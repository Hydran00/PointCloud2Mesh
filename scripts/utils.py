from sensor_msgs.msg import PointCloud2, PointField
from ctypes import *
import numpy as np
import open3d
from sensor_msgs_py import point_cloud2 as pc2
import copy

# if open3d.__DEVICE_API__ == 'cuda':
#     import open3d.cuda.pybind.t.pipelines.registration as treg
# else:
treg = open3d.t.pipelines.registration


CAMERA="ZED"
SHOW_RESULT = True
gt = open3d.t.io.read_point_cloud("../assets/Pepsi_Can.ply")



# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)



def convertCloudFromRosToOpen3d(ros_cloud):

    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = open3d.t.geometry.PointCloud()
    if len(cloud_data)==0:
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==np.float32: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        # combine
        open3d_cloud.point.positions = open3d.core.Tensor(np.array(xyz), dtype=open3d.core.Dtype.Float32)
        open3d_cloud.point.colors = open3d.core.Tensor(np.array(rgb)/255.0, dtype=open3d.core.Dtype.Float32)
    else:
        pass
        # xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        # open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return

    return open3d_cloud

def draw_registration_result(source, target, transformation, paint=False):
    # source_temp = copy.deepcopy(source)
    # target_temp = copy.deepcopy(target)
    if paint:
        source.paint_uniform_color([1, 0.706, 0])
        target.paint_uniform_color([0, 0.651, 0.929])
    source.transform(transformation)
    source = source.to_legacy()
    target = target.to_legacy()
    open3d.visualization.draw_geometries([source, target])


def trasform_cloud(cloud,gt):
    # open3d.visualization.draw_geometries([cloud])

    # crop on the z-axis
    # bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.04, -0.0, 0.0), max_bound=(0.06, 0.15, 0.6)) # @ 50cm

    # cloud = cloud.crop(bbox)

    if CAMERA == "RS":
        # flip by 180 degrees in the x-axis
        flip = np.eye(4)
        flip[1, 1] = -1.0
        flip[2, 2] = -1.0
        cloud.transform(flip)
    else:
        # flip by 110 degrees in the x-axis
        flip = np.array([[1, 0, 0, 0],
                         [0, 0, -1, 0],
                         [0, 1, 0, 0],
                         [0, 0, 0, 1]])
        cloud.transform(flip)
        # # flip by 180 degrees in the y-axis
        flip = np.array([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])
        cloud.transform(flip)
        
        bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, -0.05), max_bound=(1, 1 , 0.05))
        cloud = cloud.crop(bbox)        
    # if SHOW_RESULT:
    #     print("Transformed cloud 1")
    #     open3d.visualization.draw_geometries([cloud,gt])
    
    # translate the cloud by 0.5 in the x-axis
    trans = np.eye(4)
    trans[0, 3] = 0.5
    
    cloud_bef = copy.deepcopy(cloud)
    cloud_bef.transform(trans)
    # draw_registration_result(cloud, cloud_bef, trans)

    # translate the cloud by 0.5 in the x-axis
    flip = np.eye(4)
    # flip[0, 3] =0.05
    if CAMERA == "RS":
        flip[1, 3] = 0.15
        flip[2, 3] = 0.45
    else:
        pass
        flip[1, 3] = 0.17
        flip[2, 3] = 0.0
        flip[0, 3] = -0.5

    cloud.transform(flip)
    return cloud





def evaluate_sensor(cloud):
    print("Evaluate sensor")

    global gt
    cloud = cloud.to_legacy()
    gt = gt.to_legacy()
    cloud = trasform_cloud(cloud,gt)

    if SHOW_RESULT:
        print("Transformed cloud 2")
        open3d.visualization.draw_geometries([cloud,gt])


    print("Apply Vanilla point-to-plane ICP")
    threshold = 0.1

    # algorithm = open3d.pipelines.registration.TransformationEstimationPointToPoint()
    
    cloud = open3d.t.geometry.PointCloud.from_legacy(cloud).cuda(0)
    gt = open3d.t.geometry.PointCloud.from_legacy(gt).cuda(0)

    # voxel_sizes = open3d.utility.DoubleVector([0.1, 0.05, 0.025])    
    # criteria_list = [
    # treg.ICPConvergenceCriteria(relative_fitness=0.0001,
    #                             relative_rmse=0.0001,
    #                             max_iteration=1000),
    # treg.ICPConvergenceCriteria(0.00001, 0.00001, 15),
    # treg.ICPConvergenceCriteria(0.000001, 0.000001, 10)
    # ]
    # max_correspondence_distances = open3d.utility.DoubleVector([0.1,0.1,0.1])
    # init_source_to_target = open3d.core.Tensor.eye(4, open3d.core.Dtype.Float32)
    # estimation = treg.TransformationEstimationPointToPlane()
    
    # # Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
    # callback_after_iteration = lambda loss_log_map : print("Iteration Index: {}, Scale Index: {}, Scale Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
    #     loss_log_map["iteration_index"].item(), 
    #     loss_log_map["scale_index"].item(), 
    #     loss_log_map["scale_iteration_index"].item(), 
    #     loss_log_map["fitness"].item(), 
    #     loss_log_map["inlier_rmse"].item()))

    # registration_ms_icp = treg.multi_scale_icp(source=cloud,target=gt,voxel_sizes=voxel_sizes,
    #                                         criteria_list=criteria_list,
    #                                         max_correspondence_distances=max_correspondence_distances,
    #                                         init_source_to_target=init_source_to_target,
    #                                         estimation=estimation)
    voxel_sizes = open3d.utility.DoubleVector([0.1, 0.05, 0.025])

    # List of Convergence-Criteria for Multi-Scale ICP:
    criteria_list = [
        treg.ICPConvergenceCriteria(relative_fitness=0.0001,
                                    relative_rmse=0.0001,
                                    max_iteration=20),
        treg.ICPConvergenceCriteria(0.00001, 0.00001, 15),
        treg.ICPConvergenceCriteria(0.000001, 0.000001, 10)
    ]

    # `max_correspondence_distances` for Multi-Scale ICP (open3d.utility.DoubleVector):
    max_correspondence_distances = open3d.utility.DoubleVector([0.2, 0.2, 0.2])

    # Initial alignment or source to target transform.
    init_source_to_target = open3d.core.Tensor.eye(4, open3d.core.Dtype.Float32)

    # Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
    estimation = treg.TransformationEstimationPointToPlane()

    # Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
    callback_after_iteration = lambda loss_log_map : print("Iteration Index: {}, Scale Index: {}, Scale Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
        loss_log_map["iteration_index"].item(),
        loss_log_map["scale_index"].item(),
        loss_log_map["scale_iteration_index"].item(),
        loss_log_map["fitness"].item(),
        loss_log_map["inlier_rmse"].item()))
    
    
    
    registration_ms_icp = treg.multi_scale_icp(cloud, gt, voxel_sizes,
                                           criteria_list,
                                           max_correspondence_distances,
                                           init_source_to_target, estimation,
                                           callback_after_iteration)

    print("Inlier Fitness: ", registration_ms_icp.fitness)
    print("Inlier RMSE: ", registration_ms_icp.inlier_rmse)

    # draw_registration_result(cloud, gt, registration_ms_icp.transformation)
 
    # cloud = cloud.to_legacy()
    # gt = gt.to_legacy()
    draw_registration_result(cloud, gt,registration_ms_icp.transformation, paint=True)


    # if SHOW_RESULT:
    #     draw_registration_result(cloud.cpu(), gt.cpu(), reg_p2p.transformation, paint=True)

    
    # print("Apply point-to-plane ICP with robust kernel")
    # sigma = 0.01
    # loss = open3d.pipelines.registration.TukeyLoss(k=sigma)
    # algorithm = open3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    # reg_p2p = open3d.pipelines.registration.registration_icp(
    #     cloud, gt, threshold, np.eye(4), algorithm,
    #       open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
    # )
    
    # draw_registration_result(cloud, gt, reg_p2p.transformation, paint=True)

    with open("zed@50.txt", "a") as myfile:
        # save relative fitness and inlier RMSE 
        myfile.write(str(registration_ms_icp.fitness) + " " + str(registration_ms_icp.inlier_rmse) + "\n")

    return 

def crop_cloud(cloud):
    x_min = -1.6
    x_max = 1
    y_min = -1.9
    y_max = 0.8 
    z_min = 0.0
    z_max = 2.46

    bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
    cloud = cloud.crop(bbox)
    return cloud

if __name__ == "__main__":
    cloud = open3d.io.read_point_cloud("clown_cloud2.ply")
    cloud = crop_cloud(cloud)
    # rotate by 180 degrees in the z-axis
    flip = np.eye(4)
    flip[0, 0] = -1.0
    flip[1, 1] = -1.0
    cloud = cloud.transform(flip)
    # rotate by 180 degrees in the y-axis
    flip = np.eye(4)
    flip[0, 0] = -1.0
    flip[2, 2] = -1.0
    cloud = cloud.transform(flip)
    # translate the cloud by -2.5 in the y-axis
    flip = np.eye(4)
    flip[0, 3] = 0.3
    flip[1, 3] = -0.3
    flip[2, 3] = 2.4

    cloud = cloud.transform(flip)
    open3d.visualization.draw_geometries([cloud])
    # outlier removal
    cloud, ind = cloud.remove_statistical_outlier(nb_neighbors=1000, std_ratio=2.0)
    open3d.visualization.draw_geometries([cloud])

    # save
    open3d.io.write_point_cloud("crop_clown_cloud2.ply", cloud)