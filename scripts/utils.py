from sensor_msgs.msg import PointCloud2, PointField
from ctypes import *
import numpy as np
import open3d
from sensor_msgs_py import point_cloud2 as pc2
import copy
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
    open3d_cloud = open3d.geometry.PointCloud()
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
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return

    return open3d_cloud

def draw_registration_result(source, target, transformation, paint=False):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    if paint:
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp])


def trasform_cloud(cloud):
    # crop on the z-axis
    # bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.04, -0.0, 0.0), max_bound=(0.06, 0.15, 0.6)) # @ 50cm
    bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.08, -0.02, 0.0), max_bound=(0.08, 0.14, 0.6)) # @ 40cm
    cloud = cloud.crop(bbox)

    # flip by 180 degrees in the x-axis
    flip = np.eye(4)
    flip[1, 1] = -1
    flip[2, 2] = -1
    cloud.transform(flip)
    
    # translate the cloud by 0.5 in the x-axis
    trans = np.eye(4)
    trans[0, 3] = 0.5
    
    cloud_bef = copy.deepcopy(cloud)
    cloud_bef.transform(trans)
    # draw_registration_result(cloud, cloud_bef, trans)

    # translate the cloud by 0.5 in the x-axis
    flip = np.eye(4)
    # flip[0, 3] =0.05
    flip[1, 3] =0.15
    flip[2, 3] =0.45
    cloud.transform(flip)
    return cloud


def evaluate_sensor(cloud):
    # load ground truth
    gt = open3d.io.read_point_cloud("../assets/Pepsi_Can.ply")

    # open3d.visualization.draw_geometries([cloud,gt])

    # perform point-to-point ICP
    threshold = 0.2
    
    cloud = trasform_cloud(cloud)

    print("Transformed cloud")
    open3d.visualization.draw_geometries([cloud,gt])


    print("Apply Vanilla point-to-plane ICP")
    algorithm = open3d.pipelines.registration.TransformationEstimationPointToPoint()
    reg_p2p = open3d.pipelines.registration.registration_icp(
        cloud, gt, threshold, np.eye(4), algorithm)
    
    # draw_registration_result(cloud, gt, reg_p2p.transformation, paint=True)

    
    # print("Apply point-to-plane ICP with robust kernel")
    # sigma = 0.01
    # loss = open3d.pipelines.registration.TukeyLoss(k=sigma)
    # algorithm = open3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    # reg_p2p = open3d.pipelines.registration.registration_icp(
    #     cloud, gt, threshold, np.eye(4), algorithm,
    #       open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
    # )
    
    # draw_registration_result(cloud, gt, reg_p2p.transformation, paint=True)

    with open("realsense@40.txt", "a") as myfile:
        # save relative fitness and inlier RMSE 
        myfile.write(str(reg_p2p.fitness) + " " + str(reg_p2p.inlier_rmse) + "\n")

    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    
    # evaluation = open3d.pipelines.registration.evaluate_registration(
    # cloud, gt, threshold, np.eye(4))
    
    # print(evaluation, "\n")
    
    # exit()                         
    # show the result

    # # Estimate normals
    # cloud.estimate_normals(
    #     search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # # Flip the normal
    # cloud.orient_normals_towards_camera_location()
    # # Compute the FPFH feature
    # radius_normal = voxel_size * 2


    return 