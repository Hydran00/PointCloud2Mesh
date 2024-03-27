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

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp])

def evaluate_sensor(cloud):
    # load ground truth
    gt = open3d.io.read_point_cloud("/root/shared/Pepsi_Can.ply")
    
    # perform point-to-point ICP
    threshold = 10.0
    cloud_bef = copy.deepcopy(cloud)
    # crop on the z-axis
    bbox = open3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.04, -0.0, 0.0), max_bound=(0.06, 0.15, 0.6))
    cloud = cloud.crop(bbox)

    # translate the cloud by 0.5 in the x-axis
    trans = np.eye(4)
    trans[0, 3] = 0.5
    draw_registration_result(cloud, cloud_bef, trans)

    print("Apply point-to-point ICP")
    reg_p2p = open3d.pipelines.registration.registration_icp(
        cloud, gt, threshold, np.eye(4),
        open3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    # draw_registration_result(cloud, gt, reg_p2p.transformation)
    exit()                         
    # show the result

    # # Estimate normals
    # cloud.estimate_normals(
    #     search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # # Flip the normal
    # cloud.orient_normals_towards_camera_location()
    # # Compute the FPFH feature
    # radius_normal = voxel_size * 2


    return 