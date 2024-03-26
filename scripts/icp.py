
# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

import torch
import open3d
import os

import utils as rec_utils
import time

# NKSR
from pycg import vis as visualizer, exp
from nksr import Reconstructor, utils, fields
import copy
DEVICE = torch.device("cuda:0")      
VISUALIZE_WITH_OPEN3D = True # use open3D or pycg
LOAD_CAMERA_VIEW = False  # if using open3D, load camera view set point from json, if false then the position is overwritten
CAMERA_FILE_NAME = 'icp.json' # file with configuration
UPDATE_VIEWER = LOAD_CAMERA_VIEW # load just the first reconstruction for a visual inspection

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])
    
class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subscriber_node')
        # point cloud
        self.pc2_ros = None
        self.device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
        self.xyz = None
        self.normal = None
        self.color = None
        self.mesh = open3d.geometry.TriangleMesh()
        self.cloud = None
        # visualizer
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window(width=1280, height=720)
        self.view_control = self.vis.get_view_control()
        self.vis.add_geometry(self.mesh)
        self.load_camera_view = LOAD_CAMERA_VIEW
        self.param = None
        if self.load_camera_view:
            self.param = open3d.io.read_pinhole_camera_parameters(CAMERA_FILE_NAME)
            self.view_control.convert_from_pinhole_camera_parameters(self.param, allow_arbitrary=True)
        # flags
        self.rendered_last = True
        self.cloud_received = False
        self.first_already_rendered = False
        # subscriber    
        self.pcd_subscriber_ = self.create_subscription(
            PointCloud2, '/filtered_pc2', self.listener_callback, 10)

        self.timer_ = self.create_timer(0.01, self.timer_callback)
        # self.timer_2 = self.create_timer(0.01, self.timer_callback2)

    # def timer_callback2(self):
    #     self.vis.poll_events()
    #     self.vis.update_renderer()

    def timer_callback(self):

        self.vis.poll_events()
        self.vis.update_renderer()

        if self.cloud_received:
            if ((not UPDATE_VIEWER and not self.first_already_rendered) or (UPDATE_VIEWER)):
                self.cloud_received = False
                self.create_mesh(self.cloud)
                self.rendered_last = True
                self.first_already_rendered = True
        if self.load_camera_view and UPDATE_VIEWER:
            # load camera config
            self.view_control.convert_from_pinhole_camera_parameters(self.param, allow_arbitrary=True)
        else:
            # save camera config
            if os.path.exists(CAMERA_FILE_NAME):
                os.remove(CAMERA_FILE_NAME)
            param = self.vis.get_view_control().convert_to_pinhole_camera_parameters()
            open3d.io.write_pinhole_camera_parameters(CAMERA_FILE_NAME, param)
            print("Open3D Camera parameters saved")
    

    def listener_callback(self, msg):
        # do not convert msg if the previous is not already processed
        if self.rendered_last and ((not UPDATE_VIEWER and not self.first_already_rendered) or (UPDATE_VIEWER)):
            # Convert ROS PointCloud2 message to numpy arrays)
            converted_cloud = self.convertCloudFromRosToOpen3d(msg)
            # estimate normals
            converted_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=40))
            # orient normals towards the camera
            converted_cloud.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
            self.cloud = converted_cloud
            self.rendered_last = False
            self.cloud_received = True


    def convertCloudFromRosToOpen3d(self,ros_cloud):
    
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check empty
        open3d_cloud = open3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        print("Converting a cloud with {} points".format(len(cloud_data)))
        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==np.float32: # if float (from pcl::toROSMsg)
                rgb = [rec_utils.convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [rec_utils.convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            # combine
            # print color size
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        # return

        return open3d_cloud


def main(args=None):
    # rclpy.init(args=args)
    # node = PCDListener()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    demo_icp_pcds = open3d.data.DemoICPPointClouds()
    source = open3d.io.read_point_cloud(demo_icp_pcds.paths[0])
    target = open3d.io.read_point_cloud(demo_icp_pcds.paths[1])
    threshold = 0.02
    trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                            [-0.139, 0.967, -0.215, 0.7],
                            [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    draw_registration_result(source, target, trans_init)
    print("Initial alignment")
    evaluation = open3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print(evaluation)

    print("Apply point-to-point ICP")
    reg_p2p = open3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
    open3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)



if __name__ == '__main__':
    main()
