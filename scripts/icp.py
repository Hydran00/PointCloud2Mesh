
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
from time import sleep

# NKSR
import copy
DEVICE = torch.device("cuda:0")      
VISUALIZE_WITH_OPEN3D = True # use open3D or pycg
LOAD_CAMERA_VIEW = True  # if using open3D, load camera view set point from json, if false then the position is overwritten
CAMERA_FILE_NAME = 'zed.json' # file with configuration
UPDATE_VIEWER = LOAD_CAMERA_VIEW # load just the first reconstruction for a visual inspection

    
    
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
        self.clouds = []
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
        
        self.cloud_received = False
        self.first_already_rendered = False
        # subscriber    
        self.pcd_subscriber_ = self.create_subscription(
            PointCloud2, '/filtered_pc2', self.listener_callback, 10)

        self.timer_ = self.create_timer(0.01, self.timer_callback)
        # self.timer_2 = self.create_timer(0.01, self.timer_callback2)
        self.num_clouds = 0
    # def timer_callback2(self):
    #     self.vis.poll_events()
    #     self.vis.update_renderer()
    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        self.vis.add_geometry(source_temp)
        self.vis.add_geometry(target_temp)
        self.vis.poll_events()
        self.vis.run()

    def timer_callback(self):

        self.vis.poll_events()
        self.vis.update_renderer()

        if self.cloud_received:
            self.cloud_received = False
        
        if self.load_camera_view and UPDATE_VIEWER:
            # load camera config
            self.view_control.convert_from_pinhole_camera_parameters(self.param, allow_arbitrary=True)
            if self.num_clouds == 2:
                self.icp(self.clouds[0], self.clouds[1]) 
        else:
            # save camera config
            if os.path.exists(CAMERA_FILE_NAME):
                os.remove(CAMERA_FILE_NAME)
            param = self.vis.get_view_control().convert_to_pinhole_camera_parameters()
            open3d.io.write_pinhole_camera_parameters(CAMERA_FILE_NAME, param)
            # print("Open3D Camera parameters saved")

    
    def icp(self, source, target):
        threshold = 10.0
        trans_init = np.eye(4)
        reg_p2p = open3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
        open3d.pipelines.registration.TransformationEstimationPointToPoint())


        print("Apply point-to-point ICP")
    
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        self.draw_registration_result(source, target, reg_p2p.transformation)
    
    def listener_callback(self, msg):
        self.num_clouds += 1    
        if self.num_clouds > 2:
            return
        if self.num_clouds == 1:
            print("Please move camera and press enter...")
            sleep(3)
        # do not convert msg if the previous is not already processed
        # Convert ROS PointCloud2 message to numpy arrays)
        converted_cloud = self.convertCloudFromRosToOpen3d(msg)
        # estimate normals
        converted_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=40))
        # orient normals towards the camera
        converted_cloud.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
        
        self.clouds.append(converted_cloud)
        print("Received cloud num: ", self.num_clouds)
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


    # print("Apply point-to-point ICP")

    # print(reg_p2p)
    # print("Transformation is:")
    # print(reg_p2p.transformation)
    # draw_registration_result(source, target, reg_p2p.transformation)

    # read txt file that contains two values with numpy
    with open('realsense_50.txt', 'r') as data:
            x = []
            y = []
            for line in data:
                p = line.split()
                x.append(float(p[0]))
                y.append(float(p[1]))
    # print(x)
    # print(y)
    print("Mean relative fitness: ",np.mean(x))
    print("Var relative fitness: ",np.var(x))
    print("Mean inlier RMSE: ",np.mean(y))
    print("Var inlier RMSE: ",np.var(y))

if __name__ == '__main__':
    main()
