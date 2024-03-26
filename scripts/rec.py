
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

DEVICE = torch.device("cuda:0")      
VISUALIZE_WITH_OPEN3D = True # use open3D or pycg
LOAD_CAMERA_VIEW = False  # if using open3D, load camera view set point from json, if false then the position is overwritten
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
        # subscriber    
        self.pcd_subscriber_ = self.create_subscription(
            PointCloud2, '/filtered_pc2', self.listener_callback, 10)

        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):

        self.vis.poll_events()
        self.vis.update_renderer()

        if self.cloud_received:
            self.cloud_received = False
            self.create_mesh(self.cloud)
            self.rendered_last = True


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
        if self.rendered_last:
            # Convert ROS PointCloud2 message to numpy arrays)
            converted_cloud = self.convertCloudFromRosToOpen3d(msg)
            # estimate normals
            converted_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=40))
            # orient normals towards the camera
            converted_cloud.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
            self.cloud = converted_cloud
            self.rendered_last = False
            self.cloud_received = True

    def create_mesh(self, cloud):

        input_xyz = torch.from_numpy(np.asarray(cloud.points)).float().to(DEVICE)
        input_normal = torch.from_numpy(np.asarray(cloud.normals)).float().to(DEVICE)
        input_color = torch.from_numpy(np.asarray(cloud.colors)).float().to(DEVICE)
        nksr = Reconstructor(DEVICE)
        field = nksr.reconstruct(input_xyz, input_normal, detail_level=1.0)
        field.set_texture_field(fields.PCNNField(input_xyz, input_color))
        new_mesh = field.extract_dual_mesh(max_points=2 ** 22, mise_iter=1)

        if VISUALIZE_WITH_OPEN3D:
            # open3d visualizer
            vertices_np = new_mesh.v.cpu().detach().numpy()
            self.mesh.vertices = open3d.utility.Vector3dVector(vertices_np)
            faces_np = new_mesh.f.cpu().detach().numpy()
            self.mesh.triangles = open3d.utility.Vector3iVector(faces_np)
            colors_np = new_mesh.c.cpu().detach().numpy()
            self.mesh.vertex_colors = open3d.utility.Vector3dVector(colors_np)
            self.vis.add_geometry(self.mesh)
            # add point cloud
            # self.vis.add_geometry(cloud)
            if not UPDATE_VIEWER:
                self.vis.run()
            if self.load_camera_view and UPDATE_VIEWER:
                # load camera config
                self.view_control.convert_from_pinhole_camera_parameters(self.param, allow_arbitrary=True)
            self.vis.poll_events()
            self.vis.update_renderer()
        else:
            # pycg visualizer
            new_mesh = visualizer.mesh(new_mesh.v, new_mesh.f, color=new_mesh.c)
            visualizer.show_3d([new_mesh], [cloud])


    def convertCloudFromRosToOpen3d(self,ros_cloud):
    
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
                rgb = [rec_utils.convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [rec_utils.convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            # combine
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        # return

        return open3d_cloud


def main(args=None):
    rclpy.init(args=args)
    node = PCDListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
