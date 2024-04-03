
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
UPDATE_VIEWER = True#LOAD_CAMERA_VIEW # load just the first reconstruction for a visual inspection


class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subscriber_node')
        # point cloud
        self.pc2_ros = None
        self.device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
        print("Using device: ", self.device)
        self.xyz = None
        self.normal = None
        self.color = None
        self.mesh = open3d.t.geometry.TriangleMesh()
        self.cloud = None
        # visualizer
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window(width=1280, height=720)
        self.view_control = self.vis.get_view_control()
        self.vis.add_geometry(self.mesh.to_legacy())
        self.load_camera_view = LOAD_CAMERA_VIEW
        self.param = None
        if self.load_camera_view:
            self.param = open3d.t.io.read_pinhole_camera_parameters(CAMERA_FILE_NAME)
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
            # self.create_mesh(self.cloud)
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
            # print("Open3D Camera parameters saved")
    

    def listener_callback(self, msg):
        print("Received point cloud")
        # do not convert msg if the previous is not already processed
        if self.rendered_last:
            # Convert ROS PointCloud2 message to numpy arrays)
            converted_cloud = rec_utils.convertCloudFromRosToOpen3d(msg)
            # estimate normals
            converted_cloud.estimate_normals(radius=1, max_nn=40)
            # orient normals towards the camera
            converted_cloud.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
            # draw geom
            open3d.visualization.draw_geometries([converted_cloud.to_legacy()])
            self.cloud = converted_cloud
            self.rendered_last = False
            self.cloud_received = True
            # save ply
            print("Saving point cloud")
            open3d.t.io.write_point_cloud("cloud.ply", converted_cloud)
            time.sleep(5.0)

    def create_mesh(self, cloud):
        cloud = cloud.cuda(0)
        input_xyz = torch.utils.dlpack.from_dlpack(cloud.point.positions.to_dlpack())
        input_normal = torch.utils.dlpack.from_dlpack(cloud.point.normals.to_dlpack())
        input_color = torch.utils.dlpack.from_dlpack(cloud.point.colors.to_dlpack())

        nksr = Reconstructor(DEVICE)
        field = nksr.reconstruct(input_xyz, input_normal, detail_level=1.0)
        field.set_texture_field(fields.PCNNField(input_xyz, input_color))
        new_mesh = field.extract_dual_mesh(max_points=2 ** 22, mise_iter=1)

        if VISUALIZE_WITH_OPEN3D:
            # open3d visualizer
            vertices_np = new_mesh.v.cpu().detach().numpy()
            self.mesh.vertex.positions = open3d.core.Tensor(vertices_np, dtype=open3d.core.Dtype.Float32)
            faces_np = new_mesh.f.cpu().detach().numpy()
            self.mesh.triangle.indices = open3d.core.Tensor(faces_np, dtype=open3d.core.Dtype.Int32)
            colors_np = new_mesh.c.cpu().detach().numpy()
            self.mesh.vertex.colors = open3d.core.Tensor(colors_np, dtype=open3d.core.Dtype.Float32)
            # self.vis.add_geometry(self.mesh)
            # add point cloud
            # self.vis.add_geometry(cloud)
            if not UPDATE_VIEWER:
                self.vis.run()
                # rec_utils.evaluate_sensor(cloud)
            if self.load_camera_view and UPDATE_VIEWER:
                # load camera config
                self.view_control.convert_from_pinhole_camera_parameters(self.param, allow_arbitrary=True)
            self.vis.poll_events()
            self.vis.update_renderer()
        else:
            # pycg visualizer
            new_mesh = visualizer.mesh(new_mesh.v, new_mesh.f, color=new_mesh.c)
            visualizer.show_3d([new_mesh], [cloud])





def main(args=None):
    rclpy.init(args=args)
    node = PCDListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
