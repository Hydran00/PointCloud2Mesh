import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import torch
import nksr
from sensor_msgs_py import point_cloud2 as pc2
from pycg import vis

import torch
import open3d
import utils
import time
# Visualizing 
#   (Use whatever other libraries you like, here mesh.v is Vx3 array and mesh.f is Tx3 triangle index array)
from pycg import vis

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subscriber_node')
        self.received = False
        self.pc2_ros = None
        self.device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
        self.xyz = None
        self.normal = None
        self.color = None
        self.mesh = None
        self.vis = open3d.visualization.Visualizer()
        self.vis.create_window()

        self.pcd_subscriber_ = self.create_subscription(
            PointCloud2, '/filtered_pc2', self.listener_callback, 10)
            # PointCloud2, '/camera/camera/depth/color/points', self.listener_callback, 10)
        self.timer_ = self.create_timer(0.04, self.timer_callback)

    def timer_callback(self):
        if self.received:
            pass

    def listener_callback(self, msg):
        # Convert ROS PointCloud2 message to numpy arrays
        # points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
        # normals = np.array([[n[0], n[1], n[2]] for n in pc2.read_points(msg, field_names=("normal_x", "normal_y", "normal_z"), skip_nans=True)])
        # colors = np.array([[c[0], c[1], c[2]] for c in pc2.read_points(msg, field_names=("r", "g", "b"), skip_nans=True)])

        # # Convert numpy arrays to torch tensors
        # self.xyz = torch.tensor(points, dtype=torch.float32, device=self.device)
        # self.normal = torch.tensor(normals, dtype=torch.float32, device=self.device)
        # self.color = torch.tensor(colors, dtype=torch.float32, device=self.device)

        # self.received = True
        # self.get_logger().info("Received point cloud")
        converted_cloud = self.convertCloudFromRosToOpen3d(msg)
        # save ply
        # print("Saving the point cloud as a ply file")
        open3d.io.write_point_cloud("box.pcd", converted_cloud)

        # converted_cloud = open3d.io.read_point_cloud("copy_of_fragment.pcd")
        start = time.time()
        # estimate normals
        converted_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=40))

        # orient normals towards the camera
        converted_cloud.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
        end = time.time()
        print("Time taken: ",end-start)
        open3d.io.write_point_cloud("box_normals.pcd", converted_cloud)
        # visualize
        self.vis.add_geometry(converted_cloud)
        print("Visualizing the point cloud")
        rclpy.shutdown()

    def convertCloudFromRosToOpen3d(self,ros_cloud):
    
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check empty
        open3d_cloud = open3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==np.float32: # if float (from pcl::toROSMsg)
                rgb = [utils.convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [utils.convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            # combine
            # print color size
            print("Color size: ",np.array(rgb).shape)
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
