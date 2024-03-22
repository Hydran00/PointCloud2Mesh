import sys
import os

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import time
from sensor_msgs_py import point_cloud2


import numpy as np
import open3d as o3d

import ctypes
import struct

class PCDListener(Node):
    
    def __init__(self):
        super().__init__('pcd_subsriber_node')

        self.msg = None
        self.received = False
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.Visualizer()
        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/camera/camera/depth/color/points',                      # topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )
        
        self.vis.create_window()
        self.create_timer(0.01, self.timer_callback)
        
    def timer_callback(self):
        if (self.received):
            pcd_as_numpy_array, rgb = my_decode(self.msg)
            # if(self.first):
            self.vis.remove_geometry(self.o3d_pcd)
            self.o3d_pcd = o3d.geometry.PointCloud(
                                o3d.utility.Vector3dVector(pcd_as_numpy_array[:, :3]))
            self.o3d_pcd.colors = o3d.utility.Vector3dVector(rgb/255.0)
            self.vis.add_geometry(self.o3d_pcd)
            self.vis.update_renderer()
            self.vis.poll_events()

            self.received = False

    def listener_callback(self, msg):
        self.msg = msg
        self.received = True


def my_decode(msg):
    gen = point_cloud2.read_points(msg, skip_nans=True)

    xyz = np.empty((gen.shape[0], 3))
    rgb = np.empty((gen.shape[0], 3))
    idx = 0
    r,g,b = 0,0,0
    s = 0
    i = 0
    pack = 0
    test = 0
    for x in gen:
        test = x[3] 
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        xyz[idx] = [x[0],x[1],x[2]]
        rgb[idx] = [r,g,b]
        idx = idx + 1
    return xyz, rgb

def main(args=None):
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
