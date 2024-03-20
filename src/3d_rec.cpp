#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "open3d_conversions/open3d_conversions.hpp"

#include <open3d/Open3D.h>

#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

class PCDListener : public rclcpp::Node
{
public:
    PCDListener()
        : Node("pcd_subsriber_node")
    {
        received_ = false;

        pcd_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points",
            10,
            std::bind(&PCDListener::listener_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(40ms, std::bind(&PCDListener::timer_callback, this));
        
        // init visualization
        visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
        // visualizer.AddGeometry(std::make_shared<const open3d::geometry::PointCloud>(pc2_o3d));
        // visualizer.Run();
    }
    // visualizer
    open3d::visualization::Visualizer visualizer;
private:
    void timer_callback()
    {
        if (received_ && first)
        {
            open3d_conversions::rosToOpen3d(pc2_ros, pc2_o3d);
            geometries.clear();
            visualizer.ClearGeometries();
            geometries.push_back(std::make_shared<const open3d::geometry::PointCloud>(pc2_o3d)); // Convert to const Geometry
            visualizer.AddGeometry(geometries[0]);
            // visualizer.UpdateGeometry(std::make_shared<const open3d::geometry::PointCloud>(pc2_o3d));
            
            received_ = false;
            first = false;
        }
        visualizer.UpdateRender();
        visualizer.PollEvents();
    }

    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pc2_ros = msg;
        received_ = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_subscriber_;
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_ros;
    open3d::geometry::PointCloud pc2_o3d;
    bool received_;
    bool first = true;


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDListener>();
    rclcpp::spin(node);
    node->visualizer.DestroyVisualizerWindow();
    rclcpp::shutdown();
    return 0;
}
