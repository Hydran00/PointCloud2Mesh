#include <open3d/Open3D.h>

#include <chrono>
#include <iostream>

#include "open3d_conversions/open3d_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

class PCDListener : public rclcpp::Node
{
public:
  open3d::visualization::Visualizer visualizer;

  PCDListener() : Node("pcd_subsriber_node")
  {
    received_ = false;

    pcd_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/filtered_pc2", 10,
        std::bind(&PCDListener::listener_callback, this,
                  std::placeholders::_1));
    timer_ = this->create_wall_timer(
        40ms, std::bind(&PCDListener::timer_callback, this));

    // init visualization
    visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
    std::cout << "Visualizer created" << std::endl;
  }

private:
  void timer_callback()
  {
    if (received_ && first)
    {
      pc2_o3d_ptr = std::make_shared<open3d::geometry::PointCloud>(pc2_o3d);
      open3d_conversions::rosToOpen3d(pc2_ros, *pc2_o3d_ptr);
      first = false;
      received_ = false;
    }
    if (received_ && a)
    {
      std::cout << "Timer callback" << std::endl;
      visualizer.RemoveGeometry(pc2_o3d_ptr);
      open3d_conversions::rosToOpen3d(pc2_ros, *pc2_o3d_ptr);
      visualizer.AddGeometry(pc2_o3d_ptr);
      pc2_o3d_ptr->EstimateNormals();

      //       # estimate radius for rolling ball
      // distances = pcd.compute_nearest_neighbor_distance()
      // avg_dist = np.mean(distances)
      // radius = 1.5 * avg_dist
      auto distances = pc2_o3d_ptr->ComputeNearestNeighborDistance();
      double avg_dist = 0;
      for (auto dist : distances) {
        avg_dist += dist;
      }
      avg_dist /= distances.size();

      std::vector<double> radii = {5 * avg_dist};
      std::cout << "Waiting for the mesh" << std::endl;
      auto rec_mesh =
          open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(
              *pc2_o3d_ptr, radii);
      // visualizer.AddGeometry(pc2_open3d_ptr);
      visualizer.AddGeometry(rec_mesh);
      received_ = false;
      first = false;
      a = false;
      std::cout << "Mesh created" << std::endl;
      // open3d::io::WriteTriangleMesh("mesh.ply", *rec_mesh);
      // exit(0);
    }
    Eigen::Vector3d front = Eigen::Vector3d(
        0.014839371860051669, 0.38389557435654953, -0.92325726698047395);
    Eigen::Vector3d lookat = Eigen::Vector3d(
        -0.30690958816078184, -1.0919699054014078, 2.653967750598559);
    Eigen::Vector3d up = Eigen::Vector3d(
        -0.078439667039420083, -0.92006636317111679, -0.38382952725893771);
    double zoom = 0.42;
    // visualizer.GetViewControl().SetFront(front);
    // visualizer.GetViewControl().SetLookat(lookat);
    // visualizer.GetViewControl().SetUp(up);
    // visualizer.GetViewControl().SetZoom(zoom);

    visualizer.UpdateRender();
    visualizer.PollEvents();
  }

  void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pc2_ros = msg;
    received_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pcd_subscriber_;
  std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr pc2_ros;
  open3d::geometry::PointCloud pc2_o3d;
  std::shared_ptr<open3d::geometry::PointCloud> pc2_o3d_ptr;
  bool received_ = false;
  bool first = true;
  bool a = true;
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
