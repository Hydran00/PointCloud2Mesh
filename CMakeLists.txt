cmake_minimum_required(VERSION 3.8)
project(3d_rec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)   
find_package(sensor_msgs REQUIRED)
find_package(open3d_conversions REQUIRED)
find_package(Open3D REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(rec src/3d_rec.cpp)
ament_target_dependencies(rec rclcpp sensor_msgs open3d_conversions)
target_link_libraries(rec Open3D::Open3D)

install(TARGETS
  rec
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME})


ament_package()
