cmake_minimum_required(VERSION 3.8)
project(imu_odom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME}_node src/imu_odom_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    tf2_geometry_msgs
    tf2_ros
    nav_msgs
)
target_link_libraries(${PROJECT_NAME}_node
  Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
