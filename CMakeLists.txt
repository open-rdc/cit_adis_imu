cmake_minimum_required(VERSION 2.8.3)
project(cit_adis_imu)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    tf
    std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_node src/imu_node.cpp src/Comm.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)

target_link_libraries(imu_node ${catkin_LIBRARIES})
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

