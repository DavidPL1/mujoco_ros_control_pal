cmake_minimum_required(VERSION 3.0.2)

project(mujoco_ros_control_pal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  hardware_interface 
  joint_limits_interface
  transmission_interface
  controller_manager
  control_toolbox
  pluginlib
  angles
  mujoco_ros
  urdf
  roscpp
)

# Depend on system install of Gazebo 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
    hardware_interface
    transmission_interface
    controller_manager
    pluginlib
    urdf
    mujoco_ros
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} default_robot_hw_sim_pal
)

message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME} src/mujoco_ros_control_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(default_robot_hw_sim_pal src/default_robot_hw_sim.cpp
                                 src/internal/joint_state.cpp
                                 src/internal/position_joint.cpp
                                 src/internal/velocity_joint.cpp
                                 src/internal/effort_joint.cpp)
target_link_libraries(default_robot_hw_sim_pal ${catkin_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME} default_robot_hw_sim_pal
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES robot_hw_sim_pal_plugins.xml mujoco_ros_control_pal_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )