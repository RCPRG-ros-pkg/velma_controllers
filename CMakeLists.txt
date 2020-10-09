cmake_minimum_required(VERSION 2.8.3)
project(velma_controller)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_rosclock
  rtt_rosparam
  std_msgs
  eigen_conversions
  cmake_modules
  image_transport
  controller_common
  rtt_actionlib
  kdl_parser
  )

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR} ${USE_OROCOS_INCLUDE_DIRS})

ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

### Orocos Targets ###

orocos_component( velma_components 
  src/components/joint_impedance.cpp
#  src/components/vector_concate1_7_7_0.cpp
#  src/components/vector_concate1_1_0_0.cpp
#  src/components/vector_concate15_2_8_8.cpp
#  src/components/vector_split1_7_7_0.cpp
#  src/components/vector_split15_2_8_8.cpp
  src/components/decimator_wrench.cpp
  src/components/cartesian_impedance.cpp
  src/components/robot_mass_matrix.cpp
  src/components/mass_test.cpp
  src/components/torque_publisher7.cpp
  src/components/join_limit_avoidance15.cpp
  src/components/double_aggregate1.cpp
  src/components/double_aggregate2.cpp
  src/components/double_split1.cpp
  src/components/double_split2.cpp
  src/components/joint_state_publisher33.cpp
  src/components/library.cpp
)
target_link_libraries(velma_components ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_service(velma_service src/velma_service.cpp)
target_link_libraries(velma_service ${catkin_LIBRARIES})

orocos_component(torso_teleop_joy src/torso_teleop_joy.cpp)
target_link_libraries(torso_teleop_joy ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(head_trajectory src/head_trajectory_generator.cpp)
target_link_libraries(head_trajectory ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(head_lookat3d src/head_lookat3d.cpp src/head_kinematics.cpp)
target_link_libraries(head_trajectory ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(velma_grav src/velma_grav.cpp)
target_link_libraries(velma_grav ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(trigger_generator src/trigger_generator.cpp)
target_link_libraries(trigger_generator ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(velocity_limiter src/velocity_limiter.cpp)
target_link_libraries(velocity_limiter ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_component(velma_fk src/velma_fk.cpp)
target_link_libraries(velma_fk ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

add_executable(image_stamper_node src/image_stamper.cpp)
target_link_libraries(image_stamper_node ${catkin_LIBRARIES})
# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  DEPENDS rtt_ros kdl_parser rtt_rosparam
)
