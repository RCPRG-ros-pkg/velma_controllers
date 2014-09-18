cmake_minimum_required(VERSION 2.8.3)
project(velma_controller)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros rtt_rosclock std_msgs cmake_modules
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

### Orocos Targets ###

#orocos_component(velma_controller src/velma_controller.cpp)
#target_link_libraries(velma_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

#orocos_component(velma_sim src/velma_sim.cpp)
#target_link_libraries(velma_sim ${orocos_kdl_LIBRARIES})

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

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  DEPENDS rtt_ros
)
