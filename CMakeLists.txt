cmake_minimum_required(VERSION 2.8.3)
project(velma_controller)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)

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

if(${OROCOS_TARGET} STREQUAL "xenomai" )
# message("Found RTNET: ${RTNET_INCLUDE_DIR}")
# include_directories(${RTNET_INCLUDE_DIR})
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}" -DXENOMAI)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  #
  orocos_add_compile_flags(torso_teleop_joy ${XENO_CFLAGS})
  orocos_add_link_flags(torso_teleop_joy ${XENO_LDFLAGS})
  TARGET_LINK_LIBRARIES(torso_teleop_joy "-lrtdm")
  
  orocos_add_compile_flags(head_trajectory ${XENO_CFLAGS})
  orocos_add_link_flags(head_trajectory ${XENO_LDFLAGS})
  TARGET_LINK_LIBRARIES(head_trajectory "-lrtdm")
endif()



# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  DEPENDS rtt_ros
)
