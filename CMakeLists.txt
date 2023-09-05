cmake_minimum_required(VERSION 3.5)
project(kinova_pid VERSION 0.1)
set(CMAKE_CXX_STANDARD 11)


set(CMAKE_BUILD_TYPE Release)

set(KORTEX_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/kortex_api")
set(KORTEX_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/kortex_api")


# pinocchio
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED "pinocchio")
add_definitions(${pinocchio_CFLAGS})

# ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)

catkin_package()


link_directories(${pinocchio_LIBRARY_DIRS})
include_directories(
  include/
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
)


#for kortex API
include_directories(
  ${KORTEX_INCLUDE_DIR}
  ${KORTEX_INCLUDE_DIR}/client
  ${KORTEX_INCLUDE_DIR}/common
  ${KORTEX_INCLUDE_DIR}/messages
  ${KORTEX_INCLUDE_DIR}/client_stubs
  ${KORTEX_INCLUDE_DIR}/cxxopts/
  ${KORTEX_INCLUDE_DIR}/messages/
)
add_definitions(-D_OS_UNIX)

set(
  SRC_FILES 
  src/gen3.cpp
  src/gen3_node.cpp
)

add_executable(gen3_node ${SRC_FILES})
target_link_libraries(gen3_node ${KORTEX_LIB_DIR}/libKortexApiCpp.a)
target_link_libraries(gen3_node ${pinocchio_LIBRARIES})
target_link_libraries(gen3_node pthread)
target_link_libraries(gen3_node ${catkin_LIBRARIES})