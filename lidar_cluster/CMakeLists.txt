cmake_minimum_required(VERSION 3.0.2)
project(lidar_detection)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare the C++ executable and the node
add_executable(lidar_detection src/cluster.cpp)

# Specify libraries to link against
target_link_libraries(lidar_detection
  ${catkin_LIBRARIES}
)

# Install executable
install(TARGETS lidar_detection
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation (if you use any)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

