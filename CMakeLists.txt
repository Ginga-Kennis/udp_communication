cmake_minimum_required(VERSION 3.0.2)
project(udp_communication)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(base_udp src/base_udp.cpp)

add_executable(sender_node src/sender_node.cpp)
add_dependencies(sender_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sender_node base_udp ${catkin_LIBRARIES})

add_executable(receiver_node src/sender_node.cpp)
add_dependencies(receiver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(receiver_node base_udp ${catkin_LIBRARIES})
