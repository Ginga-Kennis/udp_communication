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

add_executable(udp_sender src/udp_sender.cpp)

add_dependencies(udp_sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(udp_sender ${catkin_LIBRARIES})

add_executable(udp_receiver src/udp_receiver.cpp)

add_dependencies(udp_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(udp_receiver ${catkin_LIBRARIES})
