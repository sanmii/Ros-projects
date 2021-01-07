# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(agitr_chapter8_plus)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    turtlesim
    std_msgs
    message_generation
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Changerate.srv
   Changevelocity.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

# Declare our catkin package.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demopkg
   CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(pubvel_toggle src/pubvel_toggle.cpp)
add_executable(pubvel_toggle_plus src/pubvel_toggle_plus.cpp)
add_executable(spawn_turtle src/spawn_turtle.cpp)
add_executable(spawn_turtle_plus src/spawn_turtle_plus.cpp)

add_executable(improved_pubvel_toggle src/improved_pubvel_toggle.cpp)

# Specify libraries against which to link.
target_link_libraries(pubvel_toggle ${catkin_LIBRARIES})
target_link_libraries(pubvel_toggle_plus ${catkin_LIBRARIES})
target_link_libraries(improved_pubvel_toggle ${catkin_LIBRARIES})
add_dependencies(pubvel_toggle_plus agitr_chapter8_plus_gencpp)
add_dependencies(improved_pubvel_toggle agitr_chapter8_plus_gencpp)
target_link_libraries(spawn_turtle ${catkin_LIBRARIES})
target_link_libraries(spawn_turtle_plus ${catkin_LIBRARIES})
