cmake_minimum_required(VERSION 2.8.3)
project(gazebo_control_tutorial)
 
add_definitions(-std=c++11 -Wall -Wextra )#-Wpedantic -Werror)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  seven_dof_arm_gazebo
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    seven_dof_arm_gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})



# Declare the executable, along with its source files.
add_executable(pubcontrols src/pubcontrols.cpp)

# Specify libraries against which to link.
target_link_libraries(pubcontrols ${catkin_LIBRARIES})


# add_library(simple_world_plugin src/simple_world_plugin.cpp)
# target_link_libraries(simple_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(coke_world_plugin src/coke_world_plugin.cpp)
# target_link_libraries(coke_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
