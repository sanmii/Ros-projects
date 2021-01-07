cmake_minimum_required(VERSION 2.8.3)
project(leire_ane)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
    cv_bridge
    rostest
    aruco_ros
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    message_generation
    chesslab_setup

)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Pickup.srv
   MovePiece.srv
   ThrowPiece.srv
   EnroquePlay.srv
   Trajectory.srv
   TrajectorySensing.srv
 )

 generate_messages(
 DEPENDENCIES
 std_msgs
 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    DEPENDS
        roscpp
        gazebo_ros
        aruco_ros
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(
    ${PROJECT_NAME}
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(aruco_frames_detection src/aruco_frames_detection.cpp)
add_dependencies(aruco_frames_detection ${leire_ane_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_frames_detection ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(movement src/movement.cpp)
add_dependencies(movement ${leire_ane_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(movement ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



install(
    TARGETS
        ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
