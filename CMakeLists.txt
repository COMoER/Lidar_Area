cmake_minimum_required(VERSION 3.0.2)
project(lidar_ros_area)


find_package(catkin REQUIRED COMPONENTS
  roscpp rospy rosbag pcl_ros cv_bridge sensor_msgs std_msgs message_generation
)


add_message_files(
  FILES
  Grid.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs  
)
## To declare and build dynamic reconfigure parameters within this
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_test
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Find pcl lib
find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)

#find_package(apriltag REQUIRED)

find_package(OpenCV REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
       # ${APRILTAG_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include/
)


add_library(${PROJECT_NAME}_area
        include/lidar_area.h
        include/lidar_area.cpp
        include/common.h
        )
#add_library(${PROJECT_NAME}_detect
#        include/common.h
#        ${PROJECT_SOURCE_DIR}/include/detect.cpp
#        ${PROJECT_SOURCE_DIR}/include/detect.hpp
#        )

target_link_libraries(${PROJECT_NAME}_area    ${catkin_LIBRARIES} 	${PCL_LIBRARIES} ${EIGEN3_LIB})

#target_link_libraries(${PROJECT_NAME}_detect    ${catkin_LIBRARIES} 	${PCL_LIBRARIES} ${APRILTAG_LIBS} ${OpenCV_LIBS} ${EIGEN3_LIB})

add_executable(${PROJECT_NAME}_node src/main.cpp)



 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} 	${PCL_LIBRARIES} ${PROJECT_NAME}_area ${EIGEN3_LIB}
 )

#add_executable(camera_position src/camera_position.cpp)

#target_link_libraries(camera_position ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${APRILTAG_LIBS} ${OpenCV_LIBS} ${PROJECT_NAME}_detect ${EIGEN3_LIB})

