set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_ip)

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	cv_bridge
	roscpp
	std_msgs
	image_transport
	message_generation
	pcl_conversions
  	pcl_ros
  	s8_common_node
)

add_message_files(FILES
	distPose.msg
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime s8_common_node
)
include_directories(${catkin_INCLUDE_DIRS})


add_executable(imageConverter src/image_converter.cpp)
target_link_libraries(imageConverter ${catkin_LIBRARIES})

add_executable(objectTracker src/object_tracker.cpp)
target_link_libraries(objectTracker ${catkin_LIBRARIES})

add_executable(objectRecognition src/object_recognition.cpp)
target_link_libraries(objectRecognition ${catkin_LIBRARIES})

add_executable(houghcircle src/houghcircle.cpp)
target_link_libraries(houghcircle ${catkin_LIBRARIES})

add_executable(ball_detector src/ball_detector.cpp)
target_link_libraries(ball_detector ${catkin_LIBRARIES})

add_executable(shape_detector src/shape_detector.cpp)
target_link_libraries(shape_detector ${catkin_LIBRARIES})
