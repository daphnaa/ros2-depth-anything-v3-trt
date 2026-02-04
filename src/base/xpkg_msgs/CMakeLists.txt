cmake_minimum_required(VERSION 3.0.2)
project(xpkg_msgs)

set(X_ROS_VERSION $ENV{ROS_VERSION})
message(STATUS "Your ROS version is ROS${X_ROS_VERSION}")

add_definitions(-DX_ROS_VERSION=${X_ROS_VERSION}) 

if (${X_ROS_VERSION} STREQUAL "1")

	find_package(catkin REQUIRED COMPONENTS
	  message_generation
	  roscpp
	  rospy
	  std_msgs
	)

	add_message_files(
	  FILES
      XmsgCommData.msg
	  XmsgDevice.msg
	  XmsgDeviceList.msg
	)

	generate_messages(
	  DEPENDENCIES
	  std_msgs
	)

	catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES xpkg_msgs
	 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
	#  DEPENDS system_lib
	)

	include_directories(
	# include
	  ${catkin_INCLUDE_DIRS}
	)

#//////////////////////////////////////////////////////////////////////////////////////
elseif (${X_ROS_VERSION} STREQUAL "2")

	cmake_minimum_required(VERSION 3.5)
	if(NOT CMAKE_C_STANDARD)
	  set(CMAKE_C_STANDARD 99)
	endif()

	# Default to C++14
	if(NOT CMAKE_CXX_STANDARD)
	  set(CMAKE_CXX_STANDARD 14)
	endif()

	if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	  add_compile_options(-Wall -Wextra -Wpedantic)
	endif()

	# find dependencies
	find_package(ament_cmake REQUIRED)
	find_package(rclcpp REQUIRED)
	find_package(builtin_interfaces REQUIRED)
	find_package(rosidl_default_generators REQUIRED)

	rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/XmsgCommData.msg"
	  "msg/XmsgDeviceList.msg"
	  "msg/XmsgDevice.msg"
	  DEPENDENCIES builtin_interfaces
	)

	if(BUILD_TESTING)
	  find_package(ament_lint_auto REQUIRED)
	  # the following line skips the linter which checks for copyrights
	  # uncomment the line when a copyright and license is not present in all source files
	  #set(ament_cmake_copyright_FOUND TRUE)
	  # the following line skips cpplint (only works in a git repo)
	  # uncomment the line when this package is not in a git repo
	  #set(ament_cmake_cpplint_FOUND TRUE)
	  ament_lint_auto_find_test_dependencies()
	endif()

	ament_package()

else ()
    message(FATAL_ERROR "Unknown ROS version, please install the ros_environment package")
endif ()
