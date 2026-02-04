cmake_minimum_required(VERSION 3.0.2)
project(xpkg_comm)

set(X_ROS_VERSION $ENV{ROS_VERSION})
message(STATUS "Your ROS version is ROS${X_ROS_VERSION}")

add_definitions(-DX_ROS_VERSION=${X_ROS_VERSION}) 

if (${X_ROS_VERSION} STREQUAL "1")

# 这里填写 ROS1 的相关的CMakeList
# 如果这个包不支持 ROS1 可以 写 message(FATAL_ERROR "This package does not support ROS1")
	find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  rospy
	  std_msgs
	  xpkg_msgs
	)

	catkin_package(

	)

	add_executable(xnode_comm
	src/xnode_comm.cpp
	src/ROS_INF/ros1_interface.cpp
	src/COMM_INF/comm_inf_com.cpp
	src/COMM_INF/comm_inf_tcp.cpp
	src/FUNC/comm_func.cpp
	)

	add_dependencies(xnode_comm xpkg_msgs_generate_messages_cpp)

	include_directories(
	include
	include/LIB_JSON
	include/ROS_INF
	include/COMM_INF
	include/FUNC
	  ${catkin_INCLUDE_DIRS}
	)

	target_link_libraries(xnode_comm
	   ${catkin_LIBRARIES}
	 )

#//////////////////////////////////////////////////////////////////////////////////////////////////
elseif (${X_ROS_VERSION} STREQUAL "2")

# 这里填写 ROS2 的相关的CMakeList
# 如果这个包不支持 ROS2 可以 写 message(FATAL_ERROR "This package does not support ROS2")
# 提示: cmake_minimum_required(VERSION X.X.X) 可以写多次, 建议ROS2部分顶部加一句 cmake_minimum_required 并设置为 ROS2 的 CMake 版本
#       否则可能会出现 cmake policy 相关的 警告/错误
cmake_minimum_required(VERSION 3.8)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

	# find dependencies
	find_package(ament_cmake REQUIRED)
	find_package(rclcpp REQUIRED)
	find_package(std_msgs REQUIRED)
	find_package(xpkg_msgs REQUIRED)

	include_directories(
	  ../../
	  ../
	  ./
	  include
	  include/LIB_JSON
	  include/ROS_INF
	  include/COMM_INF
	  include/FUNC
	  ${colcon_INCLUDE_DIRS}
	)


	if(BUILD_TESTING)
	  find_package(ament_lint_auto REQUIRED) 
	  ament_lint_auto_find_test_dependencies()
	endif()

	ament_package()

	add_executable(xnode_comm
	  src/xnode_comm.cpp
	  src/ROS_INF/ros2_interface.cpp
	  src/COMM_INF/comm_inf_com.cpp
	  src/COMM_INF/comm_inf_tcp.cpp
	  src/FUNC/comm_func.cpp
	)

	ament_target_dependencies(xnode_comm rclcpp std_msgs xpkg_msgs)

	install(TARGETS
	  xnode_comm
	  DESTINATION lib/${PROJECT_NAME}
	)

	install(DIRECTORY launch
	  DESTINATION share/${PROJECT_NAME}
	)

	install(DIRECTORY scripts
	  DESTINATION share/${PROJECT_NAME}
	)

else ()
    message(FATAL_ERROR "Unknown ROS version, please install the ros_environment package")
endif ()
