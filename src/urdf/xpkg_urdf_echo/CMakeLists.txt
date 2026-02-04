cmake_minimum_required(VERSION 3.0.2)
project(xpkg_urdf_echo)

set(X_ROS_VERSION $ENV{ROS_VERSION})
message(STATUS "Your ROS version is ROS${X_ROS_VERSION}")

add_definitions(-DX_ROS_VERSION=${X_ROS_VERSION}) 

if (${X_ROS_VERSION} STREQUAL "1")

    find_package(catkin REQUIRED)

    catkin_package()

    find_package(roslaunch)

    foreach(dir config launch meshes urdf)
        install(DIRECTORY ${dir}/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
    endforeach(dir)

elseif (${X_ROS_VERSION} STREQUAL "2")

    cmake_minimum_required(VERSION 3.8)
    
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(xacro)

    ament_package()

    foreach(dir config launch meshes urdf)
        install(DIRECTORY ${dir}
            DESTINATION share/${PROJECT_NAME})
    endforeach(dir)

else ()
    message(FATAL_ERROR "Unknown ROS version, please install the ros_environment package")
endif ()
