cmake_minimum_required(VERSION 3.5)
project(slam_visualization VERSION 3.1.0)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)

if (NOT WIN32)
    find_package(rclcpp REQUIRED)
    find_package(lidar_slam REQUIRED)
    find_package(rviz_common REQUIRED)
    find_package(rviz_rendering REQUIRED)
    find_package(rviz_default_plugins REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(Qt5 REQUIRED COMPONENTS Widgets)

    ## This setting causes Qt's "MOC" generation to happen automatically.
    set(CMAKE_AUTOMOC ON)

    ###########
    ## Build ##
    ###########

    add_library(${PROJECT_NAME} SHARED
        src/SlamControlPanel.cxx
    )

    ament_target_dependencies(${PROJECT_NAME}
        lidar_slam
        rclcpp
        rviz_common
        rviz_rendering
        rviz_default_plugins
        sensor_msgs
        geometry_msgs
    )

    target_include_directories(${PROJECT_NAME} PUBLIC
        ${Qt5Widgets_INCLUDE_DIRS})


    # Export plugin description
    pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

    # Causes the visibility macros to use dllexport rather than dllimport (for Windows, when your plugin should be used as library)
    target_compile_definitions(${PROJECT_NAME} PRIVATE "SLAM_VISUALIZATION_BUILDING_LIBRARY")

    ament_export_dependencies(
        geometry_msgs
        rclcpp
        rviz_common
        rviz_rendering
        rviz_default_plugins
        sensor_msgs
    )

    #############
    ## Install ##
    #############

    ## Mark libraries for installation
    ## See https://github.com/ros-visualization/visualization_tutorials/blob/ros2/rviz_plugin_tutorials/CMakeLists.txt
    ## For tutorial to integrate a plugin in rviz2
    install(TARGETS ${PROJECT_NAME}
            EXPORT ${PROJECT_NAME}
            ARCHIVE DESTINATION lib/${PROJECT_NAME}
            LIBRARY DESTINATION lib/${PROJECT_NAME}
            RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
else()
    message(WARNING "rviz plugin is not supported on Windows.\nslam_visualization not build")
endif()

ament_package()
