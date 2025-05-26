cmake_minimum_required(VERSION 3.5)
project(cobot_demo)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
set(THIS_PACKAGE_FIND
  ament_cmake
  rclcpp
  class_loader
  pluginlib
  rviz2
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  sensor_msgs
  visualization_msgs
  moveit
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
  image_transport
  cv_bridge
  OpenCV
  Eigen3
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_FIND})
  find_package(${Dependency} REQUIRED)
endforeach()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  class_loader
  pluginlib
  Qt5
  rviz2
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  sensor_msgs
  visualization_msgs
  moveit
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  Eigen3
  tf2
  tf2_ros
  tf2_eigen
  image_transport
  cv_bridge
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(move_to_pose src/move_to_pose.cpp)
ament_target_dependencies(move_to_pose ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(aruco_node src/aruco_node.cpp)
ament_target_dependencies(aruco_node ${THIS_PACKAGE_INCLUDE_DEPENDS} OpenCV ${EIGEN3_LIBRARIES})

include_directories(
  include
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#add_definitions(-DQT_NO_KEYWORDS)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/demo_panel.cpp
  src/robot_commander.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/demo_panel.h
  include/${PROJECT_NAME}/robot_commander.h
)

# Define ui file
set(${PROJECT_NAME}_UIS
  ui/form.ui
)

message(STATUS "Generate header for ui with rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
foreach(header "${${PROJECT_NAME}_HDRS}")
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${header})
endforeach()

## Add library is needed in order to generate the header file from ui file.
#add_library(${PROJECT_NAME} SHARED
add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS}
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} rviz_common::rviz_common)
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")


pluginlib_export_plugin_description_file(rviz_common demo_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  move_to_pose
  aruco_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src urdf ui rviz launch config 
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
