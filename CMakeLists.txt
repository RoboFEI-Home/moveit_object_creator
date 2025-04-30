cmake_minimum_required(VERSION 3.8)
project(moveit_object_creator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

set(object_creator_plugins_headers_to_moc
  include/moveit_object_creator/object_tool.hpp
)

set(object_creator_plugins_source_files
  src/moveit_object_creator/object_panel.cpp
  src/moveit_object_creator/object_tool.cpp
)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/moveit_object_creator/object_panel.hpp
  include/moveit_object_creator/object_tool.hpp
)

add_library(object_creator_plugins 
  SHARED
  ${object_creator_plugins_source_files}
  ${MOC_FILES}
)
target_include_directories(object_creator_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(object_creator_plugins
  geometry_msgs
  pluginlib
  Qt5
  rclcpp
  rclcpp_lifecycle
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  tf2_geometry_msgs
  moveit_msgs
)

target_include_directories(object_creator_plugins PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(object_creator_plugins
  rviz_common::rviz_common
)

target_compile_definitions(object_creator_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

install(TARGETS object_creator_plugins
        EXPORT export_object_creator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/ObjectPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_object_creator)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()