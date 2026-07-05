cmake_minimum_required(VERSION 3.8)
project(mdetect_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Qt meta-object compiler for the Q_OBJECT panel class.
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/goto_panel.cpp
  src/goto_panel.hpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rviz_common
  nav2_msgs
  geometry_msgs
  pluginlib
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
ament_export_libraries(${PROJECT_NAME})
ament_package()
