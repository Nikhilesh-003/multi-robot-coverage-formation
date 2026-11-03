cmake_minimum_required(VERSION 3.8)
project(mrc_costmap_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(mrc_peer_layer SHARED src/peer_layer.cpp)
target_include_directories(mrc_peer_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mrc_peer_layer PUBLIC cxx_std_17)
ament_target_dependencies(mrc_peer_layer
  rclcpp nav2_costmap_2d pluginlib geometry_msgs tf2 tf2_geometry_msgs)

pluginlib_export_plugin_description_file(nav2_costmap_2d peer_layer_plugin.xml)

install(TARGETS mrc_peer_layer
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(mrc_peer_layer)
ament_export_dependencies(rclcpp nav2_costmap_2d pluginlib geometry_msgs tf2 tf2_geometry_msgs)
ament_package()
