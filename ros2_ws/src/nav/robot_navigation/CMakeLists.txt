cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(relative_translation SHARED src/relative_translation.cpp)
target_compile_features(relative_translation PRIVATE cxx_std_17)
ament_target_dependencies(relative_translation nav2_behaviors nav2_core nav_msgs pluginlib)
pluginlib_export_plugin_description_file(nav2_core relative_behaviors.xml)
install(TARGETS relative_translation LIBRARY DESTINATION lib)

install(
  DIRECTORY config launch maps rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
