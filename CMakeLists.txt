cmake_minimum_required(VERSION 3.8)
project(ouster_http_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

# --- Generate the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# --- Build as a shared library for composable node
add_library(ouster_http_node_component SHARED
  src/ouster_http_node.cpp
)

ament_target_dependencies(ouster_http_node_component
  rclcpp
  rclcpp_components
  nlohmann_json
)

target_link_libraries(ouster_http_node_component CURL::libcurl)

# --- Link ROS2 message type support
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ouster_http_node_component "${cpp_typesupport_target}")

# --- Register the component
rclcpp_components_register_node(ouster_http_node_component
  PLUGIN "ouster_http_node::OusterHttpNode"
  EXECUTABLE exe
)

# --- Install the library
install(TARGETS ouster_http_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Install the executable
install(TARGETS exe
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
