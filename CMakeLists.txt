cmake_minimum_required(VERSION 3.8)
project(ouster_http_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

# --- Generate the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# --- Create executable
add_executable(exe src/ouster_http_node.cpp)

ament_target_dependencies(exe
  rclcpp
  nlohmann_json
)

target_link_libraries(exe CURL::libcurl)

# --- Link ROS2 message type support
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(exe "${cpp_typesupport_target}")

# --- Install the executable
install(TARGETS exe
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
