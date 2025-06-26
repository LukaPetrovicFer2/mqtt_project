cmake_minimum_required(VERSION 3.8)
project(rovc_comm_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(nlohmann_json REQUIRED)  # ✅ JSON support

# Add executable for publisher node
add_executable(publisher_node src/publisher_node.cpp)
target_link_libraries(publisher_node
  PahoMqttCpp::paho-mqttpp3
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(publisher_node rclcpp std_msgs PahoMqttCpp nlohmann_json)

# Add executable for subscriber node
add_executable(subscriber_node src/subscriber_node.cpp)
target_link_libraries(subscriber_node
  PahoMqttCpp::paho-mqttpp3
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(subscriber_node rclcpp std_msgs PahoMqttCpp nlohmann_json)

# Add executable for MQTT bridge node (pub_node)
add_executable(pub_node src/pub_node.cpp)
target_link_libraries(pub_node
  PahoMqttCpp::paho-mqttpp3
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(pub_node rclcpp std_msgs PahoMqttCpp nlohmann_json)

# Install executables
install(TARGETS
  publisher_node
  subscriber_node
  pub_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
