cmake_minimum_required(VERSION 3.8)
project(kruger_guariste_ej_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ProcessText.action"
  DEPENDENCIES std_msgs
)

# Text processor server
add_executable(text_processor_server src/text_processor_server.cpp)
ament_target_dependencies(text_processor_server rclcpp rclcpp_action std_msgs)

# Text processor client
add_executable(text_processor_client src/text_processor_client.cpp) 
ament_target_dependencies(text_processor_client rclcpp rclcpp_action std_msgs)

# Link the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(text_processor_server "${cpp_typesupport_target}")
target_link_libraries(text_processor_client "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  text_processor_server
  text_processor_client
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()