cmake_minimum_required(VERSION 3.20)
project(cpp_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(server src/add_two_ints_server.cpp)
add_executable(client src/add_two_ints_client.cpp)
#ament_target_dependencies(server rclpp example_interfaces)
#target_link_libraries(server PUBLIC
#  example_interfaces
#  rclcpp::rclcpp
#)ros
target_link_libraries(server PUBLIC
  ${example_interfaces_TARGETS}
  rclcpp::rclcpp
)

target_link_libraries(client PUBLIC
  ${example_interfaces_TARGETS}
  rclcpp::rclcpp
)
# ===== INSTALACIÓN =====
install(TARGETS server client
  DESTINATION lib/${PROJECT_NAME}
)

# Exportar dependencias (opcional pero recomendado)
ament_export_dependencies(
  rclcpp
  example_interfaces  
)


ament_package()
