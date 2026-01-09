cmake_minimum_required(VERSION 3.20)
project(sensor_proximidad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(sensor_proximidad src/sensor_proximidad.cpp)
target_link_libraries(sensor_proximidad PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

install(TARGETS sensor_proximidad
  DESTINATION lib/${PROJECT_NAME}
) 
# Exportar dependencias (opcional pero recomendado)
ament_export_dependencies(
  rclcpp
  std_msgs  
)

ament_package()
