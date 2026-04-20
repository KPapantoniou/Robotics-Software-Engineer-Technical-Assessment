cmake_minimum_required(VERSION 3.8)
project(linear_algebra_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(linear_algebra_service REQUIRED)
find_package(Eigen3 REQUIRED)

# set(SOURCE_FILES src/client_node.cpp server_node.cpp)

add_executable(server_node src/server_node.cpp)
add_executable(client_node src/client_node.cpp)

ament_target_dependencies(server_node rclcpp geometry_msgs linear_algebra_service Eigen3)
ament_target_dependencies(client_node rclcpp geometry_msgs linear_algebra_service Eigen3)

target_link_libraries(server_node yaml-cpp)
target_link_libraries(client_node yaml-cpp)

install(TARGETS
  server_node
  client_node
  DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
