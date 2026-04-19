cmake_minimum_required(VERSION 3.8)
project(linear_algebra_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) # for generating the service code as a header file
find_package(geometry_msgs REQUIRED) # for the service definition

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeastSquares.srv"
  DEPENDENCIES geometry_msgs
) # this is needed to generate the service code as a header file
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)


  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
