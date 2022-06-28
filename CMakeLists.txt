cmake_minimum_required(VERSION 3.8)
project(bea_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Parameters.msg"
  "msg/Emergency.msg"
  "msg/Heartbeat.msg"
  "srv/Configure.srv"
  DEPENDENCIES std_msgs
)

add_executable(flatscan
  src/demo.cc
  src/flatscan.cc
  src/parser.cc
  src/protocol.cc
)
ament_target_dependencies(flatscan
  "rclcpp"
  "sensor_msgs"
)
rosidl_target_interfaces(flatscan
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)


install(TARGETS
  flatscan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
