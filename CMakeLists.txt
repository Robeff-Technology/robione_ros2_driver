cmake_minimum_required(VERSION 3.8)
project(robione_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

set(TARGET_SRC
    src/AsyncSerial.cpp
    src/robione_ros2_node.cpp
    src/robione.cpp
    src/serial_converter.cpp
)

set(msg_files
    "msg/ControlCmd.msg"
    "msg/SystemStatus.msg"
    "msg/VehicleInfo.msg"
    "msg/VehicleStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES
    std_msgs
)

set(LIBRARY_DIR ${CMAKE_SOURCE_DIR}/lib)
link_directories(${LIBRARY_DIR})

add_executable(robione_ros2_node ${TARGET_SRC})

ament_target_dependencies(robione_ros2_node
                          rclcpp
                          sensor_msgs
)


target_link_libraries(robione_ros2_node libcomm_pro.a)


rosidl_target_interfaces(robione_ros2_node
        ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS robione_ros2_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

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
