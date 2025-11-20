cmake_minimum_required(VERSION 3.8)
project(servo_latch)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(servo_latch_node src/servo_latch_node.cpp)
ament_target_dependencies(servo_latch_node rclcpp std_srvs)

# pigpio daemon client library is in /usr/local/lib after "sudo make install"
link_directories(/usr/local/lib)

target_link_libraries(servo_latch_node
  pigpiod_if2
  pthread
  rt
)

install(TARGETS
  servo_latch_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
