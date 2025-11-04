cmake_minimum_required(VERSION 3.8)
project(hybrid_pkg)

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# === C++ Node ===
add_executable(simple_cpp_node src/simple_cpp_node.cpp)
ament_target_dependencies(simple_cpp_node rclcpp)
install(TARGETS
  simple_cpp_node
  DESTINATION lib/${PROJECT_NAME})

# === Python Node ===
# Install Python scripts
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/simple_py_node.py
  DESTINATION lib/${PROJECT_NAME})

# === Launch files ===
install(DIRECTORY launch worlds model DESTINATION share/${PROJECT_NAME}/)

# === Finalize ===
ament_package()

