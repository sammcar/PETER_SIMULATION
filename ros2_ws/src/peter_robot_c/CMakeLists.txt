cmake_minimum_required(VERSION 3.8)
project(peter_robot_c)

# Encuentra dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Para que reconozca los scripts como ejecutables
install(PROGRAMS
  src/peter_controller.py
  src/peter_teleop_keyboard.py
  src/test_move_joints.py
  src/red_neuronal.py
  src/camera_node.py
  src/camera_Adjust.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalación de archivos adicionales como URDF, meshes, config, etc.
install(DIRECTORY
  config
  launch
  meshes
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Instalación del archivo package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Para registrar los ejecutables (como entry_points en setup.py)
# ament_python_install_package(${PROJECT_NAME})

# Ejecutable C++
add_executable(red_cpp src/red_cpp.cpp)
ament_target_dependencies(red_cpp
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)


install(TARGETS
  red_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
