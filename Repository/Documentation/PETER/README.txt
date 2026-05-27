1. Dentro de este workspace, dirigete a src/peter_robot/gazebo_model y copia la carpeta "peter_robot" dentro de /.gazebo/models/ (Ctrl + H desde home para ver la carpeta)
3. colcon build + source install/setup.bash
4. Para RVIZ: ros2 launch peter_robot display.launch.py
5. Para Gazebo: ros2 launch peter_robot gazebo.launch.py

Para moverlo en gazebo: En otra terminal source install/setup.bash + ros2 run peter_robot peter_teleop_keyboard

Dentro del launch se puede configurar para abrir cualquier mundo (agregarlo en la carpeta worlds y modificar el launch en la parte respectiva)