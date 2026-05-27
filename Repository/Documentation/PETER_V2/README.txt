1. Dentro de este workspace, dirigete a src/peter_robot/gazebo_model y copia la carpeta "peter_robot" dentro de /.gazebo/models/ (Ctrl + H desde home para ver la carpeta)
3. colcon build + source install/setup.bash
4. Para RVIZ: ros2 launch peter_robot display.launch.py
5. Para Gazebo: ros2 launch peter_robot gazebo.launch.py

Para moverlo en gazebo teleoperado: En otra terminal source install/setup.bash + ros2 run peter_robot peter_teleop_keyboard
Para moverlo en gazebo con la red: En otra terminal source install/setup.bash + ros2 run peter_robot red_parcial
Para abrirlo dentro del terreno irregular: ros2 launch peter_robot gazebo.launch.py world:=terrain.world

El robot es capaz de ejecutar la rutina de caminata de cuadrúpedo si se maneja desde el teleop PERO primero debe darse clic en "C" para que inicie la rutina.
Por el momento hace la rutina pero por problemas en fricciones en las patas no tiene desplazamiento. 

Dentro del launch se puede configurar para abrir cualquier mundo (agregarlo en la carpeta worlds y modificar el launch en la parte respectiva)