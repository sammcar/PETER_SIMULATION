1. colcon build + source install/setup.bash
2. Para RVIZ: ros2 launch peter_robot display.launch.py
3. Para Gazebo: ros2 launch peter_robot gazebo.launch.py
4. Para abrir un mundo en gazebo ros2 launch peter_robot gazebo.launch.py world_name:=(nombre del mundo sin extensiones)

Nombres de mundo disponibles:

- depot (entorno industrial)
- terrain (mapa de dieguito)
- empty (entorno clasico)
- contextos (mapa con estimulos fieles a simulacion)
- fabrica (mapa con iluminacion diferente industrial)

*Si no se especifica un mundo se empty por defecto.

Para moverlo en gazebo teleoperado: En otra terminal source install/setup.bash + ros2 run peter_robot peter_teleop_keyboard
Para moverlo en gazebo con la red: En otra terminal source install/setup.bash + ros2 run peter_robot red_neuro
Para ver las gráficas puede correr en otra terminal mientras la simulación está activa: source install/setup.bash + ros2 launch peter_robot graficas_launch.py

Para visualizar LIDAR, Superficies de contacto y otras herramientas dentro de Gazebo, se recomienda ver el video "Resultados Red" dentro de 
la carpeta de Samuel/Entregas Pasadas en Onedrive.