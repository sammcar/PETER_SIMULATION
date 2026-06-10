#!/usr/bin/env python3
import os
import subprocess
import time
import shutil
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TransitionManager(Node):
    def __init__(self, output_dir):
        super().__init__('transition_manager')
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_transition = "none"
        self.transition_active = False
        self.start_time = 0.0
        
        # Escucha los cambios de modo inducidos por el teleop
        self.create_subscription(String, '/peter_mode', self._mode_callback, 10)
        self.get_logger().info(f"Orquestador iniciado. Los resultados se guardarán en: {self.output_dir}")

    def _mode_callback(self, msg):
        new_mode = msg.data.upper()
        self.get_logger().info(f"Detectado comando de teleop hacia modo: {new_mode}")
        self.current_transition = f"transition_to_{new_mode}_{int(time.time())}"
        self.transition_active = True
        self.start_time = time.monotonic()

def main():
    # Rutas internas dentro del contenedor de Docker
    home = Path.home()
    default_log = home / 'stability_log.csv'
    results_base_dir = Path('/ros2_ws/src/peter_robot/results')
    
    print("--> Lanzando simulación y nodos de control de PETER...")
    launch_cmd = ["ros2", "launch", "peter_robot", "transition_evaluation.launch.py"]
    launch_proc = subprocess.Popen(launch_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    print("--> Esperando inicialización física del robot (12s)...")
    time.sleep(12)
    
    rclpy.init()
    manager = TransitionManager(results_base_dir)
    
    print("\n=========================================================================")
    print(" ¡LISTO! Utiliza el teleop en otra terminal para inducir transformaciones.")
    print(" Presiona CTRL+C en ESTA terminal para finalizar y exportar el reporte.")
    print("=========================================================================\n")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(manager, timeout_sec=0.5)
    except KeyboardInterrupt:
        print("\n--> Finalizando pruebas y deteniendo procesos...")
    finally:
        manager.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass
        
        launch_proc.terminate()
        launch_proc.wait()
        
        # Kill remanentes internos del contenedor
        os.system("pkill -f gz_sim")
        os.system("pkill -f peter_controller")
        os.system("pkill -f stability_monitor")
        
        # Exportación y guardado de resultados en el volumen compartido
        if default_log.exists():
            final_dest = results_base_dir / f"stability_session_report_{int(time.time())}.csv"
            shutil.move(str(default_log), str(final_dest))
            print(f"--> [ÉXITO] Reporte consolidado exportado a: {final_dest}")
        else:
            print("--> [ADVERTENCIA] No se encontró el archivo base log de estabilidad.")

if __name__ == '__main__':
    main()