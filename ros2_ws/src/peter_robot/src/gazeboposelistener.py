
import time
import subprocess
import re
import numpy as np

goal = np.array([2.0, 0.0])
start = np.array([0.0, 0.0])

direction = goal - start
direction_norm = np.linalg.norm(direction)

errors_ct = []
errors_goal = []


def get_peter_pose():
    cmd = [
        "gz", "topic",
        "-e",
        "-t", "/world/default/pose/info",
        "-n", "1"
    ]

    try:
        output = subprocess.check_output(cmd, timeout=0.3).decode("utf-8")
    except subprocess.TimeoutExpired:
        return None

    pattern = r'name: "peter".*?position\s*{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)'
    match = re.search(pattern, output, re.DOTALL)

    if match:
        return tuple(map(float, match.groups()))

    return None


def cross_track_error(p):
    p2 = p[:2]
    return abs(np.cross(direction, p2 - start)) / direction_norm


def goal_error(p):
    p2 = p[:2]
    return np.linalg.norm(p2 - goal)


while True:
    pose = get_peter_pose()

    if pose is None:
        continue

    ct = cross_track_error(pose)
    ge = goal_error(pose)

    errors_ct.append(ct)
    errors_goal.append(ge)

    rmse_ct = np.sqrt(np.mean(np.array(errors_ct)**2))
    #rmse_goal = np.sqrt(np.mean(np.array(errors_goal)**2))

    print("\nREAL TIME TRACKING")
    print(f"GOAL: ({goal[0]:.2f}, {goal[1]:.2f})")
    print(f"POS:  ({pose[0]:.2f}, {pose[1]:.2f})")
    print(f"CT error: {ct:.3f} m")
    print(f"Goal error: {ge:.3f} m")
    print(f"RMSE CT (tracking accuracy): {rmse_ct:.3f} m") #esta métrica nos indica qué tan bien siguió el robot la linea recta que conecta su posición inicial al objetivo que busca
    #print(f"RMSE goal error: {rmse_goal:.3f} m")

    time.sleep(0.2)
    
#NOTA DE DIEGO: Este código proporciona la posición x,y,z de un robot en gazebo a tiempo real, puede ser implementada en ROS si se necesita
#El tópico de Gazebo también tiene orientación disponible pero no fue extraída en este ejemplo, la posición es el centro del STL que se use en la simulación
#es importante correr el código en un contenedor