import subprocess
import re

def get_peter_pose():
    cmd = [
    "gz", "topic",
    "-e",
    "-t", "/world/default/pose/info",
    "-n", "1"
]
    output = subprocess.check_output(cmd).decode("utf-8")

    # Buscar bloque del modelo "peter"
    pattern = r'name: "peter".*?position\s*{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)'
    match = re.search(pattern, output, re.DOTALL)

    if match:
        x, y, z = map(float, match.groups())
        return x, y, z
    else:
        return None

while True:
    pose = get_peter_pose()
    print("POSICION A TIEMPO REAL")
    print("")
    print(f"X: {pose[0]:3f}")
    print(f"Y: {pose[1]:3f}")
    print(f"Z: {pose[2]:3f}")
    print("")