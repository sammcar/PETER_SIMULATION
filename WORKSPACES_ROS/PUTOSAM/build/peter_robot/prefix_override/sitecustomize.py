import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alejo/Paper/PETER_SIMULATION/WORKSPACES_ROS/PUTOSAM/install/peter_robot'
