"""
Centralized topic names and ZMQ port assignments.
All nodes import from here so renaming a topic is a one-line change.
"""

# ── Broker ports ──────────────────────────────────────────────────────────────
BROKER_FRONTEND = "tcp://localhost:5555"   # publishers connect here (XSUB)
BROKER_BACKEND  = "tcp://localhost:5556"   # subscribers connect here (XPUB)

# ── Topic names ───────────────────────────────────────────────────────────────
TOPIC_NEURON_ACTIVITY    = "neuron_activity"
TOPIC_CMD_VEL            = "/cmd_vel"
TOPIC_BBOX_RED           = "/bounding_box/red"
TOPIC_BBOX_BLUE          = "/bounding_box/blue"
TOPIC_PETER_MODE         = "/peter_mode"

TOPIC_EXPERIMENT_METRICS = "/experiment/metrics"   # neural_recorder → metrics_recorder
TOPIC_EXPERIMENT_STATUS  = "/experiment/status"    # neural_recorder → (debug)

TOPIC_METRICS            = "/Metrics"              # controller → metrics_recorder + robustness
TOPIC_RMSE_CT            = "/rmse_ct"              # rmse_node → metrics_recorder + robustness

TOPIC_STABILITY_DATA     = "/peter/stability_data" # controller → stability_monitor

TOPIC_POSE               = "/pose"                 # odometry publisher → rmse_node

TOPIC_SCAN               = "/scan"                 # lidar_node → neural network
TOPIC_IMU                = "/imu/data"             # imu_ina_node → neural network + stability
TOPIC_INA                = "/ina/data"             # imu_ina_node → power recording

TOPIC_ROBOT_CMD          = "/robot_cmd"            # cmd_translator → cmd_vel_server → Pi serial
