#!/usr/bin/env python3

import json
import math
import re
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray, String

SPHERE_XY      = (-2.0, 2.0)
Z17_IDX        = 29                 # index in /neuron_activity
LIDAR4_IDX     = 36                 # index in /neuron_activity
Z17_THRESH     = 0.25
LIDAR4_THRESH  = 0.3
BB_AREA_THRESH = 500.0
OUTPUT_PATH    = Path.home() / 'inspection_summary.json'


class InspectionRecorder(Node):

    def __init__(self) -> None:
        super().__init__('inspection_recorder')

        self._sim_time_s       = 0.0
        self._lidar4_prev      = 0.0
        self._z17_peak         = 0.0
        self._success          = False
        self._d_final          = None
        self._inspection_pose  = None
        self._T_acquisition    = None   # sim time (s) of first blue detection
        self._N_lidar_events   = 0
        self._mode_prev        = ''
        self._N_mode_X_events  = 0
        self._summary_written  = False

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Clock,              '/clock',             self._cb_clock,   qos_be)
        self.create_subscription(Float32MultiArray,  '/neuron_activity',   self._cb_neurons,  50)
        self.create_subscription(Float32MultiArray,  '/bounding_box/blue', self._cb_bb_blue, 100)
        self.create_subscription(String,             '/peter_mode',        self._cb_mode,     10)

        self.get_logger().info(f'InspectionRecorder ready. Output → {OUTPUT_PATH}')

    # ------------------------------------------------------------------
    def _cb_clock(self, msg: Clock) -> None:
        self._sim_time_s = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _cb_neurons(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) <= max(Z17_IDX, LIDAR4_IDX):
            return

        lidar4 = data[LIDAR4_IDX]
        z17    = data[Z17_IDX]

        # Ascending crossing: lidar[4] goes from below to above threshold
        if self._lidar4_prev < LIDAR4_THRESH and lidar4 >= LIDAR4_THRESH:
            self._N_lidar_events += 1
            self.get_logger().info(
                f'Lidar evasion event #{self._N_lidar_events}: lidar[4]={lidar4:.3f}'
            )
        self._lidar4_prev = lidar4

        if z17 > self._z17_peak:
            self._z17_peak = z17

        # First crossing of z[17] > Z17_THRESH → capture final approach pose
        if not self._success and z17 > Z17_THRESH:
            self._success = True
            pose = self._get_peter_pose()
            if pose is not None:
                dx = pose[0] - SPHERE_XY[0]
                dy = pose[1] - SPHERE_XY[1]
                self._d_final         = round(math.hypot(dx, dy), 4)
                self._inspection_pose = {
                    'x': round(pose[0], 4),
                    'y': round(pose[1], 4),
                    'z': round(pose[2], 4),
                }
                self.get_logger().info(
                    f'[STOP] z[17]={z17:.3f} | d_final={self._d_final:.3f} m | '
                    f'pose=({pose[0]:.3f}, {pose[1]:.3f})'
                )
            else:
                self.get_logger().warn('[STOP] z[17] threshold crossed but pose unavailable')

    def _cb_bb_blue(self, msg: Float32MultiArray) -> None:
        if self._T_acquisition is not None:
            return
        area = float(msg.data[1]) if len(msg.data) >= 2 else 0.0
        if area > BB_AREA_THRESH:
            self._T_acquisition = round(self._sim_time_s, 3)
            self.get_logger().info(
                f'T_acquisition={self._T_acquisition:.3f} s sim (area={area:.0f} px²)'
            )

    def _cb_mode(self, msg: String) -> None:
        mode = msg.data
        if mode != self._mode_prev:
            if mode == 'X':
                self._N_mode_X_events += 1
                self.get_logger().info(f'Mode → X (event #{self._N_mode_X_events})')
            self._mode_prev = mode

    # ------------------------------------------------------------------
    def _get_peter_pose(self):
        """One-shot gz topic query for robot pose; returns [x, y, z] or None."""
        cmd = ['gz', 'topic', '-e', '-t', '/world/obstaculos/pose/info', '-n', '1']
        try:
            out = subprocess.check_output(cmd, timeout=1.5).decode('utf-8')
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
            return None
        pattern = (
            r'name: "peter".*?'
            r'position\s*\{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)'
        )
        m = re.search(pattern, out, re.DOTALL)
        return [float(v) for v in m.groups()] if m else None

    # ------------------------------------------------------------------
    def _write_summary(self) -> None:
        if self._summary_written:
            return
        self._summary_written = True
        summary = {
            'suite_name':       'familia_e_inspeccion',
            'success':          self._success,
            'T_acquisition_s':  self._T_acquisition,
            'N_lidar_events':   self._N_lidar_events,
            'N_mode_X_events':  self._N_mode_X_events,
            'd_final_m':        self._d_final,
            'z17_peak':         round(self._z17_peak, 4),
            'inspection_pose':  self._inspection_pose,
        }
        OUTPUT_PATH.write_text(json.dumps(summary, indent=2))
        self.get_logger().info(f'inspection_summary.json written → {OUTPUT_PATH}')

    def destroy_node(self) -> None:
        self._write_summary()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node._summary_written:
            node._write_summary()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
