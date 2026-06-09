"""
transport.py — ZMQ pub/sub abstraction that mimics the rclpy API.

Drop-in replacement for the ROS2 node boilerplate.  The metric logic inside
each node is untouched; only the import line and the main() bootstrap change.

Architecture
────────────
  publishers ──PUB──► broker XSUB (port 5555)
                              │
                       broker XPUB (port 5556) ──SUB──► subscribers

Run broker.py once before starting any node.

Message wire format
───────────────────
  b"TOPIC_NAME " + JSON payload
  e.g. b"/cmd_vel {\"linear\":{\"x\":0.5,\"y\":0.0,\"z\":0.0},\"angular\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}"
"""

import json
import logging
import threading
import time

import zmq

from topics import BROKER_FRONTEND, BROKER_BACKEND

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")


# ── Message types ─────────────────────────────────────────────────────────────
# Each type exposes the same attribute structure as the ROS2 equivalent and
# implements to_dict() / from_dict() for JSON serialization.

class Float32MultiArray:
    def __init__(self, data=None):
        self.data = list(data) if data is not None else []

    def to_dict(self):
        return {"data": self.data}

    @classmethod
    def from_dict(cls, d):
        return cls(d.get("data", []))


class Float64:
    def __init__(self, data=0.0):
        self.data = float(data)

    def to_dict(self):
        return {"data": self.data}

    @classmethod
    def from_dict(cls, d):
        return cls(d.get("data", 0.0))


class _Vec3:
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z}

    @classmethod
    def from_dict(cls, d):
        return cls(d.get("x", 0.0), d.get("y", 0.0), d.get("z", 0.0))


class Twist:
    def __init__(self):
        self.linear  = _Vec3()
        self.angular = _Vec3()

    def to_dict(self):
        return {"linear": self.linear.to_dict(), "angular": self.angular.to_dict()}

    @classmethod
    def from_dict(cls, d):
        obj = cls()
        obj.linear  = _Vec3.from_dict(d.get("linear",  {}))
        obj.angular = _Vec3.from_dict(d.get("angular", {}))
        return obj


class String:
    def __init__(self, data=""):
        self.data = str(data)

    def to_dict(self):
        return {"data": self.data}

    @classmethod
    def from_dict(cls, d):
        return cls(d.get("data", ""))


class _Position:
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Orientation:
    __slots__ = ("x", "y", "z", "w")

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = float(x), float(y), float(z), float(w)


class Pose:
    def __init__(self):
        self.position    = _Position()
        self.orientation = _Orientation()

    @classmethod
    def from_dict(cls, d):
        obj = cls()
        p = d.get("position", {})
        obj.position = _Position(p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0))
        o = d.get("orientation", {})
        obj.orientation = _Orientation(
            o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0), o.get("w", 1.0)
        )
        return obj


class PoseArray:
    def __init__(self):
        self.poses = []

    def to_dict(self):
        return {
            "poses": [
                {
                    "position":    {"x": p.position.x, "y": p.position.y, "z": p.position.z},
                    "orientation": {"x": p.orientation.x, "y": p.orientation.y,
                                    "z": p.orientation.z, "w": p.orientation.w},
                }
                for p in self.poses
            ]
        }

    @classmethod
    def from_dict(cls, d):
        obj = cls()
        obj.poses = [Pose.from_dict(p) for p in d.get("poses", [])]
        return obj


# Pose message used by /pose topic (plain x/y/z from odometry)
class PoseStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z}

    @classmethod
    def from_dict(cls, d):
        return cls(d.get("x", 0.0), d.get("y", 0.0), d.get("z", 0.0))


# LaserScan — equivalent to sensor_msgs/LaserScan
class LaserScan:
    def __init__(self):
        self.angle_min       = 0.0
        self.angle_max       = 0.0
        self.angle_increment = 0.0
        self.ranges          = []      # list of float (meters), inf = no return
        self.intensities     = []      # list of float (optional)

    def to_dict(self):
        return {
            "angle_min":       self.angle_min,
            "angle_max":       self.angle_max,
            "angle_increment": self.angle_increment,
            "ranges":          [r if r != float('inf') else None for r in self.ranges],
            "intensities":     self.intensities,
        }

    @classmethod
    def from_dict(cls, d):
        obj = cls()
        obj.angle_min       = float(d.get("angle_min",       0.0))
        obj.angle_max       = float(d.get("angle_max",       0.0))
        obj.angle_increment = float(d.get("angle_increment", 0.0))
        obj.ranges          = [float('inf') if r is None else float(r)
                               for r in d.get("ranges", [])]
        obj.intensities     = [float(v) for v in d.get("intensities", [])]
        return obj


# Imu — equivalent to sensor_msgs/Imu
class Imu:
    def __init__(self):
        self.orientation            = _Orientation()
        self.linear_acceleration    = _Vec3()
        self.angular_velocity       = _Vec3()

    def to_dict(self):
        return {
            "orientation": {
                "x": self.orientation.x, "y": self.orientation.y,
                "z": self.orientation.z, "w": self.orientation.w,
            },
            "linear_acceleration": self.linear_acceleration.to_dict(),
            "angular_velocity":    self.angular_velocity.to_dict(),
        }

    @classmethod
    def from_dict(cls, d):
        obj = cls()
        o = d.get("orientation", {})
        obj.orientation = _Orientation(
            o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0), o.get("w", 1.0)
        )
        obj.linear_acceleration = _Vec3.from_dict(d.get("linear_acceleration", {}))
        obj.angular_velocity    = _Vec3.from_dict(d.get("angular_velocity",    {}))
        return obj


# ── Internal helpers ──────────────────────────────────────────────────────────

class _LoggerWrapper:
    """Wraps Python's logging.Logger to add a .warn() alias."""

    def __init__(self, name):
        self._log = logging.getLogger(name)

    def info(self, msg):
        self._log.info(msg)

    def warn(self, msg):
        self._log.warning(msg)

    def warning(self, msg):
        self._log.warning(msg)

    def error(self, msg):
        self._log.error(msg)

    def debug(self, msg):
        self._log.debug(msg)


class _ClockNow:
    @property
    def nanoseconds(self):
        return int(time.time() * 1e9)


class _Clock:
    def now(self):
        return _ClockNow()


class _RepeatTimer:
    def __init__(self, period, callback, name="timer"):
        self._period   = period
        self._callback = callback
        self._running  = False
        self._thread   = None
        self._name     = name

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True, name=self._name)
        self._thread.start()

    def _run(self):
        next_tick = time.monotonic() + self._period
        while self._running:
            now = time.monotonic()
            sleep_time = next_tick - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            if self._running:
                try:
                    self._callback()
                except Exception as e:
                    logging.getLogger(self._name).error(f"Timer callback error: {e}")
            next_tick += self._period

    def cancel(self):
        self._running = False


class _Publisher:
    def __init__(self, topic, ctx):
        self._topic = topic.encode()
        self._sock  = ctx.socket(zmq.PUB)
        self._sock.connect(BROKER_FRONTEND)
        self._lock  = threading.Lock()
        time.sleep(0.05)   # let the connection register with the broker

    def publish(self, msg):
        payload = json.dumps(msg.to_dict()).encode()
        frame   = self._topic + b" " + payload
        with self._lock:
            self._sock.send(frame)


# ── Public Node class ─────────────────────────────────────────────────────────

class Node:
    """
    Minimal rclpy.Node replacement backed by ZMQ.

    Usage:
        class MyNode(Node):
            def __init__(self):
                super().__init__('my_node')
                self.pub = self.create_publisher(Float64, '/my_topic', 10)
                self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
                self.create_timer(0.2, self._tick)

        node = MyNode()
        spin(node)           # blocks until Ctrl-C
    """

    def __init__(self, name):
        self.name    = name
        self._ctx    = zmq.Context.instance()
        self._logger = _LoggerWrapper(name)
        self._clock  = _Clock()
        self._subs   = []   # list of (topic, callback, msg_cls, zmq_socket)
        self._timers = []
        self._running = False

    # ── ROS2-compatible API ────────────────────────────────────────────────

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def create_publisher(self, msg_type, topic, qsize=10):
        return _Publisher(topic, self._ctx)

    def create_subscription(self, msg_type, topic, callback, qsize=10):
        sock = self._ctx.socket(zmq.SUB)
        sock.connect(BROKER_BACKEND)
        sock.setsockopt(zmq.SUBSCRIBE, topic.encode())
        entry = (topic, callback, msg_type, sock)
        self._subs.append(entry)
        return entry

    def create_timer(self, period, callback):
        t = _RepeatTimer(period, callback, name=f"{self.name}/timer")
        self._timers.append(t)
        return t

    # ── Spin / teardown ────────────────────────────────────────────────────

    def spin(self):
        """Block and process messages until KeyboardInterrupt."""
        self._running = True
        for t in self._timers:
            t.start()

        poller       = zmq.Poller()
        sock_to_info = {}
        for topic, callback, msg_cls, sock in self._subs:
            poller.register(sock, zmq.POLLIN)
            sock_to_info[sock] = (callback, msg_cls)

        self._logger.info(f"Node '{self.name}' spinning.")
        try:
            while self._running:
                events = dict(poller.poll(timeout=100))
                for sock, flag in events.items():
                    if flag & zmq.POLLIN:
                        raw = sock.recv()
                        _, _, payload = raw.partition(b" ")
                        callback, msg_cls = sock_to_info[sock]
                        try:
                            msg = msg_cls.from_dict(json.loads(payload))
                            callback(msg)
                        except Exception as e:
                            self._logger.error(f"Callback error on {sock}: {e}")
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            for t in self._timers:
                t.cancel()

    def destroy_node(self):
        self._running = False
        for t in self._timers:
            t.cancel()


# ── Module-level spin (mirrors rclpy.spin) ────────────────────────────────────

def spin(node: Node):
    node.spin()
