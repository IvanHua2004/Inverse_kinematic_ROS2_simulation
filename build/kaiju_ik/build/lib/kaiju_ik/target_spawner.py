import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from turtlesim_msgs.msg import Pose
from turtlesim_msgs.srv import TeleportAbsolute

VIEW_MIN, VIEW_MAX = 1.5, 9.0
TOUCH_THRESHOLD = 0.55
PAUSE_DURATION = 1.5

class TargetSpawner(Node):
    def __init__(self):
        super().__init__('target_spawner')
        self._target_pub = self.create_publisher(Point, '/arm_target_input', 10)
        self._teleport = self.create_client(TeleportAbsolute, '/turtle4/teleport_absolute')
        self._pose_sub = self.create_subscription(Pose, '/turtle1/pose', self._claw_pose_cb, 10)
        self._claw_pose = None
        self._target_x = 5.0
        self._target_y = 5.0
        self._cooldown_start = None
        self.create_timer(1.0, self._start)
        self._started = False

    def _start(self):
        if self._started: return
        if not self._teleport.wait_for_service(1.0): return
        self._started = True
        self._respawn_target()
        self._timer = self.create_timer(0.05, self._check_touch)

    def _respawn_target(self):
        self._target_x = random.uniform(VIEW_MIN, VIEW_MAX)
        self._target_y = random.uniform(VIEW_MIN, VIEW_MAX)
        msg = Point()
        msg.x, msg.y, msg.z = self._target_x, self._target_y, 0.0
        self._target_pub.publish(msg)
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = self._target_x, self._target_y, 0.0
        self._teleport.call_async(req)

    def _claw_pose_cb(self, msg):
        self._claw_pose = msg

    def _check_touch(self):
        if self._claw_pose is None: return
        if self._cooldown_start is not None:
            elapsed = (self.get_clock().now() - self._cooldown_start).nanoseconds / 1e9
            if elapsed >= PAUSE_DURATION:
                self._cooldown_start = None
                self._respawn_target()
            return
        dx = self._claw_pose.x - self._target_x
        dy = self._claw_pose.y - self._target_y
        if math.hypot(dx, dy) < TOUCH_THRESHOLD:
            self._cooldown_start = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = TargetSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
