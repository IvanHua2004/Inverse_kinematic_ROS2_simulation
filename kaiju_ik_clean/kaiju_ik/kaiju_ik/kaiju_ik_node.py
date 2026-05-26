import time
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point
from turtlesim_msgs.srv import Spawn, TeleportAbsolute
import numpy as np

# Longueurs des segments
L1 = 2.0
L2 = 1.5
L3 = 1.0
SHOULDER_X = 5.54
SHOULDER_Y = 5.54

DT = 0.064
PAUSE_DURATION = 2.0
MOVE_DURATION = 5.0

def forward_kinematics(theta1, theta2, theta3):
    elbowX = SHOULDER_X + L1 * math.cos(theta1)
    elbowY = SHOULDER_Y + L1 * math.sin(theta1)
    wristX = elbowX + L2 * math.cos(theta1 + theta2)
    wristY = elbowY + L2 * math.sin(theta1 + theta2)
    clawX = wristX + L3 * math.cos(theta1 + theta2 + theta3)
    clawY = wristY + L3 * math.sin(theta1 + theta2 + theta3)
    return clawX, clawY, elbowX, elbowY, wristX, wristY

def solve_ik_jacobian(target_x, target_y, theta_init=None, max_iter=50, tolerance=1e-2, step_gain=0.3):
    if theta_init is None:
        theta1, theta2, theta3 = math.pi/2, 0.0, 0.0 
    else:
        theta1, theta2, theta3 = theta_init
    
    for _ in range(max_iter):
        clawX, clawY, _, _, _, _ = forward_kinematics(theta1, theta2, theta3)
        dx = target_x - clawX
        dy = target_y - clawY
        error = math.hypot(dx, dy)
        if error < tolerance:
            break
        
        s1 = math.sin(theta1)
        c1 = math.cos(theta1)
        s12 = math.sin(theta1 + theta2)
        c12 = math.cos(theta1 + theta2)
        s123 = math.sin(theta1 + theta2 + theta3)
        c123 = math.cos(theta1 + theta2 + theta3)
        
        J11 = -L1*s1 - L2*s12 - L3*s123
        J12 = -L2*s12 - L3*s123
        J13 = -L3*s123
        J21 = L1*c1 + L2*c12 + L3*c123
        J22 = L2*c12 + L3*c123
        J23 = L3*c123
        
        matrix = np.array([[J11, J12, J13], [J21, J22, J23]])
        error = np.array([dx, dy])
        J_pninv = np.linalg.pinv(matrix)
        dtheta = step_gain * (J_pninv @ error)
        dtheta1, dtheta2, dtheta3 = dtheta[0], dtheta[1], dtheta[2]

        theta1 += dtheta1
        theta2 += dtheta2
        theta3 += dtheta3
    
    clawX, clawY, elbowX, elbowY, wristX, wristY = forward_kinematics(theta1, theta2, theta3)
    return elbowX, elbowY, wristX, wristY, clawX, clawY, theta1, theta2, theta3

def ease_in_out(t):
    return t * t * (3.0 - 2.0 * t)

class KaijuIKNode(Node):
    def __init__(self):
        super().__init__('kaiju_ik_node')
        self._spawn = self.create_client(Spawn, '/spawn')
        self._tele_elbow = self.create_client(TeleportAbsolute, '/turtle2/teleport_absolute')
        self._tele_wrist = self.create_client(TeleportAbsolute, '/turtle5/teleport_absolute')
        self._tele_claw = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self._tele_shoulder = self.create_client(TeleportAbsolute, '/turtle3/teleport_absolute')
        self._claw_angle = math.pi/2
        
        init_t1, init_t2, init_t3 = math.pi/2, 0.0, 0.0
        self._last_theta = [init_t1, init_t2, init_t3]
        clawX, clawY, _, _, _, _ = forward_kinematics(init_t1, init_t2, init_t3)
        self._current_x = clawX
        self._current_y = clawY
        self._start_x = clawX
        self._start_y = clawY
        self._target_x = clawX
        self._target_y = clawY
        self._incoming_x = clawX
        self._incoming_y = clawY
        
        self._anim_t = 1.0
        self._move_start_time = time.monotonic()
        self._pausing = False
        self._pause_start_time = None
        
        self._state = 0
        self._spawned = False
        self._init_timer = self.create_timer(0.1, self._state_machine)
        self.create_subscription(Point, '/arm_target_input', self._target_cb, 10)
    
    def _update_claw_angle(self, clawX, clawY):
        dx = self._target_x - clawX
        dy = self._target_y - clawY
        if math.hypot(dx, dy) > 0.02:
            desired = math.atan2(dy, dx)
            diff = (desired - self._claw_angle + math.pi) % (2 * math.pi) - math.pi
            self._claw_angle += 0.02 * diff 
        return self._claw_angle


    def _target_cb(self, msg):
        self._incoming_x = msg.x
        self._incoming_y = msg.y
        self.get_logger().info(f"Cible reçue: ({msg.x:.2f}, {msg.y:.2f})")
    
    def _teleport(self, client, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        client.call_async(req)
    
    def _spawn_turtle(self, name, x, y):
        req = Spawn.Request()
        req.x, req.y, req.theta, req.name = x, y, 0.0, name
        future = self._spawn.call_async(req)
        future.add_done_callback(lambda f: self._spawn_callback(f, name))
    
    def _spawn_callback(self, future, name):
        try:
            future.result()
            self.get_logger().info(f"Tortue {name} créée")
        except Exception:
            self.get_logger().warn(f"Tortue {name} existe déjà")
    
    def _state_machine(self):
        if self._state == 0:
            if not self._spawn.service_is_ready():
                return
            if not self._spawned:
                self._spawned = True
                turtles = [
                    ('turtle2', SHOULDER_X, SHOULDER_Y),
                    ('turtle3', SHOULDER_X, SHOULDER_Y),
                    ('turtle4', SHOULDER_X + 2.0, SHOULDER_Y + 2.0),
                    ('turtle5', SHOULDER_X + L1, SHOULDER_Y)
                ]
                for name, x, y in turtles:
                    self._spawn_turtle(name, x, y)
                self.create_timer(1.0, self._init_robots)
                self._state = 1
    
    def _init_robots(self):
        if self._state == 1:
            self._setup_robots()
            self._init_timer.cancel()
    
    def _setup_robots(self):
        if (self._tele_elbow.service_is_ready() and
            self._tele_wrist.service_is_ready() and
            self._tele_claw.service_is_ready()):
            
            elbowX, elbowY, wristX, wristY, clawX, clawY, t1, t2, t3 = solve_ik_jacobian(
            self._current_x, self._current_y, theta_init=self._last_theta)
            self._last_theta = [t1, t2, t3]
            
            self._teleport(self._tele_shoulder, SHOULDER_X, SHOULDER_Y, t1)
            self._teleport(self._tele_elbow, elbowX, elbowY, t1 + t2)
            self._teleport(self._tele_wrist, wristX, wristY, t1 + t2 + t3)

            self._teleport(self._tele_claw, clawX, clawY, self._update_claw_angle(clawX, clawY))
            
            self._state = 2
            self.create_timer(DT, self._control_loop)
            self.get_logger().info("Bras Jacobien démarré (sans numpy)")
    
    def _start_move_to(self, targetX, targetY):
        self._start_x = self._current_x
        self._start_y = self._current_y
        self._target_x = targetX
        self._target_y = targetY
        self._anim_t = 0.0
        self._move_start_time = time.monotonic()
    
    def _control_loop(self):
        if self._state != 2:
            return
        
        if self._pausing:
            elapsed = time.monotonic() - self._pause_start_time
            if elapsed >= PAUSE_DURATION:
                self._pausing = False
                if (self._incoming_x != self._target_x or
                    self._incoming_y != self._target_y):
                    self._start_move_to(self._incoming_x, self._incoming_y)
            return
        
        if self._anim_t < 1.0:
            elapsed = time.monotonic() - self._move_start_time
            self._anim_t = min(1.0, elapsed / MOVE_DURATION)
            eased = ease_in_out(self._anim_t)
            
            self._current_x = self._start_x + eased * (self._target_x - self._start_x)
            self._current_y = self._start_y + eased * (self._target_y - self._start_y)
            
            elbowX, elbowY, wristX, wristY, clawX, clawY, t1, t2, t3 = solve_ik_jacobian(
                self._current_x, self._current_y, theta_init=self._last_theta)
            self._last_theta = [t1, t2, t3]
            
            self._teleport(self._tele_shoulder, SHOULDER_X, SHOULDER_Y, t1)
            self._teleport(self._tele_elbow, elbowX, elbowY, t1 + t2)
            self._teleport(self._tele_wrist, wristX, wristY, t1 + t2 + t3)
            self._teleport(self._tele_claw, clawX, clawY, self._update_claw_angle(clawX, clawY))
            
            
            if self._anim_t >= 1.0:
                self._pausing = True
                self._pause_start_time = time.monotonic()
            return
        
        if (self._incoming_x != self._target_x or
            self._incoming_y != self._target_y):
            self._start_move_to(self._incoming_x, self._incoming_y)

def main(args=None):
    rclpy.init(args=args)
    node = KaijuIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()