"""
arm_overlay.py  –  Pygame overlay node for the Kaiju IK arm.

Subscribes to the four turtle poses and draws the arm skeleton in a
separate window at ~30 Hz, independent of turtlesim.

Turtles expected
----------------
  turtle3  →  shoulder  (fixed anchor)
  turtle2  →  elbow
  turtle5  →  wrist
  turtle1  →  claw  (end-effector)
  turtle4  →  target marker (optional, drawn as a cross)

Run alongside your existing nodes:
  ros2 run <your_pkg> arm_overlay
"""

import threading
import math

import pygame
import rclpy
from rclpy.node import Node
from turtlesim_msgs.msg import Pose

# ── Window / rendering constants ────────────────────────────────────────────
WIN_W, WIN_H   = 600, 600          # window size in pixels
SIM_MIN        = 0.0               # turtlesim world coordinates
SIM_MAX        = 11.08

FPS            = 30

# Colours  (R, G, B)
BG_COLOR       = (15,  15,  30)    # dark navy
GRID_COLOR     = (35,  35,  60)
SEGMENT_COLOR  = (0,  220, 180)    # teal / cyan
JOINT_COLOR    = (255, 200,  50)   # amber
CLAW_COLOR     = (255,  80,  80)   # red
TARGET_COLOR   = (100, 255, 100)   # green
LABEL_COLOR    = (200, 200, 220)

SEGMENT_WIDTH  = 3
JOINT_RADIUS   = 7
CLAW_RADIUS    = 9
TARGET_SIZE    = 12                # half-size of the cross


# ── Coordinate helpers ───────────────────────────────────────────────────────
def sim_to_px(x: float, y: float) -> tuple[int, int]:
    """Convert turtlesim world coords → window pixel coords."""
    px = int((x - SIM_MIN) / (SIM_MAX - SIM_MIN) * WIN_W)
    # turtlesim Y=0 is bottom; pygame Y=0 is top → flip
    py = int((1.0 - (y - SIM_MIN) / (SIM_MAX - SIM_MIN)) * WIN_H)
    return px, py


# ── ROS2 node ────────────────────────────────────────────────────────────────
class ArmOverlayNode(Node):
    def __init__(self):
        super().__init__('arm_overlay')

        # Latest poses, keyed by turtle name
        self._poses: dict[str, Pose] = {}
        self._lock  = threading.Lock()

        turtles = ['turtle1', 'turtle2', 'turtle3', 'turtle4', 'turtle5']
        for name in turtles:
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, n=name: self._pose_cb(n, msg),
                10
            )

        self.get_logger().info('arm_overlay node ready – waiting for poses.')

    def _pose_cb(self, name: str, msg: Pose):
        with self._lock:
            self._poses[name] = msg

    def get_poses(self) -> dict[str, Pose]:
        with self._lock:
            return dict(self._poses)


# ── Pygame rendering ─────────────────────────────────────────────────────────
def draw_cross(surface, color, cx, cy, half, width=2):
    pygame.draw.line(surface, color, (cx - half, cy), (cx + half, cy), width)
    pygame.draw.line(surface, color, (cx, cy - half), (cx, cy + half), width)


def draw_grid(surface, font):
    """Faint grid lines every 1 turtlesim unit."""
    for i in range(12):
        world = float(i)
        px, _ = sim_to_px(world, 0.0)
        _, py  = sim_to_px(0.0, world)
        pygame.draw.line(surface, GRID_COLOR, (px, 0),     (px, WIN_H), 1)
        pygame.draw.line(surface, GRID_COLOR, (0, py),     (WIN_W, py), 1)


def render_frame(surface, font, poses: dict):
    surface.fill(BG_COLOR)
    draw_grid(surface, font)

    # Joint order: shoulder → elbow → wrist → claw
    joint_keys   = ['turtle3', 'turtle2', 'turtle5', 'turtle1']
    joint_labels = ['shoulder', 'elbow', 'wrist', 'claw']

    # Collect pixel positions for joints that have pose data
    pts = {}
    for key in joint_keys:
        if key in poses:
            p = poses[key]
            pts[key] = sim_to_px(p.x, p.y)

    # ── Draw arm segments ────────────────────────────────────────────────────
    for a, b in [('turtle3', 'turtle2'),
                 ('turtle2', 'turtle5'),
                 ('turtle5', 'turtle1')]:
        if a in pts and b in pts:
            pygame.draw.line(surface, SEGMENT_COLOR, pts[a], pts[b], SEGMENT_WIDTH)

    # ── Draw joints ──────────────────────────────────────────────────────────
    for key, label in zip(joint_keys, joint_labels):
        if key not in pts:
            continue
        px, py = pts[key]
        color  = CLAW_COLOR if key == 'turtle1' else JOINT_COLOR
        radius = CLAW_RADIUS if key == 'turtle1' else JOINT_RADIUS
        pygame.draw.circle(surface, color, (px, py), radius)
        pygame.draw.circle(surface, BG_COLOR, (px, py), radius - 2)   # hollow

        # label
        txt = font.render(label, True, LABEL_COLOR)
        surface.blit(txt, (px + radius + 3, py - 7))

    # ── Draw claw heading line ───────────────────────────────────────────────
    if 'turtle1' in poses and 'turtle1' in pts:
        theta = poses['turtle1'].theta
        cx, cy = pts['turtle1']
        length = 20
        ex = int(cx + length * math.cos(theta))
        ey = int(cy - length * math.sin(theta))   # flip Y
        pygame.draw.line(surface, CLAW_COLOR, (cx, cy), (ex, ey), 2)

    # ── Draw target marker (turtle4) ────────────────────────────────────────
    if 'turtle4' in poses:
        tx, ty = sim_to_px(poses['turtle4'].x, poses['turtle4'].y)
        draw_cross(surface, TARGET_COLOR, tx, ty, TARGET_SIZE, 2)
        pygame.draw.circle(surface, TARGET_COLOR, (tx, ty), TARGET_SIZE, 1)
        lbl = font.render('target', True, TARGET_COLOR)
        surface.blit(lbl, (tx + TARGET_SIZE + 3, ty - 7))

    # ── HUD ─────────────────────────────────────────────────────────────────
    if 'turtle1' in poses:
        p = poses['turtle1']
        hud = font.render(f'claw  x={p.x:.2f}  y={p.y:.2f}', True, LABEL_COLOR)
        surface.blit(hud, (8, 8))

    pygame.display.flip()


# ── Main ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ArmOverlayNode()

    # Spin ROS in a background thread so pygame owns the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    pygame.init()
    surface = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption('Kaiju Arm Overlay')
    font  = pygame.font.SysFont('monospace', 13)
    clock = pygame.time.Clock()

    try:
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return

            poses = node.get_poses()
            render_frame(surface, font, poses)
            clock.tick(FPS)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()