#!/usr/bin/env python3
# Waypoint Manager: parses a route string, extracts spawn + goals,
# publishes each goal sequentially with a configurable wait between them.
# route format: "spawn_x,spawn_y;goal1_x,goal1_y;goal2_x,goal2_y;..."
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(Pose2D, 'goal', latched)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # --- Parameters ---
        self.declare_parameter('route',         '0.0,0.0;1.0,0.0')
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('goal_wait_time', 30.0)

        route_str       = self.get_parameter('route').value
        self.goal_tol   = self.get_parameter('goal_tolerance').value
        self.wait_time  = self.get_parameter('goal_wait_time').value

        # --- Parse route: first pair = spawn, rest = goals ---
        self.spawn, self.goals = self._parse_route(route_str)

        # --- State ---
        self.current_index  = 0          # index into self.goals
        self.goal_published = False
        self.waiting        = False       # True while timer between goals is running
        self.wait_start     = None        # wall-clock time when wait started
        self.goal_start     = None        # wall-clock time when current goal was published
        self.robot_x        = self.spawn[0]
        self.robot_y        = self.spawn[1]

        self.create_timer(0.2, self.tick)  # 5 Hz orchestration loop

        self.get_logger().info(
            f'[WP] Route parsed | spawn=({self.spawn[0]:.2f},{self.spawn[1]:.2f}) '
            f'| {len(self.goals)} goal(s): {self.goals}')
        self.get_logger().info(
            f'[WP] goal_tolerance={self.goal_tol}m  wait_between_goals={self.wait_time}s')

    # ─── Route parser ────────────────────────────────────────────────────────

    def _parse_route(self, route_str):
        """Parse 'x0,y0;x1,y1;...' → (spawn_tuple, [goal_tuples]).
        First pair is spawn, rest are goals.
        """
        pairs = [p.strip() for p in route_str.strip().split(';') if p.strip()]
        if len(pairs) < 2:
            self.get_logger().error(
                f'[WP] route must have at least 2 pairs (spawn + 1 goal). Got: "{route_str}"')
            return (0.0, 0.0), [(1.0, 0.0)]

        coords = []
        for pair in pairs:
            xy = pair.split(',')
            coords.append((float(xy[0].strip()), float(xy[1].strip())))

        spawn = coords[0]
        goals = coords[1:]
        return spawn, goals

    # ─── Main orchestration tick (5 Hz) ──────────────────────────────────────

    def tick(self):
        # All goals done
        if self.current_index >= len(self.goals):
            return

        # Waiting between goals
        if self.waiting:
            elapsed = time.monotonic() - self.wait_start
            remaining = self.wait_time - elapsed
            if remaining > 0:
                if int(remaining) % 5 == 0 and abs(remaining - int(remaining)) < 0.3:
                    self.get_logger().info(
                        f'[WP] Waiting... {remaining:.0f}s until goal '
                        f'{self.current_index + 1}/{len(self.goals)}')
                return
            # Wait over: publish next goal
            self.waiting = False
            self._publish_current_goal()
            return

        # Publish goal if not yet published
        if not self.goal_published:
            self._publish_current_goal()

    # ─── Odom callback: check if current goal reached ────────────────────────

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        if self.current_index >= len(self.goals):
            return
        if not self.goal_published or self.waiting:
            return

        gx, gy = self.goals[self.current_index]
        dist = math.hypot(gx - self.robot_x, gy - self.robot_y)

        if dist < self.goal_tol:
            elapsed = time.monotonic() - self.goal_start
            self.get_logger().info(
                f'[WP] ✓ Goal {self.current_index + 1}/{len(self.goals)} '
                f'({gx:.2f},{gy:.2f}) reached in {elapsed:.1f}s')

            self.current_index += 1

            if self.current_index >= len(self.goals):
                self.get_logger().info('[WP] ✓ Route complete. All goals reached.')
                return

            # Start wait before next goal
            self.get_logger().info(
                f'[WP] Waiting {self.wait_time:.0f}s before goal '
                f'{self.current_index + 1}/{len(self.goals)} '
                f'({self.goals[self.current_index][0]:.2f},'
                f'{self.goals[self.current_index][1]:.2f})')
            self.waiting    = True
            self.wait_start = time.monotonic()
            self.goal_published = False

    # ─── Goal publisher ───────────────────────────────────────────────────────

    def _publish_current_goal(self):
        gx, gy = self.goals[self.current_index]
        msg = Pose2D()
        msg.x = gx
        msg.y = gy
        self.publisher_.publish(msg)
        self.goal_published = True
        self.goal_start     = time.monotonic()
        self.get_logger().info(
            f'[WP] → Publishing goal {self.current_index + 1}/{len(self.goals)}: '
            f'({gx:.2f},{gy:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()