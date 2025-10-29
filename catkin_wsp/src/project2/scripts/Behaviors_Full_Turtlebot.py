#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def normalize_angle(a):
    """Keep angle in [-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a

class ReactiveController(object):
    """
    Priorities (high -> low):
      1) Halt if bumper pressed.
      2) Accept keyboard teleop (/cmd_vel).
      3) Escape symmetric front obstacle (≤1 ft both sides, nearly equal).
      4) Avoid asymmetric front obstacle (≤1 ft on one side).
      5) Turn randomly (±15°) after every ~1 ft of travel.
      6) Drive forward.
    """

    def __init__(self):
        rospy.init_node('reactive_controller', anonymous=True)

        # Publisher to robot's velocity topic
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.teleop_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # --- State ---
        self.bumped = False
        self.last_teleop_cmd = None  # None means no teleop input

        # Current pose / yaw (from odom)
        self.last_x = None
        self.last_y = None
        self.current_yaw = 0.0

        # Escape (two-phase: back -> turn-to-angle)
        self.escaping = False
        self.escape_phase = None      # "back" or "turn"
        self.escape_end_time = rospy.Time.now()
        self.escape_target_yaw = None

        # Random turn (angle-based ±15° after 1 ft movement)
        self.distance_accum = 0.0
        self.turning_random = False
        self.random_target_yaw = None
        self.k_yaw = 3.0
        self.w_max = 1.8
        self.yaw_tolerance = math.radians(2.0)

        # ---- Thresholds (KEEPING YOUR ORIGINAL VALUES) ----
        # NOTE: you previously commented that 1 ft ≈ 0.3048 m, but set these to 0.50.
        # Per your request, we keep them unchanged.
        self.front_thresh = 0.50      # meters
        self.side_thresh  = 0.50      # meters

        rospy.loginfo("ReactiveController started (bumpers + teleop + scan) with original sectors & thresholds.")

    # ------------------- BUMPER -------------------
    def bumper_callback(self, msg):
        if msg.state == BumperEvent.PRESSED:
            rospy.logwarn("Bumper pressed! Starting escape maneuver.")
            self.start_escape()
            self.bumped = True
            self.cmd_pub.publish(Twist())  # stop immediately
        else:
            rospy.loginfo("Bumper released! Robot can move again.")
            self.bumped = False

    # ------------------- TELEOP -------------------
    def teleop_callback(self, msg):
        """Store last teleop command if active, reset if released."""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_teleop_cmd = msg
        else:
            self.last_teleop_cmd = None  # reset when keys released

    # ------------------- ODOM (distance + yaw) -------------------
    def odom_callback(self, msg):
        """Track distance traveled and update current yaw."""
        # Current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Current yaw from quaternion
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

        # Accumulate planar distance for random-turn trigger
        if self.last_x is not None and self.last_y is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            self.distance_accum += math.sqrt(dx*dx + dy*dy)

        self.last_x = x
        self.last_y = y

        # Trigger random turn after 0.30 m (~1 ft), only when safe
        if (self.distance_accum >= 0.30 and
            not self.turning_random and
            not self.escaping and
            not self.bumped):
            self.start_random_turn()

    # ------------------- ESCAPE MANEUVER -------------------
    def start_escape(self):
        """Start a two-phase escape: reverse 0.5s + angle turn to 180±30° away."""
        self.escaping = True
        self.escape_phase = "back"
        self.escape_end_time = rospy.Time.now() + rospy.Duration.from_sec(0.5)  # fixed reverse
        self.escape_target_yaw = None  # will be set at start of "turn" phase

    def do_escape(self):
        """Perform current escape phase (time-based reverse, then angle-based turn)."""
        twist = Twist()
        if self.escape_phase == "back":
            twist.linear.x = -0.2
            self.cmd_pub.publish(twist)
            if rospy.Time.now() >= self.escape_end_time:
                # Switch to angle-turn phase
                self.escape_phase = "turn"
                # set angle target = current_yaw + (pi ± 30°), normalized
                delta = math.pi + random.uniform(-math.radians(30.0), math.radians(30.0))
                self.escape_target_yaw = normalize_angle(self.current_yaw + delta)
        elif self.escape_phase == "turn":
            # angle-based turn using P-control to target yaw
            err = normalize_angle(self.escape_target_yaw - self.current_yaw)
            if abs(err) > self.yaw_tolerance:
                twist.angular.z = max(-self.w_max, min(self.k_yaw * err, self.w_max))
                self.cmd_pub.publish(twist)
            else:
                # Done
                self.escaping = False
                self.escape_phase = None
                self.escape_target_yaw = None
                rospy.loginfo("Escape complete. Resuming normal operation.")

    # ------------------- RANDOM TURN (±15°) -------------------
    def start_random_turn(self):
        """Set a small ±15° target relative to current yaw (angle-based)."""
        self.turning_random = True
        self.distance_accum = 0.0
        delta = random.uniform(-math.radians(35.0), math.radians(35.0))
        self.random_target_yaw = normalize_angle(self.current_yaw + delta)
        rospy.logwarn("Random turn triggered (±15°) per 1 ft requirement.")

    def do_random_turn(self):
        """Angle-based random turn to random_target_yaw."""
        twist = Twist()
        err = normalize_angle(self.random_target_yaw - self.current_yaw)
        if abs(err) > self.yaw_tolerance:
            twist.angular.z = max(-self.w_max, min(self.k_yaw * err, self.w_max))
            self.cmd_pub.publish(twist)
        else:
            self.turning_random = False
            self.random_target_yaw = None

    # ------------------- SCAN -------------------
    def scan_callback(self, msg):
        # Highest-priority behaviors first
        if self.bumped:
            return

        # Escape in progress
        if self.escaping:
            self.do_escape()
            return

        # Random small turn takes precedence over default/autonomy
        if self.turning_random:
            self.do_random_turn()
            return

        ranges = msg.ranges
        num_ranges = len(ranges)
        if num_ranges == 0:
            return

        # --- KEEP YOUR ORIGINAL SECTORS ---
        # FRONT SECTORS (~70° each) using your indices
        left_sector  = ranges[num_ranges//2 + 70 : num_ranges//2 + 110]
        right_sector = ranges[num_ranges//2 - 110 : num_ranges//2 - 70]

        # SIDE SECTORS (~30° each at ±90°) using your indices
        left_side_sector  = ranges[num_ranges - 100 : num_ranges - 30]   # left side
        right_side_sector = ranges[30:100]                               # right side

        def valid(vals):
            return [r for r in vals if not math.isinf(r) and not math.isnan(r) and r > 0.0]

        left_min        = min(valid(left_sector))       if valid(left_sector)       else 10.0
        right_min       = min(valid(right_sector))      if valid(right_sector)      else 10.0
        left_side_min   = min(valid(left_side_sector))  if valid(left_side_sector)  else 10.0
        right_side_min  = min(valid(right_side_sector)) if valid(right_side_sector) else 10.0

        # Thresholds (meters) — KEEPING YOUR VALUES
        front_thresh = self.front_thresh   # 0.50 m
        side_thresh  = self.side_thresh    # 0.50 m

        twist = Twist()

        # --- Symmetric Escape (≤ 1 ft both sides, nearly equal) ---
        if (left_min < front_thresh and right_min < front_thresh and
            abs(left_min - right_min) < 0.10):
            rospy.logwarn("Symmetric obstacle detected! Escaping (back + 180±30°).")
            self.start_escape()
            return

        # --- (Optional) Side Wall Avoidance ---
        # You had this in your earlier code; keeping it since side_thresh is part of your thresholds.
        if left_side_min < side_thresh:
            rospy.logwarn("Too close to LEFT wall → nudging RIGHT")
            twist.angular.z = -0.6
            twist.linear.x = 0.05
        elif right_side_min < side_thresh:
            rospy.logwarn("Too close to RIGHT wall → nudging LEFT")
            twist.angular.z = 0.6
            twist.linear.x = 0.05
        # --- Asymmetric Front Avoidance (≤ 1 ft) ---
        elif left_min < front_thresh and right_min >= front_thresh:
            rospy.logwarn("Obstacle on LEFT → turning RIGHT (auto)")
            twist.angular.z = -1.8
        elif right_min < front_thresh and left_min >= front_thresh:
            rospy.logwarn("Obstacle on RIGHT → turning LEFT (auto)")
            twist.angular.z = 1.8
        else:
            # Default autonomous forward motion
            twist.linear.x = 0.25

        # --- Priority: teleop if active (but only if safe) ---
        if self.last_teleop_cmd is not None:
            # Only pass through teleop if NO front obstacle within front_thresh
            if left_min > front_thresh and right_min > front_thresh:
                self.cmd_pub.publish(self.last_teleop_cmd)
            else:
                rospy.logwarn("Teleop blocked: obstacle ≤ threshold!")
                self.cmd_pub.publish(Twist())  # stop instead of crash
        else:
            self.cmd_pub.publish(twist)

    # ------------------- RUN -------------------
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        ReactiveController().run()
    except rospy.ROSInterruptException:
        pass
