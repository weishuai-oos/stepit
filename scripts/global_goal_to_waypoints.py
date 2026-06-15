#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def angle_lerp(start: float, end: float, ratio: float) -> float:
    return wrap_to_pi(start + wrap_to_pi(end - start) * ratio)


class GlobalGoalToWaypoints(Node):
    def __init__(self) -> None:
        super().__init__("global_goal_to_waypoints")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("waypoints_topic", "/waypoints_b")
        self.declare_parameter("remain_time_topic", "/remain_time")
        self.declare_parameter("debug_path_topic", "/global_goal_waypoints_path")
        self.declare_parameter("v_des", 0.50)
        self.declare_parameter("min_speed", 0.08)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("target_tolerance", 0.08)
        self.declare_parameter("heading_tolerance", 0.10)
        self.declare_parameter("slowdown_distance", 0.80)
        self.declare_parameter("final_heading_distance", 0.80)
        self.declare_parameter("yaw_rate_des", 0.80)
        self.declare_parameter("max_waypoint_distance", 1.35)
        self.declare_parameter("remain_time", [0.5, 1.0, 1.5, 2.0, 2.5])
        self.declare_parameter("use_goal_heading", False)
        self.declare_parameter("hold_goal", True)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value
        self.remain_time_topic = self.get_parameter("remain_time_topic").value
        self.debug_path_topic = self.get_parameter("debug_path_topic").value

        self.v_des = float(self.get_parameter("v_des").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.target_tolerance = float(self.get_parameter("target_tolerance").value)
        self.heading_tolerance = float(self.get_parameter("heading_tolerance").value)
        self.slowdown_distance = float(self.get_parameter("slowdown_distance").value)
        self.final_heading_distance = float(self.get_parameter("final_heading_distance").value)
        self.yaw_rate_des = float(self.get_parameter("yaw_rate_des").value)
        self.max_waypoint_distance = float(self.get_parameter("max_waypoint_distance").value)
        self.remain_time = [float(x) for x in self.get_parameter("remain_time").value]
        self.use_goal_heading = bool(self.get_parameter("use_goal_heading").value)
        self.hold_goal = bool(self.get_parameter("hold_goal").value)

        if len(self.remain_time) != 5:
            raise ValueError("remain_time must contain exactly 5 values for g1_traj_finetune_v2.")
        if self.v_des < 0.0:
            raise ValueError("v_des must be non-negative.")
        if self.min_speed < 0.0:
            raise ValueError("min_speed must be non-negative.")
        if self.target_tolerance <= 0.0:
            raise ValueError("target_tolerance must be positive.")
        if self.heading_tolerance <= 0.0:
            raise ValueError("heading_tolerance must be positive.")
        if self.slowdown_distance <= 0.0:
            raise ValueError("slowdown_distance must be positive.")
        if self.final_heading_distance <= 0.0:
            raise ValueError("final_heading_distance must be positive.")
        if self.yaw_rate_des <= 0.0:
            raise ValueError("yaw_rate_des must be positive.")
        if self.max_waypoint_distance <= 0.0:
            raise ValueError("max_waypoint_distance must be positive.")

        self.current_xy: Optional[tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.odom_frame: Optional[str] = None

        self.start_xy: Optional[tuple[float, float]] = None
        self.goal_xy: Optional[tuple[float, float]] = None
        self.goal_heading: Optional[float] = None
        self.goal_frame: Optional[str] = None
        self.frame_warned = False
        self.reached_reported = False

        self.waypoints_pub = self.create_publisher(Float32MultiArray, self.waypoints_topic, 10)
        self.remain_time_pub = self.create_publisher(Float32MultiArray, self.remain_time_topic, 10)
        self.debug_path_pub = self.create_publisher(Path, self.debug_path_topic, 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive.")
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Publishing {self.waypoints_topic} from {self.goal_topic} + {self.odom_topic}; "
            f"v_des={self.v_des:.2f} m/s, use_goal_heading={self.use_goal_heading}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.current_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.odom_frame = msg.header.frame_id or "odom"

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.current_xy is None:
            self.get_logger().warn("Ignoring goal: no odometry received yet.")
            return

        self.start_xy = self.current_xy
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_heading = yaw_from_quat(msg.pose.orientation)
        self.goal_frame = msg.header.frame_id or self.odom_frame
        self.frame_warned = False
        self.reached_reported = False

        distance = math.hypot(self.goal_xy[0] - self.start_xy[0], self.goal_xy[1] - self.start_xy[1])
        self.get_logger().info(
            f"New goal: start=({self.start_xy[0]:.3f}, {self.start_xy[1]:.3f}), "
            f"goal=({self.goal_xy[0]:.3f}, {self.goal_xy[1]:.3f}), distance={distance:.3f} m, "
            f"goal_yaw={self.goal_heading:.3f} rad"
        )

    def clear_goal(self) -> None:
        self.start_xy = None
        self.goal_xy = None
        self.goal_heading = None
        self.goal_frame = None
        self.frame_warned = False
        self.reached_reported = False

    def timer_callback(self) -> None:
        self.publish_remain_time()

        if self.current_xy is None or self.current_yaw is None:
            self.publish_zero_waypoints()
            self.publish_debug_path([])
            return

        if self.start_xy is None or self.goal_xy is None:
            self.publish_zero_waypoints()
            self.publish_debug_path([])
            return

        if self.goal_frame and self.odom_frame and self.goal_frame != self.odom_frame and not self.frame_warned:
            self.get_logger().warn(
                f"Goal frame '{self.goal_frame}' differs from odom frame '{self.odom_frame}'. "
                "This node assumes both are already in the same frame."
            )
            self.frame_warned = True

        waypoints, debug_poses, reached = self.compute_command()
        self.publish_waypoints(waypoints)
        self.publish_debug_path(debug_poses)

        if reached:
            if not self.reached_reported:
                self.get_logger().info("Goal pose reached; holding final pose.")
                self.reached_reported = True
            if not self.hold_goal:
                self.clear_goal()

    def compute_command(self) -> tuple[list[float], list[tuple[float, float, float]], bool]:
        assert self.current_xy is not None
        assert self.current_yaw is not None
        assert self.goal_xy is not None

        pos_error = math.hypot(self.goal_xy[0] - self.current_xy[0], self.goal_xy[1] - self.current_xy[1])
        yaw_error = self.goal_yaw_error()
        yaw_reached = (not self.use_goal_heading) or abs(yaw_error) <= self.heading_tolerance

        if pos_error <= self.target_tolerance:
            waypoints, debug_poses = self.compute_goal_pose_waypoints()
            return waypoints, debug_poses, yaw_reached

        waypoints, debug_poses = self.compute_tracking_waypoints()
        return waypoints, debug_poses, False

    def compute_tracking_waypoints(self) -> tuple[list[float], list[tuple[float, float, float]]]:
        assert self.start_xy is not None
        assert self.goal_xy is not None
        assert self.current_xy is not None
        assert self.current_yaw is not None

        seg_x = self.goal_xy[0] - self.start_xy[0]
        seg_y = self.goal_xy[1] - self.start_xy[1]
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len <= 1e-6:
            return self.compute_goal_pose_waypoints()

        dir_x = seg_x / seg_len
        dir_y = seg_y / seg_len
        line_heading = math.atan2(dir_y, dir_x)

        cur_x = self.current_xy[0] - self.start_xy[0]
        cur_y = self.current_xy[1] - self.start_xy[1]
        progress = cur_x * dir_x + cur_y * dir_y
        progress = max(0.0, min(progress, seg_len))
        remaining_along_path = max(0.0, seg_len - progress)

        speed_scale = min(1.0, remaining_along_path / self.slowdown_distance)
        speed = min(self.v_des, max(self.min_speed, self.v_des * speed_scale))

        blend_start = max(0.0, seg_len - self.final_heading_distance)
        blend_span = max(1e-6, seg_len - blend_start)

        waypoints: list[float] = []
        debug_poses: list[tuple[float, float, float]] = []
        for t in self.remain_time:
            s = min(progress + speed * t, seg_len)
            target_x = self.start_xy[0] + dir_x * s
            target_y = self.start_xy[1] + dir_y * s
            target_heading = line_heading

            if self.use_goal_heading and self.goal_heading is not None:
                blend = smoothstep((s - blend_start) / blend_span)
                target_heading = angle_lerp(line_heading, self.goal_heading, blend)

            waypoints.extend(self.global_pose_to_local(target_x, target_y, target_heading))
            debug_poses.append((target_x, target_y, target_heading))

        return waypoints, debug_poses

    def compute_goal_pose_waypoints(self) -> tuple[list[float], list[tuple[float, float, float]]]:
        assert self.current_yaw is not None
        assert self.goal_xy is not None

        yaw_error = self.goal_yaw_error()
        waypoints: list[float] = []
        debug_poses: list[tuple[float, float, float]] = []

        for t in self.remain_time:
            if self.use_goal_heading:
                heading_step = math.copysign(min(abs(yaw_error), self.yaw_rate_des * t), yaw_error)
                target_heading = wrap_to_pi(self.current_yaw + heading_step)
            else:
                target_heading = self.current_yaw

            waypoints.extend(self.global_pose_to_local(self.goal_xy[0], self.goal_xy[1], target_heading))
            debug_poses.append((self.goal_xy[0], self.goal_xy[1], target_heading))

        return waypoints, debug_poses

    def goal_yaw_error(self) -> float:
        if self.current_yaw is None or self.goal_heading is None:
            return 0.0
        return wrap_to_pi(self.goal_heading - self.current_yaw)

    def global_pose_to_local(self, target_x: float, target_y: float, target_heading: float) -> list[float]:
        assert self.current_xy is not None
        assert self.current_yaw is not None

        dx = target_x - self.current_xy[0]
        dy = target_y - self.current_xy[1]
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        x_b = cos_yaw * dx + sin_yaw * dy
        y_b = -sin_yaw * dx + cos_yaw * dy

        distance = math.hypot(x_b, y_b)
        if distance > self.max_waypoint_distance:
            scale = self.max_waypoint_distance / distance
            x_b *= scale
            y_b *= scale

        heading_b = wrap_to_pi(target_heading - self.current_yaw)
        return [x_b, y_b, heading_b]

    def publish_waypoints(self, waypoints: list[float]) -> None:
        msg = Float32MultiArray()
        msg.data = waypoints
        self.waypoints_pub.publish(msg)

    def publish_zero_waypoints(self) -> None:
        self.publish_waypoints([0.0] * 15)

    def publish_remain_time(self) -> None:
        msg = Float32MultiArray()
        msg.data = self.remain_time
        self.remain_time_pub.publish(msg)

    def publish_debug_path(self, poses: list[tuple[float, float, float]]) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame or self.goal_frame or "odom"

        for x, y, yaw in poses:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = quat_from_yaw(yaw)
            msg.poses.append(pose)

        self.debug_path_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GlobalGoalToWaypoints()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
