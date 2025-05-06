#!/usr/bin/env python3

'''Copyright [2025] [Walter Glockner]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.'''


"""
PoseArray → MoveIt2 Cartesian path (chunks ≥2 way‑points)
Humble‑compatible (GetCartesianPath w/out scaling fields)
"""

import rclpy, math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState, Constraints, PositionConstraint
from std_msgs.msg import Header

# ── CONFIG ────────────────────────────────────────────────────────────────
GROUP      = "khi_cx110l"
TCP_LINK   = "khi_cx110l_link6"
EXEC_TOPIC = "/joint_trajectory_position_controller/joint_trajectory"
PRINT_VEL, TRAVEL_VEL = 0.1, 0.5
Z_JUMP_TH   = 0.005         # >5 mm jump → travel
CHUNK_SIZE  = 1000          # split very long segments
# ──────────────────────────────────────────────────────────────────────────


class ToolpathPlanner(Node):
    def __init__(self):
        super().__init__("toolpath_planner")

        # I/O
        self.create_subscription(PoseArray, "/tool_poses",
                                 self.cb_toolpath, 10)
        self.traj_pub = self.create_publisher(JointTrajectory,
                                              EXEC_TOPIC, 10)
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states",
                                 self.cb_joint, 10)

        # MoveIt connections
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.move_client.wait_for_server()

        self.cart_client = self.create_client(GetCartesianPath,
                                              '/compute_cartesian_path')
        while not self.cart_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /compute_cartesian_path …")

        self.get_logger().info("✔︎ Connected to MoveIt 2")

        self.last_z = None

    # -------------------------------------------------------------- #
    def cb_joint(self, msg: JointState):
        self.joint_state = msg                          # always keep latest

    # -------------------------------------------------------------- #
    def cb_toolpath(self, msg: PoseArray):
        self.get_logger().info(f"Received {len(msg.poses)} way‑points")

        segments, cur, cur_type = [], [], False
        for p in msg.poses:
            travel = (self.last_z is not None and
                      abs(p.position.z - self.last_z) > Z_JUMP_TH)
            if cur and travel != cur_type:
                segments.append((cur, cur_type))
                cur = []
            cur.append(p)
            cur_type, self.last_z = travel, p.position.z
        if cur:
            segments.append((cur, cur_type))

        for poses, travel in segments:
            self.plan_segment(poses, travel)

    # ──────────────────────────────────────────────────────────────────
    #  Cartesian planning with fraction check & chunk subdivision      #
    # ──────────────────────────────────────────────────────────────────
    def plan_segment(self, poses, travel):
        vel_scale = TRAVEL_VEL if travel else PRINT_VEL
        kind      = "travel"    if travel else "print"

        stack = [poses]                              # start with whole segment
        while stack:
            chunk = stack.pop(0)
            if len(chunk) == 1:
                self.simple_joint_goal(chunk[0], vel_scale, kind)
                continue

            ok, traj = self.cartesian_chunk(chunk, vel_scale)
            if ok:
                self.traj_pub.publish(traj)
                self.get_logger().info(f"✓ {kind}: {len(traj.points)} pts")
            else:
                # If a large chunk failed, split it and try again
                if len(chunk) > 10:
                    mid = len(chunk) // 2
                    stack.insert(0, chunk[mid:])
                    stack.insert(0, chunk[:mid])
                else:
                    # Too small to split – fall back to joint‑goals
                    self.get_logger().warn(f"{kind}: fallback jt goals for "
                                           f"{len(chunk)} poses")
                    for p in chunk:
                        self.simple_joint_goal(p, vel_scale, kind)

    # ------------------------------------------------------------------
    def cartesian_chunk(self, poses, vel_scale):
        """Returns (success, joint_trajectory)"""
        req = GetCartesianPath.Request()
        req.header.frame_id  = "map"            # same frame as PoseArray
        req.group_name       = GROUP
        req.link_name        = TCP_LINK
        req.max_step         = 0.005
        req.jump_threshold   = 0.0
        req.avoid_collisions = True
        req.waypoints        = poses

        if self.joint_state:
            rs = RobotState()
            rs.joint_state = self.joint_state
            req.start_state = rs

        fut = self.cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()

        if (res is None or
            res.error_code.val != res.error_code.SUCCESS or
            res.fraction < 0.99 or                    # <── key new check
            len(res.solution.joint_trajectory.points) < 2):
            self.get_logger().debug(f"Cartesian frac={getattr(res,'fraction',0)} "
                                     "pts="f"{len(res.solution.joint_trajectory.points) if res else 0}")
            return False, None

        # simple time‑scaling
        if vel_scale < 1.0:
            for pt in res.solution.joint_trajectory.points:
                t = pt.time_from_start
                pt.time_from_start = Duration(
                    sec=int(t.sec/vel_scale),
                    nanosec=int(t.nanosec/vel_scale))

        return True, res.solution.joint_trajectory

    def simple_joint_goal(self, pose, vel_scale, kind):
        """
        MoveGroup joint goal to reach a single pose
        """
        if not self.joint_state:
            self.get_logger().error("No /joint_states yet; cannot fallback")
            return

        goal = MoveGroup.Goal()
        goal.request.group_name              = GROUP
        goal.request.allowed_planning_time   = 5.0
        goal.request.max_velocity_scaling_factor = vel_scale

        # create a PositionConstraint using the pose
        pc = PositionConstraint()
        pc.header         = Header(frame_id="world")
        pc.link_name      = TCP_LINK
        pc.target_point_offset.x = pc.target_point_offset.y = pc.target_point_offset.z = 0.0
        pc.constraint_region.primitives.append(
            # small box around the target point
            self.create_box(0.001))
        pc.constraint_region.primitive_poses.append(pose)

        c = Constraints()
        c.name = "reach_single_pose"
        c.position_constraints.append(pc)
        goal.request.goal_constraints.append(c)
        goal.request.start_state.joint_state = self.joint_state

        self.move_client.send_goal_async(goal)
        self.get_logger().info(f"✓ {kind}: fallback joint goal sent")

    # helper: tiny box Primitive
    @staticmethod
    def create_box(size):
        from shape_msgs.msg import SolidPrimitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size, size, size]
        return box


def main():
    rclpy.init()
    rclpy.spin(ToolpathPlanner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
