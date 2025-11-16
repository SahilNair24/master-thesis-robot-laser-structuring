#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState


class TestMotionPlanNode(Node):
    def __init__(self):
        super().__init__('test_motion_plan_node')

        # Create the MoveGroup action client
        self._client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info("Waiting for move_group action server...")
        self._client.wait_for_server()
        self.get_logger().info("Connected to move_group.")

        # Send a planning goal
        self.send_motion_plan()

    def send_motion_plan(self):
        self.get_logger().info("Building motion planning goal...")

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()

        # Name of the planning group in your SRDF (e.g., "arm")
        goal.request.group_name = "arm"

        # Pose target
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = 1.4
        pose_stamped.pose.position.y = 0.3
        pose_stamped.pose.position.z = 0.5
        pose_stamped.pose.orientation.w = 1.0

        joint_state = JointState()
        joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]  # Replace with your joint names
        joint_state.position = [0.0, -1.57, 1.2, 0.0, 0.5, 0.0]  # Replace with current joint values

        # Create and assign start state
        start_state = RobotState()
        start_state.joint_state = joint_state
        goal.request.start_state = start_state

        constraints = Constraints()

        # PositionConstraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_stamped.header
        pos_constraint.link_name = "link_6"  # <-- Replace with your end-effector link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
        )
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)

        # OrientationConstraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose_stamped.header
        ori_constraint.link_name = "link_6"
        ori_constraint.orientation = pose_stamped.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(constraints)

        # Planning options
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.planning_options.plan_only = False

        # Send the goal
        self.get_logger().info("Sending motion plan goal...")
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Motion goal rejected by move_group")
            return

        self.get_logger().info("✅ Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"📦 Planning result code: {result.error_code.val}")
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("🎉 Motion plan executed successfully!")
        else:
            self.get_logger().error("🚫 Motion planning failed.")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = TestMotionPlanNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
