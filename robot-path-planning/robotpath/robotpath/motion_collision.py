#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_msgs.srv import ApplyPlanningScene
import os
import csv
import math
import time

class MotionPlannerWithCollision(Node):
    def __init__(self):
        super().__init__("motion_planner_with_collision")
        self.logger = self.get_logger()

        # Create publisher for planning scene updates
        self.planning_scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # Add the collision object
        self.publish_collision_object()

        # Proceed with motion planning
        self.run_motion_planning()

    def apply_planning_scene(self, planning_scene_msg):
        client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not client.wait_for_service(timeout_sec=5.0):
            self.logger.error('ApplyPlanningScene service not available')
            return

        req = ApplyPlanningScene.Request()
        req.scene = planning_scene_msg
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.logger.info("Planning scene successfully applied.")
        else:
            self.logger.error("Failed to apply planning scene.")

    def publish_collision_object(self):
        self.logger.info("Publishing collision object...")

        collision_object = CollisionObject()
        collision_object.id = "box1"
        collision_object.header = Header()
        collision_object.header.frame_id = "base_link"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.5, 0.5, 0.05]

        pose = Pose()
        pose.position.x = 1.5
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(collision_object)
        planning_scene_msg.is_diff = True

        # Apply the planning scene to ensure MoveIt uses it in planning
        self.apply_planning_scene(planning_scene_msg)

        self.planning_scene_pub.publish(planning_scene_msg)
        self.logger.info("Collision object published.")
        time.sleep(2.0)  # Ensure collision is registered

    def plan_and_execute(self, robot, planning_component, pose_goal, save_path):
        planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")
        self.logger.info("Planning trajectory")

        plan_result = planning_component.plan()
        if plan_result:
            self.logger.info("Executing plan")
            trajectory_msg = plan_result.trajectory.get_robot_trajectory_msg()

            robot.execute(plan_result.trajectory, controllers=[])

            try:
                os.makedirs(os.path.dirname(save_path), exist_ok=True)
                points = trajectory_msg.joint_trajectory.points
                positions = [
                    [round(angle, 4) for angle in point.positions]
                    for point in points
                ]
                with open(save_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerows(positions)
                self.logger.info(f"Trajectory saved to {save_path}")
            except Exception as e:
                self.logger.error(f"Failed to save trajectory: {e}")
        else:
            self.logger.error("Planning failed")

        time.sleep(3.0)

    def run_motion_planning(self):
        panda = MoveItPy(node_name="moveit_py")
        panda_arm = panda.get_planning_component("arm")
        panda_arm.set_start_state(configuration_name="home")
        panda_arm.set_start_state_to_current_state()
        robot_model = panda.get_robot_model()

        trajectory_file_path = '/home/sahilsnair/Desktop/laser-toolpath/scripts/joint_positions.csv'
        input_csv = '/home/sahilsnair/Desktop/laser-toolpath/scripts/robotpath_single.csv'

        with open(trajectory_file_path, mode='w', newline='') as file:
            pass

        with open(input_csv, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                data = [float(x) for x in row]
                pose_goal = PoseStamped()
                pose_goal.header.frame_id = "base_link"
                pose_goal.pose.orientation.w = 1.0
                pose_goal.pose.position.x = data[0]
                pose_goal.pose.position.y = data[1]
                pose_goal.pose.position.z = data[2]
                self.plan_and_execute(panda, panda_arm, pose_goal, trajectory_file_path)


def main():
    rclpy.init()
    node = MotionPlannerWithCollision()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
