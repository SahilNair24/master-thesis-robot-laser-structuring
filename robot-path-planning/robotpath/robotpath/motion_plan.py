#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
from moveit_msgs.msg import JointConstraint, Constraints
# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import PoseStamped
# from trajectory_msgs.msg import JointTrajectoryPoint

import os
import csv
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import JointState

# def simulate_joint_state_update(node, trajectory_msg):
#     joint_pub = node.create_publisher(JointState, '/joint_states', 10)

#     last_point = trajectory_msg.joint_trajectory.points[-1]
#     joint_names = trajectory_msg.joint_trajectory.joint_names

#     joint_state_msg = JointState()
#     joint_state_msg.header.stamp = node.get_clock().now().to_msg()
#     joint_state_msg.name = joint_names
#     joint_state_msg.position = list(last_point.positions)

#     # Publish a few times to ensure RViz catches it
#     for _ in range(5):
#         joint_pub.publish(joint_state_msg)
#         time.sleep(0.1)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
    save_trajectory_path=None,
):
    
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()
    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])

        # Save the trajectory if a path is provided
        if save_trajectory_path:
            try:
                # Convert RobotTrajectory to moveit_msgs.msg.RobotTrajectory
                trajectory_msg = robot_trajectory.get_robot_trajectory_msg()
                # Ensure the directory exists
                os.makedirs(os.path.dirname(save_trajectory_path), exist_ok=True)
                # Serialize and save the trajectory
                points = trajectory_msg.joint_trajectory.points
                # joint_names = trajectory_msg.joint_trajectory.joint_names   
                positions = []

                for point in points:
                    positions_raw = point.positions
                    positions_radians = [round(angle,4) for angle in positions_raw]
                    # positions_degrees = [round(math.degrees(angle),4) for angle in positions_raw]
                    # positions.append(positions_degrees)
                    positions.append(positions_radians)

                with open(save_trajectory_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerows(positions)
                logger.info(f"Trajectory saved to {save_trajectory_path}")
            except Exception as e:
                logger.error(f"Failed to save trajectory: {e}")
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    panda = MoveItPy(node_name="moveit_py")
    panda_arm = panda.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    trajectory_file_path = '/home/sahilsnair/Desktop/laser-toolpath/scripts/joint_positions.csv'

    # # set plan start state using predefined state
    panda_arm.set_start_state(configuration_name="home")

    # instantiate a RobotState instance using the current robot model
    robot_model = panda.get_robot_model()
    # robot_state = RobotState(robot_model)

    # # set plan start state to current state
    panda_arm.set_start_state_to_current_state()

    # This just clears the file every time the node is run
    with open(trajectory_file_path, mode='w', newline='') as file:
        pass  

    with open('/home/sahilsnair/Desktop/laser-toolpath/scripts/robotpath.csv', 'r') as file:
        reader = csv.reader(file)
        
        # Iterate through each row (line)
        for row in reader:
            # Convert each row into a list of floats
            # print(row)
            data = [float(value) for value in row]
            # print(data)
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "base_link"
            position = np.array([data[0],data[1],data[2]])
            safe_distance = 0.3


            #### Normal to Quaternion implementation begins
            normal = np.array([data[3], data[4], data[5]])  # replace with your normal
            normal = normal / np.linalg.norm(normal)
            # Desired tool X-axis is INTO the surface → opposite of the normal
            desired_x = -normal

            # Pick an arbitrary "up" vector not colinear with desired_x (e.g., Z-up)
            up = np.array([0, 0, 1])
            if np.allclose(np.abs(np.dot(up, desired_x)), 1.0):  # if colinear
                up = np.array([0, 1, 0])  # choose another

            # Compute Y and Z axes to form a rotation matrix
            z_axis = np.cross(desired_x, up)
            z_axis /= np.linalg.norm(z_axis)

            y_axis = np.cross(z_axis, desired_x)
            y_axis /= np.linalg.norm(y_axis)

            # Rotation matrix: columns = X Y Z axes
            rotation_matrix = np.column_stack((desired_x, y_axis, z_axis))

            # Convert to quaternion
            quat = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
            qx, qy, qz, qw = quat
            # implementation ends


            offset_position = position + safe_distance * normal
            # qx, qy, qz, qw = R.from_euler('xyz', [data[3], data[4], data[5]]).as_quat()  # [x, y, z, w]
            pose_goal.pose.orientation.x = qx
            pose_goal.pose.orientation.y = qy
            pose_goal.pose.orientation.z = qz
            pose_goal.pose.orientation.w = qw
            # pose_goal.pose.position.x = data[0]
            # pose_goal.pose.position.y = data[1]
            # pose_goal.pose.position.z = data[2]
            pose_goal.pose.position.x = offset_position[0]
            pose_goal.pose.position.y = offset_position[1]
            pose_goal.pose.position.z = offset_position[2]
            panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link_6")
            plan_and_execute(panda, panda_arm, logger, sleep_time=3.0,save_trajectory_path=trajectory_file_path)




if __name__ == "__main__":
    main()
