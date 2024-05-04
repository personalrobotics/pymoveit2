#!/usr/bin/env python3

"""
Example of moving two robot arms to joint configurations simultaneously.
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.executors
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import geodude_dual_arms
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()

    node = Node("trial_dual_arms")

    callback_group = ReentrantCallbackGroup()

    moveit2_dual_arms = MoveIt2(
        node=node,
        joint_names=geodude_dual_arms.joint_names(),
        base_link_name="world",
        end_effector_name="left_wam7",
        group_name=geodude_dual_arms.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    moveit2_dual_arms.max_velocity = 0.25
    moveit2_dual_arms.max_acceleration = 0.25

    pos_left = PoseStamped()
    pos_left.pose.position.x = -0.33
    pos_left.pose.position.y = -0.49
    pos_left.pose.position.z = 1.95
    pos_left.pose.orientation.x = 0.00
    pos_left.pose.orientation.y = 0.00
    pos_left.pose.orientation.z = 0.00
    pos_left.pose.orientation.w = 1.00

    pos_right = PoseStamped()
    pos_right.pose.position.x = 0.32
    pos_right.pose.position.y = -0.49
    pos_right.pose.position.z = 1.95
    pos_right.pose.orientation.x = 0.00
    pos_right.pose.orientation.y = 0.00
    pos_right.pose.orientation.z = 0.00
    pos_right.pose.orientation.w = 1.00

    moveit2_dual_arms.set_pose_goal(pos_left, target_link="left_wam7", frame_id="world")
    moveit2_dual_arms.set_pose_goal(pos_right, target_link="right_wam7", frame_id="world")

    # moveit2_dual_arms.set_position_goal(position=(-0.33, -0.49, 1.95), frame_id="world", target_link="left_wam7")
    # moveit2_dual_arms.set_orientation_goal(quat_xyzw=(0.00, 0.00, 0.00, 1.00), frame_id="world", target_link="left_wam7")
    # moveit2_dual_arms.set_position_goal(pos_right.pose.position, frame_id="world", target_link="right_wam7")
    # moveit2_dual_arms.set_orientation_goal(pos_right.pose.orientation, frame_id="world", target_link="right_wam7")

    moveit2_dual_arms.plan()
    rate = node.create_rate(10)

    while moveit2_dual_arms.query_state() != MoveIt2State.EXECUTING:
        rate.sleep()
    future_ = moveit2_dual_arms.get_execution_future()

    while not future_.done():
        rate.sleep()

    node.get_logger().info("status: " + str(future_.result().status))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
