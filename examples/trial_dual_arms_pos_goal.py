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
        end_effector_name="world",
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
    pos_left.pose.position.x = 0.300
    pos_left.pose.position.y = -0.539
    pos_left.pose.position.z = 0.302
    pos_left.pose.orientation.x = -0.086
    pos_left.pose.orientation.y = 0.701
    pos_left.pose.orientation.z = -0.701
    pos_left.pose.orientation.w = -0.086

    pos_right = PoseStamped()
    pos_right.pose.position.x = -0.026
    pos_right.pose.position.y = -0.545
    pos_right.pose.position.z = 0.620
    pos_right.pose.orientation.x = 0.577
    pos_right.pose.orientation.y = 0.605
    pos_right.pose.orientation.z = -0.523
    pos_right.pose.orientation.w = -0.160

    # moveit2_dual_arms.set_pose_goal(pos_left, target_link="left_wam7", frame_id="left_wam_base")
    # moveit2_dual_arms.set_pose_goal(pos_right, target_link="right_wam7", frame_id="right_wam_base")
    moveit2_dual_arms.set_position_goal(pos_left.pose.position, frame_id="left_wam_base", target_link="left_wam7")
    moveit2_dual_arms.set_orientation_goal(pos_left.pose.orientation, frame_id="left_wam_base", target_link="left_wam7")
    moveit2_dual_arms.set_position_goal(pos_right.pose.position, frame_id="right_wam_base", target_link="right_wam7")
    moveit2_dual_arms.set_orientation_goal(pos_right.pose.orientation, frame_id="right_wam_base", target_link="right_wam7")

    plan = moveit2_dual_arms.plan()
    moveit2_dual_arms.execute(plan)

    # rate = node.create_rate(5)

    # while moveit2_dual_arms.query_state() != MoveIt2State.EXECUTING or moveit2_dual_arms.query_state() != MoveIt2State.REQUESTING or moveit2_dual_arms.query_state() != MoveIt2State.IDLE:
    #     rate.sleep()
    # future_ = moveit2_dual_arms.get_execution_future()

    # while not future_.done():
    #     rate.sleep()

    # node.get_logger().info("status: " + str(future_.result().status))

    moveit2_dual_arms.clear_goal_constraints()
    moveit2_dual_arms.clear_path_constraints()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
