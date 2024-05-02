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
from pymoveit2.robots import geodude_left, geodude_right


def main():
    rclpy.init()

    node = Node("trial")

    callback_group = ReentrantCallbackGroup()

    moveit2_left = MoveIt2(
        node=node,
        joint_names=geodude_left.joint_names(),
        base_link_name=geodude_left.base_link_name(),
        end_effector_name=geodude_left.end_effector_name(),
        group_name=geodude_left.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2_right = MoveIt2(
        node=node,
        joint_names=geodude_right.joint_names(),
        base_link_name=geodude_right.base_link_name(),
        end_effector_name=geodude_right.end_effector_name(),
        group_name=geodude_right.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    moveit2_left.max_velocity = 0.5
    moveit2_left.max_acceleration = 0.5
    moveit2_right.max_velocity = 0.5
    moveit2_right.max_acceleration = 0.5

    left_plan = moveit2_left.plan(joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    right_plan = moveit2_right.plan(joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    moveit2_left.execute(joint_trajectory=left_plan)
    moveit2_right.execute(joint_trajectory=right_plan)

    rate = node.create_rate(10)

    while moveit2_left.query_state() != MoveIt2State.EXECUTING:
        rate.sleep()
    future_left = moveit2_left.get_execution_future()

    while moveit2_right.query_state() != MoveIt2State.EXECUTING:
        rate.sleep()
    future_right = moveit2_right.get_execution_future()

    while not future_left.done() or not future_right.done():
        rate.sleep()

    node.get_logger().info("Left arm result status: " + str(future_left.result().status))
    node.get_logger().info("Right arm result status: " + str(future_right.result().status))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
