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

    moveit2_dual_arms.max_velocity = 0.5
    moveit2_dual_arms.max_acceleration = 0.5

    plan = moveit2_dual_arms.plan(joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    moveit2_dual_arms.execute(joint_trajectory=plan)

    # rate = node.create_rate(10)

    # while moveit2_dual_arms.query_state() != MoveIt2State.EXECUTING:
    #     rate.sleep()
    # future_ = moveit2_dual_arms.get_execution_future()

    # while not future_.done():
    #     rate.sleep()

    # node.get_logger().info("Left arm result status: " + str(future_.result().status))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
