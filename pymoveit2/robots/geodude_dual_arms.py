from typing import List

MOVE_GROUP_ARM: str = "dual_arms"

def joint_names() -> List[str]:
    return [
        "left_j1",
        "left_j2",
        "left_j3",
        "left_j4",
        "left_j5",
        "left_j6",
        "left_j7",
        "right_j1",
        "right_j2",
        "right_j3",
        "right_j4",
        "right_j5",
        "right_j6",
        "right_j7",
    ]

def base_link_name(prefix: str = "left_") -> str:
    return prefix + "wam1"


def end_effector_name(prefix: str = "left_") -> str:
    return prefix + "hand"


def gripper_joint_names() -> List[str]:
    return [
        "left_j01",
        "left_j11",
        "left_j21",
        "left_j00",
        "right_j01",
        "right_j11",
        "right_j21",
        "right_j00",
    ]
    