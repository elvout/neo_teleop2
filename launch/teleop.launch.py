# Neobotix GmbH

import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("neo_teleop2"), "launch", "default.yaml"
    )
    ur_teleop_config = os.path.join(
        get_package_share_directory("neo_teleop2"), "launch", "ur_teleop.yaml"
    )

    # urdf_path = os.path.join(
    #     get_package_share_directory("neo_mpo_700-2"),
    #     "robot_model",
    #     "mmo_700.urdf.xacro",
    # )
    srdf_path = os.path.join(
        get_package_share_directory("neo_ur_moveit_config"),
        "srdf",
        "mpo_700.srdf.xacro",
    )

    # TODO(elvout): properly parameterize, see neo_ur_moveit.launch.py
    moveit_config = (
        MoveItConfigsBuilder(robot_name="mpo_700", package_name="neo_ur_moveit_config")
        .robot_description_semantic(
            file_path=srdf_path,
            mappings={
                "prefix": "ur10e",
                "gripper_type": "vg10",
            },
        )
        # TODO(elvout): For some reason this is unable to parse the urdf.
        # moveit will obtain the urdf from /robot_description with a warning.
        # .robot_description(
        #     file_path=urdf_path,
        #     mappings={
        #         "arm_type": "ur10e",
        #         "disable_scanners": True,
        #         "gripper_type": "vg10",
        #         "use_ur_dc": True,
        #         "use_gz": False,
        #         "force_abs_paths": False,
        #         "use_mock_hardware": False,
        #         "mock_sensor_commands": False,
        #     },
        # )
        .to_moveit_configs()
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="neo_teleop2",
                executable="neo_teleop2_node",
                output="screen",
                name="neo_teleop2_node",
                parameters=[config],
            ),
            launch_ros.actions.Node(
                package="neo_teleop2",
                executable="ur_teleop_node",
                output="screen",
                name="ur_teleop_node",
                parameters=[
                    ur_teleop_config,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            ),
            launch_ros.actions.Node(
                package="joy",
                executable="joy_node",
                output="screen",
                name="joy_node",
                parameters=[{"dev": "/dev/input/js0"}, {"deadzone": 0.12}],
            ),
        ]
    )
