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

    # TODO(elvout): properly parameterize, see neo_ur_moveit.launch.py
    moveit_config = (
        MoveItConfigsBuilder(robot_name="mpo_700", package_name="neo_ur_moveit_config")
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("neo_ur_moveit_config"),
                "srdf",
                "mpo_700.srdf.xacro",
            ),
            mappings={
                "prefix": "ur10e",
                "gripper_type": "vg10",
                "disable_scanners": "true",
            },
        )
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("neo_mpo_700-2"),
                "robot_model",
                "mpo_700.urdf.xacro",
            ),
            mappings={
                "arm_type": "ur10e",
                "disable_scanners": "true",
                "use_imu": "false",
                "use_d435": "false",
                "use_ur_dc": "true",
                "use_gz": "false",
                "force_abs_paths": "false",
                "use_mock_hardware": "false",
                "mock_sensor_commands": "false",
                "gripper_type": "vg10",
            },
        )
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
                # TODO: using the "name" kwarg creates a duplicate node in the
                # graph, but only for this node. The duplicate node has no
                # associated process.
                # name="ur_teleop_node",
                parameters=[
                    ur_teleop_config,
                    moveit_config.robot_description,
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
