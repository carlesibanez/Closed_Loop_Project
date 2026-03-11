import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Setup configurations
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 2. Point MoveIt to YOUR custom description and SRDF
    # This replaces the hardcoded UR logic
    pkg_description = get_package_share_directory('closed_loop_dalsa_description')

    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["scaled_joint_trajectory_controller", "gripper_controller"],
            "scaled_joint_trajectory_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            },
            "gripper_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": ["finger_joint"],
            },
        }
    }
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="bio_lab", package_name="closed_loop_dalsa_description")
        .robot_description(file_path="urdf/lab_setup.urdf.xacro", mappings={"ur_type": "ur5e"})
        .robot_description_semantic(Path("config") / "bio_lab.srdf.xacro", {"name": ur_type})
        # Load kinematics for BOTH the arm and the gripper
        .robot_description_kinematics(file_path=os.path.join(
            get_package_share_directory("ur_moveit_config"), "config", "kinematics.yaml"))
        .joint_limits(file_path=os.path.join(
            get_package_share_directory("closed_loop_dalsa_description"), "config", "joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 3. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 4. RViz Node
    rviz_config_file = os.path.join(
        get_package_share_directory('closed_loop_dalsa_description'),
        'rviz',
        'moveit.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        move_group_node,
        rviz_node,
    ])