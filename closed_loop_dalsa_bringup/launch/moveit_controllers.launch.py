"""
A launch file for running MoveIt with Gazebo simulation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  
        return None

def generate_launch_description():

    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="closed_loop_dalsa_description"
        )
        .robot_description(file_path="urdf/lab_setup2.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .moveit_cpp(
            file_path="config/motion_planning_python_cpp.yaml"
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )


    # example_file = DeclareLaunchArgument(
    #     "example_file",
    #     default_value="motion_planning_python_api_tutorial.py",
    #     description="Python API tutorial file name",
    # )

    # moveit_py_node = Node(
    #     name="moveit_py",
    #     package="moveit2_tutorials",
    #     executable=LaunchConfiguration("example_file"),
    #     output="both",
    #     parameters=[moveit_config.to_dict()],
    # )

    rviz_config_file = os.path.join(
        get_package_share_directory("closed_loop_dalsa_description"),
        "rviz",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            # moveit_config.planning_pipelines,
            # moveit_config.joint_limits,
        ],
    )

    # NOTE: robot_state_publisher is already launched by ur_sim_control.launch.py
    # It listens to /joint_states and publishes the TF tree based on actual Gazebo state
    # DO NOT launch another one here to avoid conflicts
    
    # NOTE: static_tf for world->tool0 is not needed since robot_state_publisher handles all TF
    # from the URDF based on joint states. Removing this to prevent TF conflicts.

    moveit_controllers_dict = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 10.0,
            "allowed_goal_duration_margin": 2.0,
            "allowed_start_tolerance": 0.05,
        },
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["joint_trajectory_controller", "gripper_controller"],
            "joint_trajectory_controller": {
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
                "default": False,
                "joints": ["finger_joint"],
            },
        },
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers_dict,
            {"use_sim_time": use_sim_time,
            "capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    servo_yaml = load_yaml("closed_loop_dalsa_description", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}


    servo_node = Node(
        package="moveit_servo",
        # condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            move_group_node,
            rviz_node,
            servo_node,
        ]
    )
