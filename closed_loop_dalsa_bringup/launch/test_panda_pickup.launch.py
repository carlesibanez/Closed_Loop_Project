"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="closed_loop_dalsa_description"
        )
        .robot_description(file_path="urdf/lab_setup2.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(
            file_path="config/motion_planning_python_cpp.yaml"
        )
        .to_moveit_configs()
    )

    example_file = DeclareLaunchArgument(
        "example_file",
        default_value="test_panda.py",
        description="Python API tutorial file name",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="closed_loop_dalsa_bringup",
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True}],
    )
    
    # moveit_py_node = Node(
    #         package="closed_loop_dalsa_bringup",
    #         executable="test_panda.py",
    #         name="moveit_py",
    #         output="both",
    #         parameters=[
    #             moveit_config.to_dict(),
    #             # moveit_cpp_params,
    #             {"use_sim_time": True}
    #         ],
    #     )

    

    # rviz_config_file = os.path.join(
    #     get_package_share_directory("moveit2_tutorials"),
    #     "config",
    #     "motion_planning_python_api_tutorial.rviz",
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #     ],
    # )

    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    # )

    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="log",
    #     parameters=[moveit_config.robot_description],
    # )

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("moveit_resources_panda_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="log",
    # )

    # load_controllers = []
    # for controller in [
    #     "panda_arm_controller",
    #     "panda_hand_controller",
    #     "joint_state_broadcaster",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner {}".format(controller)],
    #             shell=True,
    #             output="log",
    #         )
    #     ]

    return LaunchDescription(
        [
            example_file,
            moveit_py_node,
            # robot_state_publisher,
            # ros2_control_node,
            # rviz_node,
            # static_tf,
        ]
        # + load_controllers
    )
