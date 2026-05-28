import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        return None

def generate_launch_description():
    description_pkg = "closed_loop_dalsa_description"
    
    # 1. Load the Robot Description (URDF/SRDF)
    # Your logs show this part IS working, so we keep using the builder for it
    moveit_config = (
        MoveItConfigsBuilder("bio_lab", package_name="closed_loop_dalsa_description")
        .robot_description(file_path="urdf/lab_setup.urdf.xacro", mappings={"ur_type": "ur5e"})
        .robot_description_semantic(file_path="config/bio_lab.srdf.xacro", mappings={"name": "ur5e"})
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(file_path=get_package_share_directory("closed_loop_dalsa_description")+ "/config/moveit_cpp.yaml")
        # This now works because the file exists!
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # 2. MANUALLY load the missing YAMLs to bypass "inference"
    ompl_config = load_yaml(description_pkg, "config/ompl_planning.yaml")
    kinematics_config = load_yaml(description_pkg, "config/kinematics.yaml")
    joint_limits_config = load_yaml(description_pkg, "config/joint_limits.yaml")

    # 3. Construct the exact dictionary MoveItPy expects
    # This structure is what the C++ backend (MoveItCpp) requires
    moveit_cpp_params = {
        "moveit_cpp": {
            "planning_pipelines": ["ompl"],
            "plan_request_params": {
                "planning_attempts": 10,
                "planning_time": 10.0,
                "max_velocity_scaling_factor": 0.2,
                "max_acceleration_scaling_factor": 0.2,
            },
        },
        "ompl": ompl_config,
        "robot_description_kinematics": kinematics_config,
        "joint_limits": joint_limits_config,
    }

    # 4. Start the Node
    return LaunchDescription([
        Node(
            package="closed_loop_dalsa_bringup",
            executable="test.py",
            name="moveit_py",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                moveit_cpp_params,
                {"use_sim_time": True}
            ],
        )
    ])