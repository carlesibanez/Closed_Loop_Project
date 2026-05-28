import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml_file(package_name, file_path):
    """Load and return the contents of a YAML file."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Failed to load {absolute_file_path}: {e}")
        return {}


def generate_launch_description():
    # Build the moveit configs to get the structure
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur", package_name="closed_loop_dalsa_description"
        )
        .robot_description(file_path="urdf/lab_setup2.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(file_path="config/motion_planning_python_cpp.yaml")
        .to_moveit_configs()
    )

    # Directly load and parse the YAML configuration files
    ompl_planning_config = load_yaml_file("closed_loop_dalsa_description", "config/ompl_planning.yaml")
    moveit_cpp_config = load_yaml_file("closed_loop_dalsa_description", "config/motion_planning_python_cpp.yaml")
    pilz_config = load_yaml_file("closed_loop_dalsa_description", "config/pilz_cartesian_limits.yaml")
    
    position_a_arg = DeclareLaunchArgument(
        "pos_a",
        default_value="[0.35, 0.0, 0.15, 0.0, 1.0, 0.0, 0.0]",
        description="Source location as [x, y, z, qw, qx, qy, qz] in base_link frame",
    )

    position_b_arg = DeclareLaunchArgument(
        "pos_b",
        default_value="[-0.1, -0.35, 0.15, 0.0, 1.0, 0.0, 0.0]",
        description="Position B as [x, y, z] in base_link frame",
    )
    
    # Launch our pick_place node
    pick_place_node = Node(
        package="closed_loop_dalsa_manipulation",
        executable="pick_place_task",
        name="mtc_node",
        output="screen",
        parameters=[
            # Pass the complete moveit config dict
            moveit_config.to_dict(),
            # Also explicitly pass the planning configs as backup
            {"ompl": ompl_planning_config},
            {"moveit_cpp": moveit_cpp_config},
            {"pilz_industrial_motion_planner": pilz_config},
            # Pass our custom parameters
            {"source_loc": LaunchConfiguration("pos_a")},
            {"dest_loc": LaunchConfiguration("pos_b")},
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        position_a_arg,
        position_b_arg,
        pick_place_node,
    ])