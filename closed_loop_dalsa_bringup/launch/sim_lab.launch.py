import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get paths using share directory (symlinked to your src)
    pkg_description = get_package_share_directory('closed_loop_dalsa_description')
    pkg_bringup = get_package_share_directory('closed_loop_dalsa_bringup')
    pkg_robotiq = get_package_share_directory('robotiq_hande_description')
    
    # Robot XACRO and world paths
    urdf_path = os.path.join(pkg_description, 'urdf', 'lab_setup.urdf.xacro')

    world_file_path = os.path.join(pkg_description, 'world', 'bio_lab.world')

    # 2. Gazebo Resource Path (Vital for finding your .stl meshes)
    # We point Gazebo to the 'share' directory so it can resolve package:// paths
    model_path = os.path.dirname(pkg_description)
    robotiq_path = os.path.dirname(pkg_robotiq)

    resource_paths = [model_path, robotiq_path]
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ":" + ":".join(resource_paths)
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = ":".join(resource_paths)

    my_moveit_launch = os.path.join(
        get_package_share_directory('closed_loop_dalsa_bringup'), 
        'launch', 
        'bio_lab_moveit.launch.py' 
    )

    # 3. Include the UR Simulation 
    # This launch file handles MoveGroup, Gazebo, and RViz
    ur_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_simulation_gz'), 'launch', 'ur_sim_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'description_file': urdf_path, 
            'world_file': world_file_path,
            'moveit_launch_file': my_moveit_launch,
            'launch_rviz': 'true'
        }.items()
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 4. Perception & Control Nodes
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # This name MUST match the name inside your ros2_controllers.yaml
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"]
    )


    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # '/gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            # '/gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        output='screen'
    )


    # Use the description package to find the plate model
    # plate_urdf_path = os.path.join(pkg_description, 'models', 'plate.urdf')
    # spawn_plate = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-file', plate_urdf_path,
    #         '-name', 'greiner_plate',
    #         '-x', '0.4', '-y', '0.0', '-z', '0.76', # Adjusted for your 0.75m table
    #     ],
    #     output='screen',
    # )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        ur_simulation,
        gripper_spawner,
        camera_bridge,
        # spawn_plate
    ])