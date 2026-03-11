import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_desc = get_package_share_directory('closed_loop_dalsa_description')
    
    # 1. GZ Resource Path
    model_path = os.path.dirname(pkg_desc)
    # Ensure robotiq meshes are also found
    pkg_robotiq = get_package_share_directory('robotiq_hande_description')
    robotiq_path = os.path.dirname(pkg_robotiq)
    
    resource_paths = [model_path, robotiq_path]
    os.environ['GZ_SIM_RESOURCE_PATH'] = ":".join(resource_paths)

    # 2. Xacro Processing
    xacro_file = os.path.join(pkg_desc, 'urdf', 'test_gripper.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 3. Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {os.path.join(pkg_desc, "world", "bio_lab.world")}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'test_gripper', '-z', '0.5'],
        output='screen',
    )

    # Bridge for Image and TF (if needed)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/wrist_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # If you have a saved rviz config for the camera, add it here:
        # arguments=['-d', os.path.join(pkg_desc, 'rviz', 'camera_test.rviz')]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        rviz_node
    ])