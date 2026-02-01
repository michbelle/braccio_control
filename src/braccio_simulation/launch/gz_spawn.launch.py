import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    urdf = os.path.join(get_package_share_directory(
        'braccio_simulation'), 'model','urdf', 'model_braccio.urdf')

    robot_desc = ParameterValue(Command(['xacro ', urdf]),
                                       value_type=str)
    
    
    
    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])
    
    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : ['empty.sdf'], # " -r "
            "headless": "true"
        }.items()
    )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "braccio",
            "-allow_renaming", "true",
            "-x", "0.00",
            "-y","0.00",
            "-z", "0.001",
            "-P","0.0",
            "-R","0.0",
            "-Y","0.0",
        ]
    )
    
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/braccio/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/braccio/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
    )
    
    rviz_config_dir = os.path.join(
            get_package_share_directory('braccio_simulation'),
            'rviz',
            'visu.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    # ld.add_action(gz_ros2_bridge)


    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_node)

    return ld