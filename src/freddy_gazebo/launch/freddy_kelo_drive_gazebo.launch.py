import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def select_controller(context, *args, **kwargs):
    declared_base_controller = LaunchConfiguration('base_controller').perform(context)

    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'base_position_controller'],
        output='screen'
    )

    load_joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'base_velocity_controller'],
        output='screen'
    )

    load_joint_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'base_effort_controller'],
        output='screen'
    )

    base_controller_map = {
        "position": load_joint_position_controller,
        "velocity": load_joint_velocity_controller,
        "effort": load_joint_effort_controller,
    }
    if declared_base_controller not in base_controller_map:
        raise ValueError(f"Unsupported base_controller_type: {declared_base_controller}")
    else: 
        return [base_controller_map[declared_base_controller]]
    

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    declare_base_controller_type = DeclareLaunchArgument(
        'base_controller',
        default_value='velocity',
        description='Specify controller for base (position, velocity, effort) (default: velocity)'
    )

    # Package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_freddy_gazebo = get_package_share_directory('freddy_gazebo')
    pkg_freddy_description = get_package_share_directory('freddy_description')

    # Resource paths
    resource_paths = [
        pkg_freddy_description,
        os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes'),
        os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes', 'sensors'),
        os.path.join(pkg_freddy_description, 'freddy_torso_description', 'meshes'),
    ]
    resource_paths_str = os.pathsep.join(resource_paths)
    set_env_vars_resources = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths_str,
    )


    # World and robot files
    world_file = os.path.join(pkg_freddy_gazebo, 'worlds', 'my_world.sdf')
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('freddy_description'),
                 'robots', 'freddy_gz.urdf.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    bridge_yaml = os.path.join(pkg_freddy_gazebo, 'config', 'bridge.yaml')

    # ROS bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen'
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'robot', '-allow_renaming', 'true'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        # Declare base_controller launch argument
        declare_base_controller_type,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [f' -r -v 4 {world_file}'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        # Launch controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[OpaqueFunction(function=select_controller)],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
