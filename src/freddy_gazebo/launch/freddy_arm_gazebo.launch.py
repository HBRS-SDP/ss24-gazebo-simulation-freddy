import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_freddy_gazebo = get_package_share_directory('freddy_gazebo')
    pkg_freddy_description = get_package_share_directory('freddy_description')
    pkg_kortex_description = get_package_share_directory('kortex_description')

    # Resource paths
    # resource_paths = [
    #     pkg_freddy_description,
    #     os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes'),
    #     os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes', 'sensors'),
    #     os.path.join(pkg_freddy_description, 'freddy_torso_description', 'meshes'),
    # ]
    # resource_paths_str = os.pathsep.join(resource_paths)
    # set_env_vars_resources = AppendEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value=resource_paths_str,
    # )

    # World and robot files
    world_file = os.path.join(pkg_freddy_gazebo, 'worlds', 'my_world.sdf')
    xacro_file = os.path.join(pkg_freddy_description,
                                    'robots',
                                    'freddy_arms_gz.urdf.xacro')
    bridge_yaml = os.path.join(pkg_freddy_gazebo, 'config', 'bridge.yaml')

    robot_description = xacro.process_file(xacro_file, \
                    mappings={'sim_gz': 'true'}).toxml()
    robot_description_params = {'robot_description': robot_description}

    kortex_controllers = PathJoinSubstitution([
        pkg_kortex_description,
        "arms/" + "gen3/" + "7dof/config",
        "ros2_controllers.yaml",
    ])

    # Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    # ROS2 Bridge to Gazebo Harmonic
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description_params]
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'freddy', '-topic', 'robot_description'],
        output='screen'
    )

    # Kinova Kortex additions below
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[kortex_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    return LaunchDescription([
        # set_env_vars_resources,
        gz_sim,
        control_node,
        robot_state_publisher_node,
        spawn_entity,
        ros_gz_bridge
    ])
