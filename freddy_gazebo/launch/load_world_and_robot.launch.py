import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.actions import AppendEnvironmentVariable
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Get the package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_freddy_gazebo = get_package_share_directory('freddy_gazebo')
    pkg_freddy_description = get_package_share_directory('freddy_description')
    
    resource_paths = [
        pkg_freddy_description,
        os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes'),
        os.path.join(pkg_freddy_description, 'freddy_base_description', 'meshes', 'sensors'),
        os.path.join(pkg_freddy_description, 'freddy_torso_description', 'meshes'),
    ]
    # Join the paths with the appropriate separator
    resource_paths_str = os.pathsep.join(resource_paths)
    # Append the environment variable
    set_env_vars_resources = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths_str,
    )
    
    # Paths to the SDF world and robot URDF files
    world_file = os.path.join(pkg_freddy_gazebo, 'worlds', 'my_world.sdf')
    # robot_file = os.path.join(pkg_freddy_gazebo, 'urdf', 'freddy.urdf')
    xacro_file = os.path.join(pkg_freddy_description, 'robots', 'freddy_gz.urdf.xacro')
    # Convert xacro to urdf
    urdf_file = os.path.join(pkg_freddy_gazebo, 'urdf', 'freddy.urdf')
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution(
    #             [
    #                 FindExecutable(name="xacro")]), 
    #                 " ", 
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("freddy_description"),
    #                 "robots",
    #                 "freddy_gz.urdf.xacro"
    #             ]
    #         ), 
    #         " ",
    #         "sim_gz:=true",
    #     ]
    # )

    # Gazebo launch file
    gz_sim_srv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    gz_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-g -v4 '}.items()
    )


    # robot_description = {'robot_description': robot_description_content}
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[robot_description]
    # )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'freddy', 
                #    '-topic', 'robot_description'],
                   '-file', urdf_file],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg_freddy_gazebo, 'config', 'controller.yaml')],
        output='screen'
    )

    return LaunchDescription([
        set_env_vars_resources,
        gz_sim_srv,
        gz_client_cmd,
        # node_robot_state_publisher,
        spawn_entity,
        controller_manager,
    ])
