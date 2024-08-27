

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node





ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
]


def generate_launch_description():

    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory(
        'turtlebot4_ignition_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_irobot_create_ignition_plugins = get_package_share_directory(
        'irobot_create_ignition_plugins')
    # pkg_ros_ign_gazebo = get_package_share_directory(
    #     'ros_ign_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set ignition resource path
    # ign_resource_path = SetEnvironmentVariable(
    #     name='IGN_GAZEBO_RESOURCE_PATH',
    #     value=[
    #         os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
    #         os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
    #         str(Path(pkg_turtlebot4_description).parent.resolve()), ':' +
    #         str(Path(pkg_irobot_create_description).parent.resolve())])

    # ign_gui_plugin_path = SetEnvironmentVariable(
    #     name='IGN_GUI_PLUGIN_PATH',
    #     value=[
    #         os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'), ':' +
    #         os.path.join(pkg_irobot_create_ignition_plugins, 'lib')])

    # Paths
    # ign_gazebo_launch = PathJoinSubstitution(
    #     [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    gz_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Ignition gazebo
    # ignition_gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([ign_gazebo_launch]),
    #     launch_arguments=[
    #         ('ign_args', [LaunchConfiguration('world'),
    #                       '.sdf',
    #                       ' -v 4',
    #                       ' --gui-config ',
    #                       PathJoinSubstitution(
    #                         [pkg_turtlebot4_ignition_bringup,
    #                          'gui',
    #                          LaunchConfiguration('model'),
    #                          'gui.config'])])
    #     ]
    # )

      harmonic_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [
                    # PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds',
                    #                               LaunchConfiguration('world_file')])
                            ],
                'on_exit_shutdown': 'True'
            }.items(),
        )



    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(ign_resource_path)
    # ld.add_action(ign_gui_plugin_path)
    ld.add_action(harmonic_gazebo)
    ld.add_action(clock_bridge)
    return ld



#####################################
#####################################
#####################################
#####################################




def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spaceros_gz_sim = get_package_share_directory('spaceros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_spaceros_gz_sim, 'models'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='moon',
            choices=['moon', 'mars', 'enceladus'],
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds',
                                                  LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[],
            remappings=[],
            output='screen'
        ),
    ])