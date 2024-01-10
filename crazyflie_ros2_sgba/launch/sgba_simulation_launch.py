import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
#from webots_ros2_driver.utils import controller_url_prefix


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('crazyflie_ros2_sgba')
    robot_description1 = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie1.urdf')).read_text()
    robot_description2 = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie2.urdf')).read_text()
    beacon_description = pathlib.Path(os.path.join(package_dir, 'resource', 'beacon_drone.urdf')).read_text()

    crazyflie_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'Crazyflie1'},
        parameters=[
            {'robot_description': robot_description1},
        ]
    )
    crazyflie_driver2 = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'Crazyflie2'},
        parameters=[
            {'robot_description': robot_description2},
        ]
    )
    beacon_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'beaconDrone'},
        parameters=[
            {'robot_description': beacon_description},
        ]
    )


    return [
        crazyflie_driver, crazyflie_driver2, beacon_driver
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_ros2_sgba')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world])
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='crazyflie_apartment2.wbt',
            description='Choose one of the world files from `/webots_ros2_mavic/worlds` directory'
        ),
        webots,
        ros2_supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.UnregisterEventHandler(
                        event_handler=reset_handler.event_handler
                    ),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),

        # Add the reset event handler
        reset_handler
    ] + get_ros2_nodes())
