import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
import yaml

def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_simulation')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie.urdf')).read_text()
    supervisor_description = pathlib.Path(os.path.join(package_dir, 'resource', 'supervisor.urdf')).read_text()

    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies_yml = yaml.safe_load(ymlfile)

    robot_data = crazyflies_yml['robots']

    # Create easy lookup tables for uri, name and types
    names = []
    for crazyflie in robot_data:
        if robot_data[crazyflie]["enabled"]:
            names.append(crazyflie)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'crazyflie_world.wbt')
    )

    supervisor = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'Ros2Supervisor'},
        respawn=True,
        parameters=[
            {'robot_description': supervisor_description},
        ]
    )
    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(supervisor)

    for name in names:
        crazyflie_driver = Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': name},
            parameters=[
                {'robot_description': robot_description}
            ]
        )
        ld.add_action(crazyflie_driver)
        ld.add_action(launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ))




    return ld