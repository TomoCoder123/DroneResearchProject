import rclpy
from ament_index_python.packages import get_package_share_directory
import os
import yaml

class CrazyflieSpawner:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_spawner')

        # load crazyflies
        crazyflies_yaml = os.path.join(
            get_package_share_directory('crazyflie'),
            'config',
            'crazyflies.yaml')

        with open(crazyflies_yaml, 'r') as ymlfile:
            crazyflies_yml = yaml.safe_load(ymlfile)

        robot_data = crazyflies_yml['robots']

        # Create easy lookup tables for uri, name and types
        self.names = []
        self.location_dict = {}
        for crazyflie in robot_data:
            if robot_data[crazyflie]["enabled"]:
                self.location_dict[crazyflie]=robot_data[crazyflie]["initial_position"]
                self.names.append(crazyflie)
            
            self.node.get_logger().info(crazyflie)

        self.robot = webots_node.robot
        root_node = self.robot.getRoot()
        children_field = root_node.getField('children')
        for name in self.names:
            location = self.location_dict[name]
            children_field.importMFNodeFromString(-1, 'Crazyflie {  translation '+str(location[0])+' '+ str(location[1])+' '+str(location[2])+' name "'+ name +'"  controller "<extern>"}')

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        #self.node.get_logger().info('step...')

