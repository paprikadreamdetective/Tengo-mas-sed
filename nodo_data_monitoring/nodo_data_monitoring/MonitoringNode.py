import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import requests

# Importar el paquete measure del nodo planta
from kalanchoe_interface.msg import Measure

class MonitoringNode(Node):

    def __init__(self, flask_url):
        super().__init__('nodo_monitor_sub')
        self.humidity_value = 0.0  
        self.temperature_value = 0.0  
        self.water_level_value = ""  
        self.robot_state_value = ""  
        self.flask_url = flask_url

        self.humidity_subscription = self.create_subscription(
            Measure,
            '/humedad',
            self.callback_humidity,
            10)
        self.temperature_subscription = self.create_subscription(
            Measure,
            '/temperatura',
            self.callback_temperature,
            10)
        self.water_level_subscription = self.create_subscription(
            String,
            '/niv_agua',
            self.callback_water_level,
            10)
        self.robot_state_subscription = self.create_subscription(
            String,
            '/xolobot_state',
            self.callback_robot_state,
            10)

    def callback_humidity(self, msg):
        self.get_logger().info('Nuevo valor de humedad recibido: %f' % msg.value)
        self.humidity_value = msg.value
        self.send_data_to_flask()

    def callback_temperature(self, msg):
        self.get_logger().info('Nuevo valor de temperatura recibido: %f' % msg.value)
        self.temperature_value = msg.value
        self.send_data_to_flask()

    def callback_water_level(self, msg):
        self.get_logger().info('Nuevo valor de nivel de agua recibido: %s' % msg.data)
        self.water_level_value = msg.data
        self.send_data_to_flask()

    def callback_robot_state(self, msg):
        self.get_logger().info('Nuevo estado del robot recibido: %s' % msg.data)
        self.robot_state_value = msg.data
        self.send_data_to_flask()
    
    def send_data_to_flask(self):
        data = {
            'humidity': self.humidity_value,
            'temperature': self.temperature_value,
            'water_level': self.water_level_value,
            'robot_state': self.robot_state_value
        }
        requests.post(self.flask_url + '/update_data', json=data)
        self.get_logger().info('Datos enviados al servidor Flask')

def main(args=None):
    rclpy.init(args=args)
    flask_url = 'http://localhost:5000'  # URL del servidor Flask
    nodo_monitor = MonitoringNode(flask_url)
    rclpy.spin(nodo_monitor)
    nodo_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

