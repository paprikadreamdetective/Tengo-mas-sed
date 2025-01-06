import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import random

class NodoMonitor(Node):

    def __init__(self):
        super().__init__('nodo_monitor_pub')

        # Crear temporizadores y publicadores para cada tipo de dato
        timer_period = 1  # segundos
        
        self.humidity_publisher = self.create_publisher(Float64, '/humedad', 10)
        self.timer_humidity = self.create_timer(timer_period, self.timer_callback_humidity)

        self.temperature_publisher = self.create_publisher(Float64, '/temperatura', 10)
        self.timer_temperature = self.create_timer(timer_period, self.timer_callback_temperature)

        self.mode_publisher = self.create_publisher(String, '/avanzar', 10)
        self.timer_mode = self.create_timer(timer_period, self.timer_callback_mode)

        self.water_level_publisher = self.create_publisher(String, '/niv_agua', 10)
        self.timer_water_level = self.create_timer(timer_period, self.timer_callback_water_level)

    def timer_callback_humidity(self):
        humidity = random.uniform(0, 100)
        msg = Float64()
        msg.data = humidity
        self.humidity_publisher.publish(msg)
        self.get_logger().info('Publicando valor de humedad: %f' % msg.data)

    def timer_callback_temperature(self):
        temperature = random.uniform(0, 50)
        msg = Float64()
        msg.data = temperature
        self.temperature_publisher.publish(msg)
        self.get_logger().info('Publicando valor de temperatura: %f' % msg.data)

    def timer_callback_mode(self):
        mode = random.randint(0, 1)  # Generar número entero aleatorio entre 0 y 1
        msg = String()
        msg.data = str(mode)
        self.mode_publisher.publish(msg)
        self.get_logger().info('Publicando valor de modo: %s' % msg.data)

    def timer_callback_water_level(self):
        water_level = random.randint(0, 100)
        msg = String()
        msg.data = str(water_level)
        self.water_level_publisher.publish(msg)
        self.get_logger().info('Publicando valor de nivel de agua: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    nodo_monitor = NodoMonitor()
    rclpy.spin(nodo_monitor)
    nodo_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
