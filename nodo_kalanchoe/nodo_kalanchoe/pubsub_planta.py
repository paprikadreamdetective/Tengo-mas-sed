import rclpy
import random
import sys

from rclpy.node import Node
from kalanchoe_interface.msg import Measure
from std_msgs.msg import Float64
from std_msgs.msg import String

period = 0.5

class PlantPubSub(Node):
    
    def __init__(self):
        super().__init__('Plant_PubSub')
        #Plublicadores
        self.publisher_humedad = self.create_publisher(Measure, '/humedad', 10)
        self.publisher_temp = self.create_publisher(Measure, '/temperatura', 10)
        timer_period = period
        self.time_h = 2.5 #diferente a time.t para evitar traslape 
        self.time_t = 1.0
        self.time = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #suscriptores
        self.subscription_h = self.create_subscription(String,'/planta_regada', self.listener_callback, 10)
        self.subscription_h  # prevent unused variable warning
        self.subscription_t = self.create_subscription(String,'/avanzar', self.listener_callback, 10)
        self.subscription_t # prevent unused variable warning

    #publica periodicamente el valor de ambos topicos
    def timer_callback(self):
    	
        #Humedad
        msgH = Measure()
        msgH.time = self.time
        msgH.value = self.funcion_dec(self.time_h)
        self.publisher_humedad.publish(msgH)
        print('valores humedad: ', msgH.time,", ", msgH.value, " , ", self.time_h)

        #Temperatura
        msgT = Measure()
        msgT.time = self.time
        msgT.value = self.funcion_dec(self.time_t)
        self.publisher_temp.publish(msgT)
        print('valores temperatura: ', msgT.time,", ", msgT.value, " , ", self.time_t)

        #se incrementa el tiempo "real" y simulado
        self.time_t += 0.001 #llega al 45% en un min
        self.time_h += 0.002 #llega al 45% en un min
        self.time += 0.5
        
    #funcion decreciente la usan ambos topicos
    def funcion_dec(self, t):
        value = (1/pow(t, 2))+random.uniform(-0.008,0.008)
        
        if(value > 1):
            value = 1
        elif(value < 0):
            value = 0
        
        return float(value)
    
    #La planta ha sido regada o asoleada se reinicia el tiempo de la funcion
    def listener_callback(self, msg):
        self.get_logger().info('Entrada: "%s"' % msg.data)
        if(msg.data == 'planta_regada'):
            self.time_h = 1.0
        elif(msg.data == 'planta_asoleada'):
            self.time_t = 1.0


def main(args=None):
    rclpy.init(args=args)

    Plant_PubSub = PlantPubSub()
    rclpy.spin(Plant_PubSub)

    Plant_PubSub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
