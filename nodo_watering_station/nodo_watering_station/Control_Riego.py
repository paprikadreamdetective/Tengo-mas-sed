# Ejemplo de nodo para conducir el robot Xolobot.

from math import sqrt
from enum import Enum
from std_msgs.msg import String

import rclpy
import threading
import time
import numpy as np
from rclpy.node import Node

from station_interface.srv import Reload

# Importar la clase del tipo de mensajes para Xolobot
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

#Nuestras clases se deben derivar de la clase Node de ROS2
class XolobotController(Node):

    def __init__(self):
        super().__init__('water_controller')

        self.srvReload = self.create_service(Reload, 'recharge_water', self.watering_service) 

        # Declaramos que vamos a publicar el mensaje para que ya pueda avanzar Xolobot.
        self.pub_avanzar = self.create_publisher(String, '/avanzar', 10)
        
        # Publicador para el tópico /planta_regada
        self.pub_planta_servida = self.create_publisher(String, '/planta_regada', 10)
        
        # Publicador para el tópico /niv_agua
        self.pub_nivel_agua = self.create_publisher(String, '/niv_agua', 10)

        # Posición actual del robot, se actualizará con la suscripción
        self.xoloPose = Pose()

        # Posición de la zona de RIEGO.
        # Importante: se puso la posición en claro, si se cambia en 
        #             xolobot_world_simple.sdf se debe cambiar acá y viceversa.
        self.wateringPos = Point(x=4.0, y=4.0)
        
        # Tolerancia (en metros) para determinar que xolobot llegó
        # a la zona indicada (agua o sol). 
        self.goalThreshold = 0.25

        # Variable booleana para indicar que ya llegamos a la zona.
        self.goalReached = False

        self.aguatotal = 20  # Inicializar del agua 
        self.nivel_agua_msg = String()
        self.nivel_agua_msg.data = str(self.aguatotal)
        
        self.secondsEnergy = 0.1 # 1 segundo por 100 mililitros. 


    def watering_service(self, request, response):
        print("Watering the plant 💦💦🪴...")

        # Simula que la estación de iluminación está dando el servicio.
        time.sleep(request.load * self.secondsEnergy)

        # Cuando se cumple el tiempo, publicar al robot /avanzar para que pueda irse.
        print("Recarga lista 🔋🔋")

        # Publicar mensaje en el tópico /planta_regada
        planta_servida_msg = String()
        planta_servida_msg.data = "planta_regada"
        self.pub_planta_servida.publish(planta_servida_msg)

        response.success = True
        return response


 
    ### FUNCIÓN IMPORTANTE ###
    # Esta función se invoca automáticamente cada que llega un mensaje 
    # del tópico /model/arlo_xolobot/odometry
    def updatePosition(self, odom):
        self.xoloPose = odom.pose.pose ## Obtiene campos position y pose (dirección)
        self.get_logger().info('Actualizando orientación: z=%f, w=%f' % (odom.pose.pose.orientation.z, odom.pose.pose.orientation.w))

 
    def contarSegundosRiego(self, segundos):
        self.get_logger().info('Iniciando conteo de {} segundos de riego.'.format(segundos))
        time.sleep(segundos)
        self.get_logger().info('Finalizado conteo de {} segundos de riego.'.format(segundos))

    # Obtiene la distancia Euclideana entre dos puntos.
    # Recibe: dos puntos de tipo Punto()
    # Regresa: la distancia de tipo flotante.
    def dist(self, p1: Point, p2: Point):
        sum = (p1.x - p2.x)**2 + (p1.y - p2.y)**2
        return sqrt(sum)

def main(args=None):
    rclpy.init(args=args)

    waterController = XolobotController()

    rclpy.spin(waterController)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waterController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()