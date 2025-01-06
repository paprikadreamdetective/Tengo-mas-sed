# Este es el nodo que controla la estación de iluminación.

from math import sqrt
from enum import Enum
import rclpy
import threading
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String

from station_interface.srv import Reload

# Importar la clase del tipo de mensajes para Xolobot
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


#Nuestras clases se deben derivar de la clase Node de ROS2
class SunController(Node):

    def __init__(self):
        super().__init__('sun_controller')

        # Para imprimir mensajes solamente cierto número de iteraciones
        self.i = 0

        self.srvReload = self.create_service(Reload, 'recharge_light', self.lightning_service) 

        # Publicador para el tópico /planta_regada
        self.pub_planta_servida = self.create_publisher(String, '/planta_regada', 10)

        # Declaramos que vamos a publicar el mensaje para que ya pueda avanzar Xolobot.
        self.pubAvanzar = self.create_publisher(String, '/avanzar', 10)

        # Declaramos que nos suscribimos a la información de posición y orientación del robot
        self.susOdome = self.create_subscription(Odometry, '/model/arlo_xolobot/odometry', self.updatePosition, 1)

        # Declaramos que nos suscribimos a la información de posición y orientación del robot
        self.subsSol   = self.create_subscription(Odometry, '/light_station/odom', self.updatePosSol, 2)

        # Posición actual del robot, se actualizará con la suscripción
        self.xoloPose = Pose()

        # Posición de la zona de RIEGO.
        # Importante: se puso la posición en claro, si se cambia en 
        #             xolobot_world_simple.sdf se debe cambiar acá y viceversa.
        # TODO: Pedir esta posición en tiempo de ejecución con el tópico 
        self.sunPos = Point(x=-4.0, y=-4.0)

        # Tolerancia (en metros) para determinar que xolobot llegó
        # a la zona indicada (agua o sol). 
        self.goalThreshold = 0.25

        # Variable booleana para indicar que ya llegamos a la zona.
        self.goalReached = False

        self.secondsEnergy = 0.1 # 1 segundo por 100 mililitros. 


    def lightning_service(self, request, response):
        print("Lighting the plant 🌞🌞🪴...")

        # Simula que la estación de iluminación está dando el servicio.
        time.sleep(request.load * self.secondsEnergy)

        # Cuando se cumple el tiempo, publicar al robot /avanzar para que pueda irse.
        print("Recarga lista 🔋🔋")

        # Publicar mensaje en el tópico /planta_regada
        planta_servida_msg = String()
        planta_servida_msg.data = "planta_asoleada"
        self.pub_planta_servida.publish(planta_servida_msg)

        response.success = True
        return response
 

    def monitoringSunStation(self):

        while rclpy.ok():
            ## El ciclo de abajo no se ejecutará a la máxima velocidad
            ## del CPU sino a la taza (rate) que determinemos dado en Hertz (ciclos por segundo).
            ## De esta manera, le diremos qué hacer al robot cada 0.5 segundos por ejemplo.

            thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
            thread.start() 

            ## Decirle qué hacer a xolobot 5 veces por segundo.
            rate = self.create_rate(5)  ## 5Hz: es decir, 5 veces por segundo

            ## Este ciclo se ejecutará infinitamente (mientras no haya ERRO en ROS)
            self.goalReached = False    
            while rclpy.ok() and not self.goalReached:
                self.detectRobot()
            
                ## Duerme lo necesario para que esta iteración la frecuencia dada arriba (Hz)
                rate.sleep()
            ### FIN del ciclo que detecta al Robot 
                
            if self.goalReached:
                print("\nXolobot en posición correcta. Comenzar a iluminar (station sleeping)...")
                # Iluminar a la planta por N segundos.
                time.sleep(10)

                # Cuando se cumple el tiempo, publicar al robot /avanzar para que pueda irse.
                print("Recarga lista. Avisar a Xolobot que ya se puede ir.")
                msg = String(data="Planta asoleada")
                self.pubAvanzar.publish(msg)

                # Esperar un tiempo para esperar que se vaya el robot y no detectarlo otra vez.
                print("Dar tiempo para que ya se pueda ir el robot.")
                time.sleep(10)
        ### FIN del ciclo infinito que siempre está controlando la estación de iluminación 
            


    ### FUNCIÓN IMPORTANTE ###
    # Esta función se invoca automáticamente cada que llega un mensaje 
    # del tópico /model/arlo_xolobot/odometry
    def updatePosition(self, odom):
        self.xoloPose = odom.pose.pose ## Obtiene campos position y pose (dirección)
        if self.i % 2 == 0:
            self.get_logger().info('Robot position: x=%f, y=%f' % (odom.pose.pose.position.x, odom.pose.pose.position.y))

        self.i = self.i+1

    def updatePosSol(self, odomSol):
        self.sunPos = odomSol.pose.pose.position

    def detectRobot(self):
        # Calcular distancia entre zona de riego y el robot
        distToGo = self.dist(self.xoloPose.position, self.sunPos)
        print("La distancia a la meta es %f" % distToGo)
    
        if distToGo <= self.goalThreshold:
            self.goalReached = True


    def dist(self, p1, p2):
        sum = (p1.x - p2.x)**2 + (p1.y - p2.y)**2
        return sqrt(sum)
    


def main(args=None):
    rclpy.init(args=args)

    sunController = SunController()
    #sunController.monitoringSunStation()

    rclpy.spin(sunController)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sunController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
