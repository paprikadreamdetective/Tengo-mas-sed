
from math import sqrt
from enum import Enum
import rclpy
import threading
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from random import random
from sensor_msgs.msg import Joy


# Importar la clase del tipo de mensajes para Xolobot
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from station_interface.srv import Reload

# Constantes para indicar en qué estados puede estar
# el robot: wandering (vagar), go2goal (ir a objetivo), holdon (esperar), refill (repostando agua/luz)
Estado = Enum('Estado', 'wandering go2Sun go2Water holdon rechargin rechargeWater rechargeLight')

# Class that implements the driver that controls Xolobot. 
# It receives environment stimuli and produces the values for the actuators (robot velocities)
class XolobotDriver(Node):

    def __init__(self):
        self.manejaUsuario = False

        super().__init__('xolobot_driver')

        # Contador para imprimir mensajes solamente cierto número de iteraciones.
        self.i = 0

        #self.susJoy = self.create_subscription(Joy, '/joy', self.checkHand, 1)

        ###############
        ##  Stubs, fuentes y sumideros para invocar servicios remotos 
        ##  y publicar/consumir mensajes.
        ###############

        # Stub del cliente para invocar el servicio de la estación de iluminación.
        self.clientLight = self.create_client(Reload, 'recharge_light')      
        while not self.clientLight.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Estación de iluminación no responde, esperando...')

        # Stub del cliente para invocar el servicio de la estación de riego.
        self.clientWater = self.create_client(Reload, 'recharge_water')       
        while not self.clientWater.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Estación de riego no responde, esperando...')

        # Fuente para publicar comandos de velocidad a Xolobot.
        self.pubVelocities = self.create_publisher(Twist, '/model/arlo_xolobot/cmd_vel', 10)

        # Fuente para publicar el estado de Xolobot
        self.pubState = self.create_publisher(String, '/xolobot_state', 10)

        # Suscripción a la información de posición y orientación del robot.
        self.subsOdom = self.create_subscription(Odometry, '/model/arlo_xolobot/gz_odometry', self.updatePosition, 10)

        # Suscripción a la información de posición de la estación de riego.
        self.subsRiego = self.create_subscription(Odometry, '/watering_station/odom', self.updatePosRiego, 2)

        # Suscripción a la información de posición de la estación de iluminación.
        self.subsSol   = self.create_subscription(Odometry, '/light_station/odom', self.updatePosSol, 2)

        # Suscripción al tópico /riego que le ordena al robot ir a la estación de riego.
        self.subsRiego = self.create_subscription(String, '/riego', self.checkWatering, 10)

        # Suscripción al tópico /sol que le ordena al robot ir a la estación de iluminación.
        self.subsSun = self.create_subscription(String, '/sol', self.checkSun, 10)

        # Suscripción al tópico /avanzar que le ordena al robot volver a moverse.
        self.subsAvanzar = self.create_subscription(String, '/avanzar', self.checkAvanzar, 10)


        #########
        ### Atributos del robot
        ########

        # Estado del robot, que condiciona qué hacer.
        self.estado = Estado.wandering
        self.pubState.publish(String(data="Persiguiendo mariposas 🦋🦋"))

        # Posición actual del robot, se actualizará con la suscripción a /odom
        self.xoloPose = Pose()


        # Posición de las zonas de RIEGO y de SOL.
        # Importante: ahora esta posición se actualiza con una publicación que se recibe
        # continuamanete. 
        # PERO como la posición NO cambia, lo mejor sería obtenerla con un servicio remoto una vez al inicio.
        self.wateringPos = Point(x=4.0, y=4.0)
        self.sunPos = Point(x=-4.0, y=-4.0)

        # Tolerancia (en metros) para considerar que Xolobot llegó
        # a la estación indicada (agua o sol). 
        self.goalThreshold = 0.1

        # Variable booleana para indicar que ya llegamos a la estación destino.
        self.goalReached = False

        # Tolerancia (en radianes) para ajustar la dirección de Xolobot hacia su destino.
        self.deviationThreshold = 0.01

        # Velocidad base de xolobot
        self.vLinearBase = 0.4  # Si va muy rápido no tendrá tiempo de corregir dirección.
        
        # Ángulo de corrección cuando Xolobot va a una estación de servicio.
        self.vAngularBase = 1.0

        # Cuando Xolobot está deambulando, esto sirve para saber si iba derecho o no.
        self.wanderingStraight = True

        #self.fileVelocities = ""


    ## Este es el método principal donde le indicamos a Xolobot cómo moverse 
    ## según su estado: wandering, go2goal, holdon, etc.
    ## En cada iteración tomamos información del entorno de Xolobot y 
    ## calculamos su velocidad lineal (avanzar) y angular (giro) para moverse un paso.
    ## 
    def drive(self):
        ## El ciclo de abajo no se ejecutará a la máxima velocidad del CPU 
        ## sino a la tasa (rate) que determinemos dada en Hertz (ciclos por segundo).
        ## Es decir, le diremos qué hacer al robot N veces por segundos.

        #self.fileVelocities = open("vels.txt", "w")

        ## Por default, ROS2 tiene un solo hilo para que el nodo reciba o emita publicaciones, y
        ## ejecute el código de sus métodos (como este). Si ponemos un sleep, ya no se recibirían
        ## publicaciones porque el hilo estaría ocupado. Para que se sigan haciendo la tareas
        ## de fondo, debemos crear un hilo que ejecute la función spin (la que ejecuta este método).
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start() 

        ## Decirle qué hacer a xolobot N veces por segundo.
        rate = self.create_rate(20)  ## Hz: es decir, N veces por segundo

        ## Este ciclo se ejecutará infinitamente (mientras no haya ERROR en ROS)    
        while rclpy.ok():
            if self.estado == Estado.wandering:
                self.wandering()
            elif self.estado == Estado.go2Water:
                self.gotoGoal(self.wateringPos)
            elif self.estado == Estado.go2Sun:
                self.gotoGoal(self.sunPos)
            elif self.estado == Estado.rechargeWater:
                self.recharging()
            elif self.estado == Estado.rechargeLight:
                self.recharging()
            elif self.estado == Estado.holdon:
                self.holdon()

            ## Duerme lo necesario para que esta iteración la frecuencia dada arriba (Hz)
            rate.sleep()
        ## FIN del while
            
        thread.join()
 
    # Método para que Xolobot tome una dirección para deambular.
    # Con probabilidad pequeña gira, y con probabilidad mayor sigue derecho.
    def wandering(self):
        print("Buscando mariposas 🦋🦋🦋...")

        if self.manejaUsuario == True:
            print("\tEl usuario al mando 🎮🎮")
        else:
            ## Con baja probabilidad cambiamos de dirección.
            if self.wanderingStraight:
                if random() < 0.05:
                    print("Girar...")
                    self.wanderingStraight = False
                    if random() < 0.5:
                        self.turnRight()
                    else:
                        self.turnLeft()
            else:
                if random() < 0.2:
                    print("Ir derecho")
                    self.wanderingStraight = True
                    self.goStraight()

    # Método que ordena a Xolobot frenarse completamente.
    def holdon(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        self.pubVelocities.publish(vel_msg)


    def recharging(self):
        self.holdon()

        if self.estado == Estado.rechargeLight:
            self.rechargeLight()
        elif self.estado == Estado.rechargeWater:
            self.rechargeWater()

        # Después de recargar debe pasar otra vez a wandering.
        self.estado = Estado.wandering
        self.pubState.publish(String(data="Persiguiendo mariposas 🦋🦋"))
        print("Listo, vámonos 🪴🎊🎉")


    def rechargeLight(self):
        print("Voy a recargar LUZ 🌞🌞⛽️⛽️...")
        self.pubState.publish(String(data="Recargando luz"))
        request = Reload.Request()
        request.load = 200.0

        ## Esta llamada tomará algunos segundos porque está recargando luz.
        self.clientLight.call(request)


    def rechargeWater(self):
        print("Voy a recargar AGUA 💦💦⛽️⛽️...")
        self.pubState.publish(String(data="Recargando agua"))
        request = Reload.Request()
        request.load = 200.0

        ## Esta llamada tomará algunos segundos porque está recargando agua.
        self.clientWater.call(request)


    def gotoGoal(self, goalPosition):
        # Calcular distancia entre zona de riego y el robot
        distToGo = self.dist(self.xoloPose.position, goalPosition)
        #print("La distancia a la meta es %f" % distToGo)
    
        if distToGo <= self.goalThreshold:
            self.goalReached = True
            if self.estado == Estado.go2Water:
                self.estado = Estado.rechargeWater
            elif self.estado == Estado.go2Sun:
                self.estado = Estado.rechargeLight
            #self.fileVelocities.close()

            print("Se ha llegado a la meta 🏁🏁🏁")
        else:
            # Corregir dirección hacia la zona de riego

            #1. Obtener la dirección actual del robot
            robotDir = self.getRobotDirection(self.xoloPose.orientation)

            #2. Obtener la dirección actual hacia donde queda la zona de riego
            goalDir = self.getGoalDirection(self.xoloPose.position, goalPosition)

            #3. Obtener el ángulo de desviación a la meta.
            deviationAngle = np.arctan2( np.sin(goalDir-robotDir), np.cos(goalDir-robotDir) )

            print("\nRobot direction: %f" % ((robotDir*180.0) / np.pi))
            print("Goal direction : %f" % ((goalDir*180.0) / np.pi))
            print("Goal deviation : %f" % ((deviationAngle*180.0) / np.pi))
        
            #4. Corregir si es necesario la dirección actual del robot.
            vel_msg = Twist()
            vel_msg.linear.x = self.vLinearBase * self.sigmoid(distToGo, shift=2.0)
            vel_msg.angular.z = 0.0
            magDeviation = abs(deviationAngle)
            if magDeviation > self.deviationThreshold:
                shift = self.vAngularBase * self.sigmoid(magDeviation, shift=2.0) 
                if deviationAngle > 0.0:  # Si cotrasentido de las manecillas
                    vel_msg.angular.z =  shift    # Girar a la izquierda
                else:                     # Sentido de las manecillas
                    vel_msg.angular.z = -1*shift  # Girar a la derecha

            if self.manejaUsuario == True:
                print("Usuario!! Tienes que llevar al robot a la ZONA")
            else:
                print("Lin: %f, Ang: %f" % (vel_msg.linear.x, vel_msg.angular.z))
                #5. Enviar la nueva orden de conducción a xolobot

                #self.fileVelocities.write("%f\t%f\t%f\t%f\t%f\t%f\n" % (self.xoloPose.position.x, self.xoloPose.position.y, distToGo, vel_msg.linear.x, magDeviation, vel_msg.angular.z))
                self.pubVelocities.publish(vel_msg)


    # Método que se invoca automáticamente cada que llega un mensaje 
    # del tópico /model/arlo_xolobot/odometry
    def updatePosition(self, odom):
        self.xoloPose = odom.pose.pose ## Obtiene campos position y pose (dirección)
        if self.i % 60 == 0:
            #self.get_logger().info('Orientación: z=%f, w=%f' % (odom.pose.pose.orientation.z, odom.pose.pose.orientation.w))
            robotDir = self.getRobotDirection(self.xoloPose.orientation)
            print("\nRobot direction: %f" % ((robotDir*180.0) / np.pi))
            #print("Yaw robot: %f" % (robotDir))

        self.i = self.i+1

    # Método que se invoca automáticamente cada que llega un mensaje
    # del tópico /watering_station/odom
    def updatePosRiego(self, odomRiego):
        self.wateringPos = odomRiego.pose.pose.position

    # Método que se invoca automáticamente cada que llega un mensaje
    # del tópico /ligth_station/odom
    def updatePosSol(self, odomSol):
        self.sunPos = odomSol.pose.pose.position

    # Método que se invoca automáticamente cada que llega un mensaje
    # del tópico /riego
    def checkWatering(self, msgRiego):
        print("Me llegó el mensaje '%s' para que vaya a la estación de Riego..." % msgRiego.data)
        if self.estado == Estado.wandering:
            print("Voy a ir a la estación de riego 🚿")
            self.estado = Estado.go2Water
            self.pubState.publish(String(data="Buscando agua"))
        else:
            print("Ahora no puedo ir la estación de Riego, estoy en otra cosa ⛔️")
        
    # Método que se invoca automáticamente cada que llega un mensaje
    # del tópico /sol
    def checkSun(self, msgSol):
        print("Me llegó el mensaje '%s' para que vaya a la estación de SOL." % msgSol.data)
        if self.estado == Estado.wandering:
            print("Voy a ir a la estación de iluminación 🌞")
            self.estado = Estado.go2Sun
            self.pubState.publish(String(data="Buscando luz"))
        else:
            print("Ahora no puedo ir la estación de SOL, estoy en otra cosa ⛔️")

    # Esta función se invoca automáticamente cada que llega un mensaje 
    # del tópico /model/arlo_xolobot/odometry
    def checkAvanzar(self, msgAvanzar):
        print("Me llegó el mensaje '%s' para avanzar 🏎💨" % msgAvanzar.data)
        self.estado = Estado.wandering
        self.pubState.publish(String(data="Persiguiendo mariposas 🦋🦋"))

    def goStraight(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.5*self.vLinearBase
        vel_msg.angular.z = 0.0
        self.pubVelocities.publish(vel_msg)

    def turn(self, signo):
        vel_msg = Twist()
        vel_msg.linear.x = 0.25*self.vLinearBase
        vel_msg.angular.z = signo*self.vAngularBase
        self.pubVelocities.publish(vel_msg)

    def turnLeft(self):
        self.turn(1)
        
    def turnRight(self):
        self.turn(-1)

    def dist(self, p1, p2):
        sum = (p1.x - p2.x)**2 + (p1.y - p2.y)**2
        return sqrt(sum)
    
    def getRobotDirection(self, robotOrient):
        quatDir = Quaternion(x=robotOrient.x, y=robotOrient.y, z=robotOrient.z, w=robotOrient.w)
        [roll, pitch, yaw] = self.euler_from_quaternion(quatDir)

        # yaw es la dirección del robot de 0 a 360 grados, pero dado en radianes.
        return yaw
    
    # Método que toma la posición del robot y de la meta y regresa el ángulo
    # hacia donde está la meta.
    def getGoalDirection(self, robotPos, goalPos):
        deltaX = goalPos.x - robotPos.x
        deltaY = goalPos.y - robotPos.y

        # Angle in format (0, PI), (0,-PI)
        targetDirection = np.arctan(abs(deltaY/deltaX))

        if deltaX < 0 and deltaY > 0:    # 2o cuadrante -,+
            targetDirection = np.pi - targetDirection
        elif deltaX < 0 and deltaY < 0:  # 3er cuadrante -,-
            targetDirection = -np.pi + targetDirection
        elif deltaX > 0 and deltaY < 0:  # 4o cuadrante +,- 
            targetDirection = -targetDirection           

        return targetDirection
    

    ## Función tomada de:
    #  https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    ##
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def sigmoid(self, x, shift=0):
        return 1.0 / (1.0 + np.exp(-(x-shift)))


def main(args=None):
    rclpy.init(args=args)

    xolobot_drv = XolobotDriver()
    xolobot_drv.drive()

    #rclpy.spin(xolobot_drv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    xolobot_drv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
