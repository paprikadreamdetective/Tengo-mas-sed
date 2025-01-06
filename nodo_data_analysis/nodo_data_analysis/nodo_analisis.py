# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Autor: Larios Vega Erick Efrain
# Compilation: python3 nodo_analisis.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String
from playsound import playsound
from datetime import datetime

# Importar el paquete measure del nodo planta
from kalanchoe_interface.msg import Measure



class MinimalSubscriber(Node):

    def __init__(self):

        super().__init__('nodo_Analisis')

        self.N = 10      # Numero de datos necesarios para promediar
        self.Hmin = 0.6  # Humedad minima para la planta
        self.Emin = 0.5  # Energia minima para la planta
        self.counterTemperature = 0
        self.counterHumidity = 0
        self.audioNumber = 0
        self.numAudioFiles = 2

        self.humidity_data = []  # Lista para almacenar los datos de humedad
        self.temperature_data = []  # Lista para almacenar los datos de temperatura
        
        self.humidity_subscription = self.create_subscription(
            Measure,
            '/humedad',
            self.updateHumidity,
            10)
        
        self.temperature_subscription = self.create_subscription(
            Measure,
            '/temperatura',
            self.updateTemperature,
            10)
        
        self.humidity_subscription
        self.temperature_subscription

        self.riego_publisher = self.create_publisher(String, '/riego', 10)
        self.sol_publisher = self.create_publisher(String, '/sol', 10)


    def updateHumidity(self, msg):
        self.get_logger().info('Datos de humedad recibidos: "%f"' % msg.value)
        self.counterHumidity += 1
        if(self.counterHumidity >= 10):
            self.humidity_data.append(msg.value)  # Agregar el valor de humedad a la lista
        
        if len(self.humidity_data) == self.N: 
            self.dataAnalysis(self.humidity_data,1)


    def updateTemperature(self, msg):
        self.get_logger().info('Datos de temperatura recibidos: "%f"' % msg.value)
        self.counterTemperature += 1
        if(self.counterTemperature >= 10):
            self.temperature_data.append(msg.value)  # Agregar el valor de temperatura a la lista
        
        if len(self.temperature_data) == self.N: 
            self.dataAnalysis(self.temperature_data,2)
    

    def dataAnalysis(self, data, modo):
        average = sum(data) / len(data)
        if modo==1:
            if average < self.Hmin:
                self.riego_publisher.publish(String(data=str(average)))  # Publicar el promedio en el tópico 'riego'
                self.get_logger().info('\nPromedio de humedad enviado a riego: "%f"\n' % average)
                if self.audioNumber == 0:
                    playsound("tengo_mucha_sed_m11.mp3")  # Reproducir el archivo de audio
                elif self.audioNumber == 1:
                    playsound("tengo_sed_m11.mp3")

                self.audioNumber = (self.audioNumber+1) % self.numAudioFiles
            self.humidity_data.clear()  # Limpiar la lista de datos de humedad
        if modo==2:
            if average < self.Emin:
                self.sol_publisher.publish(String(data=str(average)))  # Publicar el promedio en el tópico 'sol'
                self.get_logger().info('\nPromedio de temperatura enviado a sol: "%f"\n' % average)
            self.temperature_data.clear()  # Limpiar la lista de datos de temperatura
        

    def print_data(self):
        self.get_logger().info('Datos de humedad: %s' % str(self.humidity_data))
        self.get_logger().info('Datos de temperatura: %s' % str(self.temperature_data))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Imprimir los datos antes de salir en caso de interrupción de teclado
        minimal_subscriber.print_data()  
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
