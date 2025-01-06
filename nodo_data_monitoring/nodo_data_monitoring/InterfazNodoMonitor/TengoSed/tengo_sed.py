from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

# Definir variables globales para almacenar los datos recibidos y los últimos 10 valores de temperatura, humedad y nivel de agua
data_received = {
    'humidity': None,
    'temperature': None,
    'water_level': None,
    'robot_state': None
}
last_ten_temperatures = []
last_ten_humidity = []
last_water_level = None

@app.route('/')
def index():
    # Formatear los datos para que solo tengan dos decimales y el estado del robot sea "Activado" o "Desactivado"
    formatted_data = {key: format_value(key, value) for key, value in data_received.items()}
    # Pasar los datos formateados, los últimos 10 valores de temperatura, humedad y nivel de agua a la plantilla index.html
    return render_template('index.html', **formatted_data, last_ten_temperatures=last_ten_temperatures, last_ten_humidity=last_ten_humidity, last_water_level=last_water_level)

@app.route('/update_data', methods=['POST'])
def update_data():
    global data_received, last_ten_temperatures, last_ten_humidity, last_water_level
    data = request.json
    print("Datos recibidos:", data)  # Imprimir los datos recibidos en la consola del servidor
    # Actualizar los datos almacenados con los nuevos datos recibidos
    data_received['humidity'] = data.get('humidity', None)
    data_received['temperature'] = data.get('temperature', None)
    data_received['water_level'] = data.get('water_level', None)
    data_received['robot_state'] = data.get('robot_state', None)
    # Actualizar los últimos 10 valores de temperatura, humedad y nivel de agua
    temperature = data.get('temperature', None)
    humidity = data.get('humidity', None)
    water_level = data.get('water_level', None)
    if temperature is not None:
        last_ten_temperatures.append(temperature)
        if len(last_ten_temperatures) > 10:
            last_ten_temperatures = last_ten_temperatures[-10:]
    if humidity is not None:
        last_ten_humidity.append(humidity)
        if len(last_ten_humidity) > 10:
            last_ten_humidity = last_ten_humidity[-10:]
    if water_level is not None:
        last_water_level = water_level
    # Puedes retornar los datos recibidos como JSON si lo deseas
    return jsonify({'success': True})

def format_value(key, value):
    # Redondear y formatear el valor a dos decimales
    if isinstance(value, float):
        return "{:.2f}".format(value)
    # Formatear el estado del robot
    # Verificar si la clave es 'robot_state'
    # if key == 'robot_state':
    #     # Si es así, comprobar el valor
    #     if value == "1":
    #         # Si el valor es 1, retornar "Activado"
    #         return "Activado"
    #     else:
    #         # Si el valor no es 1, retornar "Desactivado"
    #         return "Desactivado"
    # Si la clave no es 'robot_state', retornar el valor sin modificar
    return value

if __name__ == '__main__':
    app.run(debug=True)
