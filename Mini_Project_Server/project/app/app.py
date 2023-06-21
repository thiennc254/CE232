import eventlet
import csv
from flask import Flask, render_template
from flask_mqtt import Mqtt
from flask_socketio import SocketIO
import os.path

eventlet.monkey_patch()
app = Flask(__name__)

app.config['MQTT_BROKER_URL'] = 'mqtt.flespi.io'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = 'rMQSCejChXfMkMsf1HNPgh6i9li86UbmxR4XTdp00uA84TeUoPzsWtBTCJSyuRIP'
app.config['MQTT_PASSWORD'] = ''
app.config['MQTT_REFRESH_TIME'] = 1.0

mqtt = Mqtt(app)
socketio = SocketIO(app)
mqtt_topic = "lab5"

my_data = ""
file_name = 'data/data.csv'
messages = []

# if not os.path.isfile(file_name):
with open(file_name, "w", newline='') as csv_file:
    field_name = ['Temperature', 'Humidity']
    writer = csv.DictWriter(csv_file, fieldnames=field_name)
    writer.writeheader()
# else:
#     file_size = os.path.getsize(file_name)
#     if file_size == 0:
#         with open(file_name, "a", newline='') as csv_file:
#             field_name = ['Temperature', 'Humidity']
#             writer = csv.DictWriter(csv_file, fieldnames=field_name)
#             writer.writeheader()

@app.route('/')
def main_page():
    return render_template('index.html')

@mqtt.on_connect()
def handle_connect(client, userdata, flags, rc):
    print("On connect")
    mqtt.subscribe(mqtt_topic, qos=2)

counter = 0
@mqtt.on_message()
def handle_mqtt_message(client, userdate, message):
    global my_data
    global messages, counter
    my_data = dict (
        topic = message.topic,
        payload = message.payload.decode(),
    )

    messages.append(str(message.payload.decode("utf-8")))
    if len(messages) > 0:
        write_to_csv()
    counter = counter + 1
    delete_rudundant()

    socketio.start_background_task(
        send_value,
        data=my_data
    )
    
def delete_rudundant():
    global counter
    with open(file_name, 'r') as file:
        data = file.readlines()
    if counter != len(data) - 1:
        data.pop()
        with open(file_name, 'w') as file:
            file.writelines(data)

def write_to_csv():
    global counter
    with open(file_name, "a", newline='') as file:
        writer = csv.writer(file)
        string = messages[0]
        n = len(string)
        half_length = n // 2
        str1 = string[:half_length]
        str2 = string[half_length:]
        writer.writerow([str1, str2])
    messages.clear()

def send_value(data):
    socketio.emit('mqtt_message', data=data)

if __name__ == '__main__':
    socketio.run(app, host='127.0.0.1', port=5000, use_reloader=True, debug=True)
