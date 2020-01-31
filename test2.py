#! /usr/bin/python3
import time

import board
import busio
import json
import paho.mqtt.client as mqtt

from adafruit_as726x import Adafruit_AS726x

#maximum value for sensor reading
max_val = 16000
GAIN = 64
#max number of characters in each graph
max_graph = 80

def graph_map(x):
    # return min(int(x * max_graph / max_val), max_graph)
    return x
# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = Adafruit_AS726x(i2c)

sensor.conversion_mode = sensor.MODE_2
sensor.gain = GAIN
client = mqtt.Client()
success = client.connect("test.mosquitto.org",port=1883)
if (success!=0):
    print(mqtt.error_string(success))

index=0

while True:
    # Wait for data to be ready
    while not sensor.data_ready:
        time.sleep(.1)

    #plot plot the data
    print("\n")
    print(f"V: {graph_map(sensor.violet)}")
    print(f"B: {graph_map(sensor.blue)}")
    print(f"G: {graph_map(sensor.green)}")
    print(f"Y: {graph_map(sensor.yellow)}")
    print(f"O: {graph_map(sensor.orange)}")
    print(f"R: {graph_map(sensor.red)}")

    dicts = {"V":sensor.violet,"B":sensor.blue,"G":sensor.green,"Y":sensor.yellow,"O":sensor.orange,"R":sensor.red}
    y = json.dumps(dicts)
    if (index==0):
        pub_success=client.publish("IC.embedded/Fat Duck/test","Hello")
        print(mqtt.error_string(pub_success.rc))
        index+=1

    

    time.sleep(1)

