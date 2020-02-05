#! /usr/bin/python3
# Simple demo of the TCS34725 color sensor.
# Will detect the color from the sensor and print it out every second.
import time
 
import board
import busio
import json
import paho.mqtt.client as mqtt
import ssl
import adafruit_tcs34725
 
def on_message(client, userdata, message):
    print("Received message:{} on topic {}".format(message.payload, message.topic))
 
# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)
index=0
client = mqtt.Client()
client.on_message = on_message
success = client.connect("test.mosquitto.org",port=8884)
client.tls_set(ca_certs="mosquitto.org.crt", certfile="client.crt",keyfile="client.key",tls_version=ssl.PROTOCOL_TLSv1_2)
client.subscribe("IC.embedded/Fat Duck/#")
client.loop()
print(mqtt.error_string(success))
print("before loop")
client.loop()
pub_success=client.publish("IC.embedded/Fat Duck/1","hello")
print(mqtt.error_string(pub_success.rc))
# Main loop reading color and printing it every second.
while True:
    # Read the color temperature and lux of the sensor too.
    #print('Color: ({0}, {1}, {2})'.format(*sensor.color_rgb_bytes))
    #red, green,blue = (sensor.color_rgb_bytes)
    #dicts = {"R":red, "G":green, "B":blue}
    dicts = {"R":1}
    y = json.dumps(dicts)
    #print(index)
    if (index==0):
        #pub_success=client.publish("IC.embedded/Fat Duck","hello")
        #print(mqtt.error_string(pub_success.rc))
        print("good transmit")
        index+=1
    #temp = sensor.color_temperature
    #lux = sensor.lux
    #print('Temperature: {0}K Lux: {1}'.format(temp, lux))
    # Delay for a second and repeat.
    time.sleep(1.0)
