#!/usr/bin/env python3
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''


import logging

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import paho.mqtt.client as mqtt
import ssl

def on_message(client, userdata, message):
    print ("Received message:{} on topic {}".format(message.payload,message.topic))
    if (message.topic=="IC.embedded/FatDuck/motor" and message.payload.decode()=='start'):
        start_motor()
    elif(message.topic=="IC.embedded/FatDuck/motor" and message.payload.decode()=='stop'):
        end_motor()
        
def start_motor():
    """
        TODO While loop is true: 
                client.loop_start -> end, wait for incoming message to start scanning
                    if flag is TRUE, run the motor, when finish, reset all parameters for next run
                    else pass
    """
    try:
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder D
    except IOError as error:
        print(error)
    BP.set_motor_dps(BP.PORT_B, target)
    print(("Motor C Target Degrees Per Second: %d" % target), "  Motor C Status: ", BP.get_motor_status(BP.PORT_B))

def end_motor():
    BP.set_motor_dps(BP.PORT_B, 0)
#************************************************ MAIN ******************************************************

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
target = 100 # was 40
port = 8080
server = "test.mosquitto.org"
client = mqtt.Client()

client.on_message = on_message
# client.on_log = on_log

logger = logging.getLogger('xx')
logger.setLevel('DEBUG')
client.enable_logger(logger)
client.tls_set(ca_certs="mosquitto.org.crt",certfile="client.crt",keyfile="client.key",tls_version=ssl.PROTOCOL_TLSv1_2)
success = client.connect("test.mosquitto.org",port=8883)
client.subscribe("IC.embedded/FatDuck/motor")

client.loop_start()
# client.loop()

if (success!=0):
    print(mqtt.error_string(success))

pub_success=client.publish("IC.embedded/FatDuck","Brickpi_online")
print(mqtt.error_string(pub_success.rc))

try:
    while(True):
        pass
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all() # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.