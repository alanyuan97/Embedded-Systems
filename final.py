#!/usr/bin/env python3
import logging

import time     # import the time library for the sleep function
import paho.mqtt.client as mqtt
import ssl
import smbus
import json
from gpiozero import LED
import numpy as np

i2c_ch = 1
# ********************* RGB *****************************
# RGB address on the I2C bus
RGB_i2c = 0x29
# Register addresses
reg_config = 0xAD
# ******** CONTROL REG *************
reg_control = 0xAF # CONTROL registers

reg_status = 0xB3 # LSB flag 
# ********** SENSOR RAW DATA *************
reg_B_HIGH = 0xBB
reg_B_LOW = 0xBA

reg_G_HIGH = 0xB9
reg_G_LOW = 0xB8

reg_R_HIGH = 0xB7
reg_R_LOW = 0xB6

reg_Data = 0xB4

#*********************************************************

# Distance sensor 
DIS_i2c = 0x30
#  register addr 
VL53L0X_REG_SYSRANGE_START = 0x00
VL53L0X_REG_RESULT_INTERRUPT_STATUS = 0x0013
VL53L0X_REG_RESULT_RANGE_STATUS = 0x0014

lblue = LED(27)
lred = LED(17)
lgreen = LED(22)

def on_message(client, userdata, message):
    print ("Received message:{} on topic {}".format(message.payload,message.topic))
    if (message.topic=="IC.embedded/FatDuck/motor" and message.payload.decode()=='start'):
        start_reading()
    elif(message.topic=="IC.embedded/FatDuck/motor" and message.payload.decode()=='stop'):
        pass

def on_log(client, userdata, level, buf):
    print("log: ",buf)

def read_rgb_data(address):
    data = bus.read_i2c_block_data(RGB_i2c,address,8)
    # return a list of 8 bytes
    return data

def read_dis_data(address):
    bus.write_byte_data(DIS_i2c, VL53L0X_REG_SYSRANGE_START, 0x01)

    cnt = 0
    while (cnt < 100): # 1 second waiting time max
        val = bus.read_byte_data(DIS_i2c, VL53L0X_REG_RESULT_RANGE_STATUS)
        if (val & 0x01):
            break
        cnt += 1

    if (val & 0x01):
        print("TOF ready")
    else:
        print("TOF not ready")
    data = bus.read_i2c_block_data(DIS_i2c,address,12)
    return data

def init(): 
    print("INIT")
    bus.write_i2c_block_data(RGB_i2c,0xa0,[0b11])
    # status = bus.read_i2c_block_data(RGB_i2c,0xa0,1)
    # print(bin(status[0]))
    return

def set_gain(gain): # 0 for 1x, 1 for 4x, 2for 16x, 3 for 64x
    REG = GET_CONTROL()
    temp = [(REG[0] & 0xFC) + (gain & 0x11)]
    bus.write_i2c_block_data(RGB_i2c, reg_control,temp)

def GET_CONTROL():
    """
        Control Registers : Bits 7-2 are reserved
                            Bits 1-0 are used for gain mode
                            00: 1x
                            01: 4x
                            10: 16x
                            11: 60x

    """
    return bus.read_i2c_block_data(RGB_i2c, reg_control, 1)  


def start_reading():
    client.loop_stop()
    index=0 #
    counter=0
    threshold=200

    init() # setup RGB  & distance sensor
    set_gain(0) # RGB gain to 0
    start_time = time.time() 
    reds=[]
    diss=[]
    lblue.blink()
    for _ in range(10):
        # Enable led blink
        rgbdata = read_rgb_data(reg_Data)
        disdata = read_dis_data(VL53L0X_REG_RESULT_RANGE_STATUS)

        # Not used cData
        # cData = rgbdata[1] * 256 + rgbdata[0]
        red = rgbdata[3] * 256 + rgbdata[2]
        green = rgbdata[5] * 256 + rgbdata[4]
        blue = rgbdata[7] * 256 + rgbdata[6]
        distance = disdata[10]*256 + disdata[11]

        # Prevent distance reading not vaild
        if distance>200:
            distance = 200

        reds.append(red)
        diss.append(distance)
        # print("\n C:{0} \n R:{1} \n G:{2} \n B:{3} ".format(cData,red,green,blue))
        # print(index)
        print(f"Distance: {distance}")
        # Sync time with motor
        time.sleep(0.9)
        # Disable blink blue

    amp_dis = np.multiply(reds,diss)
    print(amp_dis)
    max_amp_dis = np.max(amp_dis)
    min_amp_dis = np.min(amp_dis)
    # Normalize data
    numerator = np.subtract(amp_dis,min_amp_dis)
    red_norm = 255* (np.divide(numerator,max_amp_dis-min_amp_dis))
    red_array = np.array(red_norm)
    print(red_array.astype(int))
    red_int= red_array.astype(int)
    print(len(red_int[red_int>threshold]))
    lblue.off()
    if (len(red_int[red_int>threshold])>=6):
        lgreen.on()
        y=json.dumps("good")
        pub_success=client.publish("IC.embedded/FatDuck/motor",y)
        print(mqtt.error_string(pub_success.rc))
    else:
        lred.on()
        y=json.dumps("bad")
        pub_success=client.publish("IC.embedded/FatDuck/motor",y)
        print(mqtt.error_string(pub_success.rc))
    end_time= time.time()
    # print(f"Execution time for one loop:{end_time-start_reading}s")
#************************************************ MAIN ******************************************************

bus = smbus.SMBus(i2c_ch)
port = 8080
server = "test.mosquitto.org"
client = mqtt.Client()
client.on_message = on_message
client.on_log = on_log

client.tls_set(ca_certs="mosquitto.org.crt",certfile="client.crt",keyfile="client.key",tls_version=ssl.PROTOCOL_TLSv1_2)
success = client.connect("test.mosquitto.org",port=8883)
client.subscribe("IC.embedded/FatDuck/motor")

client.loop_start()

if (success!=0):
    print(mqtt.error_string(success))

pub_success=client.publish("IC.embedded/FatDuck","ZeroPi_online")
print(mqtt.error_string(pub_success.rc))

try:
    while True:
        pass
except KeyboardInterrupt as error:
    print(error)
