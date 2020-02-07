#! /usr/bin/python3
import time
import smbus
import json
import paho.mqtt.client as mqtt
import ssl

i2c_ch = 1

# TMP102 address on the I2C bus
i2c_address = 0x29

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


# Functions definition

def read_raw_data(address):
    
    data = bus.read_i2c_block_data(i2c_address,address,8)
    # return a list of 8 bytes
    return data


def on_message(client, userdata, message):
    print ("Received message:{} on topic {}".format(message.payload,message.topic))
    if (message.topic=="IC.embedded/FatDuck/motor" and message.payload.decode()=='start'):
        pass
    elif(message.topic=="IC.embedded/FatDuck/motor" and message.payload.decode()=='stop'):
        pass


def on_log(client, userdata, level, buf):
    print("log: ",buf)

def start_reading():
    index=0
    counter=0
    threshold=11110

    init()
    set_gain(0)

    while(True):
        while (index<10):
            data = read_raw_data(reg_Data)
            cData = data[1] * 256 + data[0]
            red = data[3] * 256 + data[2]
            green = data[5] * 256 + data[4]
            blue = data[7] * 256 + data[6]

            print("\n C:{0} \n R:{1} \n G:{2} \n B:{3} ".format(cData,red,green,blue))
            print(index)

            if (red>threshold):
                counter=counter+1
                
            if (index ==9):
                if (counter>4):
                    y=json.dumps("good")
                    pub_success=client.publish("IC.embedded/FatDuck/",y)
                    #print(mqtt.error_string(pub_success.rc))
                else: 
                    y=json.dumps("bad")
                    pub_success=client.publish("IC.embedded/FatDuck/",y)
                    #print(mqtt.error_string(pub_success.rc))
            #if (index==0):
            
                #pub_success=client.publish("IC.embedded/Fat Duck/",y)
                #print(mqtt.error_string(pub_success.rc))
            #index+=1
            time.sleep(1)
            index=index+1
        else:
            index =0
            counter=0

def GET_CONTROL():
    """
        Control Registers : Bits 7-2 are reserved
                            Bits 1-0 are used for gain mode
                            00: 1x
                            01: 4x
                            10: 16x
                            11: 60x

    """
    return bus.read_i2c_block_data(i2c_address, reg_control, 1)

def set_gain(gain): # 0 for 1x, 1 for 4x, 2for 16x, 3 for 64x
    REG = GET_CONTROL()
    temp = [(REG[0] & 0xFC) + (gain & 0x11)]
    bus.write_i2c_block_data(i2c_address, reg_control,temp)

def init():
    bus.write_i2c_block_data(i2c_address,0xa0,[0b11])
    status = bus.read_i2c_block_data(i2c_address,0xa0,1)
    print(bin(status[0]))
    return

# Initialize I2C (SMBus)
bus = smbus.SMBus(i2c_ch)
client = mqtt.Client()

client.on_message = on_message
client.on_log = on_log

client.tls_set(ca_certs="mosquitto.org.crt",certfile="client.crt",keyfile="client.key",tls_version=ssl.PROTOCOL_TLSv1_2)
success = client.connect("test.mosquitto.org",port=8883)
client.subscribe("IC.embedded/FatDuck/motor")
if (success!=0):
    print(mqtt.error_string(success))
client.loop()
print("Start loop")
pub_success=client.publish("IC.embedded/FatDuck","pi_online")
print(mqtt.error_string(pub_success.rc))

while(True):
    pass