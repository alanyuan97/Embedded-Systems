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
reg_config = 0x0D
# ******** CONTROL REG *************
reg_control = 0x0F # CONTROL registers

reg_status = 0x13 # LSB flag 
# ********** SENSOR RAW DATA *************
reg_B_HIGH = 0x1B
reg_B_LOW = 0x1A

reg_G_HIGH = 0x19
reg_G_LOW = 0x18

reg_R_HIGH = 0x17
reg_R_LOW = 0x16

reg_Data = 0x14


# Functions definition

def read_raw_data(address):
    
    data = bus.read_i2c_block_data(i2c_address,address,8)
    # return a list of 8 bytes
    return data


def on_message(client,userdata,message):
    print("Received message:{} on topic {}".format(message.payload,message.topic()))

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
    while(True):
        status = bus.read_i2c_block_data(i2c_address,reg_status,1)
        print(bin(status[0]))
        if (status[0] & 0b1)==1:
            print("Chip power on")
            break
        else:
            pass
    bus.write_i2c_block_data(i2c_address,0x00,[0b11])
    status = bus.read_i2c_block_data(i2c_address,0x00,1)
    print(bin(status[0]))
    return
# Initialize I2C (SMBus)
bus = smbus.SMBus(i2c_ch)
client = mqtt.Client()
client.on_message = on_message
client.subscribe("IC.embedded/Fat Duck/#")
client.loop()
client.tls_set(ca_certs="mosquitto.org.crt",certfile="client.crt",keyfile="client.key",tls_version=ssl.PROTOCOL_TLSv1_2)
success = client.connect("test.mosquitto.org",port=8884)
if (success!=0):
    print(mqtt.error_string(success))

index=0
try:
    # set_gain(0b11)
    init()
    set_gain(0)
    while(True):
        data = read_raw_data(reg_Data)
        print(data)
        cData = data[1] * 256 + data[0]
        red = data[3] * 256 + data[2]
        green = data[5] * 256 + data[4]
        blue = data[7] * 256 + data[6]

        print("\n C:{0} \n R:{1} \n G:{2} \n B:{3} ".format(cData,red,green,blue))

        newData=[]
        newData.append(cData)
        newData.append(red)
        newData.append(green)
        newData.append(blue)
        y = json.dumps(data)
        if (index==0):
            pub_success=client.publish("IC.embedded/Fat Duck/test",y)
            print(mqtt.error_string(pub_success.rc))
            index+=1
        time.sleep(0.5)
except KeyboardInterrupt:
    quit()
