#! /usr/bin/python3
import time
import smbus
import json
import paho.mqtt.client as mqtt
import ssl

i2c_ch = 1

# TMP102 address on the I2C bus
i2c_address = 0x49

# Register addresses
# ******** CONTROL REG *************
reg_config = 0x04 # CONTROL registers BIT7:0 RST:INT:GAIN*2:BANK:2:DATARDY:RSVD
reg_INT = 0x05 # Intergration time 8 bit
reg_DEV = 0x06 # Device temperature 8 bit
reg_LED = 0x07 # BIT7:0 RSVD*2:ICL_DRV*2:LED_DRV:ICL_IND*2:LED_IND
# ********** SENSOR RAW DATA *************
reg_V_HIGH = 0x08
reg_V_LOW = 0x09

reg_B_HIGH = 0x0A
reg_B_LOW = 0x0B

reg_G_HIGH = 0x0C
reg_G_LOW = 0x0D

reg_Y_HIGH = 0x0E
reg_Y_LOW = 0x0F

reg_O_HIGH = 0x10
reg_O_LOW = 0x11

reg_R_HIGH = 0x12
reg_R_LOW = 0x13

# ********* CALIBRATED DATA *************
reg_V = 0x14
reg_B = 0x18
reg_G = 0x1C
reg_Y = 0x20
reg_O = 0x24
reg_R = 0x28
# Functions definition
def read_raw_data(channel):
    if (channel == "V"):
        reg_addr = reg_V_HIGH
    elif (channel == "B"):
        reg_addr = reg_B_HIGH
    elif (channel == "G"):
        reg_addr = reg_G_HIGH
    elif (channel == "Y"):
        reg_addr = reg_Y_HIGH
    elif (channel == "O"):
        reg_addr =reg_O_HIGH 
    elif (channel == "R"):
        reg_addr =reg_R_HIGH
    else:
        print("Invalid data reading. ")
    bus.write_byte_data(i2c_address, 0x01, reg_addr)

    while(True):
        status = bus.read_byte_data(i2c_address,0x00)
        if (status&0b00000001)==1:
            break
        else:
            # print("-")
            pass
    data = bus.read_byte_data(i2c_address,0x02)
    # data = val2[0]<<8 & val2[0]   
    return data

def on_message(client,userdata,message):
    print("Received message:{} on topic {}".format(message.payload,message.topic()))

def CONFIG():
    return bus.read_i2c_block_data(i2c_address, reg_config, 1)

def set_mode(input):
    pass
def set_gain(gain):
    REG_MASK = 0xFF
    control_output = []
    gain = gain << 4
    control_input = CONFIG()
    #print(f"CONFIG DATA: {control_input[0:]}")
    print(f"CONFIG DATA: {bin(control_input[0])}")
    control_output.append((control_input[0] & 0b11001111) | gain)
    bus.write_i2c_block_data(i2c_address, reg_config, control_output)
    print(f"CONTROL BITS: {bin(control_output[0]&REG_MASK)}")
    control_input = CONFIG()
    print(f"CONFIG DATA: {bin(control_input[0])}")

def init():
    while True:
        status = bus.read_byte_data(i2c_address, 0x00)
        if (status&0b00000010)==0:
            break
        else:
            pass
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

    init()
    set_gain(0b11)
    while(True):
        

        data = []
        data.append(read_raw_data("V"))
        data.append(read_raw_data("B"))
        data.append(read_raw_data("G"))
        data.append(read_raw_data("Y"))
        data.append(read_raw_data("O"))
        data.append(read_raw_data("R"))
        print("\n V:{0} \n B:{1} \n G:{2} \n Y:{3} \n O:{4} \n R:{5}".format(data[0],data[1],data[2],data[3],data[4],data[5]))

        
        y = json.dumps(data)
        if (index==0):
            pub_success=client.publish("IC.embedded/Fat Duck/test",y)
            print(mqtt.error_string(pub_success.rc))
            index+=1

    

    
        time.sleep(0.5)
except KeyboardInterrupt:
    quit()
