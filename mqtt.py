import smbus
import time
import json
import ssl
import paho.mqtt.client as mqtt
from cryptography.fernet import Fernet


# Open i2c bus 1 and read one byte from address 80, offset 0
bus = smbus.SMBus(1)
bus_addr = 0x48
reg_addr = 0x01
n=4
writeBuf = []
writeBuf.append(0x86)
writeBuf.append(0x0B)
# writeBuf[0] = 0b10000110
# writeBuf[1] = 0b00000011

broker = "test.mosquitto.org"
port = 8884

#change to active high
#bus.write_i2c_block_data(bus_addr,reg_addr,writeBuf)
reg_addr=0x00

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val 
    
def on_message(client, userdata, message):
    print("Received message:{} on topic{}".format(message.payload, message.topic))

client = mqtt.Client()
client.on_message = on_message
client.tls_set(ca_certs="mosquitto.org.crt",certfile="client.crt",keyfile="client.key",tls_version=ssl.PROTOCOL_TLSv1_2)

c = client.connect(broker,port)
if (c != 0):
    print(mqtt.error_string(c))

client.subscribe("IC.embedded/Eat RaspberryPi/#")

client.loop()

####encryption###########
cipher_key=Fernet.generate_key()
cipher = Fernet(cipher_key)
# message1 = b'on'
# message2 = b'pi'
# encrypted_message1 = cipher.encrypt(message1)
# encrypted_message2 = cipher.encrypt(message2)
# out_message1=encrypted_message1.decode()
# decrypted_message1 = cipher.decrypt(encrypted_message1)
# #test show
# print("sent message1: ",message1)
# print("sent message2: ",message2)
# print("encrypted message: ", encrypted_message1)
# print("encrypted message: ", encrypted_message2)
# print("received messgae 1: ", out_message1)
# print("decrypted message 1: ",str(decrypted_message1.decode("utf-8")))

#while True:
#b = bus.read_i2c_block_data(bus_addr, reg_addr,2)
#press = twos_comp(b[0],8)+1
#force = twos_comp(b[1],8)
# print(f"force: {force}")
# print(f"press: {press}")
#msg = {
#"press": press,
#"force": force
#}

#msg_json = json.dumps(msg)
#print(msg_json)


#message=b'msg_json'
#message1=b'test'
#message2=b'test'
#print(message)
#print(message1)
#print(message2)
encrypted_message=cipher.encrypt("hello")
#encrypted_message1=cipher.encrypt(message1)
#encrypted_message2=cipher.encrypt(message2)
print("encrypted message: ", encrypted_message)
#print("encrypted message: ", encrypted_message1)
#print("encrypted message: ", encrypted_message2)
#out_message=encrypted_message.decode()
decrypted_message = cipher.decrypt(encrypted_message)
print("decrypted message: ", decrypted_message)

m = client.publish("IC.embedded/Eat RaspberryPi/test","hello")
if (m.rc != 0):
    print(mqtt.error_string(m.rc))
time.sleep(0.1)
# bus.close()





