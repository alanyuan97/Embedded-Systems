#! /usr/bin/python3
import smbus

bus = smbus.SMBus(1)
bus_addr = 0x49

try:
    while (True):
        data = bus.read_byte(bus_addr)
        print(f"Data received: {data}")
except KeyboardInterrupt:
    bus.close()
    quit()