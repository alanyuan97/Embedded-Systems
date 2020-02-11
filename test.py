#!/usr/bin/env python3
import smbus
from time import sleep

address=0x29

# sensor config
VL53L0X_REG_SYSRANGE_START = 0x000
VL53L0X_REG_RESULT_RANGE_STATUS = 0x0014

def u16(lsb, msb):
    return ((msb & 0xFF) << 8)  | (lsb & 0xFF)

bus = smbus.SMBus(1)

while True:

    bus.write_byte_data(address, VL53L0X_REG_SYSRANGE_START, 0x01)
    for _ in range(100):
        sleep(0.001)
        val = bus.read_byte_data(address, VL53L0X_REG_RESULT_RANGE_STATUS)
        if (val & 0x01):
            break

    data = bus.read_i2c_block_data(address, VL53L0X_REG_RESULT_RANGE_STATUS, 12)
    status = (data[0] & 0x78) >> 3

    if status==11:
        SPAD_Rtn_count = u16(data[3], data[2])
        signal_count = u16(data[7], data[6])
        ambient_count = u16(data[9], data[8])
        distance = u16(data[11], data[10])
    else:
        # logging.debug(f"TOF sensor error {status} {str_error(status)}")
        print(f"Not ready")

    sleep(0.2)