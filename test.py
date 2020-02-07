#!/usr/bin/env python3

from gpiozero import LED
from time import sleep

led = LED(22)

while True:
    led.on()
    sleep(0.2)
    led.off()
    sleep(0.2)