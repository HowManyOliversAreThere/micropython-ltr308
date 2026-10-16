"""Print the light level whenever a new reading is ready."""

import time
from machine import I2C, Pin

import ltr308

i2c = I2C(0, sda=Pin(19), scl=Pin(20), freq=400_000)
als = ltr308.LTR308(i2c)

while True:
    lux = als.lux(only_new=True)
    if lux is not None:
        print("{:.1f} lux{}".format(lux, " (saturated)" if als.saturated() else ""))
    time.sleep_ms(50)
