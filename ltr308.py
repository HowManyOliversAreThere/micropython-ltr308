"""
MicroPython driver for the Lite-On LTR-308ALS-01 ambient light sensor
(I2C, 16- to 20-bit, roughly 0.01 to 157,000 lux).

I2C address (7-bit): 0x53 (fixed).

Quick start
-----------
    from machine import I2C, Pin
    import ltr308

    i2c = I2C(0, sda=Pin(19), scl=Pin(20), freq=400_000)
    als = ltr308.LTR308(i2c)                 # gain 3x, 100 ms, 18-bit
    print(als.lux())                         # None until the first reading

    als.gain(ltr308.GAIN_1X)                 # for bright light
    als.standby()                            # ~1 uA until enable()
"""

import time
from micropython import const

ADDR = 0x53
PART_ID = 0xB1  # part number (0xB) and revision (0x1)

# Gain settings, and the factor each one divides the count by.
GAIN_1X = 0
GAIN_3X = 1
GAIN_6X = 2
GAIN_9X = 3
GAIN_18X = 4
_GAINS = (1, 3, 6, 9, 18)

# Resolution / conversion time. The integration factor is relative to
# 100 ms, which the lux formula is based on.
RES_20BIT_400MS = 0
RES_19BIT_200MS = 1
RES_18BIT_100MS = 2
RES_17BIT_50MS = 3
RES_16BIT_25MS = 4
_INT_FACTOR = (4, 2, 1, 0.5, 0.25)
_INT_MS = (400, 200, 100, 50, 25)

# Measurement repeat rate (must be at least the conversion time).
RATE_25MS = 0
RATE_50MS = 1
RATE_100MS = 2
RATE_500MS = 3
RATE_1000MS = 5
RATE_2000MS = 6

_REG_MAIN_CTRL = const(0x00)
_REG_MEAS_RATE = const(0x04)
_REG_GAIN = const(0x05)
_REG_PART_ID = const(0x06)
_REG_STATUS = const(0x07)
_REG_DATA = const(0x0D)
_REG_INT_CFG = const(0x19)
_REG_INT_PERSIST = const(0x1A)
_REG_THRES_UP = const(0x21)
_REG_THRES_LOW = const(0x24)

_CTRL_ALS_EN = const(0x02)
_CTRL_SW_RESET = const(0x10)
_STATUS_DATA = const(0x08)
_STATUS_INT = const(0x10)
_STATUS_POWER_ON = const(0x20)


class LTR308:
    def __init__(self, i2c, address=ADDR, gain=GAIN_3X,
                 resolution=RES_18BIT_100MS, rate=RATE_100MS,
                 window_factor=1.0, enable=True):
        """
        i2c           : machine.I2C (or SoftI2C) instance
        gain          : GAIN_1X .. GAIN_18X
        resolution    : RES_20BIT_400MS .. RES_16BIT_25MS
        rate          : RATE_25MS .. RATE_2000MS
        window_factor : light lost to a cover window, >= 1. Lux readings
                        are multiplied by it (1.0 with no window).
        Raises OSError if the part ID doesn't match.
        """
        self._i2c = i2c
        self._addr = address
        self._buf = bytearray(3)
        self.window_factor = window_factor
        part = self.part_id()
        if part != PART_ID:
            raise OSError("LTR-308 not found (part ID 0x%02X)" % part)
        self._gain = gain
        self._res = resolution
        self._rate = rate
        self._write(_REG_GAIN, gain)
        self._write(_REG_MEAS_RATE, (resolution << 4) | rate)
        if enable:
            self.enable()
        else:
            self.standby()

    # ---- Low-level I2C ----------------------------------------------------
    def _read(self, reg):
        return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

    def _write(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, bytes((value & 0xFF,)))

    def part_id(self):
        return self._read(_REG_PART_ID)

    # ---- Power ------------------------------------------------------------
    def enable(self):
        """Start measuring continuously at the configured rate."""
        self._write(_REG_MAIN_CTRL, _CTRL_ALS_EN)
        self._enabled = True

    def standby(self):
        """Stop measuring (about 1 uA). Settings are kept."""
        self._write(_REG_MAIN_CTRL, 0)
        self._enabled = False

    @property
    def enabled(self):
        return self._enabled

    def reset(self):
        """Software reset - every register back to its default."""
        try:
            self._write(_REG_MAIN_CTRL, _CTRL_SW_RESET)
        except OSError:
            pass  # the part may not acknowledge the reset write
        time.sleep_ms(10)
        self._enabled = False

    # ---- Configuration ----------------------------------------------------
    def gain(self, value=None):
        if value is None:
            return self._gain
        if not 0 <= value < len(_GAINS):
            raise ValueError("bad gain")
        self._write(_REG_GAIN, value)
        self._gain = value
        return value

    def resolution(self, value=None, rate=None):
        if value is None:
            return self._res
        if not 0 <= value < len(_INT_FACTOR):
            raise ValueError("bad resolution")
        if rate is not None:
            self._rate = rate
        self._write(_REG_MEAS_RATE, (value << 4) | self._rate)
        self._res = value
        return value

    @property
    def conversion_ms(self):
        return _INT_MS[self._res]

    # ---- Readings ---------------------------------------------------------
    def status(self):
        """Raw MAIN_STATUS register."""
        return self._read(_REG_STATUS)

    def data_ready(self):
        return bool(self.status() & _STATUS_DATA)

    def raw(self):
        """Latest 20-bit ALS count (reading it clears data-ready)."""
        self._i2c.readfrom_mem_into(self._addr, _REG_DATA, self._buf)
        b = self._buf
        return ((b[2] & 0x0F) << 16) | (b[1] << 8) | b[0]

    def lux(self, only_new=False):
        """Illuminance in lux, from the latest count. With only_new=True,
        returns None unless a conversion has finished since the last
        read."""
        if only_new and not self.data_ready():
            return None
        count = self.raw()
        return count * 0.6 * self.window_factor / (_GAINS[self._gain] * _INT_FACTOR[self._res])

    def saturated(self):
        """True if the last count is at the top of the current range, so
        a lower gain would read more accurately."""
        bits = 20 - self._res
        return self.raw() >= (1 << bits) - 1

    # ---- Interrupt (the nINT pin, active low) -----------------------------
    def set_thresholds(self, low, high, persist=1):
        """Assert nINT when the raw count leaves [low, high] for `persist`
        consecutive readings (1-16)."""
        self._i2c.writeto_mem(self._addr, _REG_THRES_UP, bytes((high & 0xFF, (high >> 8) & 0xFF, (high >> 16) & 0x0F)))
        self._i2c.writeto_mem(self._addr, _REG_THRES_LOW, bytes((low & 0xFF, (low >> 8) & 0xFF, (low >> 16) & 0x0F)))
        self._write(_REG_INT_PERSIST, (max(1, min(16, persist)) - 1) << 4)

    def interrupt(self, enable):
        """Enable or disable the threshold interrupt on nINT."""
        self._write(_REG_INT_CFG, 0x10 | (0x04 if enable else 0))
