# LTR-308ALS ambient light sensor

A MicroPython driver for the Lite-On **LTR-308ALS-01** I2C ambient light
sensor: lux readings with selectable gain (1x to 18x) and resolution
(16 to 20 bits), standby, and the threshold interrupt on `nINT`.

Copy `ltr308.py` onto your board, or install it with `mip`:

```python
import mip
mip.install("github:HowManyOliversAreThere/micropython-ltr308")
```

## Usage

```python
from machine import I2C, Pin
import ltr308

i2c = I2C(0, sda=Pin(19), scl=Pin(20), freq=400_000)
als = ltr308.LTR308(i2c, gain=ltr308.GAIN_3X, resolution=ltr308.RES_18BIT_100MS)
print(als.lux())                  # latest reading
print(als.lux(only_new=True))     # None unless a new conversion finished

if als.saturated():
    als.gain(ltr308.GAIN_1X)      # too bright for this gain

als.standby()                     # stop measuring until enable()
```

## Notes

- The address is fixed at 0x53. The constructor checks the part ID (0xB1)
  and raises `OSError` if it doesn't match.
- Lux is `count × 0.6 × window_factor / (gain × integration factor)`, with
  the integration time relative to 100 ms, per Lite-On's formula. Pass
  `window_factor` to correct for light lost to a cover window.
- The measurement rate must be at least as long as the conversion time.

## License

MIT - see [LICENSE](LICENSE).
