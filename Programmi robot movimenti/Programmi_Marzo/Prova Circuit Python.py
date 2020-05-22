import time
import board
import busio
import adafruit_tmp006
import adafruit_tcs34725
from Adafruit_BNO055 import BNO055

t=0
# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
tmp1 = adafruit_tmp006.TMP006(i2c,0x40)
tcs = adafruit_tcs34725.TCS34725(i2c,0x29)
bno = BNO055.BNO055()
# Initialize communication with the sensor, using the default 16 samples per conversion.
# This is the best accuracy but a little slower at reacting to changes.
# The first sample will be meaningless
while t<10:
    obj_temp = tmp1.temperature
    #print(obj_temp)
    r,g,b=tcs.color_rgb_bytes
    print(tmp1._read_die_temperature)
    print('Object temperature: {0:8.2f}'.format(tmp1.temperature))
    print('RED:{0:3d} GREEN:{1:3d} BLUE:{2:3d}'.format(r,g,b))
    time.sleep(5)
    t=+1

