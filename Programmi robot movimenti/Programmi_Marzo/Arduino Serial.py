#esempio di comunicazione seriale tra arduino e raspy, la potenza viene spartita tra primo e secondo byte,
# movimento sul secondo e puntatore sta sul primo

import serial
import time
ser = serial.Serial('/dev/ttyUSB0',115200)
movimento=3
puntatore=3
valore=100

while(1):
    temp=str(chr((valore>>4) | (puntatore<<4)))
    ser.write(temp.encode())
    temp=str(chr((valore&0b1111) | (movimento<<4)))
    ser.write(temp.encode())
    time.sleep(2)
