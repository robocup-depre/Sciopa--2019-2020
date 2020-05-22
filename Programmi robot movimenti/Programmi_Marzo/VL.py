# Demo of reading the range and lux from the VL6180x distance sensor and
# printing it every second.
# Author: Tony DiCola
from time import sleep
import board
import busio
import adafruit_vl6180x
import RPi.GPIO as GPIO
LD=21
LAD=20
LAS=16
LS=12
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
GPIO.output(21,GPIO.LOW)
GPIO.output(20,GPIO.LOW)
GPIO.output(12,GPIO.LOW)
GPIO.output(16,GPIO.LOW)
GPIO.output(21,GPIO.HIGH)
sleep(0.1)
i2c = busio.I2C(board.SCL, board.SDA)
LD = adafruit_vl6180x.VL6180X(i2c)
LD.set_address(i2c,0x50)
GPIO.output(20,GPIO.HIGH)
sleep(0.1)
LAD = adafruit_vl6180x.VL6180X(i2c)
LAD.set_address(i2c,0x51)
GPIO.output(16,GPIO.HIGH)
sleep(0.1)
LAS = adafruit_vl6180x.VL6180X(i2c)
LAS.set_address(i2c,0x52)
GPIO.output(12,GPIO.HIGH)
sleep(0.1)
LS = adafruit_vl6180x.VL6180X(i2c)
LS.set_address(i2c,0x53)
