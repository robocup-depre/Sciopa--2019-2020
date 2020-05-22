from time import sleep
from signal import pause
import RPi.GPIO as GPIO
AS=26
AD=6
PS=19
PD=13


def sas(k):
    print("SAS")

def sad(k):
    print("SAD")

def sps(k):
    print("SPS")

def spd(k):
    print("SPD")

GPIO.setmode(GPIO.BCM)
GPIO.setup(AS,GPIO.IN,GPIO.PUD_DOWN)
GPIO.add_event_detect(AS, GPIO.FALLING, callback=sas)
GPIO.setup(AD,GPIO.IN,GPIO.PUD_DOWN)
GPIO.add_event_detect(AD, GPIO.FALLING, callback=sad)
GPIO.setup(PS,GPIO.IN,GPIO.PUD_DOWN)
GPIO.add_event_detect(PS, GPIO.FALLING, callback=sps)
GPIO.setup(PD,GPIO.IN,GPIO.PUD_DOWN)
GPIO.add_event_detect(PD, GPIO.FALLING, callback=spd)

while(1):
    sleep(1)
