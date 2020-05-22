from time import sleep
from signal import pause
import RPi.GPIO as GPIO
AS=13
AD=19
PS=6
PD=26


def sas(k):
    print("SAS")

def sad(k):
    print("SAD")

def sps(k):
    print("")

def spd(k):
    print("")

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
    print(GPIO.input(AS))
    sleep(1)
