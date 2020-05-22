from gpiozero import PWMLED
from time import sleep
from signal import pause
import time
from time import clock
import RPi.GPIO as GPIO
import threading
import random
import serial

asx=20
psx=21
kp=0.0444
ki=0.05
kd=0.0125
errore=0
windup=5
voluto=0
voluto=2000

def saltoasx(k):
  h=ASX.old
  ASX.old=time.time()-h
  fine=errore
  errore=voluto-(old*1000000)
  ASX.errore=errore
  ASX.old=time.time()
  
def saltopsx(k):
  h=PSX.old
  PSX.old=time.time()-h
  fine=errore
  errore=voluto-(old*1000000)
  PSX.errore=errore
  PSX.old=time.time()

campionamento=200
campionamento=1/campionamento

GPIO.setmode(GPIO.BCM)
GPIO.setup(asx,GPIO.IN,GPIO.PUD_DOWN)
GPIO.add_event_detect(asx, GPIO.FALLING, callback=saltoasx)
GPIO.setup(psx,GPIO.IN,GPIO.PUD_DOWN)
GPIO.add_event_detect(psx, GPIO.FALLING, callback=saltopsx)
ser = serial.Serial('/dev/ttyACM0',9600)

def trasmissione(dato,posizione):
    LSB=str(chr(dato&31))
    MSB=str(chr(dato>>5))
    #print(LSB,MSB,dato)
    POS=str(chr(posizione))
    ser.write(LSB.encode())
    ser.write(MSB.encode())
    ser.write(POS.encode())

class PID:
  def __init__(self,errore,old):
    self.errore=errore
    self.old=old

ASX=PID(0,time.time)
PSX=PID(0,time.time)

class TimerClass(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.event = threading.Event()

    def run(self):
        global contatore,up,down,ultimo,errore,log,kp
        prop=0.0020
        integrale=0
        derivata=0
        media=0
        while((1) and not self.event.is_set()):
            media=ASX.errore
            derivata=media-derivata
            integrale=media+integrale
            if(integrale>windup):
                integrale=0
            if(integrale<-windup):
                integrale=0
            valore=(prop*voluto)-(kp*media)-(ki*integrale)-(kd*derivata)
            valore=int(valore*1023)
            if(valore>1023):
              valore=1023
            if(valore<0):
              valore=0
            trasmissione(555,0)
            derivata=errore
            media=errore
            time.sleep(campionamento)
            ultimo=errore
          
    def stop(self):
        self.event.set()

tmr = TimerClass()
tmr.start()
time.sleep(20)
trasmissione(124,1)
tmr.stop()
