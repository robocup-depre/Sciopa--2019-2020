from gpiozero import PWMLED
from time import sleep
from signal import pause
import time
from time import clock
import RPi.GPIO as GPIO
import threading
import random
import serial

encoder=15
kp=0.0444
ki=0.05
kd=0#.0125
errore=0
windup=5
voluto=0
old=0
voluto=2000

def salto(k):
  global errore,old
  old=time.time()-old
  fine=errore
  errore=voluto-(old*1000000)
  #print(errore)
  old=time.time()

campionamento=200
campionamento=1/campionamento

#def initialize():

GPIO.setmode(GPIO.BCM)
GPIO.setup(encoder,GPIO.IN,GPIO.PUD_DOWN)
GPIO.add_event_detect(encoder, GPIO.FALLING, callback=salto)  
ser = serial.Serial('/dev/ttyUSB0',9600)
#initialize()

def trasmissione(dato):
    LSB=str(chr(dato&31))
    MSB=str(chr(dato>>5))
    #print(LSB,MSB,dato)
    spazio=str(chr(10))
    ser.write(LSB.encode())
    ser.write(MSB.encode())

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
            media=errore
            derivata=media-derivata
            integrale=media+integrale
            if(integrale>windup):
                integrale=0
            if(integrale<-windup):
                integrale=0
            #print(integrale)
            valore=(prop*voluto)-(kp*media)-(ki*integrale)-(kd*derivata)
            valore=int(valore*1023)
            if(valore>1023):
              valore=1023
            if(valore<0):
              valore=0
            #print(valore, errore)
            trasmissione(555)
            derivata=errore
            media=errore
            time.sleep(campionamento)
            ultimo=errore
          
    def stop(self):
        self.event.set()

tmr = TimerClass()
tmr.start()
time.sleep(20)
trasmissione(512)
tmr.stop()
