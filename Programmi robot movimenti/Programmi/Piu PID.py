import time
import RPi.GPIO as GPIO
import threading
from random import randrange
import serial
import matplotlib.pyplot as plt
import numpy as np

encoderASX=15
encoderADX=18
windup=15500
voluto=3000
kp=0.15
ki=0.015
kd=0.07
prop=0.05
contatore=0

ser = serial.Serial('/dev/ttyUSB0',115200)

def saltoASX(variabile):
    global contatore
    ASX.enc=(time.time()-ASX.time)*1000000
    ASX.time=time.time()
    if (abs((ASX.old-ASX.enc))>100):
        ASX.enc=(ASX.old+ASX.enc)/2
    ASX.old=ASX.enc

def saltoADX(variabile):
    global contatore
    ADX.enc=(time.time()-ADX.time)*1000000
    ADX.time=time.time()
    if (abs((ADX.old-ADX.enc))>100):
        ADX.enc=(ADX.old+ADX.enc)/2
    ADX.old=ADX.enc

class TimerClass(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.event = threading.Event()

    def run(self):
        global voluto
        selettore=0
        while((1) and not self.event.is_set()):
                selettore+=1
                selettore=selettore%2
                if(selettore==0):
                     if(ASX.voluto==0):
                        serial(0,0)
                        ASX.integrale=0
                    else:
                        ASX.errore=ASX.voluto-ASX.enc
                        ASX.derivata=ASX.errore-ASX.derivata
                        ASX.integrale=ASX.integrale+ASX.errore
                        if(ASX.integrale<-windup):
                            ASX.integrale=0
                        if(ASX.integrale>windup):
                            ASX.integrale=0
                        ASX.valore=(prop*ASX.voluto)-(kp*ASX.errore)-(ki*ASX.integrale)-(kd*ASX.derivata)
                        ASX.derivata=ASX.errore
                        if(ASX.valore<0):
                           ASX.valore=0
                        if(ASX.valore>255):
                            ASX.valore=255
                        ASX.valore=int(ASX.valore)
                        serial(ASX.valore,0)
                        time.sleep(0.01)
                if(selettore==1):
                     if(ADX.voluto==0):
                        serial(0,1)
                        ADX.integrale=0
                    else:
                        ADX.errore=ADX.voluto-ADX.enc
                        ADX.derivata=ADX.errore-ADX.derivata
                        ADX.integrale=ADX.integrale+ADX.errore
                        if(ADX.integrale<-windup):
                            ADX.integrale=0
                        if(ADX.integrale>windup):
                            ADX.integrale=0
                        ADX.valore=(prop*ADX.voluto)-(kp*ADX.errore)-(ki*ADX.integrale)-(kd*ADX.derivata)
                        ADX.derivata=ADX.errore
                        if(ADX.valore<0):
                           ADX.valore=0
                        if(ADX.valore>255):
                            ADX.valore=255
                        ADX.valore=int(ADX.valore)
                        serial(ADX.valore,1)
                        time.sleep(0.01)
          
    
    def stop(self):
        self.event.set()

tmr= TimerClass()

class PID:
    def __init__(self,errore,tempo,voluto,valore,counter):
        self.errore=errore
        self.time=tempo
        self.enc=voluto
        self.old=voluto
        self.valore=valore
        self.voluto=voluto
        self.counter=counter
        self.integrale=0
        self.derivata=0
        
ASX = PID(0, time.time(),voluto,0,0)   
ADX = PID(0, time.time(),voluto,0,0)

def initialize():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoderASX,GPIO.IN,GPIO.PUD_UP)
    GPIO.add_event_detect(encoderASX, GPIO.FALLING, callback=saltoASX)
    GPIO.setup(encoderADX,GPIO.IN,GPIO.PUD_UP)
    GPIO.add_event_detect(encoderADX, GPIO.FALLING, callback=saltoADX) 
    temp=time.time()
    tmr.start()

def serial(valore,puntatore):
    temp=str(chr((valore>>4) | (puntatore<<4)))
    ser.write(temp.encode())
    temp=str(chr(valore&0b1111))
    ser.write(temp.encode())
    
    
initialize()
while(1):
    time.sleep(1.5)
