import time
import RPi.GPIO as GPIO
import threading
import random
import serial

encoder=15
windup=13780
old=0
voluto=10000
kp=0
ki=0.4
kd=5
valore=0
prop=0.01
dato=11000

ser = serial.Serial('/dev/ttyUSB0',115200)

def salto(variabile):
    global old,dato,k
    dato=time.time()-old;
    dato=dato*1000000
    #print(dato)
    old=time.time()

class TimerClass(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.event = threading.Event()

    def run(self):
        global voluto
        integrale=0
        derivata=0
        errore=10
        while((1) and not self.event.is_set()):
            errore=(voluto)-dato
            derivata=errore-derivata
            integrale=integrale+errore
            if(integrale<-windup):
                integrale=-windup
            if(integrale>windup):
                integrale=windup
            valore=(prop*voluto)-(kp*errore)-(ki*integrale)-(kd*derivata)
            #print(valore)
            derivata=errore
            if(valore<0):
                valore=45
            if(valore>255):
                valore=255
            valore=valore
            valore=int(valore)
            #print(valore)
            serial(valore)
            time.sleep(0.01)
          
    def stop(self):
        self.event.set()

tmr = TimerClass()

def initialize():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder,GPIO.IN,GPIO.PUD_DOWN)
    GPIO.add_event_detect(encoder, GPIO.FALLING, callback=salto)  
    tmr.start()

def serial(valore):
    temp=str(chr(valore>>4))
    ser.write(temp.encode())
    temp=str(chr(valore&0b1111))
    ser.write(temp.encode())
    
    
initialize()
while(1):
    time.sleep(5)
