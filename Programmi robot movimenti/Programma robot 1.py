'''
9/3 AD  10/2 AS    11/1 PD    12/0 PS
AS=13
AD=19
PS=6
PD=26
'''
import time
import RPi.GPIO as GPIO
import threading
from random import randrange
import serial
from Adafruit_BNO055 import BNO055
from time import sleep

EAS=13
EAD=19
EPD=26
EPS=6
windup=15000
voluto=3000
kp=0.1
ki=0.01
kd=0.05
prop=0.05
direzione=0
avanti=0
indietro=1
sinistra=2
destra=3

bno = BNO055.BNO055()
ser = serial.Serial('/dev/ttyUSB0',115200)

def setta(voluto,direzione):
    AS.voluto=voluto
    AD.voluto=voluto
    PD.voluto=voluto
    PS.voluto=voluto
    AS.direzione=direzione

def SAS(variabile):
    AS.counter+=1
    AS.enc=(time.time()-AS.time)*1000000
    AS.time=time.time()
    if (abs((AS.old-AS.enc))>100):
        AS.enc=(AS.old+AS.enc)/2
    AS.old=AS.enc

def SAD(variabile):
    AD.enc=(time.time()-AD.time)*1000000
    AD.time=time.time()
    if (abs((AD.old-AD.enc))>100):
        AD.enc=(AD.old+AD.enc)/2
    AD.old=AD.enc
    
def SPD(variabile):
    PD.enc=(time.time()-PD.time)*1000000
    PD.time=time.time()
    if (abs((PD.old-PD.enc))>100):
        PD.enc=(PD.old+PD.enc)/2
    PD.old=PD.enc

def SPS(variabile):
    PS.enc=(time.time()-PS.time)*1000000
    PS.time=time.time()
    if (abs((PS.old-PS.enc))>100):
        PS.enc=(PS.old+PS.enc)/2
    PS.old=PS.enc

class TimerClass(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.event = threading.Event()

    def run(self):
        global voluto,direzione
        sel=0
        while((1) and not self.event.is_set()):
            if(sel==0):
                if(AS.voluto==0):
                    serial(0,2,AS.direzione)
                    integrale=0
                else:
                    AS.errore=AS.voluto-AS.enc
                    AS.derivata=AS.errore-AS.derivata
                    AS.integrale=AS.integrale+AS.errore
                    if(AS.integrale<-windup):
                        AS.integrale=-windup
                    if(AS.integrale>windup):
                        AS.integrale=windup
                    AS.valore=(prop*voluto)-(kp*AS.errore)-(ki*AS.integrale)-(kd*AS.derivata)
                    AS.derivata=AS.errore
                    if(AS.valore<0):
                       AS.valore=0
                    if(AS.valore>255):
                        AS.valore=255
                    AS.valore=int(AS.valore)
                    serial(AS.valore,2,AS.direzione)
                    #print(AS.valore)
            if(sel==1):
                if(AD.voluto==0):
                    serial(0,3,AS.direzione)
                    integrale=0
                else:
                    AD.errore=AD.voluto-AD.enc
                    AD.derivata=AD.errore-AD.derivata
                    AD.integrale=AD.integrale+AD.errore
                    if(AD.integrale<-windup):
                        AD.integrale=-windup
                    if(AD.integrale>windup):
                        AD.integrale=windup
                    AD.valore=(prop*voluto)-(kp*AD.errore)-(ki*AD.integrale)-(kd*AD.derivata)
                    AD.derivata=AD.errore
                    if(AD.valore<0):
                       AD.valore=0
                    if(AD.valore>255):
                        AD.valore=255
                    AD.valore=int(AD.valore)
                    serial(AD.valore,3,AS.direzione)
                    #print(AD.valore)
            if(sel==2):
                if(PS.voluto==0):
                    serial(0,0,AS.direzione)
                    integrale=0
                else:
                    PS.errore=PS.voluto-PS.enc
                    PS.derivata=PS.errore-PS.derivata
                    PS.integrale=PS.integrale+PS.errore
                    if(PS.integrale<-windup):
                        PS.integrale=-windup
                    if(PS.integrale>windup):
                        PS.integrale=windup
                    PS.valore=(prop*voluto)-(kp*PS.errore)-(ki*PS.integrale)-(kd*PS.derivata)
                    PS.derivata=PS.errore
                    if(PS.valore<0):
                       PS.valore=0
                    if(PS.valore>255):
                        PS.valore=255
                    PS.valore=int(PS.valore)
                    serial(PS.valore,0,AS.direzione)
                    #print(PS.valore)
            if(sel==3):
                if(PD.voluto==0):
                    serial(0,1,AS.direzione)
                    integrale=0
                else:
                    PD.errore=PD.voluto-PD.enc
                    PD.derivata=PD.errore-PD.derivata
                    PD.integrale=PD.integrale+PD.errore
                    if(PD.integrale<-windup):
                        PD.integrale=-windup
                    if(PD.integrale>windup):
                        PD.integrale=windup
                    PD.valore=(prop*voluto)-(kp*PD.errore)-(ki*PD.integrale)-(kd*PD.derivata)
                    PD.derivata=PD.errore
                    if(PD.valore<0):
                       PD.valore=0
                    if(PD.valore>255):
                        PD.valore=255
                    PD.valore=int(PD.valore)
                    serial(PD.valore,1,AS.direzione)
                    #print(PD.valore)
            time.sleep(0.001)
            sel+=1
            sel=sel%4
            #print(sel)

    def stop(self):
        self.event.set()

tmr = TimerClass()
class PID:
    def __init__(self,errore,tempo,voluto,valore,counter):
        self.errore=errore
        self.time=tempo
        self.enc=voluto
        self.old=voluto
        self.valore=valore
        self.voluto=voluto
        self.counter=counter
        self.derivata=0
        self.integrale=0
        self.direzione=0

AS = PID(0, time.time(),voluto,0,0)
AD = PID(0, time.time(),voluto,0,0)
PS = PID(0, time.time(),voluto,0,0)
PD = PID(0, time.time(),voluto,0,0)

def initialize():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(EAS,GPIO.IN,GPIO.PUD_DOWN)
    GPIO.add_event_detect(EAS, GPIO.FALLING, callback=SAS) 
    GPIO.setup(EAD,GPIO.IN,GPIO.PUD_DOWN)
    GPIO.add_event_detect(EAD, GPIO.FALLING, callback=SAD)
    GPIO.setup(EPD,GPIO.IN,GPIO.PUD_DOWN)
    GPIO.add_event_detect(EPD, GPIO.FALLING, callback=SPD) 
    GPIO.setup(EPS,GPIO.IN,GPIO.PUD_DOWN)
    GPIO.add_event_detect(EPS, GPIO.FALLING, callback=SPS) 
    temp=time.time()
    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    setta(0,avanti)
    tmr.start()

def serial(valore,puntatore,movimento):
    temp=str(chr((valore>>4) | (puntatore<<4)))
    ser.write(temp.encode())
    temp=str(chr((valore&0b1111) | (movimento<<4)))
    ser.write(temp.encode())

def destra():
    heading,roll,pitch=bno.read_euler()
    temp=heading+90
    if(temp>360):
        temp=temp-360
    print(heading,temp)
    sleep(1)
    while(abs(temp-heading)>1):
        setta(3000,destra)
        sleep(0.02)
        heading,roll,pitch=bno.read_euler()
        print(heading)

initialize()
while(1):
    input("parto")
    setta(2000,avanti)
    while(AS.counter<1500):
        sleep(0.01)
    AS.counter=0
    setta(0,avanti)
    '''
    heading,roll,pitch=bno.read_euler()
    temp=heading+90
    if(temp>360):
        temp=temp-360
    print(heading,temp)
    sleep(1)
    while(abs(temp-heading)>1):
        setta(3000,destra)
        sleep(0.02)
        heading,roll,pitch=bno.read_euler()
        print(heading)
    setta(0,avanti)
    input()
    setta(0,avanti)
    input()'
    '''
