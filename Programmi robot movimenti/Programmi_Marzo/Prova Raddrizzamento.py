'''
9/3 AD  10/2 AS    11/1 PD    12/0 PS
AS=26
AD=6
PS=19
PD=13
avanti=1
indietro=0
destra=3
sinistra=2
'''
import time
import RPi.GPIO as GPIO
import threading
from random import randrange
import serial
from Adafruit_BNO055 import BNO055
import board
import busio
import adafruit_vl6180x

EAS=26
EAD=6
EPD=13
EPS=19
windup=15000
kp=10
ki=0.01
kd=0.6
prop=0.05
direzione=0
cm=973
avanti=1
indietro=0
destra=3
sinistra=2
voluto=7000

bno = BNO055.BNO055()
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_vl6180x.VL6180X(i2c)
ser = serial.Serial('/dev/ttyUSB0',115200)

def setta(voluto):
    AS.voluto=voluto-50
    AD.voluto=voluto
    PD.voluto=voluto
    PS.voluto=voluto
    if(voluto==0):
        PS.voluto=0
        AS.voluto=0

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
        NPD=2
        NAD=0
        NPS=3
        NAS=1
        while((1) and not self.event.is_set()):
            if(sel==0):
                if(AS.voluto==0):
                    serial(0,2,direzione)
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
                    serial(AS.valore,NAS,direzione)
                    #print(AS.valore)
            if(sel==1):
                if(AD.voluto==0):
                    serial(0,3,direzione)
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
                    serial(AD.valore,NAD,direzione)
            if(sel==2):
                if(PS.voluto==0):
                    serial(0,0,direzione)
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
                    serial(PS.valore,NPS,direzione)
                    #print(PS.valore)
            if(sel==3):
                if(PD.voluto==0):
                    serial(0,1,direzione)
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
                    serial(PD.valore,NPD,direzione)
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

AS = PID(0, time.time(),voluto,0,0)
AD = PID(0, time.time(),voluto,0,0)
PS = PID(0, time.time(),voluto,0,0)
PD = PID(0, time.time(),voluto,0,0)

def initialize():
    setta(0)
    direzione=0
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
    tmr.start()
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

def serial(valore,puntatore,movimento):
    temp=str(chr((valore>>4) | (puntatore<<4)))
    ser.write(temp.encode())
    temp=str(chr((valore&0b1111) | (movimento<<4)))
    ser.write(temp.encode())

initialize()
distAV=0
HEADM, roll, pitch = bno.read_euler()
while(1):
    input("MIAO")
    temp, roll, pitch = bno.read_euler()
    #print(abs(HEADM-temp))
    if((abs(HEADM-temp))<180):
        direzione=sinistra
    if((abs(HEADM-temp))>180):
        direzione=destra
    setta(3000)
    while(abs(HEADM-temp)>2):
        temp, roll, pitch = bno.read_euler()
        #time.sleep(0.05)
        #print(abs(head-temp))
    setta(0)

    
