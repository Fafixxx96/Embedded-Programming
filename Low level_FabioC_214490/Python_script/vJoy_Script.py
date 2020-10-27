import time
import serial                                        #libary for serial comunication
from vjoy import vj, setJoy                          #library for vJoy to send value to
                                                     #virtual joystick
ATmega328p = serial.Serial('COM8', 38400, timeout=.1)#connect script to serial COM8 with 38400 baud
print("connected on COM8")

print("vj opening", flush=True)                     
vj.open()                                            #open virstual joystick

time.sleep(3)                                        #setup wainting 3 seconds

x=0                                                  #left x value
y=0                                                  #left y value
b=0                                                  #buttons value
a=0                                                  #accellerator value
f=0                                                  #break reverse value
s=0                                                  #steering wheel value

def freno(val):                                      #fuction used for handling the break/reverse value
                                                     #the potentiometer is fixed to high value near to 255
    if(val <= 130):
        return 9
    if ((val <= 135) & (val > 130)):
        return 8
    if ((val <= 140) & (val > 135)):
        return 7
    if ((val <= 145) & (val > 140)):
        return 6
    if ((val <= 150) & (val > 145)):
        return 5
    if ((val <= 155) & (val > 150)):
        return 4
    if ((val <= 160) & (val > 155)):
        return 3
    if ((val <= 165) & (val > 160)):
        return 2
    if ((val <= 170) & (val > 165)):
        return 1
    if (val > 170):
        return 0
    return 0

def accelleratore(val):                               #fuction used for handling the accelerator value
                                                      #the potentiometer is fixed to high value near to 255
    if(val <= 125):
        return 9
    if ((val <= 130) & (val > 125)):
        return 8
    if ((val <= 135) & (val > 130)):
        return 7
    if ((val <= 140) & (val > 135)):
        return 6
    if ((val <= 145) & (val > 140)):
        return 5
    if ((val <= 150) & (val > 145)):
        return 4
    if ((val <= 155) & (val > 150)):
        return 3
    if ((val <= 160) & (val > 155)):
        return 2
    if ((val <= 165) & (val > 160)):
        return 1
    if (val > 165):
        return 0
    return 0


while True:

    data = ATmega328p.read()                           #read incoming byte from serial
    
    if(b != 0):
        vj.setButton(b, 1)                             #send to vJoy the pushed button value if 0 sends nothing
        print(b)
    else:
        print(x, y, s, a, f, b)
        setJoy(x, y, s, a, f, 30.0)                    #send to vJoy the potentiometers value
 
    if(len(data) > 0):                                 #reads data if it receives the following char 
        if chr(data[0]) == 'b':                        #read immediately the next value
            b = int(ATmega328p.read())             
        elif chr(data[0]) == 'x':
            x = ATmega328p.read()
            x = (int(x[0])-7)*5                        #mutiplied by 5 because it receives an 8 bit value 
        elif chr(data[0]) == 'y':                      #and has to shift to 10 bit value
            y = ATmega328p.read()
            y = (int(y[0])-7)*5
        elif chr(data[0]) == 's':
            s = ATmega328p.read()                      #the steering wheel has 2 ends point it starts from 40 and arrives to 180
            s = (int(s[0])-40)*6                    #it never reaches the 0 value or 255, so I apply this formula to take proportional value
        elif chr(data[0]) == 'a':                   
            a = ATmega328p.read()
            a = accelleratore(int(a[0]))*115
        elif chr(data[0]) == 'f':
            f = ATmega328p.read()
            f = freno(int(f[0]))*115



                  
   

