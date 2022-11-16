#!/usr/bin/env python
import serial
from serial.tools import list_ports
import hal
import time
import sys
import traceback
import configparser
import linuxcnc


try:
    GcSerial = serial.Serial('/dev/ttyUSB1',115200,timeout=1)
except:
    continue:
    
config = configparser.ConfigParser(strict=False)
config.read('/homes/pi/linuxcnc/configs/my-mill/my-mill.ini')
gf=config.get('GAS_CONTROL','AcetyleneGains').split(',')
go=config.get('GAS_CONTROL','OxygenGains').split(',')
gc=config.get('GAS_CONTROL','CuttingGains').split(',')
ph=config.get('GAS_CONTROL','PreheatSetpoint').split(',')
ig=config.get('GAS_CONTROL','IgnitionSetpoint').split(',')
cu=config.get('GAS_CONTROL','CuttingSetpoint').split(',')
off = [0,0,0]


c =hal.component('GC')
c.newpin("SP",hal.HAL_U32,hal.HAL_IN)
c.newpin("Confirmation",hal.HAL_BIT,hal.HAL_IN)
c.newpin("mFuel",hal.HAL_BIT,hal.HAL_IN)
c.newpin("pFuel",hal.HAL_BIT,hal.HAL_IN)
c.newpin("mPO",hal.HAL_BIT,hal.HAL_IN)
c.newpin("pPO",hal.HAL_BIT,hal.HAL_IN)
c.newpin("mCO",hal.HAL_BIT,hal.HAL_IN)
c.newpin("pCO",hal.HAL_BIT,hal.HAL_IN)
c.newpin("Pierce",hal.HAL_BIT,hal.HAL_IN)
c.newpin("gga",hal.HAL_S32,hal.HAL_IN)

c.ready()
time.sleep(.5)


#Set GC Gains
GasControl('go',go)
time.sleep(.1)
GasControl('gf',gf)
time.sleep(.1)
GasControl('gc',gc)
time.sleep(.1)
GasControl('s',off)
current = [0,0,0]
PierceDelay= 2.5
PrevPoint = 0

c.ready()

while True:
    #time.sleep(.04)
    check = c.Confirmation
    mode = c.SP
    jogs = [c.mFuel, c.pFuel, c.mPO,c.pPO, c.mCO,c.pCO]
    ready = c.Pierce
    increment = 8
    #
   
    #if any(jogs) and check ==True:
     #  if jogs[0] ==True:
      #     current[0]-=increment
      # if jogs[1] ==True:
       #    current[0]+=increment
       #if jogs[2] ==True:
       #    current[1]-=increment
       #if jogs[3] ==True:
       #    current[1]+=increment
       #if jogs[4] ==True:
       #    current[2]-=increment
       #if jogs[5] ==True:
        #   current[2]+=increment
       #GasControl('s',current)
    if mode == PrevPoint and check == True and ready ==False:
        continue
    if mode>1 and check != True:
        GasControl('s',off)
        current = off
        c.SP=0
    if mode ==0:
        GasControl('s',off)
        current=off
        c.SP=0
    if mode ==1:
        GasControl('s',ig)
        current=ig
    if mode ==2 and check==True:
        GasControl('s',ph)
        current = ph
        #time.clock_settime(CLOCK_REALTIME,0)
        c.SP=3
    if mode ==3 and check == True and ready==True :
        GasControl('s',cu)
        current = cu
        c.SP=4
        time.sleep(PierceDelay)
        
    PrevPoint = mode      
        
              
        
 def GasControl(category,valueList):
    message= "<{},{},{},{}>".format(category,valueList[0],valueList[1],valueList[2])

    #GcSerial.write(message.encode())
    #response=GcSerial.read(1)
    #print(response)
    pass
       
        
    
    
        

    
    
