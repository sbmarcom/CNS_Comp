#!/usr/bin/env python3
import spidev
import hal
import time
import sys
import traceback
import configparser
import linuxcnc
import struct
import os

os.system('bash ~/cn-seamless/GasControl/reset_module_and_STM32')

ParameterDict = {
    'Cutting' : {'Setpoint' : 100,'kp' : 1 , 'ki' : 1 , 'kd' :0  }
    #'Oxy':      {'kp' : 0 , 'ki' : 0 , 'kd' : 0 , 'Setpoint' : 0},
    #'Fuel':     {'kp' : 0 , 'ki' : 0 , 'kd' : 0 , 'Setpoint' : 0}
}

PacketHeaders = {'kp' : 0x70 , 'ki' : 0x69 , 'kd' : 0x64 , 'Setpoint' : 0x73}

config = configparser.ConfigParser(strict=False)
config.read('/homes/pi/linuxcnc/configs/my-mill/my-mill.ini')

#spi = spidev.SpiDev()
#spi.open(6,0) #spi bus 6, device 0
#spi.close()
spi = spidev.SpiDev()
spi.open(6,0) #spi bus 6, device 0
spi.mode = 0
spi.max_speed_hz = 50000

try:
    s=linuxcnc.stat()
    s.poll()
except linuxcnc.error:
    print("error ")
    sys.exit(1)

#spi.xfer([0x00],9600)
c =hal.component('GC')
c.newpin("ModuleStatus",hal.HAL_U32,hal.HAL_IN) #TBA
c.newpin("ModulePluggedIn",hal.HAL_BIT,hal.HAL_IN) #TBA
c.newpin("Pierce",hal.HAL_BIT,hal.HAL_IN) #true when user verifies material is preheated
c.newpin("FlameCommandOn",hal.HAL_BIT,hal.HAL_IN) #True when M03
c.newpin("ReadyToCutConfirmation", hal.HAL_BIT,hal.HAL_OUT) #set true when preheat is confirmed
c.newpin("ProbeDetected", hal.HAL_BIT,hal.HAL_OUT)
time.sleep(.1)


#for key in PacketHeaders.keys():
    #Go through each type of parameter and pull the data from the INI file
  #  try:
   #     ParameterDict[key] = config.get('HEADERS',f'{key}')
   # except:
   #     print("Variable {key} is not in INI file")


class module:
    def __init__(self):
        self.PluggedIn = False
        self.Initialized = False
        self.CuttingJetStatus = False
        self.CuttingJetCommand = False
        self.PreheatConfirmed = False
    def Initialize(self):
        c.ModuleStatus=1
        for p_id, p_info in ParameterDict.items():
            for key in p_info:
                print("initialized")
                #self.SendParameters(PacketHeaders[key],p_info[key])
        self.Initialized = True

    def DeInit(self):
        c.ModuleStatus = 0
        self.Initialized = False

    def SendParameters(self,header, data):
        #HeaderByte =[header] #pack the header and data into a byte array
        #if(header != 115):
           # DataByteArray = struct.pack('>f',data) #encode big endian 
            #DataByteArray = b'0x00000000'
        #else:
            #print("Encoding as 16 bit unsigned Int")
           # DataByteArray = struct.pack('>hh',data,0) #store in big endian format
        #print(f"Header: {header}, data = {data}")
        #print(f'Header Binary: {HeaderByte}  Data Binary: {DataByteArray}')
        crc = [0x00]#calculate checksum of the byte array, not implemented yet
        spi.xfer(HeaderByte,9600)
        #print(DataByteArray)
        spi.xfer(DataByteArray,9600)
        spi.xfer(crc,9600)
        time.sleep(.01)
        x = spi.readbytes(1)
        time.sleep(.01)
        #breakpoint()
        #print(f"Response from STM8S for header {header} is {x}")
    def TurnOnJet(self):
        #self.SendParameters(PacketHeaders['Setpoint'],ParameterDict['Cutting']['Setpoint'])
        spi.xfer([0x73,0x55,0xFF,0xFF,0xFF,0x00])
        time.sleep(.01)
        t = spi.xfer([0xAB])
        self.CuttingJetStatus= True
        c.ReadyToCutConfirmation =True
    def TurnOffJet(self):
        #self.SendParameters([0x73],[0x00,0x00,0x00,0x00])
        spi.xfer([0x73,0x00,0x00,0x00,0x00,0x00])
        time.sleep(.01)
        t = spi.xfer([0xAB])
        self.CuttingJetStatus= False
        print("Gas Turned Off Sucessfully")
    def Probe(self):
        f= spi.xfer([0xAB])
        time.sleep(0.1)
        f= spi.xfer([0xAB])
        time.sleep(0.1)
        f=spi.xfer([0x76,0x04,0x52,0x00,0x00,0x00])
        time.sleep(0.1)
        a = spi.xfer([0x78,0x00,0x20,0x00,0x00,0x00])
        time.sleep(0.1)
        a=spi.xfer([0x75,0x01,0x00,0x00,0x00,0x00])
        time.sleep(.01)
        #t = spi.xfer([0xAB])
        BottomedOut = [0]
        while  (BottomedOut[0]!=1 and hal.get_value("plasmac.ohmic-enable") ):
            time.sleep(.01)
            
            BottomedOut = spi.xfer([0xAB])
        print("Probe Activated")
        a = spi.xfer([0x75,0x00,0x00,0x00,0x00,0x00])
        time.sleep(0.01)
        b = spi.xfer([0x73,0,0,0,0,0])
        if BottomedOut[0] ==1:
            c.ProbeDetected = 1
        time.sleep(.1)
        os.system('bash ~/cn-seamless/GasControl/reset_module_only')
        c.ProbeDetected = 0

c.ready()
c.ProbeDetected = False
c.ReadyToCutConfirmation = False
Manifold = module()
Manifold.PluggedIn= True
Manifold.Initialize()
Manifold.TurnOffJet()
while (True):
    s.poll()
    #if (s.estop):
        #turn off gas
        #continue
    Manifold.PluggedIn = c.ModuleStatus
    if Manifold.PluggedIn ==False and Manifold.Initialized ==False:
        continue
    if Manifold.PluggedIn ==False and Manifold.Initialized==True:
        Manifold.Deinit()
        continue
    if Manifold.PluggedIn ==True and Manifold.Initialized ==False:
        Manifold.Initialize()
        continue

    if c.FlameCommandOn:
        if c.Pierce:
            #print("Pierce Confirmed")
            Manifold.CuttingJetCommand = True
    else:
        Manifold.CuttingJetCommand = False

   # print(f" CuttingJetCommand: {Manifold.CuttingJetCommand}")
    if hal.get_value("plasmac.ohmic-enable"):
        print("Going to Probe in gascontrol.py")
        Manifold.Probe()
        
    if Manifold.CuttingJetCommand:
        if Manifold.CuttingJetStatus==False:
            Manifold.TurnOnJet()  
         
            print(f"Gas Turned On")
            continue
    else:
        if Manifold.CuttingJetStatus==True:
            Manifold.TurnOffJet()
            c.ReadyToCutConfirmation = False
        
            
                
    

    
    




       

    
        

    
    
