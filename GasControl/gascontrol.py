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

os.system('bash ~/cn-seamless/GasControl/reset_module')

ParameterDict = {
    'Cutting' : {'Setpoint' : 100,'kp' : 1 , 'ki' : 1 , 'kd' :0  }
    #'Oxy':      {'kp' : 0 , 'ki' : 0 , 'kd' : 0 , 'Setpoint' : 0},
    #'Fuel':     {'kp' : 0 , 'ki' : 0 , 'kd' : 0 , 'Setpoint' : 0}
}

PacketHeaders = {'kp' : 0x70 , 'ki' : 0x69 , 'kd' : 0x64 , 'Setpoint' : 0x73}

config = configparser.ConfigParser(strict=False)
config.read('/homes/pi/linuxcnc/configs/my-mill/my-mill.ini')

spi = spidev.SpiDev()
spi.open(6,0) #spi bus 6, device 0
spi.close()
spi = spidev.SpiDev()
spi.open(6,0) #spi bus 6, device 0

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
c.newpin("Pierce",hal.HAL_BIT,hal.HAL_IN)
c.newpin("PreheatConfirmation",hal.HAL_BIT,hal.HAL_IN)
c.newpin("FlameCommandOn",hal.HAL_BIT,hal.HAL_IN)
c.newpin("CutConfirmation", hal.HAL_BIT,hal.HAL_IN)
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
                self.SendParameters(PacketHeaders[key],p_info[key])
        self.Initialized = True

    def DeInit(self):
        c.ModuleStatus = 0
        self.Initialized = False

    def SendParameters(self,header, data):
        HeaderByte =struct.pack('B',header) #pack the header and data into a byte array
        if(header != 115):
            DataByteArray = struct.pack('>f',data) #encode big endian 
            #DataByteArray = b'0x00000000'
        else:
            #print("Encoding as 16 bit unsigned Int")
            DataByteArray = struct.pack('>hh',data,0) #store in big endian format
        #print(f"Header: {header}, data = {data}")
        #print(f'Header Binary: {HeaderByte}  Data Binary: {DataByteArray}')
        crc = [0x00]#calculate checksum of the byte array, not implemented yet
        spi.xfer(HeaderByte,9600)
        print(DataByteArray)
        spi.xfer(DataByteArray,9600)
        spi.xfer(crc,9600)
        time.sleep(.01)
        x = spi.readbytes(1)
        time.sleep(.01)
        #breakpoint()
        #print(f"Response from STM8S for header {header} is {x}")
    def TurnOnJet(self):
        self.SendParameters(PacketHeaders['Setpoint'],ParameterDict['Cutting']['Setpoint'])
        self.CuttingJetStatus= True
    def TurnOffJet(self):
        self.SendParameters(PacketHeaders['Setpoint'],0)
        self.CuttingJetStatus= False
        print("Gas Turned Off Sucessfully")

c.ready()
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
        if c.PreheatConfirmation:
            if c.Pierce:
                Manifold.CuttingJetCommand = True
                Manifold.PreheatConfirmed =True
            elif not Manifold.PreheatConfirmed:
                print("Waiting for Preheat Confirmation")
        else:
            print("Turn On Preheat Flame and Confirm")
            Manifold.PreheatConfirmed = False 
    else:
        Manifold.CuttingJetCommand = False
        Manifold.PreheatConfirmed =False

   # print(f" CuttingJetCommand: {Manifold.CuttingJetCommand}")

    if Manifold.CuttingJetCommand:
        if Manifold.CuttingJetStatus==False:
            Manifold.TurnOnJet()   
           # print(f"Gas Status {Manifold.CuttingJetStatus}")
            c.CutConfirmation =True
            continue
    else:
        if Manifold.CuttingJetStatus==True:
            Manifold.TurnOffJet()
        
            
                
    

    
    




       

    
        

    
    
