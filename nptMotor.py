# -*- coding: utf-8 -*-
from pystxmcontrol.controller.motor import motor

from pylibftdi import Device, Driver
from uldaq import get_daq_device_inventory, DaqDevice, InterfaceType, TmrIdleState, PulseOutOption, TmrStatus
import numpy as np
import time
import struct

class nptMotor(motor):

    def __init__(self, ftdi_device_id = '7340012A'):
        # refer to page 29 of NPoint manual for device write/read formatting info
        self.devID = ftdi_device_id
        self.dev = self.initialize() # NOTE: SUDO PRIVILEGES NEEDED TO CONNECT

        #connect the daq_device USB-CTR08 and call the timer function
        self.daq_device = self.DaqConnect()
        self.tmr_device = self.daq_device.get_tmr_device()

        #axis base address
        self.axis1address = 0x11831000
        self.axis2address = 0x11832000

        #various offsets. To be summed with base address for an axis.
        self.positionOffset = 0x218 # "Digital Position Command" used for both reading and writing positions
        self.pOffset = 0x720 # Proportional Gain
        self.iOffset = 0x728 # Integral Gain
        self.dOffset = 0x730 # Derivative Gain
        self.sOffset = 0x084 # servo state: 1 closes the loop, 0 opens it.
        self.dsrOffset = 0x334 # digital sensor reading

        #command prefixes
        self.readCom = 0xA0 # prepends all read commands
        self.writeCom = 0xA2 # prepends all write commands
        self.readArrayCom = 0xA4 # to read multiple 32bit values, e.g. to read a single 64bit float.
        self.writeNextCom = 0xA3 # to write a single 32bit right after issuing a writeCom, at its following address.

        #some more whole addresses
        self.zeroPosAddr = 0xEC000004 # write a 32bit value to this address to set current position to zero. Setting 1st LSB to 1 zeros the first channel, 2nd LSB the second channel, ...
        self.axis1SensorGainAddr = 0x11831245
        self.axis2SensorGainAddr = 0x11832245

        #spiral trajectory whole addresses. These are full addresses that are not related to the channel base addresses. page44 on manual
        self.spiralAxis1Addr = 0x11830510  #Sets the controller channel to be used for Axis 1
        self.spiralAxis2Addr = 0x11830514  #Sets the controller channel to be used for Axis 2.
        self.spiralRadius1Addr = 0x11830520  #Sets the Axis 1 scan radius in 20 bit counts.   
        self.spiralRadius2Addr = 0x11830524  #Sets the Axis 2 scan radius in 20 bit counts.
        self.spiralRevoAddr = 0x1183051C  #Sets the spiral scan revolutions per second.
        self.spiralLineSpaAddr = 0x11830528 #Sets the spacing between lines in 20 bit counts
        self.spiralStartAddr = 0x118290D8 
   

        #trajectory_generation whole addresses. these are full addresses that are not related to the channel base addresses. page40 on manual
        self.trajGenEnabelAddr = 0xB10 # trajectory generation enable
        self.trajCoorNumAddr = 0x1182A000 #set Number of trajectory coordinates
        self.trajIterNumAddr = 0x1182A004 #set Number of trajectory iterations
        self.trajGenParamAddr = 0x1182A6A0 #trajectory generation parameters array  
        self.trajStartAddr = 0x11829048 #start trajectoy
        self.trajStopAddr = 0x1182904C #stop trajectoy

        #Sensor Gain. should really be read using above address. To be added later...
        self.sensorGain = 26.60 #10.577 # 1nm = sensorGain*[quadrature counts]

        #read in the current pid parameters and current positions
        self.curPID = (self.pidRead(1), self.pidRead(2))
        self.curPos = (self.getPos(1), self.getPos(2))

        #reset the interferometer, and start from position zero.
        #self.resetInterferometer()
        self.moveTo(1,0)
        self.moveTo(2,0)

    def initialize(self):
        return Device(device_id = self.devID)
       
    def getStatus(self, **kwags):
        pass

    def moveBy(self, axis, pos):
        final_pos = self.curPos[axis-1] + pos
        self.moveTo(axis,final_pos)
        
   
    def moveTo(self, axis, pos):
        dist = np.abs(self.curPos[axis-1] - pos)
        pid = self.disToPID(axis, dist)
        if pid != self.curPID[axis-1]:
            print("current pid is",self.curPID[axis-1], "\nNow,for this step, writing the optimal pid",pid," to axis",axis, "...") 
            self.pidWrite(axis, pid)
        self.curPos = (self.curPos[0], pos) if axis == 2 else (pos, self.curPos[1])
        writeAddr = self.posAddress(axis)
        pos = int(round(float(self.nmToSteps(pos))))
        pos = int(self.signedIntToHex(pos),16)
        self.writeToDev4B(writeAddr, pos)


    def getPos(self, axis):
        readAddr = self.posAddress(axis)
        posInSteps = self.readFromDev4B(readAddr)
        return self.stepsToNM(posInSteps)


#####################################################################################################

    def DaqConnect(self):#connect DAQ devices
        try:
            devices = get_daq_device_inventory(InterfaceType.USB)
            number_of_devices = len(devices)
            daq_device= DaqDevice(devices[0])
            daq_device.connect()
            for i in range(number_of_devices):
                print(devices[i].product_name,'is connected.')
            return daq_device
        except:
            print("No DAQ device")

    def devlist(self): # should really be done outside of this script; but I'll just put it here too. SUDO PRIVILEGES NEEDED TO WORK, OTHERWISE LIST COMES BACK EMPTY.
        return Driver().list_devices()

    def posAddress(self, axis):
        if axis == 1:
            addr = self.axis1address + self.positionOffset
        elif axis == 2:
            addr = self.axis2address + self.positionOffset
        else:
            raise("invalid axis")
        return addr

    def dsrAddress(self, axis): #digital sensor reading
        if axis == 1:
            addr = self.axis1address + self.dsrOffset
        elif axis == 2:
            addr = self.axis2address + self.dsrOffset
        else:
            raise("invalid axis")
        return addr

    def pAddress(self, axis): # proportional gain
        if axis == 1:
            addr = self.axis1address + self.pOffset
        elif axis == 2:
            addr = self.axis2address + self.pOffset
        else:
            raise("invalid axis")
        return addr

    def iAddress(self, axis):
        if axis == 1:
            addr = self.axis1address + self.iOffset
        elif axis == 2:
            addr = self.axis2address + self.iOffset
        else:
            raise("invalid axis")
        return addr

    def dAddress(self, axis):
        if axis == 1:
            addr = self.axis1address + self.dOffset
        elif axis == 2:
            addr = self.axis2address + self.dOffset
        else:
            raise("invalid axis")
        return addr

    def sAddress(self, axis):
        if axis == 1:
            addr = self.axis1address + self.sOffset
        elif axis == 2:
            addr = self.axis2address + self.sOffset
        else:
            raise("invalid axis")
        return addr

    def hexToSignedInt(self, h): #converts hex numbers to signed int
        hInt = int(h[2:],16)
        if hInt <= 0x7FFFFFFF:
            return hInt
        else:
            return hInt - 0x100000000

    def signedIntToHex(self, si):
        if si < 0:
            si += 0x100000000
        return hex(si)

    def float64ToHex(self, f):  #needed for writing the float64 pid value
        return hex(struct.unpack('<Q', struct.pack('<d', f))[0])

    def float32Tohex(self,f):   #needed for writing the float32 velocity limit value in trajectory_make 
        return hex(struct.unpack('<I', struct.pack('<f', f))[0]) 

    def hexToFloat64(self, h):
        return struct.unpack('<d', struct.pack('<Q', int(h, 16)))[0]

    def readFromDev4B(self, addr): # to read 32bit values, e.g. position or servo state
        # format: [readCom] [address] [0x55] for a total of 6 bytes
        readTX = 0x55 * 16**10 + addr * 16**2 + self.readCom
        readTX = bytearray.fromhex(hex(readTX)[2:]) # Get the bytes in decreasing significance (need to be reversed)
        readTX.reverse()
        dataw = self.dev.write(readTX)
        if dataw != 6:
            print("dataw =", dataw)
            raise("reading value: writing to \"read address\" on device failed")
        datar = self.dev.read(10)
        while datar == b'':
            datar = self.dev.read(10)
        datar = bytearray(datar)
        datar.reverse()
        val = datar[1:5]
        val = '0x' + val.hex()
        return self.hexToSignedInt(val)

    def readArray(self, numBytes, addr): # e.g. to read PID parameters as 64bit float, use numBytes=2
        # format: [readArrayCom] [address] [numBytes] [0x55] for a total of 10 bytes
        readTX = 0x55 * 16**18 + numBytes * 16**10 + addr * 16**2 + self.readArrayCom
        readTX = bytearray.fromhex(hex(readTX)[2:]) # Get the bytes in decreasing significance (need to be reversed)
        readTX.reverse()
        dataw = self.dev.write(readTX)
        if dataw != 10:
            print("dataw =", dataw)
            raise("reading value: writing to \"read address\" on device failed")
        datar = self.dev.read(6 + 4*numBytes)
        while datar == b'':
            datar = self.dev.read(6 + 4*numBytes)
        datar = bytearray(datar)
        datar.reverse()
        val = datar[1:1+4*numBytes]
        retVal = '0x' + val.hex()
        return retVal 

    def writeToDev4B(self, addr, val): # to write 32bit values, e.g. position or servo state
        # format: [writeCom] [addr] [val/data] [0x55] for a total of 10 bytes: [addr] and [val] are 4 bytes each.
        writeTX = 0x55 * 16**18 + val * 16**10 + addr * 16**2 + self.writeCom
        writeTX = bytearray.fromhex(hex(writeTX)[2:]) # Get the bytes in decreasing significance (need to be reversed)
        writeTX.reverse()
        dataw = self.dev.write(writeTX)
        if dataw != 10:
            print("dataw =", dataw)
            raise("writing value: writing to \"write address\" on device failed")

    def writeNext(self, val): # writes a single 32bit value at the current memory address.
        writeTX = 0x55 * 16**10 + val * 16**2 + self.writeNextCom
        writeTX = bytearray.fromhex(hex(writeTX)[2:])
        writeTX.reverse()
        dataw = self.dev.write(writeTX)
        if dataw != 6:
            print("dataw =", dataw)
            raise("writing value: writing to \"write address\" on device failed")

    def writeFloat64(self, addr, val):
        val = self.float64ToHex(float(val))
        val = val[2:-1] if val[-1] == 'L' else val[2:]
        val = val.zfill(16)
        val = bytearray.fromhex(val)
        self.writeToDev4B(addr, int(val[4:8].hex(),16))
        self.writeNext(int(val[:4].hex(),16))

    def writeFloat32(self,val): #needed for writing float32 velocity limit value to device(trajectory_make)    
        val = self.float32Tohex(float(val))
        val = val[2:-1] if val[-1] == 'L' else val[2:]   
        #val.zfill(8)  
        val = bytearray.fromhex(val)    
        self.writeNext(int(val.hex(),16)) 

    def dsrRead(self, axis):
        readAddr = self.dsrAddress(axis)
        dsrInSteps = self.readFromDev4B(readAddr)
        return self.stepsToNM(dsrInSteps)

    '''
    def sgRead(self, axis): # sensor gain
        if axis == 1:
            sg = self.readFromDev4B(self.axis1SensorGainAddr)
        elif axis == 2:
            sg = self.readFromDev4B(self.axis2SensorGainAddr)
        return sg
    '''



    def stepsToNM(self, steps):
        return steps / self.sensorGain

    def nmToSteps(self, nm):
        return nm * self.sensorGain

    def timeToCounts(self,dtime) #dwelltime unit is seconds
        return dtime / 0.000024

    def pRead(self, axis):
        addr = self.pAddress(axis)
        retVal = self.readArray(2, addr)
        retVal = self.hexToFloat64(retVal)
        return retVal

    def iRead(self, axis):
        addr = self.iAddress(axis)
        return self.hexToFloat64(self.readArray(2, addr))

    def dRead(self, axis):
        addr = self.dAddress(axis)
        return self.hexToFloat64(self.readArray(2, addr))

    def pidRead(self, axis):
        p = self.pRead(axis)
        i = self.iRead(axis)
        d = self.dRead(axis)
        return (p,i,d)

    def sRead(self, axis):
        #reads servo state: return value of 0 means off, 1 means on
        addr = self.sAddress(axis)
        return self.readFromDev4B(addr)

    def pWrite(self, axis, p):
        addr = self.pAddress(axis)
        self.writeFloat64(addr, p)

    def iWrite(self, axis, i):
        addr = self.iAddress(axis)
        self.writeFloat64(addr, i)

    def dWrite(self, axis, d):
        addr = self.dAddress(axis)
        self.writeFloat64(addr, d)

    def pidWrite(self, axis, pid):
        self.curPID = ((pid[0],pid[1],pid[2]), self.curPID[1]) if axis == 1 else (self.curPID[0], (pid[0],pid[1],pid[2]))
        self.pWrite(axis, pid[0])
        self.iWrite(axis, pid[1])
        self.dWrite(axis, pid[2])

    def sWrite(self, axis, state):
        #writes servo states: 0 for off, 1 for on
        if state != 0 and state != 1:
            raise("invalid state requested. valid states are 0 for open and 1 for closed.")
        addr = self.sAddress(axis)
        self.writeToDev4B(addr, state)

    def setZero(self, axis):
        #sets current position to zero
        writeTX = 10**(axis-1)
        writeTX = str(writeTX).zfill(32)
        self.writeToDev4B(self.zeroPosAddr, int(writeTX, 2))

    def resetInterferometer(self):
        # turn servo off, set current position to zero, then turn servo back on and go back to previous position
        pos1, pos2 = self.dsrRead(1), self.dsrRead(2)
        self.moveTo(1,0) # set axis 1 position to zero
        self.moveTo(2,0) # set axis 2 position to zero
        self.sWrite(1, 0) # turn servo off on axis 1
        self.sWrite(2, 0) # turn servo off on axis 2
        time.sleep(1) # let system relax for 2 seconds
        offset1 = self.dsrRead(1) # this is the offset due to resetting interferometer
        offset2 = self.dsrRead(2)
        self.setZero(1) # redefine current position as zero
        self.setZero(2)
        self.sWrite(1, 1) # turn servo back on
        self.sWrite(2, 1)
        time.sleep(0.2)
        self.moveTo(1, pos1 - offset1)
        self.moveTo(2, pos2 - offset2)

    def disToPID(self, axis, distance):
        if axis == 1:
            if 0 <= distance < 300:
                pid = (0, 180, 0)
            elif 300 <= distance < 3e3:
                pid = (0.01, 170, 0)
            elif 3e3 <= distance < 5e3:
                pid = (-0.01, 150, 0)
            elif 5e3 <= distance < 10e3:
                pid = (-0.01, 120, 0)
            elif 10e3 <= distance < 15e3:
                pid = (0, 90, 0)
            elif 15e3 <= distance < 20e3:
                pid = (0, 70, 0)
            elif 70e3 <= distance < 80e3:
                pid = (0, 115, 0)
            elif 80e3 <= distance < 95e3:
                pid = (0, 100, 0)
            elif 95e3 <= distance:
                pid = (-0.13, 67, 0)
        elif axis == 2:
            if 0 <= distance < 500:
                pid = (0, 240, 0)
            elif 500 <= distance < 5e3:
                pid = (0, 210, 0)
            elif 5e3 <= distance < 10e3:
                pid = (0, 200, 0)
            elif 5e3 <= distance < 30e3:
                pid = (0, 180, 0)
            elif 30e3 <= distance < 60e3:
                pid = (0, 120, 0)
            elif 60e3 <= distance < 70e3:
                pid = (0, 100, 0)
            elif 70e3 <= distance < 80e3:
                pid = (0, 87, 0)
            elif 80e3 <= distance < 90e3:
                pid = (0, 77, 0)
            elif 90e3 <= distance:
                pid = (0, 67, 0)
        else:
            raise('invalid axis:' + str(axis))
        return pid
