#!/usr/bin/env python3

# ============= References =============

# [ Serial Implementations of MSP ]
# https://github.com/alduxvm/pyMultiWii/blob/master/pymultiwii/__init__.py
# https://github.com/alduxvm/pyMultiWii/blob/master/demo/show-imu.py
# https://github.com/alduxvm/pyMultiWii/blob/master/demo/show-attitude.py

# [ Drona Aviation MSP Specification ]
# https://docs.google.com/document/d/1c2tjbeAuTYk3JZrkazImayqjCKx9w3rND4RTN41ol6U/edit#

# [ Struct Format Characters ]
# https://docs.python.org/3/library/struct.html#format-characters

# =======================================

import telnetlib
import re
import struct
import time

def rollleft():
    
        k=0
        startime=time.time()
        print("rl")
        increment=0
        while (time.time()-startime<0.8):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1800,1500,(1650),1500,2000,1500,1500,1500)) #arm command
            time.sleep(0.1)
            '''if (time.time() - startime) > 1:
                k+=increment
                time.sleep(0.05)
            if (k>200):
                increment = -5
            if (k<-200):
                increment = 5'''
            k+=20    
            print(k)

def rollright():
        k=1500
        p=0
        startime=time.time()
        increment=-5
        print("rr")
        while (time.time()-startime<0.75):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1300,1500,(1650),1500,2000,1500,1500,1500)) #arm command
            time.sleep(0.05)
            print(k)
            ''' if (time.time() - startime) > 1.5:
                print(k)
                k+=5
                time.sleep(0.05)
            if (k>1600):
                while(k>1300):
                    print(k)
                    c.send(*MSP.SET_RAW_RC(k,1500,(1500),1500,2000,1500,1500,1500)) #arm command
                    time.sleep(0.05)
                    k-=5
                    time.sleep(0.05)
                break
        print(k)'''

def yawleft():
        k=0
        startime=time.time()
        increment=0
        while (time.time()-startime<0.02):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1500,(1500),1300+k,2000,1500,1500,1500)) #arm command
            time.sleep(0.05)
            if (time.time() - startime) > 1:
                k+=increment
            if (k>200):
                increment = -5
            if (k<-200):
                increment = 5
            print(k)
            
def yawright():
        k=0
        p=0
        startime=time.time()
        increment=0
        print("yaw")
        while (p<400):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1500,(1750),1700,2000,1500,1500,1500)) #arm command
            time.sleep(0.1)
            print(p)
            p+=10
            if (time.time() - startime) > 0.25:
                k+=increment
            if (k>200):
                increment = -5
            if (k<-200):
                increment = 5
            print(k)

def pitchforward():
        k=0
        startime=time.time()
        increment=0
        while (time.time()-startime<2):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1600,(1650),1500,2000,1500,1500,1500)) #arm command
            time.sleep(0.05)
            if (time.time() - startime) > 1:
                k+=increment
            if (k>200):
                increment = -5
            if (k<-200):
                increment = 5
            print(k)   

def pitchback():
        k=0
        startime=time.time()
        increment=0
        print("pitchback")
        while (time.time()-startime<1):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1200,(1650),1500,2000,1500,1500,1500)) #arm command
            time.sleep(0.05)
            if (time.time() - startime) > 1:
                k+=increment
            if (k>200):
                increment = -5
            if (k<-200):
                increment = 5
            print(k)  

def throttleup():
        k=0
        startime=time.time()
        increment=0
        print("throttleup")
        while (k<150):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1500,(1700),1500,1400,1500,1500,1500)) #arm command
            #1750 throttle
            time.sleep(0.1)
            k+=10
            print(k)
            

def hover():
        k=1800
        startime=time.time()
        print("hover")
        increment=0
        while (k>1500):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1500,(1700),1500,2000,1500,1500,1500)) #arm command
            time.sleep(0.1)
            k-=10


def throttledown():
        k=0
        startime=time.time()
        increment=0
        while (time.time()-startime<1):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1500,(1500+k),1500,2000,1500,1500,1500)) #arm command
            time.sleep(0.05)
            if (time.time() - startime) > 1:
                k+=increment
            if (k>200):
                increment = -5
            if (k<-200):
                increment = 5
          

def landing():
        k=0
        startime=time.time()
        print("land")
        increment=0
        while (k<100):
            #150
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
            c.send(*MSP.SET_RAW_RC(1500,1500,(1650-k),1500,2000,1500,1100,1500)) #arm command
            time.sleep(0.05)
            k-=10
            #15
            
        c.send(*MSP.SET_COMMAND(2))
      #check if throttle shd hv a value 950 or 0 is fine
       # c.send(*MSP.SET_RAW_RC(1500,1500,0,1500,2000,1500,1100,1100))
        
def concatByteArrs(*arrs):
  res = b''
  for i in arrs:
    res += i
  
  return res

class Connection:
  def __init__(self, hostip = None, port = 23):
    self.HOSTIP = hostip
    self.HOSTPORT = port

    self.__connection = None
    self.connect(hostip, port)

  def __isValidIP(self, ipaddr):
    if type(ipaddr) is not str:
      raise TypeError("Invalid HOSTIP")

    pattern = r"^((25[0-5]|(2[0-4]|1\d|[1-9]|)\d)\.?\b){4}$"

    isValid = bool(re.search(pattern, ipaddr))

    return isValid
  
  def connect(self, hostip, port):
    if self.__connection is not None:
      self.close()

    if not self.__isValidIP(hostip):
      raise ValueError("Invalid HOSTIP")

    try:
      self.__connection = telnetlib.Telnet(hostip, port, 1)
    except Exception as err:
      print("Couldn't connect to host.", err)
      self.__connection = None

    return self

  def __getChecksum(self, *args):
    byteArr = concatByteArrs(*args)

    checksum = 0
    for byte in byteArr:
      checksum ^= byte
    
    return struct.pack("<B", checksum)
  
  def createDataPacket(self, direction, msgLength, payloadType, payloadFormat, payload):
    HEADER = struct.pack("<2c", b'$', b'M')
    DIRECTION = struct.pack("<c", direction.encode('utf-8'))
    MSG_LENGTH = struct.pack("<B", msgLength)
    PAYLOAD_TYPE = struct.pack("<c", payloadType)
    PAYLOAD = struct.pack(payloadFormat, *payload)
    CHECKSUM = self.__getChecksum(MSG_LENGTH, PAYLOAD_TYPE, PAYLOAD)

    return concatByteArrs(HEADER, DIRECTION, MSG_LENGTH, PAYLOAD_TYPE, PAYLOAD, CHECKSUM)

  def send(self, direction, msgLength, payloadType, payloadFormat, payload):
    if self.__connection is None:
      raise Exception("Attempting to send data packet over broken connection.")
    
    dataPacket = self.createDataPacket(direction, msgLength, payloadType, payloadFormat, payload)
    try:
      self.__connection.write(dataPacket)
    except Exception as err:
      print("Error while sending data packet to host.", err)

  def read(self):
    res = b''
    while True:
      try:
        res += self.__connection.read_eager()

        if not self.__connection.sock_avail():
          break
      except EOFError:
        break

    return res
  
  def test(self):
    test_msg = b"+++AT"

    try:
      print("Sent:", test_msg)
      self.__connection.write(test_msg)
      print("Received:", self.__connection.read_until(b"\r\n"))
    except:
      raise Exception("Test Failed")

  def close(self):
    if self.__connection is not None:
      self.__connection.close()

    self.__connection = None
  
  def restart(self):
    if self.__connection is not None:
      self.__connection.close()

    self.connect(self.HOSTIP, self.HOSTPORT)

class MSPWrapper:
  def __init__(self):
    self.MSP_SET_RAW_RC = ('<', 16, b'\xc8', '<8H')
    self.MSP_SET_COMMAND = ('<', 2, b'\xd9', '<H')

  def SET_RAW_RC(self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4):    
    return (*self.MSP_SET_RAW_RC, [roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4])

  def SET_COMMAND(self, command):
    return (*self.MSP_SET_COMMAND, [command])
  
MSP = MSPWrapper()

c = Connection('192.168.4.1', 23) # Pluto IP and PORT(default 23)
# time.sleep(2)
# disarm it
c.send(*MSP.SET_RAW_RC(1500,1500,1500,1500,900,900,900,900))
time.sleep(2)
#arm it
c.send(*MSP.SET_RAW_RC(1500,1500,1500,1500,1400,1500,1300,1500))
time.sleep(0.1)
c.send(*MSP.SET_COMMAND(1)) #Take off Command
time.sleep(0.1)
# c.send(*MSP.SET_RAW_RC(1500,1500,1600,1500,1400,1500,1500,1500))
# time.sleep(0.1)

# c.send(*MSP.SET_RAW_RC(1500,1500,1700,1500,1400,1500,1500,1500))
# time.sleep(0.1)

'''
yawleft()
yawright()
pitchback()
pitchforward()
rollleft()
rollright()
throttleup()
throttledown()
landing()
'''

'''while (True):
   #self, roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
   c.send(*MSP.SET_RAW_RC(1500,1500,(1500+k),1500,2000,1500,1500,1500)) #arm command
   time.sleep(0.05)
   if (time.time() - startime) > 2:
    k+=increment
   if (k>200):
    increment = -5
   if (k<-200):
    increment = 5
   print(k)'''
   
#tsk2
throttleup()
time.sleep(0.1)
landing()
'''pitchforward()
time.sleep(0.1)

# hover()
# time.sleep(0.1)

rollright()
time.sleep(0.1)

pitchback()
time.sleep(0.1)

rollleft()
time.sleep(0.1)
'''
'''pitchback()
time.sleep(0.1)
rollleft()
time.sleep(0.1)'''





'''TAKE OFF (for 2 secs)
time.sleep(5)'''


time.sleep(0.05)
time.sleep(0.05)
# disarm
c.send(*MSP.SET_RAW_RC(1500,1500,1500,1500,900,900,900,900))

c.close()
