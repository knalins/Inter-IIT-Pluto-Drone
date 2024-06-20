#!/usr/bin/env python3

# telnetlib is the implimentation of telnet protocol and helps in communicating with the drone
import telnetlib
import time

from . import utils

# connector class impliments connect, send msg, disconnect, and reconnect methods
class Connector:
  def __init__(self, hostip = None, port = 23):
    self.HOSTIP = hostip
    self.HOSTPORT = port

    self.__connection = None
    self.connect(hostip, port)
  
  # connect method to extablish a telnet communication with the given drone's IP and Port 
  def connect(self, hostip, port):
    # check for an existing connection and close it if there is one
    if self.__connection is not None:
      self.close()

    # check if the ip is valid, throws an exception if not
    if not utils.isValidIP(hostip):
      raise ValueError("Invalid HOSTIP")

    # try for connecting with the ip, in case of error throws an exception
    try:
      self.__connection = telnetlib.Telnet(hostip, port, 1)
    except Exception as err:
      print("Couldn't connect to host.", err)
      self.__connection = None

    # return the communication channel 
    return self
  
  # func. to send MSPdatapackets through the telnet communication channel
  def send(self, direction, msgLength, payloadType, payloadFormat, payload, writeDelay = 0):
    # check if the there is a valid communication channel else throws an error  
    if self.__connection is None:
      raise Exception("Attempting to send data packet over broken connection.")

    # creating datapackets with the given parameters
    dataPacket = utils.createDataPacket(direction, msgLength, payloadType, payloadFormat, payload)
    
    # try to send datapacket with given time delay else throws an error
    try:
      self.__connection.write(dataPacket)
      time.sleep(writeDelay)
    except Exception as err:
      print("Error while sending data packet to host.", err)
  
  # close the connection 
  def close(self):
    if self.__connection is not None:
      self.__connection.close()

    self.__connection = None

  # reinitiate the telnet connection
  def restart(self):
    self.connect(self.HOSTIP, self.HOSTPORT)

if __name__ == '__main__':
  quit()
