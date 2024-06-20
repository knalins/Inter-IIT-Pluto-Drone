#!/usr/bin/env python3

import telnetlib
import time

from . import utils

class Connector:
  def __init__(self, hostip = None, port = 23):
    self.HOSTIP = hostip
    self.HOSTPORT = port

    self.__connection = None
    self.connect(hostip, port)
  
  def connect(self, hostip, port):
    if self.__connection is not None:
      self.close()

    if not utils.isValidIP(hostip):
      raise ValueError("Invalid HOSTIP")

    try:
      self.__connection = telnetlib.Telnet(hostip, port, 1)
    except Exception as err:
      print("Couldn't connect to host.", err)
      self.__connection = None

    return self
  
  def send(self, direction, msgLength, payloadType, payloadFormat, payload, writeDelay = 0):
    if self.__connection is None:
      raise Exception("Attempting to send data packet over broken connection.")

    dataPacket = utils.createDataPacket(direction, msgLength, payloadType, payloadFormat, payload)
    
    try:
      self.__connection.write(dataPacket)
      time.sleep(writeDelay)
    except Exception as err:
      print("Error while sending data packet to host.", err)
  
  def close(self):
    if self.__connection is not None:
      self.__connection.close()

    self.__connection = None

  def restart(self):
    if self.__connection is not None:
      self.__connection.close()

    self.connect(self.HOSTIP, self.HOSTPORT)

  def read(self):
    res = b""
    while True:
        try:
            res += self.__connection.read_eager()

            if not self.__connection.sock_avail():
                break
        except EOFError:
            break

    return res


if __name__ == '__main__':
  quit()