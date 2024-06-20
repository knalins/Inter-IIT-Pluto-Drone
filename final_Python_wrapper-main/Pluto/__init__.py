import time

from DroneConnector import Connector
from MSPHelper import MSP

class Pluto:
    def __init__(self, hostip=None, port=23):

        self.connector = Connector(hostip, port)
        self.DELAY_TIME = 0.1  # Delay time between each data packet

        self.ZERO_TIME = 0 #Default time if each command

    def arm(self):
        print("arming")
        self.connector.send(*MSP.SET_RAW_RC(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500), self.DELAY_TIME)

    def disarm(self):
        print("disarming")
        self.connector.send(*MSP.SET_RAW_RC(1500, 1500, 1500, 1500, 900, 900, 900, 900))

    def take_off(self):
        print("takeoff")
        self.connector.send(*MSP.SET_COMMAND(1), self.DELAY_TIME)
        self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1500),1500, 1500, 1500, 1800, 1500), self.DELAY_TIME)

    def move_up(self, k, t=None):

        print(f"move_up")
        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME):
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, k, 1500, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            return

        temp = 1500
        startTime = time.time()
        while time.time() - startTime < t:

            if(temp<k):# Gardually increasing a throtle from 1500 untill it reaches user input value
              temp+=100
            else:
              temp=k

            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (k), 1500, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            k += 1

    def hover(self, k, t=None):

        print(f"hover")
        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME): ## When no time argument is provided
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1500+k), 1500, 1500, 1500, 1500, 1500), self.DELAY_TIME)
            return

        startTime = time.time()
        while time.time() - startTime < t:
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (k), 1500, 1500, 1500, 1500, 1500), self.DELAY_TIME)

    def turn_left(self, k, t=None):
        print(f"turn_left")

        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME):
            self.connector.send(*MSP.SET_RAW_RC(k, 1500, (1500), 1500, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            return

        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(*MSP.SET_RAW_RC(k, 1500, 1500, 1300, 1500, 1500, 1500, 1500), self.DELAY_TIME)

    def turn_right(self, k, t=None):
        print(f"turn_right")

        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME):
            self.connector.send(*MSP.SET_RAW_RC(k, 1500, (1500), 1500, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            return

        k = 0
        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(*MSP.SET_RAW_RC(k, 1500, 1500, 1700, 1500, 1500, 1500, 1500), self.DELAY_TIME)

    def move_down(self,k, t=None):
        pass
        

    def move_forward(self, k, t=None):

        print(f"move_foward")
        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME):
            self.connector.send(*MSP.SET_RAW_RC(1500, k, (1500), 1500, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            return

        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(*MSP.SET_RAW_RC(1500, k, 1500, 1500, 1500, 1500, 1500, 1500), self.DELAY_TIME)


    def move_backward(self, k, t=None):

        print(f"move_backward")
        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME):
            self.connector.send(*MSP.SET_RAW_RC(1500, k, (1500), 1500, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            return

        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(*MSP.SET_RAW_RC(1500, k, 1500, 1500, 1500, 1500, 1500, 1500), self.DELAY_TIME)

    def yaw_left(self,k,t=None):

        print(f"yaw_left")
        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME):
            self.connector.send(*MSP.SET_COMMAND(1), self.DELAY_TIME)
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, 1500, k, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            return

        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, 1500, k , 1500, 1500, 1500, 1500), self.DELAY_TIME)

    def yaw_right(self,k,t=None):

        print(f"yaw_right")
        if t is None:
            t = self.ZERO_TIME

        if (t <= self.ZERO_TIME):
            self.connector.send(*MSP.SET_COMMAND(1), self.DELAY_TIME)
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1500), k, 1500, 1500, 1800, 1500), self.DELAY_TIME)
            return

        startTime = time.time()
        while time.time() - startTime < t:
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500 , 1500, k, 1500, 1500, 1500, 1500), self.DELAY_TIME)   


    def land(self):

        print(f"land")
        k = 0
        startTime = time.time()
        
        while time.time() - startTime < 3: #De-throttling for smooth landing
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1400-k),1500, 1500, 1500, 1500, 1500), self.DELAY_TIME)
            k = k-4

        self.connector.send(*MSP.SET_COMMAND(2), self.DELAY_TIME)

    def disconnect(self):
        self.disarm()
        self.connector.close()

    def reconnect(self):
        self.disarm()
        self.connector.restart()
        self.arm()


if __name__ == '__main__':
    quit()
