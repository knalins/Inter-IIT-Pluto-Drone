#!/usr/bin/env python3

import time

from DroneConnector import Connector
from MSPHelper import MSP


class Pluto:

    def __init__(self, hostip=None, port=23):
        self.connector = Connector(hostip, port)
        self.DELAY_TIME = 0.2  # delay time between each command

        self.isFlying = False

        # self.MOVE_X = 0
        # self.MOVE_Y = 0
        self.MOVE_Z = 0
        self.TURN = 0

        self.MOVE_UP = 0
        self.MOVE_DOWN = 0
        self.MOVE_LEFT = 0
        self.MOVE_RIGHT = 0
        self.MOVE_FORWARD = 0
        self.MOVE_BACKWARD = 0
        self.ROTATE_CW = 0
        self.ROTATE_CCW = 0
        self.y=1800
        
    def readIMU(self):
        # print("IMU:", end=" ")
        self.connector.send(*MSP.RAW_IMU())
        time.sleep(0.3)
            

    def arm(self):
        print("arming")
        self.connector.send(*MSP.SET_RAW_RC(1500, 1500, 1500, 1500, 900, 900, 900, 900))

    def disarm(self):
        print("disarming")
        self.connector.send(*MSP.SET_RAW_RC(1500, 1500, 1500, 1500, 900, 900, 900, 900))


    def __updatePose(self, TIME):
        startTime = time.time()

        r = 0  # roll
        p = 0  # pitch
        t = 500  # throttle
        y = 0  # yaw

        while self.isFlying and time.time() - startTime < TIME:
            # t = self.MOVE_Z * 200
            # y = self.TURN * 150

            # while self.isFlying and time.time() - startTime < TIME:
            #   t += 2 * self.MOVE_Z

            #   if self.MOVE_LEFT:
            #     r -= 2
            #   if self.MOVE_RIGHT:
            #     r += 2

            #   if self.MOVE_FORWARD:
            #     p += 2
            #   if self.MOVE_BACKWARD:
            #     p -= 2

            #   y += 2 * self.TURN

            self.connector.send(*MSP.SET_RAW_RC(1500 + r, 1500 + p, 2000, 1500 + y, 1400, 1500, 1500, 1500),0.2,)

    def take_off(self):
        self.connector.send(*MSP.SET_COMMAND(1), 0.2) 
        # self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1500), 1500, 1500, 1500, 1800, 1500), 0.2)
        # self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1600), 1500, 1500, 1500, 1800, 1500), 0.2)
        

    def move_up(self, t):
        print(f"move_up({t})")
        # self.connector.send(*MSP.SET_RAW_RC(1500,1500,(1700),1500,1500,1500,1500,1500), 0)
        # time.sleep(0.2)

        k = 0
        # self.connector.send(*MSP.SET_COMMAND(1), 0.2)
        self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1500), 1500, 1500, 1500, 1800, 1500), 0.2)
        self.connector.send(*MSP.SET_RAW_RC(1500, 1500, (1600), 1500, 1500, 1500, 1800, 1500), 0.2)
        startTime = time.time()
        
        while time.time() - startTime < t:
            print(k)
            self.connector.send(*MSP.SET_RAW_RC(1500, 1500, 1800, 1500, 1500, 1500, 1800, 1500), 0.2)
            # self.connector.send(*MSP.SET_RAW_RC(1500, 1500, self.y, 1500, 1500, 1500, 1800, 1500), 0.2)
            k += 1

    # def hover(self, t):
    #     print(f"hover({t})")

    #     k = 0
    #     startTime = time.time()
    #     # self.connector.send(*MSP.SET_RAW_RC(1500,1500,(1600),1500,1500,1500,1500,1500), 0.2)
    #     while time.time() - startTime < t:
    #         print(k)
    #         self.connector.send(
    #             *MSP.SET_RAW_RC(1500, 1500, (1500), 1500, 1500, 1500, 1800, 1500), 0.2
    #         )
    #         k += 1
            
    def hover2(self,r,p,t):
        # print(f"{t} sent to fly")
        
        y=1600
        
  


        # kk=0
        # if(t==500):
            
        #     while kk<20:
        #         self.connector.send(
        #         *MSP.SET_RAW_RC(1500, 1500, (2100), 1500, 1500, 1500, 1900, 1500), 0.2
        #         )
        #         print("inside the loop")
        #         kk+=1

        self.connector.send(*MSP.SET_RAW_RC(1500+r, 1500+p, (1500+t), 1500, 1500, 1500, y, 1500), 0.2)
    
        print(f"throttle:{1500+t} roll:{1500+r} pitch:{1500+p} ")

    def turn_left(self, t):
        # FIXME test

        print(f"turn_left({t})")

        k = 0
        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            # self.connector.send(
            #     *MSP.SET_RAW_RC(1300, 1500, (self.y)1900/1800, 1500, 1500, 1500, 1500, 1500), 0.2
            # )
            self.connector.send(
                *MSP.SET_RAW_RC(1300, 1500, 1900, 1500, 1500, 1500, 1500, 1500), 0.2
            )
            k += 1

    def turn_right(self, t):
        # FIXME test

        print(f"turn_left({t})")

        k = 0
        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(
                *MSP.SET_RAW_RC(1700, 1500, 2100, 1500, 1500, 1500, 1800, 1500), 0.2
            )
            k += 1

        # k = 1800
        # self.connector.send(*MSP.SET_RAW_RC(1500,1500,(1500),1500,2000,1500,900,1500), 0.2)
        # while k > 1500:
        #   print(k)
        #   self.connector.send(*MSP.SET_RAW_RC(1500,1500,(1700),1500,2000,1500,900,1500), 0.2)
        #   self.connector.send(*MSP.SET_COMMAND(1), 0.2)
        #   k-=10
        # self.isFlying = True
        # self.connector.send(*MSP.SET_RAW_RC(1500,1500,1500,1500,1400,1500,1500,1500), 0.2)
        # self.connector.send(*MSP.SET_COMMAND(1), 5)
        # start = time.time()
        # while time.time() - start < 5:
        #   self.connector.send(*MSP.SET_RAW_RC(1500,1500,1600,1500,1400,1500,1500,1500), 0.2)

        # self.MOVE_Z = 1
        # self.__updatePose(t)
        # self.MOVE_Z = 0

    def move_down(self, t):
        pass
        # self.isFlying = True
        # self.connector.send(*MSP.SET_COMMAND(1), 0.2)

        # self.MOVE_Z = -1
        # self.__updatePose(t)
        # self.MOVE_Z = 0

    def pitchfor(self,t):
        k = 0
        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(
                *MSP.SET_RAW_RC(1500, 1700, 1900, 1500, 1500, 1500, 1500, 1500), 0.2
            )
            k += 1

    def pitchback(self,t):
        k = 0
        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(
                *MSP.SET_RAW_RC(1500, 1100, 2000, 1500, 1500, 1500, 1800, 1500), 0.2
            )
            k += 1
    def land(self):
        # FIXME have to enable smooth landing

        print("land")
        self.isFlying = False
        k, t = 0, 3
        startTime=time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(
                *MSP.SET_RAW_RC(1500, 1500, (1900-(50*k)), 1500, 1500, 1500, 1800, 1500), 0.2
            )
            k += 1
        self.connector.send(*MSP.SET_COMMAND(2), 0.2)
    
    def hover(self,t):
        k = 0
        startTime = time.time()
        while time.time() - startTime < t:
            print(k)
            self.connector.send(
                *MSP.SET_RAW_RC(1500, 1500, 1700, 1500, 1500, 1500, 1700, 1500), 0.2
            )
            k += 1

    def disconnect(self):
        self.disarm()
        self.connector.close()

    def reconnect(self):
        self.disarm()
        self.connector.restart()
        self.arm()


if __name__ == "__main__":
    quit()
