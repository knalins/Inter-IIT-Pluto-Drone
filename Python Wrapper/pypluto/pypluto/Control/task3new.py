import multiprocessing
from pypluto.drone import pluto
import numpy as np
import time
import threading

target_array = [
    [375, 162], ## 1 corner
    [538, 160],
    [652, 158],
    [788, 152],
    [882, 175], ## 2 corner
    [920, 240],
    [922, 332],
    [886, 402], ## 3 corner
    [776, 422],
    [632, 428],
    [486, 432],
    [402, 412], ## 4 corner
    [365, 355],
    [364, 248],
    [375, 162], ## 1 corner
]

hover={
    '0': False,
    '8': True
}
compliment_dict = {
    '0': '8' ,
    '8' : '0'
}

KPx, KPy, KPz, KPyaw = 0.8, 0.4, 380 , 50
KIx, KIy, KIz, KIyaw = 0.02, 0.01, 0, 0
KDx, KDy, KDz, KDyaw = 18, 25, 15, 0
#currently global , 
#for deciding drone's final yaw orientation 
YAW_TARGET = 1.5708

POSE_DICT = {'8':[630., 429.,0.97107447 ,  -2.2655346], '0':[806., 385.,   0.90877014,   0.83298127]}
class PID:

    def __init__(self, Drone, target):
        self.x_error, self.y_error, self.z_error, self.yaw_error = 0, 0, 0, 0
        self.x_errorI, self.y_errorI, self.z_errorI, self.yaw_errorI = 0, 0, 0, 0
        self.x_errorD, self.y_errorD, self.z_errorD, self.yaw_errorD = 0, 0, 0, 0
        self.x_error_old, self.y_error_old, self.z_error_old, self.yaw_error_old = 0, 0, 0, 0

        self.Err = [self.x_error, self.y_error, self.z_error, self.yaw_error]
        self.ErrI = [self.x_errorI, self.y_errorI, self.z_errorI, self.yaw_errorI]
        self.drone = Drone
        self.target = target
        self.pose_dict = {'8':[630., 429.,0.97107447 ,  -2.2655346], '0':[806., 385.,   0.90877014,   0.83298127]}
        self.targets = [target_array[target][0],target_array[target][1], 0.8]

    def pid(self,pose, target, Err, ErrI):
        """
        PID Control Loop
        """
        x_error, y_error, z_error, yaw_error = Err
        x_errorI, y_errorI, z_errorI, yaw_errorI = ErrI

        x_target, y_target, height_target = target



        x_error_old = x_error
        y_error_old = y_error
        z_error_old = z_error
        yaw_error_old = yaw_error

        x,   y,   z,   yaw = pose

        x_error = x_target-x
        y_error = y_target-y
        z_error = height_target-z
        yaw_error = YAW_TARGET-yaw


        x_errorI += x_error
        y_errorI += y_error
        z_errorI += z_error
        yaw_errorI += yaw_error

        # compute derivative (variation) of errors (D)
        x_errorD = x_error-x_error_old
        y_errorD = y_error-y_error_old
        z_errorD = z_error-z_error_old
        yaw_errorD = yaw_error-yaw_error_old

        # compute commands
        xCommand = KPx*x_error + KIx*x_errorI + KDx*x_errorD
        yCommand = KPy*y_error + KIy*y_errorI + KDy*y_errorD
        zCommand = KPz*z_error + KIz*z_errorI + KDz*z_errorD

        yaw_command    = int( KPyaw*yaw_error + KIyaw*yaw_errorI + KDyaw*yaw_errorD)
        pitch_command = int( np.cos(yaw)*xCommand + np.sin(yaw)*yCommand)
        roll_command  = int( -np.sin(yaw)*xCommand + np.cos(yaw)*yCommand ) 
        throttle_command = int(zCommand)
        Err = [x_error, y_error, z_error, yaw_error]
        ErrI = [x_errorI, y_errorI, z_errorI, yaw_errorI]

        return roll_command, pitch_command, throttle_command, yaw_command, Err, ErrI

    def DronePID(self, DroneID, conn):
        global POSE_DICT

        
        # self.drone.throttle_speed(300,2)
        self.drone.takeoff()

        start = time.time()
        tReachCount=0
        corners = [4, 7,11,0]
        Err = self.Err
        ErrI = self.ErrI
        # global POSE_DICT
        pose = self.pose_dict[DroneID]
        while True:
            # pose_dict = pose
            print(self.pose_dict)
            # self.pose_dict = POSE_DICT
            # self.pose_dict = 
            # print(f"{DroneID} , posedict : {self.pose_dict} ")
            # pose = None

            if conn.poll():                
                self.pose_dict = conn.recv()
            
            try:
                # print(f"--------{DroneID}")


                if not self.pose_dict:
                
                    now_time = time.time()
                    timeout_limit = now_time - start 

                    roll_command, pitch_command, throttle_command, yaw_command = 0, 0, 50, 0

                    if timeout_limit > 8 : 
                        print("Aruco not detected ,landing")
                        self.drone.land()
                        break

                elif (not self.pose_dict)==False:
                # if drone==Drone:
                    try:
                        pose=self.pose_dict[DroneID]
                    except KeyError:
                        print(f"No pose {DroneID}")
                        pass
                



                    start = time.time()
                    roll_command, pitch_command, throttle_command, yaw_command, Err, ErrI = self.pid(pose, self.targets, Err, ErrI)

                    if (roll_command>70):
                        roll_command=70
                    elif (roll_command<-70):
                        roll_command=-70
                    if (pitch_command>35):
                        pitch_command=35
                    elif (pitch_command<-35):
                        pitch_command=-35




                    # print(f"Movingggg {DroneID, self.targets}")
                    if self.target in corners:
                        
                        # print("cornered")
                        # hover[DroneID] = True

                        # hover[compliment_dict[DroneID]] = False
                        self.targets = [target_array[self.target%15][0], target_array[self.target%15][1], 0.8]
                        # print(f"hoveringgggg {DroneID, self.targets}")

                    if np.sqrt((self.targets[0]-pose[0])**2+(self.targets[1]-pose[1])**2)<25:
                    # print(f"Pose: {pose}")
                        # print("Target Reached.")
                        self.drone.reset_speed()
                        tReachCount += 1
                        if tReachCount>=3:
                            tReachCount=0
                            # self.target += 1

                                # for hovering the drone
                                # while hover[DroneID]:
                                #     pose_dict = POSE_DICT
                                #     # try:
                                #     if not pose_dict:
            
                                #         now_time = time.time()
                                #         timeout_limit = now_time - start 
                                #         roll_command, pitch_command, throttle_command, yaw_command = 0, 0, 50, 0
                                #         if timeout_limit > 8 : 
                                #             print("Aruco not detected ,landing")
                                #             self.drone.land()
                                #             break
                                #         elif (not pose_dict)==False:
                                #         # if drone==Drone:
                                #             try:
                                #                 pose=pose_dict[DroneID]
                                #             except KeyError:
                                #                 pass
                                            
                                #             start = time.time()

                                #     roll_command, pitch_command, throttle_command, yaw_command = self.pid(pose, self.targets, self.Err, self.ErrI)
                                #     # self.drone.set_all_speed(roll_command,pitch_command,throttle_command,yaw_command)



                            self.targets = [target_array[self.target%15][0], target_array[self.target%15][1], 0.8]

                
                print(f"sending--------{roll_command, pitch_command, throttle_command, yaw_command} {DroneID}")

                self.drone.set_all_speed(roll_command, pitch_command, throttle_command, yaw_command)
                time.sleep(0.04)

            except KeyboardInterrupt:
                break
        
        self.drone.land()
        self.drone.disarm()


def getpose(conn):
    global POSE_DICT
    # start = time.time()
    while True:
        # time.sleep(2)
        try:
            pose_dict = {}
            # pose = None

            # print("ddddddddddddddddddddddd",conn.qsize())
            if conn.poll():                
                pose_dict = conn.recv()
                POSE_DICT = pose_dict
                # print(POSE_DICT)
            
            # POSE_DICT = pose_dict

        except KeyboardInterrupt:
            break


# def PID_main(conn,conn2):

    # Drone1=pluto("192.168.4.1")
    # # Drone2=pluto("10.42.0.61")
    # # Drone2=pluto("192.168.4.1")
    # Drone1.connect()
    # # Drone2.connect()

    # # # Drone1.trim(-5,-5,0,0)
    # # # Drone2.trim(10,5,0,0)
    # Drone1.disarm()
    # # Drone2.disarm()

    # pid_drone1 = PID(Drone1,4)
    # # pid_drone2 = PID(Drone2,0)

    # # poseThread = threading.Thread(target=getpose, args=(conn,conn2))
    # drone_pid1 = threading.Thread(target=pid_drone1.DronePID, args=('0', conn))
    # # drone_pid2 = threading.Thread(target=pid_drone2.DronePID, args=('8', conn2))
    # # poseThread.start()
    # # time.sleep(0.5)
    
    # # drone_pid2.start()
    # time.sleep(0.5)
    # drone_pid1.start()

    # drone_pid1.join()
    # # drone_pid2.join()