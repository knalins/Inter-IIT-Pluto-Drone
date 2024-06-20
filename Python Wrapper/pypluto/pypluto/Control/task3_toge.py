from multiprocessing import Pipe
from pypluto.drone import pluto
import numpy as np
import time
import threading 




#To do 
# 1.what if one aruco not detected, how to make our code fault tolerant

class PID_Controller():
    def __init__(self) :
#we could use a class for pid
        pass
        


# --------------------------------------------

# xTarget,  yTarget, heightTarget = 646, 348, 0.8 # for hover at point

# target_array2=[]
# target_array2.append(target_array[0])
# x_target,  y_target, height_target = target_array[0][0],target_array[0][1], 0.8  #pixel, pixel , height(m)


def pid(pose, target, Err, ErrI):
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




class MissionPlanner():
   

    def __init__(self,conn) :
        #Target coords
        self.target_array = [
            [   [375, 162],  #0
                [538, 160],  
                [652, 158],
                [788, 152],
                [882, 175] #4
            ]    
            [   
                [882, 175] 
                [920, 240], 
                [922, 332],
                [886, 402]  #
            ]
            [
                [886, 402] 
                [776, 422],
                [632, 428],
                [486, 432],
                [402, 412]#
            ]

            [ 
                [402, 412]
                [365, 355],
                [364, 248],
                [375, 162]#
            ]
        ]
        
        #For Recieving Pose
        self.conn = conn 

        self.Drone1=pluto()
        # Drone1.connect()

        self.Drone2=pluto()
        # Drone2.connect()


        self.compliDict = {
            '0' : '8',
            '8' : '0'
        }
        self.STARTIDX = {
            '0' : 0,
            '8' : 4
        }
        self.DRONEOBJDICT = {
            '0' : self.Drone1 ,
            '8' : self.Drone2
        }
        self.Drone1ID = '0'
        self.Drone2ID = '8'


        self.moveDrone1 = True
        self.moveDrone2 = False

        self.MOVEDICT = {
            '0' : self.moveDrone1,
            '8' : self.moveDrone2,
        }

        self.poseDrone1 = None
        self.poseDrone2 = None

        self.POSEDICT = {
            '0' : self.poseDrone1,
            '8' : self.poseDrone2
        }


        self.Task_not_done = True
        self.Rectangle1_done = False
        self.Rectangle2_done = False 

        self.RECT_DONE_DICT = {
            '0' : self.Rectangle1_done ,
            '8' : self.Rectangle2_done     
        }

        self.target_idx1 = 0
        self.target_idx2 = 4

        self.TARGET_IDX_DICT = {
            '0' : self.target_idx1 ,
            '1' : self.target_idx2 
        }

        #currently global , 
        #for deciding drone's final yaw orientation 
        self.YAW_TARGET = 1.5708
         
    def recv_pose(self,conn):

        '''changes the var of poses'''

        #we need to make task not done as False somewhere
        while ( self.Task_not_done ):
            if (conn.poll()):                
                pose_dict = conn.recv()
                self.POSEDICT[self.Drone1ID] = pose_dict[self.Drone1ID]
                self.POSEDICT[self.Drone2ID] = pose_dict[self.Drone2ID]
            else: 
                self.POSEDICT[self.Drone1ID] = None
                self.POSEDICT[self.Drone2ID] = None

    
    def move( self, IDX , DroneID , Err , ErrI):
        
        '''
        temp target : list of 4-5 targets,whose endpt is nxt corner 
        DroneID : '0' or '8' '''

        roll_log = []
        pitch_log = []

        
        
        #globals
        # Drone1 , Drone1ID ,poseDrone1
        # global Drone2, Drone2ID,  poseDrone2
        # global target_array 

        #Local vars
        temp_targets = self.target_array[IDX]
        Droneobj = self.DRONEOBJDICT[DroneID]
        pose = self.POSEDICT[DroneID]


        #locals
        xTarget,  yTarget, heightTarget = temp_targets[0], 0.8  


        #
        tReachCount=0
        temp_target_idx = 0 

        while temp_target_idx != len(temp_targets)  : #loop till idx just greater than last ele


            #to change target within temp array ( one corner to other)
            xTarget,  yTarget, heightTarget = temp_targets[temp_target_idx], 0.8  


            pose = self.POSEDICT[DroneID]
            

            try:
                    
                if not pose : # pose is None
                    
                    now_time = time.time()
                    timeout_limit = now_time - start 

                    roll_command, pitch_command, throttle_command, yaw_command = 0, 0, 50, 0

                    if timeout_limit > 8 : 
                        print(f"\nAruco not detected ,landing : {DroneID}\n")
                        Droneobj.land()
                        return 'not det'
                        
                    
                elif (not pose)==False: #Pose not None
                    
                    start = time.time()
                    
                    #Target Vicinity counter block
                    if np.sqrt((xTarget-pose[0])**2+(yTarget-pose[1])**2)<25:
                        # print(f"Pose: {pose}")
                        tReachCount += 1

                        Droneobj.reset_speed() #why???


                        # corner target reached  , then idx+1   
                        if ( temp_target_idx == len(temp_targets)-1 and tReachCount >= 3):
                            print(f"\nReached Target Corner ;{DroneID}\n")
                            temp_target_idx +=1
                            
                            break 

                        #On the way targets reached , then idx+1 and reset tReachCounter
                        elif (temp_target_idx < len(temp_targets)-1 and tReachCount>= 5 ): 
                            print(f"\nOn the way Target Reached. : {DroneID} , Target : {IDX,temp_target_idx}\n")
                            temp_target_idx +=1
                            tReachCount=0


                        #Last target in last array , just land & break
                        if IDX == 3 and temp_target_idx == len(temp_targets)-1 and tReachCount >=5 : 
                            print(f"\nTask Completed\nLanding , {DroneID}")
                            Droneobj.land()
                            time.sleep(2)
                            return "stop"
                            



                    
                    roll_command, pitch_command, throttle_command, yaw_command, Err, ErrI = pid(pose, xTarget, yTarget, heightTarget, Err, ErrI)


                    #Clipping commands
                    if (roll_command>100):
                        roll_command=100
                    elif (roll_command<-100):
                        roll_command=-100
                    if (pitch_command>70):
                        pitch_command=70
                    elif (pitch_command<-70):
                        pitch_command=-70

    
                
                # Set all speeds at once
                Droneobj.set_all_speed(roll_command, pitch_command, throttle_command, yaw_command)
                time.sleep(0.04)
        
                '''this sleep adjusts the running of this files while loop, 
                so that the rate of receiving from marker files is almost matched 
                to that of this file sending commands to drone using api '''
                
                
                # if not roll_command==0:
                print(f"\nFrequecy checker(sending) , rcmd: {roll_command}, pcmd: {pitch_command}, tcmd:{throttle_command},  yawcmd:{yaw_command}")

            except KeyboardInterrupt:
                return 'interupt'
 

    def hover (self, target_coords,DroneID):

        

        #locals 
        xTarget , yTarget ,heightTarget = target_coords, 0.8
        Droneobj = self.DRONEOBJDICT[DroneID]
        pose = self.POSEDICT[DroneID]   
        move_state =  self.MOVEDICT[DroneID]

        while (not move_state):

            pose = self.POSEDICT[DroneID]   
            move_state =  self.MOVEDICT[DroneID]

            try:  
                
                move_state =  self.MOVEDICT[DroneID]

                if not pose : # pose is None
                    
                    now_time = time.time()
                    timeout_limit = now_time - start 

                    roll_command, pitch_command, throttle_command, yaw_command = 0, 0, 50, 0

                    if timeout_limit > 8 : 
                        print("Aruco not detected ,landing")
                        Droneobj.land()

                        return "Stop"
                    
                elif (not pose)==False: #Pose not None
                    
                    start = time.time()
                    roll_command, pitch_command, throttle_command, yaw_command, Err, ErrI = pid(pose, xTarget, yTarget, heightTarget, Err, ErrI)
                    
                    # Clipping commands
                    if (roll_command>100):
                        roll_command=100
                    elif (roll_command<-100):
                        roll_command=-100
                    if (pitch_command>70):
                        pitch_command=70
                    elif (pitch_command<-70):
                        pitch_command=-70

                    

                # Set all speeds at once
                Droneobj.set_all_speed(roll_command, pitch_command, throttle_command, yaw_command)
                time.sleep(0.04)
            
                '''this sleep adjusts the running of this files while loop, 
                so that the rate of receiving from marker files is almost matched 
                to that of this file sending commands to drone using api '''
                
                
                # if roll_command !=0:
                print(f"\nFrequecy checker(sending) , rcmd: {roll_command}, pcmd: {pitch_command}, tcmd:{throttle_command},  yawcmd:{yaw_command}")

            except KeyboardInterrupt:
                return "interupt"


    def dronePlan(self,DroneID):

        

        #local scope
        x_error, y_error, z_error, yaw_error = 0, 0, 0, 0
        x_errorI, y_errorI, z_errorI, yaw_errorI = 0, 0, 0, 0
        x_errorD, y_errorD, z_errorD, yaw_errorD = 0, 0, 0, 0
        x_error_old, y_error_old, z_error_old, yaw_error_old = 0, 0, 0, 0

        Err = [x_error, y_error, z_error, yaw_error]
        ErrI = [x_errorI, y_errorI, z_errorI, yaw_errorI]
        path = [[0,0,0,0]]



        DroneObj= self.DRONEOBJDICT[DroneID]
        move_state = self.MOVEDICT[DroneID]
        # # Drone1.connect()

        #Drone1.trim(-8,20,0,0) # iit
        DroneObj.trim(23, 5,0,0)#  ??????????????
        DroneObj.disarm()
    
        DroneObj.arm()
        # DroneObj.takeoff()
        print("takeoff")


        # default values when not detected
        roll_command, pitch_command, throttle_command, yaw_command = 0, 0, 50, 0

        start = time.time()
        

        IDX = self.STARTIDX[DroneID] # goes from 0 to 4

        #need to tell when drone ids rect is done
        while not self.Rectangle1_done :
            
            move_state = self.MOVEDICT[DroneID]

            if move_state :  #true at very first
                resp = self.move(IDX= IDX, Err=Err,ErrI=ErrI  )
                IDX += 1


                #now current Drone  has reached its temp corner pt , so make itself hover and other move
                self.MOVEDICT[DroneID]                  = False
                self.MOVEDICT[self.compliDict[DroneID]] = True

                #shift to next set of temp array

                if(resp == 'stop' or 'not det'): 
                    print(f"\n\nresp : {resp}")
                    break #reached final 15th target

            else:
                resp = self.hover( self.target_array[self.target_idx_generic] )
                
                if (resp == "stop" or "not det" or 'interupt'):
                    print(f"\n\nresp : {resp}")
                    break # to break plan in case of Aruco not det

   





def PID_main(conn):

       
    MP = MissionPlanner(conn)

    t0 = threading.Thread( target= MP.recv_pose , args=(conn,))
    t1 = threading.Thread( target= MP.dronePlan, args=(MP.Drone1ID))
    t2 = threading.Thread( target= MP.dronePlan, args=(MP.Drone2ID))

    print("\nListerning to pose in a sec")
    time.sleep(1)
    t0.start()
    time.sleep(2)
    t1.start()
    t2.start()



    t0.join()
    t1.join()
    t2.join()