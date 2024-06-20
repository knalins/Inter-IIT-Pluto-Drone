from multiprocessing import Pipe
from pypluto.drone import pluto
import numpy as np
import time
import threading 


# poseDrone2 = None 
# poseDrone1 = None 


# POSE1DICT = {
#     '0' : poseDrone1,
#     '8' : poseDrone2

# }

# POSE2DICT = {
#     '0' : poseDrone1,
#     '8' : poseDrone2

# }


     


# --------------------------------------------

# xTarget,  yTarget, heightTarget = 646, 348, 0.8 # for hover at point

# target_array2=[]
# target_array2.append(target_array[0])
# x_target,  y_target, height_target = target_array[0][0],target_array[0][1], 0.8  #pixel, pixel , height(m)
YAW_TARGET = 1.5708

# --------
#pid gains

KPx, KPy, KPz, KPyaw = 0.8, 0.4, 380 , 50
KIx, KIy, KIz, KIyaw = 0.02, 0.01, 0, 0
KDx, KDy, KDz, KDyaw = 18, 25, 10, 0



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


        #For Recieving Pose
        self.conn = conn 


        self.target_array = [
            [   [375, 162],  #0
                [538, 160],  
                [652, 158],
                [788, 152],
                [882, 175] #4
            ],    
            [   
                [882, 175],
                [920, 240], 
                [922, 332],
                [886, 402]  #
            ],
            [
                [886, 402],
                [776, 422],
                [632, 428],
                [486, 432],
                [402, 412]#
            ],
            [ 
                [402, 412],
                [365, 355],
                [364, 248],
                [375, 162]#
            ]
        ]
        self.Drone1=pluto(DroneIP='10.42.0.74')
        self.Drone1.connect()
        self.Drone1.disarm()

        self.Drone2=pluto(DroneIP='10.42.0.61')
        self.Drone2.connect()
        self.Drone2.disarm()

    
        self.compliDict = {
            '0' : '8',
            '8' : '0'
        }
        self.START_OUTER_IDX = {
            '0' : 0,
            '8' : 3 # as this corresponds to 2 subarray, and is given to IDX
        }
        self.DRONEOBJDICT = {
            '0' : self.Drone1 ,
            '8' : self.Drone2
        }
        self.Drone1ID = '8'
        self.Drone2ID = '0'


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
        self.YAW_TARGET = 1.5708  # we can change the yaw for each here YAW_TARGET1 & YAW_TARGET2
         
    def recv_pose(self):

        '''changes the var of poses'''

        print("\nListerning to pose in a sec")
        time.sleep(0.5)

        #we need to make task not done as False somewhere 
        while ( self.Task_not_done ):
            if (self.conn.poll()):                
                pose_dict = self.conn.recv()
                if not (pose_dict ):
                    pass
                else:
                    self.POSEDICT[self.Drone1ID] = pose_dict[self.Drone1ID]
                    self.POSEDICT[self.Drone2ID] = pose_dict[self.Drone2ID]
            else: 
                self.POSEDICT[self.Drone1ID] = None
                self.POSEDICT[self.Drone2ID] = None

            if( self.Rectangle1_done and self.Rectangle2_done ):
                self.Task_not_done = False

        print("\n-|-$--Both Rectangles Completed, closing recv_pose--|-")
        
    def move( self, IDX , DroneID , Err , ErrI):
        
        '''
        temp target : list of 4-5 targets,whose endpt is nxt corner 
        DroneID : '0' or '8' '''

        roll_log = []
        pitch_log = []


        #Local vars
        temp_targets = self.target_array[IDX]
        Droneobj = self.DRONEOBJDICT[DroneID]
        pose = self.POSEDICT[DroneID]


        #locals
        xTarget,  yTarget, heightTarget = temp_targets[0][0] , temp_targets[0][1], 0.8  


        #
        tReachCount=0
        temp_target_idx = 0 

        start = time.time()
        while temp_target_idx != len(temp_targets)  : #loop till idx just greater than last ele


            #to change target within temp array ( one corner to other)
            xTarget,  yTarget, heightTarget = temp_targets[0][0] , temp_targets[0][1], 0.8  


            pose = self.POSEDICT[DroneID]
            

            try:
                    
                if not pose : # pose is None
                    
                    now_time = time.time()
                    time_detected = now_time - start 

                    roll_command, pitch_command, throttle_command, yaw_command = 0, 0, 50, 0

                    if time_detected> 8 : 
                        print(f"\nAruco not detected ,landing : {DroneID}\n")
                        Droneobj.land()
                        return 'not det'
                        
                    
                elif (not pose)==False: #Pose not None
                    

                    start = time.time()
                    
                    #Target Vicinity counter block
                    if np.sqrt((xTarget-pose[0])**2+(yTarget-pose[1])**2)<25:
                        # print(f"Pose: {pose}")
                        tReachCount += 1

                        Droneobj.reset_speed() #why??? to avoid overshooting after targets


                        # corner target reached  , then idx+1   
                        if ( IDX < (self.START_OUTER_IDX[DroneID]-1)%4  and temp_target_idx == len(temp_targets)-1 and tReachCount >= 5):
                            print(f"\nReached Target Corner {temp_target_idx}, ;{DroneID}\n")
                            temp_target_idx +=1
                            break 

                        #On the way targets reached , then idx+1 and reset tReachCounter
                        elif (temp_target_idx < len(temp_targets)-1 and tReachCount>= 5 ): 
                            print(f"\nOn the way Target Reached. : {DroneID} , Target : {IDX,temp_target_idx}\n")
                            temp_target_idx +=1
                            tReachCount=0


                        #Last target in last array , just land & break
                        if IDX == (self.START_OUTER_IDX[DroneID]-1)%4 and temp_target_idx == len(temp_targets)-1 and tReachCount >=10 : 
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
                print(f"\n(sending) move , rcmd: {roll_command}, pcmd: {pitch_command}, tcmd:{throttle_command},  yawcmd:{yaw_command}")

            except KeyboardInterrupt:
                Droneobj.disarm()
                print(f"Disarming Drone {DroneID}")
                time.time(2)
                return 'interupt'
 
    def hover (self, target_coords,DroneID):


        #locals 
        xTarget , yTarget ,heightTarget = target_coords[0], target_coords[1], 0.8
        Droneobj = self.DRONEOBJDICT[DroneID]
        pose = self.POSEDICT[DroneID]   
        move_state =  self.MOVEDICT[DroneID]
        start = time.time()
        while (not move_state):

            pose = self.POSEDICT[DroneID]   
            move_state =  self.MOVEDICT[DroneID]

            try:  
                

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
                    
                    print(f"\nHovering {DroneID} at {xTarget,yTarget}")
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
                print(f"\nHover :  rcmd: {roll_command}, pcmd: {pitch_command}, tcmd:{throttle_command},  yawcmd:{yaw_command}")

            except KeyboardInterrupt:
                Droneobj.disarm()
                print(f"Disarming Drone {DroneID}")

                return "interupt"

    def dronePlan(self,DroneID):
        

        print(f"\n-----Starting Drone {DroneID} plan--------")
        
        

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

        DroneObj.throttle_speed(300,3)
        # DroneObj.takeoff()
        print(f"Takeoff: {DroneID}")


        # default values when not detected
        roll_command, pitch_command, throttle_command, yaw_command = 0, 0, 50, 0

        start = time.time()
        
        Rectangle_done = self.RECT_DONE_DICT[DroneID]
        IDX = self.START_OUTER_IDX[DroneID] # goes from 0 to 4
    
        '''need to tell when drone ids rect is done'''
        while not Rectangle_done :
            
            move_state = self.MOVEDICT[DroneID]
            

            if move_state :  #true at very first for drone1 and False for drone2
                resp = self.move(IDX= IDX, DroneID=DroneID,Err=Err,ErrI=ErrI  )
                #now current Drone  has reached its temp corner pt , so make itself hover and other move
                self.MOVEDICT[DroneID] = False
                self.MOVEDICT[self.compliDict[DroneID]] = True

                #change current drones outer idx to nxt, (and pass 1st element of this new sub array to hover)
                
                #last 
                if( IDX == (self.START_OUTER_IDX[DroneID]-1) %4 ):
                    Rectangle_done  = True
                    DroneObj.land()
                    print(f"Rectangle{DroneID} Done")
                    time.sleep(2)

                #shift to next set of temp array

                if(resp == 'stop' or 'not det'): 
                    print(f"\n\nresp (move): {resp}")
                    break #reached final 15th target

            else:                                               #last element of subarray
                resp = self.hover( self.target_array[IDX][ len(self.target_array[IDX])-1 ] , DroneID=DroneID)

                #nxt IDX to get nxt subarray to move (above in nxt loop) 
                IDX += 1 
                
                self.MOVEDICT[DroneID] = True
                self.MOVEDICT[self.compliDict[DroneID]] = False


                if (resp == "stop" or "not det" or 'interupt'):
                    print(f"\nresp (hover) : {resp}")
                    break # to break plan in case of Aruco not det
            
            Rectangle_done = self.RECT_DONE_DICT[DroneID]
   



def PID_main(conn):

       
    MP = MissionPlanner(conn=conn)

    t0 = threading.Thread( target= MP.recv_pose )

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