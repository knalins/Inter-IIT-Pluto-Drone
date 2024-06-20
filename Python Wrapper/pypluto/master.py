import matplotlib
import multiprocessing
import threading
from multiprocessing import Pipe
from pypluto.drone import pluto
import time 
# from pypluto.Control.task3_toge import MissionPlanner, PID_main
# from pypluto.Control.task3new import PID_main
from pypluto.Control.task3new import PID
from pypluto.Camera.marker import marker_publisher





# builds necessary connections of drone(1,2,...) & the camera file
def build_conn():


    #creates connection btw marker1 file and drone1
    connCam,connDrone1 = Pipe(duplex = True)
    connCam2, connDrone2 = Pipe(duplex = True)
    # MP = MissionPlanner(connDrone1)
    # Drone1=pluto("192.168.4.1")
    Drone1=pluto("10.42.0.74")
    Drone2=pluto("10.42.0.61")
    # Drone2=pluto("192.168.4.1")
    Drone1.connect()
    Drone2.connect()

    # Drone1.trim(23, 5,0,0)
    # Drone2.trim(10,5,0,0)
    Drone1.disarm()
    Drone2.disarm()

    pid_drone1 = PID(Drone1,4)
    pid_drone2 = PID(Drone2,0)

    # poseThread = threading.Thread(target=getpose, args=(conn,conn2))
    # drone_pid1 = threading.Thread(target=pid_drone1.DronePID, args=('0', conn))
    # drone_pid2 = threading.Thread(target=pid_drone2.DronePID, args=('8', conn2))
    # poseThread.start()
    # time.sleep(0.5)
    
    # drone_pid2.start()
    # time.sleep(0.5)
    # drone_pid1.start()

    # drone_pid1.join()
    # drone_pid2.join()

    p0 = multiprocessing.Process(target=marker_publisher, args=([connCam, connCam2]))
    p1 = multiprocessing.Process(target=pid_drone1.DronePID, args=(['0',connDrone1]))
    p2 = multiprocessing.Process(target=pid_drone2.DronePID, args=(['8',connDrone2]))
    # PID_main(connDrone1,connDrone2)
    # p1 = threading.Thread(target=MP.recv_pose)
    # p2 = threading.Thread(target=MP.dronePlan, args=(['8']))
    # p3 = threading.Thread(target=MP.dronePlan, args=(['0']))


    p0.start()

    #first detect pose , then takeoff
    
    print('\n-------Starting process Camera-------')
    time.sleep(2)

    p1.start()

    print("\n-----Starting Drone1--------")
    p2.start()
    # p3.start()

    p0.join()
    p1.join()
    p2.join()
    # p3.join()
    

  
if __name__ == "__main__":
   

    
    
    build_conn()  #starts camera file and drone1 file , also builds connection btw the two