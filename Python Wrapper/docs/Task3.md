# Task 3

## Connecting two drones

To automate the process of connecting two drones run the following:
```
chmod +x establish_connection.sh
./establish_connection.sh
```
This file has a dependendency of ``expect``, which can be installed by ``sudo apt-get install expect``.


## Functional State Machine
![image](https://user-images.githubusercontent.com/85498394/218274029-4d499dc7-f3e4-40d7-91df-123abb794878.png)





## Running

To run the PID control task, run ``master.py`` which will further start two parallel processes for pose estimation and PID control of two drones.
