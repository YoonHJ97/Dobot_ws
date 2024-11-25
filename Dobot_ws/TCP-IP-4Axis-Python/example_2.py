import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

# Global variables (current coordinates)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
globalLockValue = threading.Lock()

def ConnectRobot():
    try:
        ip = "192.168.1.6"
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("Establishing connection...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        print(">.<Connection successful>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(Connection failed:(")
        raise e

def RunPoint(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])

def GetFeed(feed: DobotApi):
    global current_actual
    global algorithm_queue
    global enableStatus_robot
    global robotErrorStatepicj
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
            globalLockValue.acquire()
            # Refresh Properties
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['isRunQueuedCmd'][0]
            enableStatus_robot = feedInfo['EnableStatus'][0]
            robotErrorState = feedInfo['ErrorStatus'][0]
            globalLockValue.release()
        sleep(0.001)

def WaitArrive(point_list):
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 5):
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)

def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorStatep
    dataController, dataServo = alarmAlarmJsonFile()    # Read controller and servo alarm codes
    while True:
      globalLockValue.acquire()
      if robotErrorState:
                numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
                numbers = [int(num) for num in numbers]
                if (numbers[0] == 0):
                  if (len(numbers) > 1):
                    for i in numbers[1:]:
                      alarmState = False
                      if i == -2:
                          print("Robot Alarm: Collision detected", i)
                          alarmState = True
                      if alarmState:
                          continue                
                      for item in dataController:
                        if i == item["id"]:
                            print("Robot Alarm: Controller error ID", i, item["zh_CN"]["description"])
                            alarmState = True
                            break 
                      if alarmState:
                          continue
                      for item in dataServo:
                        if i == item["id"]:
                            print("Robot Alarm: Servo error ID", i, item["zh_CN"]["description"])
                            break  
                    choose = input("Enter 1 to clear errors and continue operation: ")     
                    if int(choose) == 1:
                        dashboard.ClearError()
                        sleep(0.01)
                        dashboard.Continue()

      else:  
         if int(enableStatus_robot[0]) == 1 and int(algorithm_queue[0]) == 0:
            dashboard.Continue()
      globalLockValue.release()
      sleep(5)

def ActivateVacuumGripper(dashboard: DobotApiDashboard, activate: bool):

    index = 1  # Assuming DO_01 corresponds to index 1
    status = 1 if activate else 0
    dashboard.DO(index, status)  # Activate or deactivate DO_01 based on status
    print(f"Vacuum Gripper {'activated' if activate else 'deactivated'}")

if __name__ == '__main__':
    dashboard, move, feed = ConnectRobot()
    dashboard.EnableRobot()

    feed_thread = threading.Thread(target=GetFeed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()

    feed_thread1 = threading.Thread(target=ClearRobotError, args=(dashboard,))
    feed_thread1.setDaemon(True)
    feed_thread1.start()

    print("Executing loop...")

    # Define positions
    point_a = [320, 11.93, -140.42, 0]   # pick position
    point_b = [320, 11.93, 0, 0]         # transition position
    point_c = [270, -113.76, 0, 0]       # transition position
    point_d = [270, -113.76, -140.59, 0] # place position

    RunPoint(move, point_b)
    WaitArrive(point_b)
    
    while True:
        command = input("Enter command (pick/place): ").strip().lower()

        if command == 'pick':

            RunPoint(move, point_b)
            WaitArrive(point_b)
            
            # Move to point_a (pick position)
            RunPoint(move, point_a)
            WaitArrive(point_a)

            # Activate the vacuum gripper to pick up the object
            ActivateVacuumGripper(dashboard, activate=True)
            sleep(1)  # Wait to ensure the object is picked up

            # Move to point_b as an intermediate position
            RunPoint(move, point_b)
            WaitArrive(point_b)

        elif command == 'place':
            # Move to point_c as an intermediate position
            RunPoint(move, point_c)
            WaitArrive(point_c)

            # Move to point_d (place position)
            RunPoint(move, point_d)
            WaitArrive(point_d)

            # Deactivate the vacuum gripper to place the object
            ActivateVacuumGripper(dashboard, activate=False)
            sleep(1)  # Wait to ensure the object is placed

            # Return to point_c after placing the object
            RunPoint(move, point_c)
            WaitArrive(point_c)

        else:
            print("Invalid command. Please enter 'pick' or 'place'.")