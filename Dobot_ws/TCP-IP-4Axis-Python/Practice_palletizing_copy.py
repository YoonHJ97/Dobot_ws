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
    global robotErrorState
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
    global robotErrorState
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
                            print("Robot Alarm: Controller error ID", i, item["en"]["description"])
                            alarmState = True
                            break 
                      if alarmState:
                          continue
                      for item in dataServo:
                        if i == item["id"]:
                            print("Robot Alarm: Servo error ID", i, item["en"]["description"])
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

def GeneratePallet(cols, rows, layers, start_point, offset):

    pallet = []
    for layer in range(layers):  # Loop over layers (Z axis)
        for row in range(rows):  # Loop over rows (Y axis)
            for col in range(cols):  # Loop over columns (X axis)
                # Calculate the coordinate for each position
                target_point = [
                    start_point[0] + col * offset[0],  # X offset (columns)
                    start_point[1] + row * offset[1],  # Y offset (rows)
                    start_point[2] + layer * offset[2],  # Z offset (layers)
                    start_point[3]                    # R (rotation) stays the same
                ]
                pallet.append(target_point)
    return pallet
            
if __name__ == '__main__':

    dashboard, move, feed = ConnectRobot()
    print("Starting enable...")
    dashboard.EnableRobot()
    print("Enable complete :)")

    # Thread to continuously read feedback from the robot
    feed_thread = threading.Thread(target=GetFeed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()

    # Thread to monitor and clear robot errors
    feed_thread1 = threading.Thread(target=ClearRobotError, args=(dashboard,))
    feed_thread1.setDaemon(True)
    feed_thread1.start()

    print("Executing loop...")

    ActivateVacuumGripper(dashboard, activate=False)

    rows = 3
    cols = 2
    layers = 2
    offset = [-35, -35, 35]  # 간격 (x, y, z)
    pick_point = [312.91, 28.95, -140.63, 0]
    start_point = [316.65, -46.94, -139.44, 0]  # 시작점
    pick_point_with_offset_z = [pick_point[0], pick_point[1], pick_point[2] + 50, pick_point[3]]
    home_point = [299.84, -68.83, -36.30, 0]

    pallet = GeneratePallet(rows, cols, layers, start_point, offset)

    RunPoint(move, pick_point_with_offset_z)
    WaitArrive(pick_point_with_offset_z)

    for point in pallet:
        point_with_offset_z = [point[0], point[1], point[2] + 50, point[3]]

        RunPoint(move, pick_point)
        WaitArrive(pick_point)

        ActivateVacuumGripper(dashboard, activate=True)

        RunPoint(move, pick_point_with_offset_z)
        WaitArrive(pick_point_with_offset_z)

        RunPoint(move, home_point)
        WaitArrive(home_point)

        RunPoint(move, point_with_offset_z)
        WaitArrive(point_with_offset_z)

        RunPoint(move, point)
        WaitArrive(point)

        ActivateVacuumGripper(dashboard, activate=False)

        RunPoint(move, point_with_offset_z)
        WaitArrive(point_with_offset_z)

        RunPoint(move, home_point)
        WaitArrive(home_point)

        RunPoint(move, pick_point_with_offset_z)
        WaitArrive(pick_point_with_offset_z)