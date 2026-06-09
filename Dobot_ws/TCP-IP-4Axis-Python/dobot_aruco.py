import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
import cv2

# ----------------------[추가: 카메라 및 호모그래피 초기화]----------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
cap = cv2.VideoCapture(0)  # USB 카메라 사용

homography = np.load("homography_matrix.npy")  # 사전 저장된 호모그래피 행렬

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
                if (abs(current_actual[index] - point_list[index]) > 1):
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

# ----------------------[추가: 카메라로 ArUco 인식 후 Dobot 이동]----------------------
def ArucoTrackingLoop(move: DobotApiMove):
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 카메라 프레임 읽기 실패")
            sleep(5)
            continue

        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None and len(ids) > 0:
            for c in corners:
                pts = c[0]
                center_pixel = np.mean(pts, axis=0).reshape(1, 1, 2).astype(np.float32)
                target_robot = cv2.perspectiveTransform(center_pixel, homography)[0][0]

                x, y = target_robot[0], target_robot[1]
                z, r = 0.0, 0.0  # 고정값: 필요시 수정

                print(f"[INFO] ArUco 위치: {x:.2f}, {y:.2f}")
                RunPoint(move, [x, y, z, r])
                WaitArrive([x, y, z, r])
                break  # 첫 번째 마커만 추적
        else:
            print("[INFO] ArUco 마커 없음")

        sleep(5)

# ----------------------[메인 실행]----------------------
if __name__ == '__main__':
    dashboard, move, feed = ConnectRobot()
    print("Starting enable...")
    dashboard.EnableRobot()
    print("Enable complete :)")

    threading.Thread(target=GetFeed, args=(feed,), daemon=True).start()
    threading.Thread(target=ClearRobotError, args=(dashboard,), daemon=True).start()

    print("[INFO] 5초마다 ArUco 추적 시작...")
    ArucoTrackingLoop(move)
