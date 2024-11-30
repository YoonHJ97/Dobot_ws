import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
import serial
import time
import cv2

# Global variables (current coordinates)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
current_color = ""

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

def ActivateVacuumGripper(dashboard: DobotApiDashboard, activate: bool):

    index = 1  # Assuming DO_01 corresponds to index 1
    status = 1 if activate else 0
    dashboard.DO(index, status)  # Activate or deactivate DO_01 based on status
    print(f"Vacuum Gripper {'activated' if activate else 'deactivated'}")

def capture_image():
    # 카메라를 열기 (디폴트 카메라는 0번 장치)
    global current_color
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다!")
        return

    while True:
        # 카메라에서 프레임 읽기
        ret, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width, _ = frame.shape

        cx = int(width / 2)
        cy = int(height / 2)

        pixel_center = hsv_frame[cy,cx]
        hue_value = pixel_center[0]

        current_color = "Undefined"
        if hue_value < 5 :
            current_color = "RED"
        elif hue_value < 35:
            current_color = "YELLOW"
        elif hue_value < 90:
            current_color = "GREEN"    
        elif hue_value < 120:
            current_color = "BLUE"
        pixel_center_bgr = frame[cy, cx]
        b, g, r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
        cv2.putText(frame, current_color, (10,50), 0, 1, (b, g, r), 2)
        cv2.circle(frame, (cx,cy), 5, (255, 0, 0), 3)

        if not ret:
            print("프레임을 읽을 수 없습니다!")
            break

        # 프레임을 화면에 표시
        cv2.imshow('Camera Capture', frame)

        # 키 입력 처리
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 's'를 누르면 이미지 저장
            print("종료합니다.")
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    ser = serial.Serial('COM4', 9600, timeout=1)  # COM 포트는 아두이노에 맞게 변경
    time.sleep(2)  # 아두이노 초기화 시간 대기
    pick = False
    dashboard, move, feed = ConnectRobot()
    print("Starting enable...")
    dashboard.EnableRobot()
    print("Enable complete :)")

    capture_thread = threading.Thread(target=capture_image)
    capture_thread.daemon = True  # 메인 스레드 종료 시 함께 종료되도록 설정
    capture_thread.start()

    # Thread to continuously read feedback from the robot
    feed_thread = threading.Thread(target=GetFeed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()

    # Thread to monitor and clear robot errors
    feed_thread1 = threading.Thread(target=ClearRobotError, args=(dashboard,))
    feed_thread1.setDaemon(True)
    feed_thread1.start()

    point_pick = [273.36, -65.42, -57.98, 0]
    point_pick = [273.36, -65.42, 0, 0]

    point_pick_offset = [273.36, -65.42, 0, 0]
    point_red = [382.89, -40.66, -142.84, 0]
    point_red_offset = [382.89, -40.66, 0, 0]
    point_yellow = [379.64, 13.60, -142.34, 0]
    point_yellow_offset = [379.64, 13.60, 0, 0]
    point_green = [371.82, 72.20, -142.43, 0]
    point_green_offset = [371.82, 72.20, 0, 0]
    point_blue = [369.24, 118.45, -142.35, 0]
    point_blue_offset = [369.24, 118.45, 0, 0]

    ActivateVacuumGripper(dashboard, activate=False)
    RunPoint(move, point_pick_offset)
    WaitArrive(point_pick_offset)

    print("Executing loop...")

    print("시리얼 통신 시작!")

    try:
        while True:
            if ser.in_waiting > 0:  # 수신된 데이터가 있는지 확인
                signal = ser.readline().decode('utf-8').strip()  # 데이터 읽기 및 디코딩
                print(f"Received Signal: {signal}")  # 수신된 신호 출력
                if signal == "DETECTED":
                    block_color = ""
                    pick = True
                    block_color = current_color
                    print(block_color)

                if pick :
                    RunPoint(move, point_pick_offset)
                    WaitArrive(point_pick_offset)

                    RunPoint(move, point_pick)
                    WaitArrive(point_pick)

                    ActivateVacuumGripper(dashboard, activate=True)

                    RunPoint(move, point_pick_offset)
                    WaitArrive(point_pick_offset)

                    if block_color == "RED":
                        RunPoint(move, point_red_offset)
                        WaitArrive(point_red_offset)

                        RunPoint(move, point_red)
                        WaitArrive(point_red)

                        # Optionally, reactivate the vacuum gripper before moving back
                        ActivateVacuumGripper(dashboard, activate=False)

                        RunPoint(move, point_red_offset)
                        WaitArrive(point_red_offset)
                    elif block_color == "YELLOW":
                        RunPoint(move, point_yellow_offset)
                        WaitArrive(point_yellow_offset)

                        RunPoint(move, point_yellow)
                        WaitArrive(point_yellow)

                        # Optionally, reactivate the vacuum gripper before moving back
                        ActivateVacuumGripper(dashboard, activate=False)

                        RunPoint(move, point_yellow_offset)
                        WaitArrive(point_yellow_offset)
                    elif block_color == "GREEN":
                        RunPoint(move, point_green_offset)
                        WaitArrive(point_green_offset)

                        RunPoint(move, point_green)
                        WaitArrive(point_green)

                        # Optionally, reactivate the vacuum gripper before moving back
                        ActivateVacuumGripper(dashboard, activate=False)

                        RunPoint(move, point_green_offset)
                        WaitArrive(point_green_offset)
                    else :
                        RunPoint(move, point_blue_offset)
                        WaitArrive(point_blue_offset)

                        RunPoint(move, point_blue)
                        WaitArrive(point_blue)

                        # Optionally, reactivate the vacuum gripper before moving back
                        ActivateVacuumGripper(dashboard, activate=False)

                        RunPoint(move, point_blue_offset)
                        WaitArrive(point_blue_offset)

                    pick = False



    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        ser.close()  
