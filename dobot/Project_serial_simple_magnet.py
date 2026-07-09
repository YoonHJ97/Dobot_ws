import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
import serial

# 컨베이어(아두이노 IR 센서)에서 "DETECTED" 시리얼 신호가 오면
# 색 구분 없이 정해진 위치로 pick and place 하는 단순 버전. (전자석 그리퍼)

# Global variables (current coordinates)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False

globalLockValue = threading.Lock()

# 전자석이 연결된 "툴(팔 끝) DO" 채널 번호. 배선에 맞게 1 또는 2로 수정.
MAGNET_TOOL_DO = 1

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
         if enableStatus_robot is not None and algorithm_queue is not None:
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

def ActivateMagnet(dashboard: DobotApiDashboard, activate: bool):
    """
    전자석 그리퍼 ON/OFF.

    전자석은 진공 그리퍼와 동일하게 디지털 출력 한 채널을 켜고 끄는 방식이다.
    팔 끝(툴) IO에 연결했으므로 ToolDO 를 사용한다.
    activate=True  -> 자화(부착), False -> 소자(분리)
    """
    status = 1 if activate else 0
    dashboard.ToolDO(MAGNET_TOOL_DO, status)
    print(f"Magnet {'ON (attach)' if activate else 'OFF (release)'}")

if __name__ == '__main__':
    ser = serial.Serial('COM4', 9600, timeout=1)  # COM 포트는 아두이노에 맞게 변경
    sleep(2)  # 아두이노 초기화 시간 대기
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

    # 집을 위치 / 놓을 위치 (작업대에 맞게 좌표 수정)
    point_pick = [212.05, -13.16, -56.33, 0]
    point_pick_offset = [212.05, -13.16, 0, 0]
    point_place = [363.63, 5.70, -143.83, 0]
    point_place_offset = [363.63, 5.70, 0, 0]

    ActivateMagnet(dashboard, activate=False)
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
                    # --- pick ---
                    RunPoint(move, point_pick_offset)
                    WaitArrive(point_pick_offset)

                    RunPoint(move, point_pick)
                    WaitArrive(point_pick)

                    ActivateMagnet(dashboard, activate=True)
                    sleep(0.3)  # 부착 안정화 대기

                    RunPoint(move, point_pick_offset)
                    WaitArrive(point_pick_offset)

                    # --- place ---
                    RunPoint(move, point_place_offset)
                    WaitArrive(point_place_offset)

                    RunPoint(move, point_place)
                    WaitArrive(point_place)

                    ActivateMagnet(dashboard, activate=False)
                    sleep(0.3)  # 분리 안정화 대기

                    RunPoint(move, point_place_offset)
                    WaitArrive(point_place_offset)

                    # 다음 물체 대기 위치로 복귀
                    RunPoint(move, point_pick_offset)
                    WaitArrive(point_pick_offset)

    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        ser.close()
