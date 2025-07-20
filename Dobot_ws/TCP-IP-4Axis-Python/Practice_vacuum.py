import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

# ——————————————————————————————
# 전역 변수 선언
# ——————————————————————————————
current_actual = None          # 로봇의 현재 좌표 피드백 저장
algorithm_queue = None         # 명령 큐 실행 여부 플래그
enableStatus_robot = None      # 로봇 활성화 상태
robotErrorState = False        # 로봇 에러 상태
globalLockValue = threading.Lock()  # 공유 데이터 보호용 락

def ConnectRobot():
    """
    로봇과의 대시보드, 모션, 피드백 소켓을 연결하고
    (dashboard, move, feed) 객체를 반환합니다.
    """
    ip = "192.168.1.6"
    print("Establishing connection...")
    dashboard = DobotApiDashboard(ip, 29999)
    move      = DobotApiMove    (ip, 30003)
    feed      = DobotApi        (ip, 30004)
    print(">.< Connection successful! <")
    return dashboard, move, feed

def RunPoint(move: DobotApiMove, point: list):
    """
    지정된 좌표(point_list)에 대해 직선 보간 이동을 실행합니다.
    """
    move.MovL(point[0], point[1], point[2], point[3])

def GetFeed(feed: DobotApi):
    """
    피드백 스레드:
    - 소켓에서 바이너리 데이터를 읽어 와서
    - current_actual, algorithm_queue, enableStatus_robot, robotErrorState 전역 변수 갱신
    """
    global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
    while True:
        # 1440 바이트씩 읽기
        data = bytes()
        read_len = 0
        while read_len < 1440:
            chunk = feed.socket_dobot.recv(1440 - read_len)
            read_len += len(chunk)
            data += chunk
        feedInfo = np.frombuffer(data, dtype=MyType)
        # 식별용 값 체크
        if hex(feedInfo['test_value'][0]) == '0x123456789abcdef':
            with globalLockValue:
                current_actual     = feedInfo["tool_vector_actual"][0]
                algorithm_queue    = feedInfo['isRunQueuedCmd'][0]
                enableStatus_robot = feedInfo['EnableStatus'][0]
                robotErrorState    = feedInfo['ErrorStatus'][0]
        sleep(0.001)

def ClearRobotError(dashboard: DobotApiDashboard):
    """
    원래 구현으로 되돌린 에러 모니터링 함수.
    로봇 에러 발생 시 코드별로 메시지 출력하고,
    사용자가 선택하면 ClearError()/Continue() 호출.
    """
    global robotErrorState, enableStatus_robot, algorithm_queue
    dataController, dataServo = alarmAlarmJsonFile()  # 컨트롤러·서보 알람 코드 로드
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            # 에러 ID 문자열에서 모든 숫자 추출
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            # 첫 숫자가 0이면 뒤에 실제 에러 코드들 처리
            if numbers and numbers[0] == 0 and len(numbers) > 1:
                for i in numbers[1:]:
                    alarmState = False
                    # 충돌 감지 예외 처리
                    if i == -2:
                        print("Robot Alarm: Collision detected", i)
                        alarmState = True
                    if alarmState:
                        continue
                    # 컨트롤러 에러 검색
                    for item in dataController:
                        if i == item["id"]:
                            print("Robot Alarm: Controller error ID", i, item["zh_CN"]["description"])
                            alarmState = True
                            break
                    if alarmState:
                        continue
                    # 서보 에러 검색
                    for item in dataServo:
                        if i == item["id"]:
                            print("Robot Alarm: Servo error ID", i, item["zh_CN"]["description"])
                            break
                # 사용자에게 클리어 여부 입력 요청
                choose = input("Enter 1 to clear errors and continue operation: ")
                if choose.strip() == '1':
                    dashboard.ClearError()
                    sleep(0.01)
                    dashboard.Continue()
        else:
            # 에러 없을 때, 로봇이 enable 상태이고 큐가 비었으면 Continue
            if int(enableStatus_robot[0]) == 1 and int(algorithm_queue[0]) == 0:
                dashboard.Continue()
        globalLockValue.release()
        sleep(5)

def WaitArrive(target: list):
    """
    목표 좌표(target)에 도달할 때까지 대기합니다.
    current_actual과 오차(1mm) 이내인지 반복 검사.
    """
    global current_actual
    while True:
        arrived = True
        with globalLockValue:
            if current_actual is not None:
                for i in range(4):
                    if abs(current_actual[i] - target[i]) > 1:
                        arrived = False
                        break
                if arrived:
                    return
        sleep(0.001)

def ActivateVacuumGripper(dashboard: DobotApiDashboard, activate: bool):
    """
    진공 그리퍼 ON/OFF 명령 전송 (DO_01 사용 가정).
    """
    idx = 1
    status = 1 if activate else 0
    dashboard.DO(idx, status)
    print(f"Vacuum Gripper {'activated' if activate else 'deactivated'}")

def main():
    # 1) 로봇 연결 및 활성화
    dashboard, move, feed = ConnectRobot()
    dashboard.EnableRobot()
    print("Enable complete :)")

    # 2) 피드백 & 에러 모니터링 스레드 시작
    t_feed  = threading.Thread(target=GetFeed, args=(feed,), daemon=True)
    t_error = threading.Thread(target=ClearRobotError, args=(dashboard,), daemon=True)
    t_feed.start()
    t_error.start()

    # 3) 주요 포인트 정의
    point_a = [369.07, -174.69, -62.96, 0]
    point_b = [369.07, -174.69,   0.00, 0]
    point_c = [365.54,   67.00,   0.00, 0]
    point_d = [365.54,   67.00, -141.03, 0]

    # 4) 초기 그리퍼 해제 및 포인트 이동
    ActivateVacuumGripper(dashboard, activate=False)
    RunPoint(move, point_b)
    WaitArrive(point_b)

    # 5) 무한 루프에서 픽 앤 플레이스 동작 반복
    while True:
        RunPoint(move, point_a)
        WaitArrive(point_a)

        ActivateVacuumGripper(dashboard, activate=True)
        RunPoint(move, point_b)
        WaitArrive(point_b)

        RunPoint(move, point_c)
        WaitArrive(point_c)
        RunPoint(move, point_d)
        WaitArrive(point_d)

        ActivateVacuumGripper(dashboard, activate=False)
        RunPoint(move, point_c)
        WaitArrive(point_c)

if __name__ == '__main__':
    main()
