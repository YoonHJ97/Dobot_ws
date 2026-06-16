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

# 전자석이 연결된 "툴(팔 끝) DO" 채널 번호. 배선에 맞게 1 또는 2로 수정.
MAGNET_TOOL_DO = 1


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
    """지정된 좌표(point)로 직선 보간 이동."""
    move.MovL(point[0], point[1], point[2], point[3])


def GetFeed(feed: DobotApi):
    """피드백 스레드: 소켓 데이터를 읽어 전역 상태 변수 갱신."""
    global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
    while True:
        data = bytes()
        read_len = 0
        while read_len < 1440:
            chunk = feed.socket_dobot.recv(1440 - read_len)
            read_len += len(chunk)
            data += chunk
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex(feedInfo['test_value'][0]) == '0x123456789abcdef':
            with globalLockValue:
                current_actual     = feedInfo["tool_vector_actual"][0]
                algorithm_queue    = feedInfo['isRunQueuedCmd'][0]
                enableStatus_robot = feedInfo['EnableStatus'][0]
                robotErrorState    = feedInfo['ErrorStatus'][0]
        sleep(0.001)


def ClearRobotError(dashboard: DobotApiDashboard):
    """로봇 에러 모니터링: 에러 발생 시 코드별 메시지 출력 후 사용자 선택으로 클리어."""
    global robotErrorState, enableStatus_robot, algorithm_queue
    dataController, dataServo = alarmAlarmJsonFile()
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            if numbers and numbers[0] == 0 and len(numbers) > 1:
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
                if choose.strip() == '1':
                    dashboard.ClearError()
                    sleep(0.01)
                    dashboard.Continue()
        else:
            if int(enableStatus_robot[0]) == 1 and int(algorithm_queue[0]) == 0:
                dashboard.Continue()
        globalLockValue.release()
        sleep(5)


def WaitArrive(target: list):
    """목표 좌표 도달(오차 1mm 이내)까지 대기."""
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


def ActivateMagnet(dashboard: DobotApiDashboard, activate: bool):
    """
    전자석 그리퍼 ON/OFF.

    전자석은 진공 그리퍼와 동일하게 디지털 출력 한 채널을 켜고 끄는 방식이다.
    팔 끝(툴) IO에 연결했으므로 ToolDO 를 사용한다.
      - ToolDO(index, status): 큐에 쌓여 순서대로 실행 (이동 명령과 동기)
      - 즉시 실행이 필요하면 dashboard.ToolDOExecute(index, status) 사용
    activate=True  -> 자화(부착), False -> 소자(분리)
    """
    status = 1 if activate else 0
    dashboard.ToolDO(MAGNET_TOOL_DO, status)
    print(f"Magnet {'ON (attach)' if activate else 'OFF (release)'}")


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

    # 3) 주요 포인트 정의  (배선/작업대에 맞게 좌표 수정)
    pick_up    = [369.07, -174.69, -62.96, 0]   # 집을 위치 (내려간 자세)
    pick_hover = [369.07, -174.69,   0.00, 0]   # 집을 위치 위 (안전 높이)
    place_hover = [365.54,   67.00,   0.00, 0]  # 놓을 위치 위
    place_down  = [365.54,   67.00, -141.03, 0] # 놓을 위치 (내려간 자세)

    # 4) 초기 전자석 OFF 후 안전 높이로 이동
    ActivateMagnet(dashboard, activate=False)
    RunPoint(move, pick_hover)
    WaitArrive(pick_hover)

    # 5) 픽 앤 플레이스 반복
    while True:
        # 집을 위치로 하강 -> 자화(부착) -> 들어올림
        RunPoint(move, pick_up)
        WaitArrive(pick_up)
        ActivateMagnet(dashboard, activate=True)
        sleep(0.3)  # 부착 안정화 대기
        RunPoint(move, pick_hover)
        WaitArrive(pick_hover)

        # 놓을 위치 위로 이동 -> 하강 -> 소자(분리)
        RunPoint(move, place_hover)
        WaitArrive(place_hover)
        RunPoint(move, place_down)
        WaitArrive(place_down)
        ActivateMagnet(dashboard, activate=False)
        sleep(0.3)  # 분리 안정화 대기

        # 안전 높이로 복귀
        RunPoint(move, place_hover)
        WaitArrive(place_hover)


if __name__ == '__main__':
    main()
