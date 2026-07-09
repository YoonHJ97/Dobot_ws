"""
robot_state.py

Dobot 로봇 상태 관리 스크립트.
대시보드 포트(29999)로 연결해서 아래 기능을 메뉴로 제공한다.

  1. Enable  robot   (EnableRobot)
  2. Disable robot   (DisableRobot)
  3. Clear  error    (알람 조회 + 설명 출력 + ClearError, 필요시 ResetRobot)
  4. Robot mode      (현재 상태 조회)
  0. Exit

Practice.py 등 다른 스크립트가 에러로 멈췄을 때 이 스크립트만 실행해서
알람 해제 / enable / disable 을 처리할 수 있다.

사용법:
    python3 robot_state.py            # 대화형 메뉴
    python3 robot_state.py enable     # 명령 한 번만 실행하고 종료
    python3 robot_state.py disable
    python3 robot_state.py clear
    python3 robot_state.py mode
"""

import sys
import re
from dobot_api import DobotApiDashboard, alarmAlarmJsonFile

IP = "192.168.1.6"
DASHBOARD_PORT = 29999

# RobotMode() 반환 코드 -> 사람이 읽는 설명
ROBOT_MODE = {
    1: "ROBOT_MODE_INIT (초기화)",
    2: "ROBOT_MODE_BRAKE_OPEN (브레이크 해제)",
    3: "ROBOT_MODE_POWEROFF (전원 꺼짐)",
    4: "ROBOT_MODE_DISABLED (비활성 / disable)",
    5: "ROBOT_MODE_ENABLE (활성 / enable, 대기)",
    6: "ROBOT_MODE_BACKDRIVE (드래그)",
    7: "ROBOT_MODE_RUNNING (실행 중)",
    8: "ROBOT_MODE_RECORDING (녹화 중)",
    9: "ROBOT_MODE_ERROR (에러 상태)",
    10: "ROBOT_MODE_PAUSE (일시정지)",
    11: "ROBOT_MODE_JOG (조그 중)",
}


def ConnectDashboard():
    print("Establishing connection...")
    dashboard = DobotApiDashboard(IP, DASHBOARD_PORT)
    print(">.<Connection successful>!<")
    return dashboard


def _parse_first_int(resp):
    """대시보드 응답에서 첫 번째 정수(보통 명령 반환 코드)를 뽑아낸다. 실패 시 None."""
    nums = re.findall(r'-?\d+', resp)
    return int(nums[0]) if nums else None


def EnableRobot(dashboard):
    print("Enabling robot...")
    resp = dashboard.EnableRobot()
    if _parse_first_int(resp) == 0:
        print("Enable complete :)")
    else:
        print("Enable failed. Response:", resp.strip())


def DisableRobot(dashboard):
    print("Disabling robot...")
    resp = dashboard.DisableRobot()
    if _parse_first_int(resp) == 0:
        print("Disable complete.")
    else:
        print("Disable failed. Response:", resp.strip())


def PrintMode(dashboard):
    """RobotMode() 조회해서 현재 상태를 출력하고 모드 코드를 반환."""
    resp = dashboard.RobotMode()
    nums = re.findall(r'-?\d+', resp)
    # 응답 형식: "0,{5},RobotMode();" -> nums[0]=반환코드, nums[1]=모드
    mode = int(nums[1]) if len(nums) > 1 else None
    if mode is None:
        print("Failed to read robot mode. Response:", resp.strip())
        return None
    print(f"Robot mode: {mode} - {ROBOT_MODE.get(mode, 'Unknown')}")
    return mode


def PrintAlarms(dashboard, dataController, dataServo):
    """현재 활성 알람 ID들을 조회해서 설명과 함께 출력. 활성 알람 개수를 반환."""
    numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
    numbers = [int(num) for num in numbers]

    # numbers[0] 은 GetErrorID 명령의 반환 코드(0=성공). 그 뒤가 실제 알람 ID들.
    if not numbers or numbers[0] != 0:
        print("Failed to read error IDs:", numbers)
        return 0

    alarm_ids = numbers[1:]
    if not alarm_ids:
        print("No active alarms. :)")
        return 0

    print(f"Active alarms: {len(alarm_ids)}")
    for i in alarm_ids:
        if i == -2:
            print("  Robot Alarm: Collision detected", i)
            continue

        described = False
        for item in dataController:
            if i == item["id"]:
                print("  Robot Alarm: Controller error ID", i, item["en"]["description"])
                described = True
                break
        if described:
            continue

        for item in dataServo:
            if i == item["id"]:
                print("  Robot Alarm: Servo error ID", i, item["en"]["description"])
                described = True
                break
        if not described:
            print("  Robot Alarm: Unknown error ID", i)

    return len(alarm_ids)


def ClearError(dashboard, dataController, dataServo, ask=True):
    """알람을 조회/출력하고, 사용자 확인 후 ClearError() 실행. 남으면 ResetRobot 제안."""
    count = PrintAlarms(dashboard, dataController, dataServo)
    if count == 0:
        return

    if ask:
        choose = input("Enter 1 to clear errors: ").strip()
        if choose != "1":
            print("Skipped. No errors cleared.")
            return

    print("Clearing errors...")
    dashboard.ClearError()

    print("Re-checking alarms...")
    remaining = PrintAlarms(dashboard, dataController, dataServo)

    if remaining > 0:
        print("일부 알람이 남아 있습니다.")
        print("리미트/위치 관련 에러는 팔이 허용 범위 밖에 있어서 클리어만으론 안 풀립니다.")
        print("팔을 허용 범위 안으로 이동(조그)한 뒤 다시 시도하세요.")
        if ask:
            choose = input("Enter 1 to run ResetRobot() and clear again: ").strip()
            if choose == "1":
                dashboard.ResetRobot()
                dashboard.ClearError()
                print("Re-checking alarms...")
                PrintAlarms(dashboard, dataController, dataServo)


def RunMenu(dashboard, dataController, dataServo):
    while True:
        print("\n===== Dobot state manager =====")
        print("  1. Enable robot")
        print("  2. Disable robot")
        print("  3. Clear error")
        print("  4. Robot mode")
        print("  0. Exit")
        choose = input("Select: ").strip()

        if choose == "1":
            EnableRobot(dashboard)
        elif choose == "2":
            DisableRobot(dashboard)
        elif choose == "3":
            ClearError(dashboard, dataController, dataServo)
        elif choose == "4":
            PrintMode(dashboard)
        elif choose == "0":
            break
        else:
            print("Invalid selection.")


def RunOnce(dashboard, dataController, dataServo, cmd):
    if cmd in ("enable", "enablerobot"):
        EnableRobot(dashboard)
    elif cmd in ("disable", "disablerobot"):
        DisableRobot(dashboard)
    elif cmd in ("clear", "clearerror", "clear_error"):
        ClearError(dashboard, dataController, dataServo)
    elif cmd in ("mode", "robotmode"):
        PrintMode(dashboard)
    else:
        print(f"Unknown command: {cmd}")
        print("Use one of: enable | disable | clear | mode")


if __name__ == '__main__':
    dashboard = ConnectDashboard()
    dataController, dataServo = alarmAlarmJsonFile()

    try:
        if len(sys.argv) > 1:
            RunOnce(dashboard, dataController, dataServo, sys.argv[1].lower())
        else:
            RunMenu(dashboard, dataController, dataServo)
    finally:
        dashboard.close()
        print("Done.")
