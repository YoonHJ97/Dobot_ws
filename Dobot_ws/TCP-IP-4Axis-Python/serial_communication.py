import serial
import time

def main():
    # 아두이노와 시리얼 통신 연결 (포트 및 보드레이트 설정 필요)
    ser = serial.Serial('COM4', 9600, timeout=1)  # COM 포트는 아두이노에 맞게 변경
    time.sleep(2)  # 아두이노 초기화 시간 대기

    print("시리얼 통신 시작!")

    try:
        while True:
            if ser.in_waiting > 0:  # 수신된 데이터가 있는지 확인
                signal = ser.readline().decode('utf-8').strip()  # 데이터 읽기 및 디코딩
                print(f"Received Signal: {signal}")  # 수신된 신호 출력
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        ser.close()  # 시리얼 포트 닫기

if __name__ == "__main__":
    main()