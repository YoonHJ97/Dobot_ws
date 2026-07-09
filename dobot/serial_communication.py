import serial
import time

# 아두이노 시리얼 포트.
# 리눅스: '/dev/ttyUSB0' (USB-시리얼 어댑터) 또는 '/dev/ttyACM0' (아두이노 우노 등)
# 윈도우: 'COM4' 처럼 지정.  `ls /dev/ttyUSB* /dev/ttyACM*` 로 확인 후 수정.
PORT = '/dev/ttyUSB0'
BAUDRATE = 9600

def main():
    # 아두이노와 시리얼 통신 연결
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    except serial.SerialException as e:
        print(f"시리얼 포트 '{PORT}' 를 열 수 없습니다: {e}")
        print("`ls /dev/ttyUSB* /dev/ttyACM*` 로 실제 포트를 확인하고 PORT 값을 수정하세요.")
        return

    time.sleep(2)  # 아두이노 초기화 시간 대기

    print(f"시리얼 통신 시작! ({PORT} @ {BAUDRATE})")

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

    