import cv2

def capture_image():
    cap = cv2.VideoCapture(1)  # 1번 카메라 장치를 엽니다. (0번은 기본 내장 카메라)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다!")  # 에러 메시지 출력
        return  # 함수 종료

    while True:
        ret, frame = cap.read()  # 프레임을 읽어 저장 (ret: 성공 여부, frame: 이미지 데이터)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # BGR -> HSV 변환
        height, width, _ = frame.shape  # 프레임의 높이/너비

        cx = int(width / 2)   # 화면 너비의 중앙 x 좌표
        cy = int(height / 2)  # 화면 높이의 중앙 y 좌표

        pixel_center = hsv_frame[cy, cx]  # 중앙 픽셀의 HSV 값
        hue_value = pixel_center[0]       # 색상(Hue) 성분

        # Hue 값으로 색상 분류
        color = "Undefined"
        if hue_value < 5 :
            color = "RED"
        elif hue_value < 35:
            color = "YELLOW"
        elif hue_value < 90:
            color = "GREEN"
        elif hue_value < 120:
            color = "BLUE"

        print(pixel_center)

        # 감지된 실제 픽셀 색(BGR)으로 텍스트를 그린다
        pixel_center_bgr = frame[cy, cx]
        b, g, r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), 3)           # 중심 마커(파란 원)
        cv2.putText(frame, color, (10, 50), 0, 1, (b, g, r), 2)  # 판별된 색상 텍스트

        if not ret:
            print("프레임을 읽을 수 없습니다!")  # 에러 메시지 출력
            break  # 루프 종료

        cv2.imshow('Camera Capture', frame)  # 현재 프레임을 창에 표시

        key = cv2.waitKey(1) & 0xFF  # 키보드 입력 대기 (1ms 간격)
        if key == ord('q'):  # 'q'를 누르면 종료
            print("종료합니다.")
            break  # 루프 종료

    cap.release()  # 카메라 장치 해제
    cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기

if __name__ == "__main__":
    # 스크립트를 직접 실행할 경우에만 실행
    capture_image()
