import cv2

def capture_image():
    # 카메라를 열기 (디폴트 카메라는 0번 장치)
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

        color = "Undefined"
        if hue_value < 5 :
            color = "RED"
        elif hue_value < 35:
            color = "YELLOW"
        elif hue_value < 90:
            color = "GREEN"    
        elif hue_value < 120:
            color = "BLUE!"

        print(pixel_center)
        pixel_center_bgr = frame[cy, cx]
        b, g, r = int(pixel_center_bgr[0]), int(pixel_center_bgr[1]), int(pixel_center_bgr[2])
        cv2.putText(frame, color, (10,50), 0, 1, (b, g, r), 2)
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

if __name__ == "__main__":
    capture_image()