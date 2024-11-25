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

        if not ret:
            print("프레임을 읽을 수 없습니다!")
            break

        # 프레임을 화면에 표시
        cv2.imshow('Camera Capture', frame)

        # 키 입력 처리
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 'q'를 누르면 종료
            print("종료합니다.")
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_image()

