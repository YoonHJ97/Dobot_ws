import cv2
import os

# 동영상 파일 경로
video_path = "C:\Block\KakaoTalk_20241127_143457952.mp4"
output_dir = "C:\Block"
os.makedirs(output_dir, exist_ok=True)

# 동영상 파일 열기
cap = cv2.VideoCapture(video_path)
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # 전체 프레임 수

# 20등분할 간격 계산
interval = max(1, frame_count // 20)

frame_idx = 0
saved_frames = 0

while cap.isOpened() and saved_frames < 20:
    ret, frame = cap.read()
    if not ret:
        break
    # 특정 간격의 프레임 저장
    if frame_idx % interval == 0:
        output_file = os.path.join(output_dir, f"frame_{saved_frames + 1:02d}.jpeg")
        # JPEG로 저장, 품질 최상으로 설정
        cv2.imwrite(output_file, frame, [cv2.IMWRITE_JPEG_QUALITY, 100])
        saved_frames += 1
    frame_idx += 1

cap.release()

print(f"이미지 저장 완료. 저장된 폴더: {output_dir}")
