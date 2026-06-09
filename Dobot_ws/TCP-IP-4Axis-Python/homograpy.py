import numpy as np
import cv2

def main():
    # ArUco 좌표 (픽셀)
    aruco_pts = np.array([
        [353.0, 180.8],
        [309.0, 177.8],
        [311.0, 224.0],
        [353.0, 223.2],
        [357.0, 270.2],
        [398.5, 266.0],
        [413.2, 309.0],
        [471.8, 327.0],
        [419.0, 365.2],
        [434.5, 271.5],
    ], dtype=np.float32)

    # 로봇 좌표 (월드 좌표 등)
    robot_pts = np.array([
        [64.874, -215.17],
        [66.97, -288.68],
        [141.575, -285.94],
        [136.391, -213.575],
        [218.352, -208.141],
        [210.326, -138.935],
        [286.175, -112.696],
        [317.469, -12.0004],
        [385.657, -98.8440],
        [219.106, -79.9620],
    ], dtype=np.float32)

    # 호모그래피 계산
    H, mask = cv2.findHomography(aruco_pts, robot_pts, cv2.RANSAC)
    if H is None:
        print("[ERROR] 호모그래피 행렬 계산 실패")
        return

    print("[INFO] 호모그래피 행렬 계산 완료:")
    print(H)

    # 저장 경로
    npy_path = "homography_matrix.npy"
    yaml_path = "homography_matrix.yaml"

    # 1. numpy 형식 저장
    np.save(npy_path, H)
    print(f"[INFO] .npy 형식으로 저장됨 → {npy_path}")