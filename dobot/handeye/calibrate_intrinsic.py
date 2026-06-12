# -*- coding: utf-8 -*-
"""
[1단계] 체커보드로 카메라 내부 캘리브레이션 -> K, distCoeffs 저장.

사용법:
    python3 calibrate_intrinsic.py

조작:
    SPACE : 현재 프레임에서 체커보드 코너 검출 성공 시 샘플 추가
    c     : 모은 샘플로 캘리브레이션 계산 후 저장
    q     : 종료

권장: 보드를 화면 곳곳/여러 각도로 기울여 15~20장 모을 것.
"""
import numpy as np
import cv2

import config as cfg


def main():
    cols, rows = cfg.CHESSBOARD_COLS, cfg.CHESSBOARD_ROWS

    # 체커보드 3D 좌표 (z=0 평면), 실제 칸 크기 반영
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= cfg.SQUARE_SIZE_MM

    objpoints, imgpoints = [], []   # 3D, 2D 대응

    cap = cv2.VideoCapture(cfg.CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    img_size = None

    print("[안내] SPACE=샘플추가, c=계산/저장, q=종료")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, (cols, rows), corners, found)
        cv2.putText(view, f"samples: {len(objpoints)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("intrinsic", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" ") and found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            print(f"  + 샘플 {len(objpoints)} 추가")
        elif key == ord("c"):
            if len(objpoints) < 5:
                print("  ! 최소 5장 이상 필요")
                continue
            rms, K, dist, _, _ = cv2.calibrateCamera(
                objpoints, imgpoints, img_size, None, None)
            print(f"[결과] RMS 재투영오차 = {rms:.4f}")
            print("K =\n", K)
            print("dist =", dist.ravel())
            np.savez(cfg.INTRINSIC_PATH, K=K, dist=dist, rms=rms)
            print(f"[저장] {cfg.INTRINSIC_PATH}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
