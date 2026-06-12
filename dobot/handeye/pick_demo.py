# -*- coding: utf-8 -*-
"""
[4단계] 캘리브레이션 결과로 ArUco 물체를 집는 데모.

흐름:
    카메라로 대상 마커 검출 -> target2cam(tvec) -> T_cam2base 로 base 좌표 변환
    -> 안전 높이로 접근 후 MovL 로 이동.

사용법:
    python3 pick_demo.py

조작:
    SPACE : 현재 검출된 마커 위치로 로봇 이동
    q     : 종료

[안전] 처음엔 PICK_Z 를 충분히 높게 두고, 실제 하강은 직접 확인하며 줄여라.
"""
import numpy as np
import cv2

import config as cfg
import robot_io as rio

PICK_OBJECT_ID = 1     # 집을 대상 마커 ID (그리퍼 마커와 다르게)
APPROACH_Z = 80.0      # 접근 높이(mm). 실험으로 조정
FIXED_R = 0.0          # 4축 yaw (deg). 필요시 마커 방향에서 계산


def main():
    Kd = np.load(cfg.INTRINSIC_PATH)
    K, dist = Kd["K"], Kd["dist"]
    He = np.load(cfg.HANDEYE_PATH)
    T_cam2base = He["T_cam2base"]

    dashboard, move = rio.connect(cfg.ROBOT_IP, cfg.DASH_PORT, cfg.MOVE_PORT)
    detector = rio.make_aruco_detector(cfg.ARUCO_DICT)

    cap = cv2.VideoCapture(cfg.CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다")

    print("[안내] SPACE=이동, q=종료")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        corners, ids, _ = detector.detectMarkers(frame)

        view = frame.copy()
        target_base = None
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(view, corners, ids)
            for c, i in zip(corners, ids.ravel()):
                if i == PICK_OBJECT_ID:
                    rvec, tvec = rio.estimate_marker_pose(
                        c, cfg.MARKER_SIZE_MM, K, dist)
                    if rvec is None:
                        continue
                    # target2cam -> base
                    p_cam = np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0], 1.0])
                    p_base = T_cam2base @ p_cam
                    target_base = p_base[:3]
                    cv2.drawFrameAxes(view, K, dist, rvec, tvec,
                                      cfg.MARKER_SIZE_MM * 0.5)
                    cv2.putText(view,
                                f"base: {target_base[0]:.0f},{target_base[1]:.0f},{target_base[2]:.0f}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 255, 0), 2)
        cv2.imshow("pick_demo", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" ") and target_base is not None:
            x, y, z = target_base
            print(f"[이동] base=({x:.1f},{y:.1f},{z:.1f}) 접근높이 {APPROACH_Z}")
            # 1) 대상 위 안전높이로
            move.MovL(float(x), float(y), float(APPROACH_Z), FIXED_R)
            # 2) TODO: WaitArrive 후 하강 / 그리퍼 ON / 상승 ...
            #    move.MovL(x, y, z_pick, FIXED_R)
            #    dashboard.DO(1, 1)   # 진공 ON 등

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
