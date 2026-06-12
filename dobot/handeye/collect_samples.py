# -*- coding: utf-8 -*-
"""
[2단계] 핸드-아이 샘플 수집.

그리퍼에 ArUco 마커를 붙이고, 로봇을 여러 자세로 옮길 때마다
  - 로봇 포즈           T_gripper2base  (GetPose)
  - 카메라가 본 마커포즈  T_target2cam    (solvePnP)
를 한 쌍씩 기록한다.

사용법:
    python3 collect_samples.py

조작:
    SPACE : 현재(로봇정지 상태) 포즈쌍 1개 캡처
    q     : 종료 및 저장

주의: 캡처 순간 로봇은 완전히 정지해 있어야 한다. 로봇은 손으로
      (teach 모드) 또는 별도 콘솔의 MovL 로 옮긴 뒤 SPACE 를 눌러라.

[중요] MG400은 4축이라 회전이 Z축 하나뿐 → 정석 핸드-아이가
       부족구속이다. README 의 'MG400 한계' 절을 반드시 읽을 것.
       최소 10자세 이상, X/Y/Z 와 R 을 골고루 변화시켜라.
"""
import numpy as np
import cv2

import config as cfg
import robot_io as rio


def main():
    data = np.load(cfg.INTRINSIC_PATH)
    K, dist = data["K"], data["dist"]

    dashboard, _ = rio.connect(cfg.ROBOT_IP, cfg.DASH_PORT, cfg.MOVE_PORT)
    detector = rio.make_aruco_detector(cfg.ARUCO_DICT)

    cap = cv2.VideoCapture(cfg.CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다")

    R_g2b, t_g2b = [], []   # gripper -> base
    R_t2c, t_t2c = [], []   # target  -> cam

    print("[안내] 로봇 정지 후 SPACE=캡처, q=저장후종료")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        corners, ids, _ = detector.detectMarkers(frame)

        view = frame.copy()
        marker_pose = None
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(view, corners, ids)
            for c, i in zip(corners, ids.ravel()):
                if i == cfg.ARUCO_ID:
                    rvec, tvec = rio.estimate_marker_pose(
                        c, cfg.MARKER_SIZE_MM, K, dist)
                    if rvec is not None:
                        marker_pose = (rvec, tvec)
                        cv2.drawFrameAxes(view, K, dist, rvec, tvec,
                                          cfg.MARKER_SIZE_MM * 0.5)

        cv2.putText(view, f"pairs: {len(R_g2b)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("collect", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord(" "):
            if marker_pose is None:
                print("  ! 마커 미검출 — 캡처 취소")
                continue
            # 로봇 포즈
            x, y, z, r = rio.get_pose_xyzr(dashboard)
            Tg2b = rio.gripper2base_matrix(x, y, z, r)
            # 마커 포즈
            rvec, tvec = marker_pose
            Tt2c = rio.rt_to_matrix(rvec, tvec)

            R_g2b.append(Tg2b[:3, :3]); t_g2b.append(Tg2b[:3, 3])
            R_t2c.append(Tt2c[:3, :3]); t_t2c.append(Tt2c[:3, 3])
            print(f"  + 쌍 {len(R_g2b)} 캡처  robot=({x:.1f},{y:.1f},{z:.1f},{r:.1f})")

    cap.release()
    cv2.destroyAllWindows()

    if len(R_g2b) < 3:
        print("  ! 샘플이 너무 적어 저장하지 않음")
        return
    np.savez(cfg.SAMPLES_PATH,
             R_gripper2base=np.array(R_g2b), t_gripper2base=np.array(t_g2b),
             R_target2cam=np.array(R_t2c), t_target2cam=np.array(t_t2c))
    print(f"[저장] {cfg.SAMPLES_PATH}  ({len(R_g2b)} 쌍)")


if __name__ == "__main__":
    main()
