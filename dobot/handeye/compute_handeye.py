# -*- coding: utf-8 -*-
"""
[3단계] 수집한 포즈쌍으로 eye-to-hand 변환(camera->base) 계산.

핵심: cv2.calibrateHandEye 는 본래 eye-in-hand(카메라가 손에 달린 경우)용으로
      cam->gripper 를 푼다. eye-to-hand(카메라 고정, 마커가 손에 부착)에서는
      gripper2base 를 '뒤집어서'(base2gripper) 넣으면 결과가 cam->base 가 된다.

      AX = XB 표준 트릭:
          입력  A = base2gripper,  B = target2cam
          출력  X = cam2base

사용법:
    python3 compute_handeye.py
"""
import numpy as np
import cv2

import config as cfg


def invert_Rt(R, t):
    """R,t (a->b) 를 b->a 로 반전."""
    Rt = R.T
    return Rt, -Rt @ t


def main():
    s = np.load(cfg.SAMPLES_PATH)
    R_g2b, t_g2b = s["R_gripper2base"], s["t_gripper2base"]
    R_t2c, t_t2c = s["R_target2cam"],  s["t_target2cam"]

    # eye-to-hand: gripper2base -> base2gripper 로 반전해서 입력
    R_b2g, t_b2g = [], []
    for R, t in zip(R_g2b, t_g2b):
        Ri, ti = invert_Rt(R, t)
        R_b2g.append(Ri); t_b2g.append(ti)

    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_b2g, t_b2g,
        list(R_t2c), [t.reshape(3, 1) for t in t_t2c],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    T = np.eye(4)
    T[:3, :3] = R_cam2base
    T[:3, 3] = t_cam2base.reshape(3)
    print("[결과] T_cam2base =\n", np.round(T, 3))

    np.savez(cfg.HANDEYE_PATH, T_cam2base=T,
             R_cam2base=R_cam2base, t_cam2base=t_cam2base)
    print(f"[저장] {cfg.HANDEYE_PATH}")

    # 간단 검증: 각 샘플에서 (cam2base * target2cam) 으로 추정한
    # 마커의 base 위치가 서로 얼마나 일관적인지(표준편차) 출력.
    pts = []
    for R, t in zip(R_t2c, t_t2c):
        Tt2c = np.eye(4); Tt2c[:3, :3] = R; Tt2c[:3, 3] = t
        pts.append((T @ Tt2c)[:3, 3])
    pts = np.array(pts)
    print("[검증] 마커 base 위치 표준편차(mm) =", np.round(pts.std(axis=0), 2))
    print("       (값이 작을수록 캘리브레이션이 일관적)")


if __name__ == "__main__":
    main()
