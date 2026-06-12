# -*- coding: utf-8 -*-
"""
Eye-to-hand 핸드-아이 캘리브레이션 공용 설정.

여기 값만 너의 환경에 맞게 바꾸면 나머지 스크립트는 그대로 동작한다.
"""
import os

# ----------------------------------------------------------------------
# 경로
# ----------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "calib_out")          # 산출물 저장 폴더
os.makedirs(OUT_DIR, exist_ok=True)

INTRINSIC_PATH = os.path.join(OUT_DIR, "intrinsic.npz")      # K, distCoeffs
SAMPLES_PATH   = os.path.join(OUT_DIR, "handeye_samples.npz")  # 로봇/마커 포즈쌍
HANDEYE_PATH   = os.path.join(OUT_DIR, "cam2base.npz")        # 최종 camera->base

# ----------------------------------------------------------------------
# 로봇 / 카메라
# ----------------------------------------------------------------------
ROBOT_IP   = "192.168.1.6"
DASH_PORT  = 29999
MOVE_PORT  = 30003
CAM_INDEX  = 0          # USB 카메라 번호 (dobot_aruco.py 와 동일하게 0)

# ----------------------------------------------------------------------
# ① 내부 캘리브레이션용 체커보드
#    가로/세로 "내부 코너" 개수 (사각형 개수 - 1). 예: 10x7 칸 보드 -> (9, 6)
# ----------------------------------------------------------------------
CHESSBOARD_COLS = 9
CHESSBOARD_ROWS = 6
SQUARE_SIZE_MM  = 25.0   # 체커보드 한 칸의 실제 한 변 길이 (mm)

# ----------------------------------------------------------------------
# ② 핸드-아이용 ArUco 마커 (그리퍼에 부착)
#    dobot_aruco.py 와 동일하게 DICT_4X4_50 사용
# ----------------------------------------------------------------------
ARUCO_DICT      = "DICT_4X4_50"
ARUCO_ID        = 0       # 그리퍼에 붙일 마커 ID
MARKER_SIZE_MM  = 40.0    # 마커 한 변 실제 길이 (mm)
