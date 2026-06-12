# -*- coding: utf-8 -*-
"""
MG400 연결 / 포즈 조회·이동 헬퍼 + ArUco 포즈 추정 유틸.

부모 폴더(dobot/)의 dobot_api 를 import 할 수 있도록 sys.path 를 보정한다.
따라서 이 스크립트들은 어느 경로에서 실행해도 된다.
"""
import os
import re
import sys
import numpy as np
import cv2

# dobot/ 를 import 경로에 추가 (handeye/ 의 부모)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dobot_api import DobotApiDashboard, DobotApiMove  # noqa: E402


# ======================================================================
# 로봇
# ======================================================================
def connect(ip, dash_port, move_port):
    """Dashboard(상태/포즈) + Move(이동) 클라이언트 반환."""
    dashboard = DobotApiDashboard(ip, dash_port)
    move = DobotApiMove(ip, move_port)
    dashboard.EnableRobot()
    return dashboard, move


def _parse_reply(reply):
    """'0,{x,y,z,r,...},GetPose();' 형태 응답에서 { } 안 숫자 리스트 추출."""
    m = re.search(r"\{([^}]*)\}", reply)
    if not m:
        raise RuntimeError(f"응답 파싱 실패: {reply!r}")
    return [float(v) for v in m.group(1).split(",") if v.strip() != ""]


def get_pose_xyzr(dashboard):
    """현재 TCP 포즈 (X, Y, Z, R) [mm, mm, mm, deg] 반환."""
    vals = _parse_reply(dashboard.GetPose())
    return vals[0], vals[1], vals[2], vals[3]


def gripper2base_matrix(x, y, z, r_deg):
    """
    MG400 포즈(X,Y,Z,R)를 4x4 동차변환 T_gripper2base 로.

    MG400은 4축이라 회전은 베이스 Z축 기준 R(yaw)뿐이고,
    툴은 항상 아래(roll=pitch=0)를 향한다고 가정한다.
    """
    rz = np.deg2rad(r_deg)
    c, s = np.cos(rz), np.sin(rz)
    T = np.eye(4)
    T[:3, :3] = np.array([[c, -s, 0],
                          [s,  c, 0],
                          [0,  0, 1]])
    T[:3, 3] = [x, y, z]
    return T


# ======================================================================
# 카메라 / ArUco
# ======================================================================
def make_aruco_detector(dict_name):
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def estimate_marker_pose(corners, marker_size_mm, K, dist):
    """
    단일 ArUco 마커 코너 -> (rvec, tvec) = target2cam.

    estimatePoseSingleMarkers 는 신버전 OpenCV에서 제거됐으므로
    solvePnP 로 직접 푼다(버전 안전). 단위는 mm.
    """
    s = marker_size_mm / 2.0
    # ArUco 코너 순서: TL, TR, BR, BL  (마커 평면 z=0)
    objp = np.array([[-s,  s, 0],
                     [ s,  s, 0],
                     [ s, -s, 0],
                     [-s, -s, 0]], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(objp, corners.reshape(-1, 2), K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None, None
    return rvec, tvec


def rt_to_matrix(rvec, tvec):
    """(rvec, tvec) -> 4x4 동차변환."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T
