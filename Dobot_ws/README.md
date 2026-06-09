# Dobot MG400 Workspace

Dobot MG400 4축 로봇을 TCP/IP로 제어하고, 비전(색상/ArUco)·컨베이어와 연동하는 실습 워크스페이스.

## 폴더 구조

| 폴더 | 설명 |
|------|------|
| `TCP-IP-4Axis-Python/` | Dobot 공식 Python SDK 및 제어 스크립트 (메인 작업 폴더) |
| `color_detection/` | OpenCV 색상 검출 / 카메라 캡처 / HSV 도구 |
| `conveyor/` | 컨베이어 제어용 아두이노 스케치 |
| `thread/` | 스레드 사용 예제 |

## 요구사항

- Python 3
- `pip install numpy opencv-python`
- Dobot MG400 (기본 IP `192.168.1.6`), PC와 같은 네트워크에 연결

## 빠른 시작

```bash
cd TCP-IP-4Axis-Python

# 로봇 통신 확인
ping 192.168.1.6

# TCP(엔드이펙터) 위치 실시간 모니터
python3 tcp_monitor.py            # 기본 IP 192.168.1.6
python3 tcp_monitor.py <로봇IP>   # 다른 IP 지정
```

## 주요 스크립트 (TCP-IP-4Axis-Python)

| 파일 | 설명 |
|------|------|
| `dobot_api.py` | Dobot TCP/IP API (Dashboard / Move / Feedback) — 공식 SDK |
| `tcp_monitor.py` | 포트 30004 피드백으로 실제 TCP 포즈 실시간 출력 |
| `main.py`, `example_1.py`, `example_2.py` | SDK 사용 기본 예제 |
| `color_detection.py`, `dobot_aruco.py`, `homograpy.py` | 비전 연동 (색상/ArUco/호모그래피) |

## Dobot TCP/IP 포트 참고

| 포트 | 용도 |
|------|------|
| 29999 | Dashboard — 명령 (EnableRobot, GetPose 등) |
| 30003 | Move — 이동 명령 |
| 30004 | Feedback — 실시간 상태 스트림 (TCP 포즈 포함) |

> SDK 자체 문서는 `TCP-IP-4Axis-Python/README.md` 참고.
