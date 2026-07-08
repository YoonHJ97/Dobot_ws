# Dobot MG400 Workspace

Dobot MG400 4축 로봇을 TCP/IP로 제어하고, 비전(색상/ArUco)·컨베이어와 연동하는 실습 워크스페이스.

## 폴더 구조

| 폴더 | 설명 |
|------|------|
| `dobot/` | Dobot 공식 Python SDK 및 제어 스크립트 (메인 작업 폴더) |
| `color_detection/` | OpenCV 색상 검출 / 카메라 캡처 / HSV 도구 |
| `conveyor/` | 컨베이어 제어용 아두이노 스케치 |
| `thread/` | 스레드 사용 예제 |

## 요구사항

- Python 3
- `pip install numpy opencv-python`
- Dobot MG400 (기본 IP `192.168.1.6`), PC와 같은 네트워크에 연결

## 빠른 시작

```bash
cd dobot

# 로봇 통신 확인
ping 192.168.1.6

# TCP(엔드이펙터) 위치 실시간 모니터
python3 tcp_monitor.py            # 기본 IP 192.168.1.6
python3 tcp_monitor.py <로봇IP>   # 다른 IP 지정
```

## 주요 스크립트 (dobot)

| 파일 | 설명 |
|------|------|
| `dobot_api.py` | Dobot TCP/IP API (Dashboard / Move / Feedback) — 공식 SDK |
| `tcp_monitor.py` | 포트 30004 피드백으로 실제 TCP 포즈 실시간 출력 |
| `mainUI.py`, `ui.py` | Tkinter 기반 로봇 제어 GUI |
| `example_1.py`, `Practice*.py`, `Project_serial.py` | SDK 사용 예제 / 실습 |
| `dobot_aruco.py`, `homograpy.py` | 비전 연동 (ArUco / 호모그래피) |

## 그리퍼 버전: 진공(vacuum) vs 전자석(magnet)

같은 픽 앤 플레이스 로직을 **진공 그리퍼**와 **전자석 그리퍼** 두 가지 버전으로 제공한다.
동작·좌표·스레드 구조는 동일하고 **그리퍼를 켜고 끄는 부분만** 다르다.

| vacuum 버전 | magnet 버전 | 내용 |
|-------------|-------------|------|
| `Practice_vacuum.py` | `Practice_magnet.py` | 단일 픽 앤 플레이스 |
| `Practice_palletizing.py` | `Practice_palletizing_magnet.py` | 팔레타이징(격자 적재) |
| `Project_serial.py` | `Project_serial_magnet.py` | 아두이노 시리얼 + 색상 분류 |
| `Practice.ipynb` | `Practice_magnet.ipynb` | 노트북 실습 |

**차이점 (그리퍼 제어)**

| 구분 | vacuum | magnet |
|------|--------|--------|
| 제어 함수 | `ActivateVacuumGripper()` | `ActivateMagnet()` |
| 출력 명령 | `dashboard.DO(1, status)` (베이스 IO) | `dashboard.ToolDO(MAGNET_TOOL_DO, status)` (툴 IO) |
| 의미 | ON=흡착 / OFF=해제 | ON=자화(부착) / OFF=소자(분리) |

> 전자석 채널 번호는 각 magnet 파일 상단의 `MAGNET_TOOL_DO`(기본 `1`)로 지정한다. 배선에 맞게 수정하고, 툴 IO가 아니라 베이스 IO에 연결했다면 `ToolDO` 대신 `DO`를 사용한다.

## Dobot TCP/IP 포트 참고

| 포트 | 용도 |
|------|------|
| 29999 | Dashboard — 명령 (EnableRobot, GetPose 등) |
| 30003 | Move — 이동 명령 |
| 30004 | Feedback — 실시간 상태 스트림 (TCP 포즈 포함) |

> 로봇 기본 IP는 `192.168.1.6`. 스크립트는 `dobot/` 폴더 안에서 실행해야 import가 정상 동작한다.
