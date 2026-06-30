# Conveyor (Arduino)

컨베이어 벨트를 스테퍼 모터로 구동하고, 적외선 센서로 물체를 감지하는 Arduino 코드입니다.

## 하드웨어 핀

| 핀 | 용도 |
|----|------|
| 8  | 모터 드라이버 enable |
| 2  | 스테퍼 방향(DIR) |
| 5  | 스테퍼 스텝(STEP) |
| 9  | 적외선(IR) 센서 |

## 파일

### `conveyor.ino`
컨베이어 메인 동작 코드.
- `AccelStepper`로 스테퍼 모터를 일정 속도로 회전시켜 벨트를 구동
- 핀 9의 IR 센서를 계속 읽어서:
  - 물체 감지(`HIGH`) → 모터 정지 + 시리얼로 `DETECTED` 전송
  - 물체 없음(`LOW`) → 모터 재가동 + 시리얼로 `CLEAR` 전송
- 상태가 바뀔 때만 메시지를 보내 시리얼 도배를 방지

### `sensor_test/sensor_test.ino`
모터 없이 **적외선 센서만 단독으로 모니터링**하는 테스트 스케치.
- 핀 9 센서 값을 0.2초마다 시리얼로 출력 (`sensor = 1 -> DETECTED` 형식)
- 센서 동작 확인 / 감지 거리(감도) 조정용

## 라이브러리 설치 (AccelStepper)

`conveyor.ino`는 `AccelStepper` 라이브러리가 필요합니다. (센서 테스트 스케치는 불필요)

**방법 1 — 라이브러리 매니저 (권장)**
1. Arduino IDE → `스케치` → `라이브러리 포함` → `라이브러리 관리...`
2. `AccelStepper` 검색 후 설치

**방법 2 — GitHub ZIP 직접 추가**
1. https://github.com/waspinator/AccelStepper 접속
2. 초록색 `Code` 버튼 → `Download ZIP`
3. Arduino IDE → `스케치` → `라이브러리 포함` → `.ZIP 라이브러리 추가...`
4. 다운로드한 ZIP 파일 선택

## 사용법
1. Arduino IDE에서 `.ino` 파일 열기
2. 보드 업로드 후 시리얼 모니터 열기 (보드레이트 **9600**)

> 참고: IR 모듈에 따라 감지 시 `LOW`가 나오는 active-low 타입도 있습니다. 물체를 댔는데 `CLEAR`가 뜨면 `HIGH`/`LOW` 해석을 반대로 하면 됩니다.
