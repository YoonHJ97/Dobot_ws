# Eye-to-Hand 핸드-아이 캘리브레이션 (방식 A · 정석)

카메라를 **고정**(삼각대/프레임)해 작업대를 내려다보게 하고, 그리퍼에 ArUco
마커를 붙여 **camera → robot base 변환**을 구한다. 이후 카메라가 본 임의의
마커/물체를 로봇 base 좌표로 바꿔 집을 수 있다.

```
camera(고정) ──본다──► ArUco 마커(그리퍼에 부착)
        │                         │
        └──── 우리가 구할 것 ──────┘
              T_cam2base  (camera → robot base)
```

## 준비물
- 고정된 USB 카메라 (작업영역 전체가 보이게)
- 인쇄한 **체커보드** (내부코너 `9x6`, 칸 `25mm` 등 — `config.py` 와 일치시킬 것)
- 인쇄한 **ArUco 마커**(DICT_4X4_50, ID=0, 한 변 40mm) → 그리퍼에 단단히 부착
- `pip install numpy opencv-contrib-python`

## 실행 순서

| 단계 | 스크립트 | 산출물 |
|------|----------|--------|
| ① 내부 캘리브레이션 | `python3 calibrate_intrinsic.py` | `calib_out/intrinsic.npz` (K, dist) |
| ② 샘플 수집 | `python3 collect_samples.py` | `calib_out/handeye_samples.npz` |
| ③ 핸드-아이 계산 | `python3 compute_handeye.py` | `calib_out/cam2base.npz` (T_cam2base) |
| ④ 픽 데모 | `python3 pick_demo.py` | — |

> 모든 스크립트는 부모의 `dobot_api.py` 를 자동으로 import 한다(`robot_io.py`가
> `sys.path` 보정). 어느 경로에서 실행해도 되지만 `handeye/` 안에서 실행 권장.

### ① 내부 캘리브레이션
체커보드를 화면 곳곳·여러 각도로 기울여 가며 `SPACE` 로 15~20장 수집 →
`c` 로 계산/저장. RMS 재투영오차가 **< 0.5px** 면 양호.

### ② 샘플 수집
1. 로봇을 한 자세로 옮긴다(teach 모드로 손으로, 또는 별도 콘솔의 `MovL`).
2. **완전히 정지**한 상태에서 `SPACE` → 로봇포즈 + 마커포즈 한 쌍 기록.
3. 자세를 바꿔 **10회 이상** 반복. X/Y/Z 와 R(yaw)을 골고루.

### ③ 핸드-아이 계산
`cv2.calibrateHandEye` 를 eye-to-hand 트릭(gripper2base 를 base2gripper 로
반전 입력)으로 풀어 `T_cam2base` 를 얻는다. 끝에 출력되는 **마커 base 위치
표준편차**가 작을수록(수 mm 이내) 일관성이 좋다.

### ④ 픽 데모
대상 마커(`PICK_OBJECT_ID`)를 검출 → base 좌표 변환 → 안전높이로 `MovL`.
**처음엔 `APPROACH_Z` 를 높게** 두고 하강은 직접 확인하며 줄여라.

---

## ⚠️ MG400(4축)의 근본적 한계 — 꼭 읽기

정석 핸드-아이(`calibrateHandEye`)는 **회전축이 2개 이상 서로 다른 방향**으로
변해야 완전히 구속된다. 그런데 **MG400은 4축 SCARA라 회전이 베이스 Z축
하나뿐**이다(툴은 항상 아래를 향함). 즉 모든 자세의 회전축이 평행 → 수학적으로
**부족구속(rank-deficient)** 이라, 카메라와 회전축 사이의 일부 성분(특히 Z
오프셋)이 잘 관측되지 않는다.

**그래서 현실적으로:**
- 이 스켈레톤은 **개념 학습 + 6축 로봇 전환 대비**용으로는 완성형이다.
- **MG400에서 평면 위 픽앤플레이스가 목적이라면**, 사실
  **방식 B(평면 호모그래피, `../homograpy.py`)가 더 적합**하다. 한 평면·고정
  높이에서는 호모그래피가 더 간단하고 안정적이다.
- 굳이 방식 A를 MG400에 쓰려면: 마커를 **기울여 부착**하거나 여러 높이에서
  샘플을 모아 병진(translation) 관측을 늘리고, 결과의 **표준편차로 신뢰도**를
  반드시 확인하라.

요약: **학습/정석 = 방식 A**, **MG400 실전 평면작업 = 방식 B** 를 권장.
