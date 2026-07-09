#include <AccelStepper.h>

// 컨베이어 프로젝트용 스케치.
// 물체를 감지하면 컨베이어를 멈추고 "DETECTED" 를 보낸 뒤,
// Dobot 쪽에서 "RESUME" 신호를 보내줄 때까지 정지 상태로 대기한다.
// (기존 conveyor.ino 처럼 물체가 치워지면 자동 재개하지 않는다.)

#define enablePin 8
#define dirxPin 2
#define stepxPin 5
#define sensorx 9
#define motorInterfaceType 1

AccelStepper stepperx = AccelStepper(motorInterfaceType, stepxPin, dirxPin);

const long RUN_SPEED = -9000;   // 컨베이어 이동 속도
bool waitingForResume = false;  // true 면 Dobot의 RESUME 를 기다리는 중 (모터 정지)

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(sensorx, INPUT);

  digitalWrite(enablePin, LOW);   // 모터 활성화 (LOW 가 활성화인 경우)
  stepperx.setMaxSpeed(10000);
  stepperx.setSpeed(RUN_SPEED);
  Serial.begin(9600);
}

void loop() {
  // 1) RESUME 대기 모드: 모터 정지 상태로 Dobot 신호만 기다린다.
  if (waitingForResume) {
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "RESUME") {
        waitingForResume = false;
        stepperx.setSpeed(RUN_SPEED);   // 컨베이어 재개
        Serial.println("RESUMED");      // (디버그용) 재개 확인 신호
      }
    }
    return;   // 대기 중에는 모터를 굴리지 않음 (정지 유지)
  }

  // 2) 정상 이동 모드: 물체 감지 시 정지 + DETECTED 전송 후 대기 모드로 전환.
  int statex = digitalRead(sensorx);
  if (statex == HIGH) {             // 물체 감지
    stepperx.setSpeed(0);           // 모터 정지
    Serial.println("DETECTED");     // Dobot 에 감지 신호 전송
    waitingForResume = true;        // RESUME 올 때까지 대기
    return;
  }

  stepperx.runSpeed();              // 물체 없으면 계속 이동
}
