#include <AccelStepper.h>

#define enablePin 8
#define dirxPin 2
#define stepxPin 5
#define sensorx 9
#define motorInterfaceType 1

AccelStepper stepperx = AccelStepper(motorInterfaceType, stepxPin, dirxPin);
int statex;         // 현재 센서 상태
int previousState;  // 이전 센서 상태 (flag 역할)

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(sensorx, INPUT);

  digitalWrite(enablePin, LOW);   // 모터 활성화 (LOW가 활성화인 경우)
  stepperx.setMaxSpeed(10000);    // 최대 속도 설정
  stepperx.setSpeed(-9000);       // 초기 회전 속도 설정
  Serial.begin(9600);             // 시리얼 통신 시작

  previousState = LOW;            // 초기 상태 설정
}

void loop() {
  statex = digitalRead(sensorx); // 적외선 센서 값 읽기

  // 상태가 변경되었을 때만 출력
  if (statex != previousState) {
    if (statex == HIGH) {          // 물체가 감지되면
      stepperx.setSpeed(0);        // 모터 정지
      Serial.println("DETECTED");  // "DETECTED" 메시지 전송
    } else {                       // 물체가 감지되지 않으면
      stepperx.setSpeed(-9000);    // 모터 속도 설정
      Serial.println("CLEAR");     // "CLEAR" 메시지 전송
    }
    previousState = statex;        // 이전 상태를 현재 상태로 업데이트
  }

  stepperx.runSpeed();             // 항상 모터 제어 실행
}

