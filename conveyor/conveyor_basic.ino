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


  stepperx.setSpeed(-9000);    // 모터 속도 설정


  stepperx.runSpeed();             // 항상 모터 제어 실행
}

