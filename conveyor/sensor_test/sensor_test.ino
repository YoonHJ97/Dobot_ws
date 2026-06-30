// 적외선 센서(sensorx, 핀 9) 단독 모니터링 스케치
// 모터 없이 센서 값만 실시간으로 시리얼 모니터에 출력합니다.

#define sensorx 9

int statex;  // 현재 센서 상태

void setup() {
  pinMode(sensorx, INPUT);
  Serial.begin(9600);
  Serial.println("=== IR sensor monitor start ===");
}

void loop() {
  statex = digitalRead(sensorx);

  // 현재 센서 값을 계속 출력
  Serial.print("sensor = ");
  Serial.print(statex);
  Serial.print("  -> ");
  Serial.println(statex == HIGH ? "DETECTED" : "CLEAR");

  delay(200);  // 출력 간격 (0.2초)
}
