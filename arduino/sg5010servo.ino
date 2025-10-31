#include <Arduino.h>
#include <Servo.h>

#define SERVO_PIN    9
#define BAUD         9600
#define MIN_US       500
#define MAX_US       2500

Servo servo_;

static int degToUs(float deg) {
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  float ratio = deg / 180.0f;                 // 0.0 .. 1.0
  int us = (int)(MIN_US + ratio * (MAX_US - MIN_US) + 0.5f);
  return us;
}

void setup() {
  Serial.begin(BAUD);
  Serial.setTimeout(10);                      // 簡易タイムアウト
  servo_.attach(SERVO_PIN, MIN_US, MAX_US);   // 明示レンジ
  servo_.writeMicroseconds((MIN_US + MAX_US) / 2); // センター(約90°)
  Serial.println(F("READY"));
}

void loop() {
  // 1行（改行まで）を文字列で受け取るだけ。例: "90.5\n"
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();                               // 余分な空白/CR除去
    if (line.length() == 0) return;

    // 小数角をパース（例: "90.5"）
    float deg = line.toFloat();                // 失敗時は0になるが、そのまま進める
    int us = degToUs(deg);

    servo_.writeMicroseconds(us);
    Serial.print(F("OK "));
    Serial.println(us);
  }
}

