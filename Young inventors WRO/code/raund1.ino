#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// --- Pinlər ---
const int IN1 = 4;
const int IN2 = 7;   // Sol motor
const int IN3 = 8;
const int IN4 = 12;  // Sağ motor
const int ENA = 5;
const int ENB = 6;

const int SERVO_PIN = 2;

// Sensorlar
const int TRIG_L = 10;
const int ECHO_L = 9;
const int TRIG_F = A0;
const int ECHO_F = A1;

// Servo limitləri
const int SERVO_CENTER = 88;
const int SERVO_LEFT = 125;

// Motor sürət
const int SPEED_SLOW = 60;
const int SPEED_FAST = 80;

// Sensor limitləri
const float FRONT_THRESHOLD_CM = 95.0; 
const float LEFT_THRESHOLD_CM  = 40.0;

// Gyro / turn param
const float TARGET_TURN_DEG = 90.0;
const int GYRO_CAL_SAMPLES = 400;

MPU6050 mpu;
Servo steer;

// Dönüş rejimi flag
bool turning = false;
unsigned long turnStart = 0;

// gyro inteqrasiyası
float gyroZBias = 0.0;
float turnedAngle = 0.0;
unsigned long lastGyroTime = 0;

// dönmə sayğacı
int turnCount = 0;
bool stopped = false;

// ⏱ Backup üçün start vaxtı
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connected.");
  }

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);

  steer.attach(SERVO_PIN);
  steer.write(SERVO_CENTER);
  Serial.print("Servo bucaq: ");
  Serial.println(SERVO_CENTER);

  // ⚡ Enerji veriləndə 2 saniyə gözləsin
  Serial.println("Power ON, waiting 2s before gyro calibration...");
  delay(2000);

  calibrateGyroZ();

  motorsForward(SPEED_FAST, SPEED_FAST);

  lastGyroTime = micros();
  startTime = millis();  // ✅ backup üçün start vaxtı

  Serial.println("Started: MPU + Front/Left sensors + gyro-based turn");
}

void loop() {
  // Əgər artıq dayanıbsa
  if (stopped) {
    motorsForward(0, 0);
    return;
  }

  // ✅ Backup: 26 saniyə keçdisə robotu dayandır
  if (millis() - startTime >= 26500) {
    stopped = true;
    motorsForward(0, 0);
    steer.write(SERVO_CENTER);
    Serial.println("⏱ Backup: 26 saniyə tamamlandı, robot dayandı.");
    return;
  }

  float frontDist = readDistanceCM(TRIG_F, ECHO_F);
  float leftDist  = readDistanceCM(TRIG_L, ECHO_L);

  int motorSpeed = (frontDist < FRONT_THRESHOLD_CM) ? SPEED_SLOW : SPEED_FAST;
  motorsForward(motorSpeed, motorSpeed);

  updateGyroIntegration();

  if (!turning && (leftDist > LEFT_THRESHOLD_CM && frontDist < FRONT_THRESHOLD_CM)) {
    turning = true;
    turnStart = millis();
    turnedAngle = 0.0;      // ✅ hər dönüşdə sıfırla
    lastGyroTime = micros();
    Serial.println("Corner detected! Starting gyro-controlled LEFT turn...");
  }

  if (turning) {
    steer.write(SERVO_LEFT);
    Serial.print("Servo bucaq: ");
    Serial.println(SERVO_LEFT);

    if (abs(turnedAngle) >= TARGET_TURN_DEG) {
      turning = false;
      steer.write(SERVO_CENTER);
      Serial.print("Servo bucaq: ");
      Serial.println(SERVO_CENTER);

      delay(120);
      Serial.println("Turn finished (by gyro). Going straight.");

      // ✅ hər 90°-də sayğac artır
      turnCount++;
      Serial.print("Turn count: ");
      Serial.println(turnCount);

      if (turnCount >= 12) {
        stopped = true;
        motorsForward(0, 0);
        steer.write(SERVO_CENTER);
        Serial.println("✅ 12 dönmə tamamlandı. Robot dayandı.");
      }
    }
    else {
      if (frontDist > FRONT_THRESHOLD_CM + 10.0) {
        turning = false;
        steer.write(SERVO_CENTER);
        Serial.print("Servo bucaq: ");
        Serial.println(SERVO_CENTER);

        Serial.println("Turn ended early: front opened. Going straight.");
      }
    }
  }

  Serial.print("Front: "); Serial.print(frontDist);
  Serial.print(" cm | Left: "); Serial.print(leftDist);
  Serial.print(" cm | Motor: "); Serial.print(motorSpeed);
  Serial.print(" | turned(deg): "); Serial.println(turnedAngle);

  delay(30);
}

// ================== Funksiyalar ==================

void calibrateGyroZ() {
  Serial.println("Calibrating gyro Z bias...");
  long sum = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    int16_t gz = mpu.getRotationZ();
    sum += gz;
    delay(3);
  }
  gyroZBias = (float)sum / (float)GYRO_CAL_SAMPLES;
  Serial.print("Gyro Z bias (LSB) = "); Serial.println(gyroZBias, 3);
}

void updateGyroIntegration() {
  unsigned long now = micros();
  float dt = (now - lastGyroTime) / 1000000.0;
  if (dt <= 0) return;
  lastGyroTime = now;

  int16_t gz_raw = mpu.getRotationZ();
  float gz = ((float)gz_raw - gyroZBias) / 131.0;
  turnedAngle += gz * dt;
}

float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return 1000.0;
  float cm = (duration / 2.0) / 29.1;
  if (cm > 1000.0) cm = 1000.0;
  return cm;
}

void motorsForward(int leftPWM, int rightPWM) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, constrain(leftPWM,0,255));
  analogWrite(ENB, constrain(rightPWM,0,255));
}
