#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <MPU9250.h>

// -------------------- Motor Pins --------------------
const int ENA = 9;   // Left motor PWM
const int IN1 = 3;   // Left motor dir
const int IN2 = 4;   // Left motor dir

const int ENB = 10;  // Right motor PWM
const int IN3 = 5;   // Right motor dir
const int IN4 = 6;   // Right motor dir

// -------------------- TCA9548A --------------------
#define TCA_ADDR 0x70
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// -------------------- Global APDS Object --------------------
Arduino_APDS9960 APDS;  // one object reused for all channels

// -------------------- PID --------------------
float Kp = 2.0, Ki = 0.0, Kd = 0.5;
float lastError = 0, integral = 0;

// -------------------- Movement --------------------
const int BASE_SPEED = 150;
const int MAX_SPEED = 255;
const unsigned long TIME_PER_CELL_MS = 700;

// -------------------- Proximity --------------------
int readProximity(uint8_t channel) {
  tcaSelect(channel);
  if (APDS.proximityAvailable()) {
    return APDS.readProximity();
  }
  return -1;
}

int calculateDynamicThreshold(uint8_t channel) {
  const int NUM_READINGS = 5;
  int total = 0;
  for (int i = 0; i < NUM_READINGS; i++) {
    int val = readProximity(channel);
    if (val > 0) total += val;
    delay(20);
  }
  return (total / NUM_READINGS) * 0.8;
}

// -------------------- Motor Control --------------------
void setMotor(int pin1, int pin2, int speed, int pwmPin) {
  if (speed > 0) {
    digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW);
  } else if (speed < 0) {
    digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW); digitalWrite(pin2, LOW);
  }
  analogWrite(pwmPin, abs(speed));
}

void driveMotors(int leftSpeed, int rightSpeed) {
  setMotor(IN1, IN2, leftSpeed, ENA);
  setMotor(IN3, IN4, rightSpeed, ENB);
}

void moveForward() {
  unsigned long start = millis();
  while (millis() - start < TIME_PER_CELL_MS) {
    int leftDist  = readProximity(0); // Left on channel 0
    int rightDist = readProximity(1); // Right on channel 1

    if (leftDist == -1 || rightDist == -1) {
      driveMotors(BASE_SPEED, BASE_SPEED);
      continue;
    }

    float error = leftDist - rightDist;
    integral += error;
    float derivative = error - lastError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    driveMotors(leftSpeed, rightSpeed);
  }
  driveMotors(0, 0);
}

void turnLeft() {
  driveMotors(-BASE_SPEED, BASE_SPEED);
  delay(400);  // adjust for 90° turn
  driveMotors(0, 0);
}

void turnRight() {
  driveMotors(BASE_SPEED, -BASE_SPEED);
  delay(400);  // adjust for 90° turn
  driveMotors(0, 0);
}

void turnAround() {
  driveMotors(BASE_SPEED, -BASE_SPEED);
  delay(800);  // adjust for 180° turn
  driveMotors(0, 0);
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Init all APDS sensors via multiplexer
  for (int ch = 0; ch < 4; ch++) {
    tcaSelect(ch);
    if (!APDS.begin()) {
      Serial.print("APDS not found on channel "); Serial.println(ch);
    } else {
      Serial.print("APDS ready on channel "); Serial.println(ch);
    }
  }

  Serial.println("Setup complete!");
}

// -------------------- Maze Solving --------------------
void solve() {
  // Dynamic thresholds
  int thLeft  = calculateDynamicThreshold(0);
  int thRight = calculateDynamicThreshold(1);
  int thFL    = calculateDynamicThreshold(2);
  int thFR    = calculateDynamicThreshold(3);

  // Read sensors
  int left       = readProximity(0);
  int right      = readProximity(1);
  int frontLeft  = readProximity(2);
  int frontRight = readProximity(3);

  Serial.print("FL: "); Serial.print(frontLeft);
  Serial.print(" | FR: "); Serial.print(frontRight);
  Serial.print(" | L: "); Serial.print(left);
  Serial.print(" | R: "); Serial.println(right);

  // Open path check
  bool leftOpen  = (left < thLeft);
  bool rightOpen = (right < thRight);
  bool frontOpen = ((frontLeft + frontRight) / 2) < ((thFL + thFR) / 2);

  if ((leftOpen + rightOpen + frontOpen) >= 2) {
    Serial.println("Crossroad detected!");
  }

  if (rightOpen) {
    Serial.println("Turning Right");
    turnRight();
    moveForward();
  } else if (frontOpen) {
    Serial.println("Moving Forward");
    moveForward();
  } else if (leftOpen) {
    Serial.println("Turning Left");
    turnLeft();
    moveForward();
  } else {
    Serial.println("Dead end! Turning Around");
    turnAround();
    moveForward();
  }
}

// -------------------- Loop --------------------
void loop() {
  solve();
  delay(200);
}
