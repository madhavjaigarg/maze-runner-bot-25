#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include <Arduino.h>
#include <queue>
#include <vector>
#include <utility>
#include <cstdint>
#include <cmath>
#include <MPU9250_WE.h>

namespace Mouse {

// -------------------- Motor Pins --------------------
const int ENA = 9;
const int IN1 = 3;
const int IN2 = 4;
const int ENB = 10;
const int IN3 = 5;
const int IN4 = 6;

// -------------------- Encoder Pins --------------------
const int ENCODER_LEFT_PIN = 2;
const int ENCODER_RIGHT_PIN = A2;

// -------------------- Parameters --------------------
const int THRESHOLD_PROX = 150;
const int BASE_SPEED = 150;
const int MAX_SPEED = 200;
const int PID_LOOP_DELAY_MS = 5;
const int CELL_DISTANCE_TICKS = 1000;
const float TURN_90_DEGREES = 90.0f;

// -------------------- APDS Proximity Sensors --------------------
Adafruit_APDS9960 sensorFrontLeft;
Adafruit_APDS9960 sensorFrontRight;
Adafruit_APDS9960 sensorLeft;
Adafruit_APDS9960 sensorRight;

// -------------------- MPU9250 --------------------
#define MPU9250_ADDR 0x68
MPU9250_WE myIMU = MPU9250_WE(MPU9250_ADDR);

float currentGyroZAngle = 0.0f;
unsigned long previousGyroReadTime = 0;

// -------------------- Stacks --------------------
#define MAX_STACK_SIZE 100
class Stack {
private:
    int items[MAX_STACK_SIZE];
    int topIndex = -1;
public:
    void push(int value) {
        if (topIndex < MAX_STACK_SIZE - 1) items[++topIndex] = value;
    }
    int pop() {
        if (topIndex >= 0) return items[topIndex--];
        return -1;
    }
    int top() {
        return (topIndex >= 0) ? items[topIndex] : -1;
    }
    bool isEmpty() {
        return topIndex < 0;
    }
    int size() {
        return topIndex + 1;
    }
};

Stack moveHistory;
Stack crossroadHistory;

// -------------------- PID Control --------------------
struct PIDGains { float Kp, Ki, Kd; };
struct PIDController {
    float Kp, Ki, Kd;
    float previousError, integral;
    unsigned long lastTime;
    float outputLimit;

    void init(float p, float i, float d, float limit) {
        Kp = p; Ki = i; Kd = d;
        previousError = 0; integral = 0;
        lastTime = millis();
        outputLimit = limit;
    }

    float calculate(float setpoint, float feedback) {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime == lastTime) ? 0.001f : (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;

        float error = setpoint - feedback;
        integral += error * deltaTime;
        float derivative = (error - previousError) / deltaTime;
        previousError = error;

        float output = Kp * error + Ki * integral + Kd * derivative;
        return constrain(output, -outputLimit, outputLimit);
    }

    void reset() {
        previousError = 0; integral = 0;
        lastTime = millis();
    }
};

PIDGains STRAIGHT_PID_GAINS = {0.5f, 0.01f, 0.005f};
PIDGains DISTANCE_PID_GAINS = {0.8f, 0.01f, 0.005f};
PIDGains TURN_PID_GAINS = {3.0f, 0.1f, 0.05f};

float STRAIGHT_PID_LIMIT = 50.0f;
float DISTANCE_PID_LIMIT = 50.0f;
float TURN_PID_LIMIT = 100.0f;

PIDController straightPID, distancePID, turnPID;

// -------------------- Motor Control --------------------
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    digitalWrite(IN1, leftSpeed >= 0); digitalWrite(IN2, leftSpeed < 0);
    analogWrite(ENA, abs(leftSpeed));
    digitalWrite(IN3, rightSpeed >= 0); digitalWrite(IN4, rightSpeed < 0);
    analogWrite(ENB, abs(rightSpeed));
}
void stopMotors() { setMotorSpeeds(0, 0); }

// -------------------- Encoder Tracking --------------------
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
void isrLeftEncoder() { leftEncoderCount++; }
void isrRightEncoder() { rightEncoderCount++; }

// -------------------- MPU9250 Functions --------------------
void initMPU() {
    if (!myIMU.init()) {
        Serial.println("MPU9250 not found!");
        while (1);
    }
    Serial.println("MPU9250 ready.");

    myIMU.autoOffsets();
    myIMU.enableGyr();
    myIMU.setGyrDLPF(MPU9250_DLPF_41HZ);
    myIMU.setSampleRateDivider(5);

    previousGyroReadTime = millis();
}

void updateGyroAngle() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousGyroReadTime) / 1000.0f;
    previousGyroReadTime = currentTime;

    xyzFloat g = myIMU.getGyrValues();  // gyro in °/s
    currentGyroZAngle += g.z * deltaTime;

    // Keep yaw between -180 and 180
    if (currentGyroZAngle > 180.0f) currentGyroZAngle -= 360.0f;
    if (currentGyroZAngle < -180.0f) currentGyroZAngle += 360.0f;
}

// -------------------- Movement Functions --------------------
void moveForwardOneCell() {
    Serial.println("Moving forward one cell (PID)");
    leftEncoderCount = 0; rightEncoderCount = 0;
    straightPID.reset(); distancePID.reset();
    long targetTicks = CELL_DISTANCE_TICKS;
    unsigned long startMillis = millis();

    while (true) {
        long left = leftEncoderCount, right = rightEncoderCount;
        long avg = (left + right) / 2;

        if (avg >= targetTicks || millis() - startMillis > 2000) break;

        float straightCorrection = straightPID.calculate(0, left - right);
        float distanceCorrection = distancePID.calculate(0, -(targetTicks - avg));

        int leftSpeed = BASE_SPEED + (int)(distanceCorrection - straightCorrection);
        int rightSpeed = BASE_SPEED + (int)(distanceCorrection + straightCorrection);
        setMotorSpeeds(constrain(leftSpeed, 0, MAX_SPEED), constrain(rightSpeed, 0, MAX_SPEED));

        delay(PID_LOOP_DELAY_MS);
    }
    stopMotors();
    Serial.println("Finished forward move.");
}

void turnDegrees(float targetDegrees) {
    Serial.print("Turning "); Serial.print(targetDegrees); Serial.println(" deg");
    float initial = currentGyroZAngle;
    turnPID.reset();
    unsigned long startMillis = millis();
    float target = initial + targetDegrees;

    while (abs(target - currentGyroZAngle) > 2.0f && millis() - startMillis < 3000) {
        updateGyroAngle();
        float error = target - currentGyroZAngle;
        float correction = turnPID.calculate(0, -error);

        int leftSpeed = (targetDegrees > 0 ? 1 : -1) * (BASE_SPEED + (int)correction);
        int rightSpeed = -leftSpeed;

        setMotorSpeeds(constrain(leftSpeed, -MAX_SPEED, MAX_SPEED), constrain(rightSpeed, -MAX_SPEED, MAX_SPEED));
        delay(PID_LOOP_DELAY_MS);
    }
    stopMotors();
    Serial.println("Turn complete.");
}

void turnLeft() { turnDegrees(-TURN_90_DEGREES); }
void turnRight() { turnDegrees(TURN_90_DEGREES); }
void turnAround() { turnDegrees(180.0f); }

// -------------------- Proximity Sensor Reading --------------------
uint8_t readProximity(Adafruit_APDS9960& sensor) {
    return sensor.readProximity();
}

// -------------------- Setup --------------------
void doStuff() {
    Wire.begin();
    Serial.begin(9600);
    delay(100);

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), isrRightEncoder, CHANGE);

    straightPID.init(STRAIGHT_PID_GAINS.Kp, STRAIGHT_PID_GAINS.Ki, STRAIGHT_PID_GAINS.Kd, STRAIGHT_PID_LIMIT);
    distancePID.init(DISTANCE_PID_GAINS.Kp, DISTANCE_PID_GAINS.Ki, DISTANCE_PID_GAINS.Kd, DISTANCE_PID_LIMIT);
    turnPID.init(TURN_PID_GAINS.Kp, TURN_PID_GAINS.Ki, TURN_PID_GAINS.Kd, TURN_PID_LIMIT);

    initMPU();  // start MPU9250

    if (!sensorFrontLeft.begin()) Serial.println("Front Left APDS not found");
    if (!sensorFrontRight.begin()) Serial.println("Front Right APDS not found");
    if (!sensorLeft.begin()) Serial.println("Left APDS not found");
    if (!sensorRight.begin()) Serial.println("Right APDS not found");

    sensorFrontLeft.enableProximity(true);
    sensorFrontRight.enableProximity(true);
    sensorLeft.enableProximity(true);
    sensorRight.enableProximity(true);

    Serial.println("Robot ready.");
    delay(100);
}

// -------------------- Maze Solving Logic --------------------
void solve() {
    updateGyroAngle();

    uint8_t leftDist = readProximity(sensorLeft);
    uint8_t rightDist = readProximity(sensorRight);
    uint8_t frontDist1 = readProximity(sensorFrontLeft);
    uint8_t frontDist2 = readProximity(sensorFrontRight);
    uint8_t frontDist = (frontDist1 + frontDist2) / 2;

    Serial.print("L: "); Serial.print(leftDist);
    Serial.print(" | F: "); Serial.print(frontDist);
    Serial.print(" | R: "); Serial.println(rightDist);

    bool leftOpen = leftDist < THRESHOLD_PROX;
    bool frontOpen = frontDist < THRESHOLD_PROX;
    bool rightOpen = rightDist < THRESHOLD_PROX;

    if ((leftOpen + frontOpen + rightOpen) >= 2) {
        if (crossroadHistory.isEmpty() || crossroadHistory.top() != moveHistory.size()) {
            crossroadHistory.push(moveHistory.size());
            Serial.println("Crossroad Detected!");
        }
    }

    if (rightOpen) {
        Serial.println("Turning Right");
        turnRight(); moveForwardOneCell(); moveHistory.push(1);
    } else if (frontOpen) {
        Serial.println("Moving Forward");
        moveForwardOneCell(); moveHistory.push(0);
    } else if (leftOpen) {
        Serial.println("Turning Left");
        turnLeft(); moveForwardOneCell(); moveHistory.push(3);
    } else {
        Serial.println("Dead End. Backtracking...");
        while (!moveHistory.isEmpty()) {
            int lastMove = moveHistory.pop();
            if (lastMove == 0) turnAround();
            else if (lastMove == 1) turnLeft();
            else if (lastMove == 3) turnRight();
            moveForwardOneCell();
            if (!crossroadHistory.isEmpty() && moveHistory.size() == crossroadHistory.top()) {
                crossroadHistory.pop();
                Serial.println("Backtracked to crossroad.");
                break;
            }
        }
        if (moveHistory.isEmpty()) {
            Serial.println("Stuck. Ending.");
            stopMotors();
            while (true);
        }
    }
}

} // namespace Mouse

void setup() { Mouse::doStuff(); }
void loop() { Mouse::solve(); }
