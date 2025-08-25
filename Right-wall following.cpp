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
const int ENCODER_LEFT_PIN  = 2;   // ensure this is interrupt-capable on your board
const int ENCODER_RIGHT_PIN = A2;  // Pico core supports interrupts on most pins; if not, move to a GPIO pin

// -------------------- Parameters --------------------
const int   THRESHOLD_PROX       = 150;   // APDS: lower = farther, higher = closer
const int   BASE_SPEED           = 150;
const int   MAX_SPEED            = 200;
const int   PID_LOOP_DELAY_MS    = 5;
const long  CELL_DISTANCE_TICKS  = 1000;  // tune for your encoders/wheel size
const float TURN_90_DEGREES      = 90.0f;

// -------------------- APDS Proximity Sensors --------------------
Adafruit_APDS9960 sensorFrontLeft;
Adafruit_APDS9960 sensorFrontRight;
Adafruit_APDS9960 sensorLeft;
Adafruit_APDS9960 sensorRight;

// -------------------- MPU9250 --------------------
#define MPU9250_ADDR 0x68
MPU9250_WE myIMU = MPU9250_WE(MPU9250_ADDR);

float currentGyroZAngle = 0.0f;  // integrated yaw, degrees
unsigned long previousGyroReadTime = 0;
float yawZero = 0.0f;

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
    float Kp=0, Ki=0, Kd=0;
    float previousError=0, integral=0;
    unsigned long lastTime=0;
    float outputLimit=0;

    void init(float p, float i, float d, float limit) {
        Kp = p; Ki = i; Kd = d;
        previousError = 0; integral = 0;
        lastTime = millis();
        outputLimit = limit;
    }

    float calculate(float error) {
        unsigned long currentTime = millis();
        float dt = (currentTime == lastTime) ? 0.001f : (currentTime - lastTime) / 1000.0f;
        lastTime = currentTime;

        integral += error * dt;
        float derivative = (error - previousError) / dt;
        previousError = error;

        float output = Kp * error + Ki * integral + Kd * derivative;
        if (output > outputLimit) output = outputLimit;
        if (output < -outputLimit) output = -outputLimit;
        return output;
    }

    void reset() {
        previousError = 0; integral = 0;
        lastTime = millis();
    }
};

PIDGains STRAIGHT_PID_GAINS = {0.6f, 0.00f, 0.01f};   // keeps left/right ticks equal
PIDGains DISTANCE_PID_GAINS = {1.0f, 0.00f, 0.02f};   // pulls toward target distance
PIDGains TURN_PID_GAINS     = {4.0f, 0.02f, 0.15f};   // turns to target yaw

float STRAIGHT_PID_LIMIT = 80.0f;
float DISTANCE_PID_LIMIT = 100.0f;
float TURN_PID_LIMIT     = 180.0f;

PIDController straightPID, distancePID, turnPID;

// -------------------- Motor Control --------------------
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Direction pins
    digitalWrite(IN1, leftSpeed  >= 0); digitalWrite(IN2, leftSpeed  < 0);
    digitalWrite(IN3, rightSpeed >= 0); digitalWrite(IN4, rightSpeed < 0);
    // PWM (0..255)
    analogWrite(ENA, constrain(abs(leftSpeed),  0, 255));
    analogWrite(ENB, constrain(abs(rightSpeed), 0, 255));
}
void stopMotors() { setMotorSpeeds(0, 0); }

// -------------------- Encoder Tracking --------------------
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
void isrLeftEncoder()  { leftEncoderCount++;  }
void isrRightEncoder() { rightEncoderCount++; }

// -------------------- Helpers --------------------
static inline float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// -------------------- MPU9250 Functions --------------------
void initMPU() {
    if (!myIMU.init()) {
        Serial.println("MPU9250 not found!");
        while (1) { delay(10); }
    }
    Serial.println("MPU9250 ready.");

    myIMU.autoOffsets();          // trims gyro/acc offsets
    myIMU.enableGyr();
    myIMU.setGyrDLPF(MPU9250_DLPF_41HZ);
    myIMU.setSampleRateDivider(5); // ~1 kHz/(1+5) = ~166 Hz

    previousGyroReadTime = millis();

    // settle + zero yaw
    delay(50);
    myIMU.readSensor();
    yawZero = myIMU.getGyrValues().z; // not strictly needed; we integrate from 0
    currentGyroZAngle = 0.0f;
}

void updateGyroAngle() {
    unsigned long now = millis();
    float dt = (now - previousGyroReadTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    previousGyroReadTime = now;

    myIMU.readSensor();                 // REQUIRED before getGyrValues()
    xyzFloat g = myIMU.getGyrValues();  // deg/s
    currentGyroZAngle = wrap180(currentGyroZAngle + g.z * dt);
}

// -------------------- Movement Functions --------------------
void moveForwardOneCell() {
    Serial.println("Moving forward one cell (PID)");
    noInterrupts();
    leftEncoderCount = 0; rightEncoderCount = 0;
    interrupts();

    straightPID.reset();
    distancePID.reset();
    long targetTicks = CELL_DISTANCE_TICKS;
    unsigned long startMillis = millis();

    while (true) {
        // time/exit conditions
        if (millis() - startMillis > 3000) break;

        noInterrupts();
        long l = leftEncoderCount;
        long r = rightEncoderCount;
        interrupts();

        long avg = (l + r) / 2;
        long distanceError = targetTicks - avg;
        if (distanceError <= 0) break;

        // keep wheels in sync (left-right = 0)
        long straightError = (l - r);

        float distOut = distancePID.calculate((float)distanceError);
        float straightOut = straightPID.calculate(-(float)straightError);

        int base = BASE_SPEED;
        int leftCmd  = base + (int)distOut + (int)straightOut;
        int rightCmd = base + (int)distOut - (int)straightOut;

        leftCmd  = constrain(leftCmd,  0, MAX_SPEED);
        rightCmd = constrain(rightCmd, 0, MAX_SPEED);

        setMotorSpeeds(leftCmd, rightCmd);

        delay(PID_LOOP_DELAY_MS);
    }
    stopMotors();
    Serial.println("Finished forward move.");
}

void turnToRelative(float targetDegrees) {
    Serial.print("Turning "); Serial.print(targetDegrees); Serial.println(" deg");
    updateGyroAngle();        // fresh starting angle
    float startYaw = currentGyroZAngle;
    float targetYaw = wrap180(startYaw + targetDegrees);

    turnPID.reset();
    unsigned long startMillis = millis();

    while (true) {
        if (millis() - startMillis > 3500) break;
        updateGyroAngle();

        float error = wrap180(targetYaw - currentGyroZAngle); // signed shortest path
        if (fabs(error) < 2.0f) break;

        float u = turnPID.calculate(error); // output is deg/s-like command

        // map PID output to wheel speeds symmetrically
        int cmd = (int)fabs(u);
        cmd = constrain(cmd, 70, MAX_SPEED); // ensure enough torque to start

        int leftCmd  = (error > 0) ?  cmd : -cmd;
        int rightCmd = -leftCmd;

        setMotorSpeeds(leftCmd, rightCmd);
        delay(PID_LOOP_DELAY_MS);
    }
    stopMotors();
    delay(60); // tiny settle
    updateGyroAngle();
    Serial.println("Turn complete.");
}

void turnLeft()   { turnToRelative(-TURN_90_DEGREES); }
void turnRight()  { turnToRelative( TURN_90_DEGREES); }
void turnAround() { turnToRelative( 180.0f); }

// -------------------- Proximity Sensor Reading --------------------
uint8_t readProximity(Adafruit_APDS9960& sensor) {
    return sensor.readProximity(); // 0..255
}

// -------------------- Setup --------------------
void doStuff() {
    Wire.begin();
    Serial.begin(115200);
    delay(100);

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);

    // Attach interrupts if supported on these pins
    int irqL = digitalPinToInterrupt(ENCODER_LEFT_PIN);
    int irqR = digitalPinToInterrupt(ENCODER_RIGHT_PIN);
    if (irqL != NOT_AN_INTERRUPT) attachInterrupt(irqL, isrLeftEncoder, CHANGE);
    else Serial.println("WARNING: Left encoder pin not interrupt-capable on this board.");

    if (irqR != NOT_AN_INTERRUPT) attachInterrupt(irqR, isrRightEncoder, CHANGE);
    else Serial.println("WARNING: Right encoder pin not interrupt-capable on this board.");

    straightPID.init(STRAIGHT_PID_GAINS.Kp, STRAIGHT_PID_GAINS.Ki, STRAIGHT_PID_GAINS.Kd, STRAIGHT_PID_LIMIT);
    distancePID.init(DISTANCE_PID_GAINS.Kp, DISTANCE_PID_GAINS.Ki, DISTANCE_PID_GAINS.Kd, DISTANCE_PID_LIMIT);
    turnPID.init(TURN_PID_GAINS.Kp, TURN_PID_GAINS.Ki, TURN_PID_GAINS.Kd, TURN_PID_LIMIT);

    initMPU();  // start MPU9250

    // Init APDS sensors (all on same I2C address; you must have them on separate buses or muxes if truly 4 units.
    // If you're actually using 4 separate boards at the default address (0x39), you'll need an I2C multiplexer.
    if (!sensorFrontLeft.begin())  Serial.println("Front Left APDS not found");
    if (!sensorFrontRight.begin()) Serial.println("Front Right APDS not found");
    if (!sensorLeft.begin())       Serial.println("Left APDS not found");
    if (!sensorRight.begin())      Serial.println("Right APDS not found");

    sensorFrontLeft.enableProximity(true);
    sensorFrontRight.enableProximity(true);
    sensorLeft.enableProximity(true);
    sensorRight.enableProximity(true);

    // Optional tuning (uncomment if needed)
    // sensorFrontLeft.setProxGain(APDS9960_PGAIN_4X);
    // sensorFrontRight.setProxGain(APDS9960_PGAIN_4X);
    // sensorLeft.setProxGain(APDS9960_PGAIN_4X);
    // sensorRight.setProxGain(APDS9960_PGAIN_4X);

    Serial.println("Robot ready.");
    delay(100);
}

// -------------------- Maze Solving Logic --------------------
void solve() {
    updateGyroAngle();

    uint8_t leftDist  = readProximity(sensorLeft);
    uint8_t rightDist = readProximity(sensorRight);
    uint8_t frontDist1 = readProximity(sensorFrontLeft);
    uint8_t frontDist2 = readProximity(sensorFrontRight);
    uint8_t frontDist  = (uint8_t)(((int)frontDist1 + (int)frontDist2) / 2);

    Serial.print("L: "); Serial.print(leftDist);
    Serial.print(" | F: "); Serial.print(frontDist);
    Serial.print(" | R: "); Serial.println(rightDist);

    bool leftOpen  = leftDist  < THRESHOLD_PROX;
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
            while (true) { delay(100); }
        }
    }
}

} // namespace Mouse

void setup() { Mouse::doStuff(); }
void loop()  { Mouse::solve(); }
