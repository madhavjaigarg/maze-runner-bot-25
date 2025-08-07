#include <cstdlib>
#include <utility>
#include <iostream>
#include <queue>
#include <array>
#include <algorithm>
#include <cassert>
#include <string>
#include <stdint.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <Arduino_APDS9960.h>

ARDUINO_APDS9960 apds;
bool initAPDS(uint8_t channel) {
    selectMuxChannel(channel);
    delay(10); // ensure the channel is set
    return apds.begin();
}

void initAPDSSensors() {
    if (!initAPDS(0)) {
        Serial.println("Failed to init APDS on channel 0 (Front)");
        while (1);
    }
    if (!initAPDS(1)) {
        Serial.println("Failed to init APDS on channel 1 (Left)");
        while (1);
    }
    if (!initAPDS(2)) {
        Serial.println("Failed to init APDS on channel 2 (Right)");
        while (1);
    }
}

#define MUX_ADDR 0x70 // default I2C address for PCS9548A

void selectMuxChannel(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

namespace Mouse {

enum Heading {N=0, E=1, S=2, W=3};

static const unsigned WALL_N = 1u<<0;
static const unsigned WALL_E = 1u<<1;
static const unsigned WALL_S = 1u<<2;
static const unsigned WALL_W = 1u<<3;
static const unsigned KNOWN_N = 1u<<4;
static const unsigned KNOWN_E = 1u<<5;
static const unsigned KNOWN_S = 1u<<6;
static const unsigned KNOWN_W = 1u<<7;

inline unsigned wallMask(Heading h){switch(h){case N:return WALL_N;case E:return WALL_E;case S:return WALL_S;default:return WALL_W;}}
inline unsigned knownMask(Heading h){switch(h){case N:return KNOWN_N;case E:return KNOWN_E;case S:return KNOWN_S;default:return KNOWN_W;}}
inline Heading opp(Heading h){return Heading((int(h)+2)&3);} 

struct Cell {unsigned bits; Cell():bits(0){}};

static int mazeW=16, mazeH=16;
static Cell maze[16][16];
static uint8_t dist[16][16];
static int x_=0,y_=0;
static Heading facing_=N;
static std::vector<std::pair<int,int> > goals;
static std::vector<std::tuple<int,int,Heading>> forwardPath;
static std::vector<std::tuple<int,int,Heading>> returnPath;
static bool runFast = false;

//PID
float kp = 1.2;
float ki = 0.0;
float kd = 0.05;

float previousError = 0;
float integral = 0;

//GYRO
#define MPU_ADDR 0x68
MPU9250_WE myMPU = MPU9250_WE(Wire, MPU_ADDR);

bool senseRelative(Heading rel) {
    const int threshold = 100;

    switch (rel) {
        case N: // Front sensor on channel 0
            selectMuxChannel(0);
            delay(2); // minimal delay
            if (apds.proximityAvailable()) {
                return apds.readProximity() > threshold;
            }
            break;

        case W: // Left sensor on channel 1
            selectMuxChannel(1);
            delay(2);
            if (apds.proximityAvailable()) {
                return apds.readProximity() > threshold;
            }
            break;

        case E: // Right sensor on channel 2
            selectMuxChannel(2);
            delay(2);
            if (apds.proximityAvailable()) {
                return apds.readProximity() > threshold;
            }
            break;

        case S:
            return false; // No rear sensor
    }

    return false;
}


float getWallError() {
    float leftDistance = senseRelative(W);  // Replace with actual left sensor
    float rightDistance = senseRelative(E); // Replace with actual right sensor

    float error = leftDistance - rightDistance; // Target is centered
    return error;
}

float computePID(float error) {
    integral += error;
    float derivative = error - previousError;
    previousError = error;

    return kp * error + ki * integral + kd * derivative;
}

inline bool inBounds(int x,int y){return x>=0 && x<mazeW && y>=0 && y<mazeH;}

bool haveWall(int x,int y,Heading h){return maze[x][y].bits & wallMask(h);}        
bool knowSide(int x,int y,Heading h){return maze[x][y].bits & knownMask(h);}       

void setWallKnown(int x,int y,Heading h,bool wall){
    if(!inBounds(x,y)) return;
    Cell &c=maze[x][y];
    if(wall) c.bits |= wallMask(h); else c.bits &= ~wallMask(h);
    c.bits |= knownMask(h);
    int nx=x,ny=y; Heading oh=opp(h);
    if(h==N) ny++; else if(h==E) nx++; else if(h==S) ny--; else nx--;
    if(inBounds(nx,ny)){
        Cell &n=maze[nx][ny];
        if(wall) n.bits |= wallMask(oh); else n.bits &= ~wallMask(oh);
        n.bits |= knownMask(oh);
    }
}

bool senseAllSidesAndCheckNew() {
    bool changed = false;
    bool wf = senseRelative(N), wr = senseRelative(E), wb = false, wl = senseRelative(W);
    auto old = maze[x_][y_].bits;
    setWallKnown(x_, y_, facing_, wf);
    setWallKnown(x_, y_, Heading((int(facing_) + 1) & 3), wr);
    setWallKnown(x_, y_, opp(facing_), wb);
    setWallKnown(x_, y_, Heading((int(facing_) + 3) & 3), wl);
    if (maze[x_][y_].bits != old) changed = true;
    return changed;
}

//ADD THE SERVO MOTOR CODE
void turnLeft() {
    float yaw = 0;
    unsigned long lastTime = millis();
    const float targetYaw = -90.0; // degrees
    const int motorSpeed = 100;
    
    // Start rotating left: left backward, right forward
    digitalWrite(LEFT_DIR_PIN1, LOW);
    digitalWrite(LEFT_DIR_PIN2, HIGH);
    digitalWrite(RIGHT_DIR_PIN1, HIGH);
    digitalWrite(RIGHT_DIR_PIN2, LOW);

    while (yaw > targetYaw) {
        myMPU.update();
        float gyroZ = myMPU.getGyrValues().z;

        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        yaw += gyroZ * dt;

        analogWrite(LEFT_PWM_PIN, motorSpeed);
        analogWrite(RIGHT_PWM_PIN, motorSpeed);
    }

    // Stop motors
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);
}


void turnRight() {
    float yaw = 0;
    unsigned long lastTime = millis();
    const float targetYaw = 90.0; // degrees
    const int motorSpeed = 100;
    
    // Start rotating right: left forward, right backward
    digitalWrite(LEFT_DIR_PIN1, HIGH);
    digitalWrite(LEFT_DIR_PIN2, LOW);
    digitalWrite(RIGHT_DIR_PIN1, LOW);
    digitalWrite(RIGHT_DIR_PIN2, HIGH);

    while (yaw < targetYaw) {
        myMPU.update();
        float gyroZ = myMPU.getGyrValues().z; // deg/s

        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;

        yaw += gyroZ * dt;

        analogWrite(LEFT_PWM_PIN, motorSpeed);
        analogWrite(RIGHT_PWM_PIN, motorSpeed);
    }

    // Stop motors
    analogWrite(LEFT_PWM_PIN, 0);
    analogWrite(RIGHT_PWM_PIN, 0);
}

void face(Heading h){
    int dt=((int)h-(int)facing_)&3;
    if(dt==1) turnRight();
    else if(dt==3) turnLeft();
    else if(dt==2){turnRight();turnRight();}
}
//ADD MOTOR CALIBRATION
void setMotorPWM(int leftPWM, int rightPWM) {
    analogWrite(LEFT_MOTOR_PIN, constrain(leftPWM, 0, 255));
    analogWrite(RIGHT_MOTOR_PIN, constrain(rightPWM, 0, 255));
}

//CALIBRATE SERVO MOTOR
void stepForward() {
    const int baseSpeed = 120;
    const int stepTime = 300; // ms to move one cell — tune this
    unsigned long startTime = millis();

    while (millis() - startTime < stepTime) {
        float error = getWallError();
        float correction = computePID(error);

        int leftPWM = baseSpeed - correction;
        int rightPWM = baseSpeed + correction;

        setMotorPWM(leftPWM, rightPWM);
    }

    setMotorPWM(0, 0); // stop motors after move
    delay(50); // optional braking delay

    // update position
    if (facing_ == N) y_++;
    else if (facing_ == E) x_++;
    else if (facing_ == S) y_--;
    else x_--;
}


void recomputeDistances(){
    const uint8_t INF = 255;
    for(int x=0;x<mazeW;x++) for(int y=0;y<mazeH;y++) dist[x][y]=INF;
    std::queue<std::pair<int,int> > q;
    for(size_t i=0;i<goals.size();++i){
        int gx=goals[i].first;
        int gy=goals[i].second;
        if(inBounds(gx,gy)){dist[gx][gy]=0;q.push(goals[i]);}
    }
    while(!q.empty()){
        int cx=q.front().first, cy=q.front().second; q.pop(); int cd=dist[cx][cy];
        static const int dx[4]={0,1,0,-1}; static const int dy[4]={1,0,-1,0}; static const Heading hh[4]={N,E,S,W};
        for(int k=0;k<4;k++){
            int nx=cx+dx[k], ny=cy+dy[k]; Heading h=hh[k];
            if(!inBounds(nx,ny)) continue;
            if(knowSide(cx,cy,h)&&haveWall(cx,cy,h)) continue;
            if(dist[nx][ny]==INF){dist[nx][ny]=cd+1; q.push(std::make_pair(nx,ny));}
        }
    }
}

void computeToStart(){
    goals.clear(); goals.push_back(std::make_pair(0,0));
    recomputeDistances();
}

bool chooseNextCell(int &tx,int &ty,Heading &th){
    int best=dist[x_][y_]; bool found=false;
    static const int dx[4]={0,1,0,-1}; static const int dy[4]={1,0,-1,0}; static const Heading hh[4]={N,E,S,W};
    for(int k=0;k<4;k++){
        int nx=x_+dx[k], ny=y_+dy[k]; Heading h=hh[k];
        if(!inBounds(nx,ny)) continue;
        if(knowSide(x_,y_,h)&&haveWall(x_,y_,h)) continue;
        int d=dist[nx][ny];
        if(d<best){best=d;tx=nx;ty=ny;th=h;found=true;}
    }
    return found;
}

bool atGoal(){
    for(size_t i=0;i<goals.size();++i)
        if(x_==goals[i].first && y_==goals[i].second) return true;
    return false;
}

void init(){
    mazeW=16; mazeH=16;
    if(mazeW>16) mazeW=16; if(mazeH>16) mazeH=16;
    for(int x=0;x<mazeW;x++){setWallKnown(x,0,S,true);setWallKnown(x,mazeH-1,N,true);} 
    for(int y=0;y<mazeH;y++){setWallKnown(0,y,W,true);setWallKnown(mazeW-1,y,E,true);} 
    int cx0=(mazeW-1)/2, cx1=mazeW/2, cy0=(mazeH-1)/2, cy1=mazeH/2;
    goals.clear(); goals.push_back(std::make_pair(cx0,cy0));
    if(cx1!=cx0||cy1!=cy0){
        goals.push_back(std::make_pair(cx1,cy0));
        goals.push_back(std::make_pair(cx0,cy1));
        goals.push_back(std::make_pair(cx1,cy1));
    }
    recomputeDistances(); senseAllSidesAndCheckNew(); recomputeDistances();
}

void returnToStart(){
    computeToStart();
    returnPath.clear();
    while(!atGoal()){
        int tx=x_, ty=y_; Heading th=facing_;
        if(!chooseNextCell(tx,ty,th)){std::cerr<<"No path to start!"<<std::endl; break;}
        face(th); stepForward();
        if (!runFast)
            returnPath.emplace_back(x_, y_, facing_);
        if(senseAllSidesAndCheckNew()) recomputeDistances();
    }
}

void solve() {
    init();

    if (runFast) {
        // Check if paths are valid
        if (forwardPath.empty() || returnPath.empty()) {
            runFast = false; // fallback
                returnPath.clear(); //Clearing Caches
                forwardPath.clear();
        }
    }

    if (runFast) {
        // Choose shorter path to goal
        bool usedForward = forwardPath.size() <= returnPath.size();
        const auto &shorterPath = usedForward ? forwardPath : returnPath;

        // Go to center (goal) using shorter path
        for (const auto &[tx, ty, th] : shorterPath) {
            face(th);
            stepForward();
        }

        // Wait 5 seconds at center
        delay(5000);

        // Return to start by walking the same path in reverse
        for (auto it = shorterPath.rbegin(); it != shorterPath.rend(); ++it) {
            const auto &[tx, ty, th] = *it;
            face(th);
            stepForward();
        }

        return;
    } else {
        while (!atGoal()) {
            int tx = x_, ty = y_; Heading th = facing_;
            if (!chooseNextCell(tx, ty, th)) {
                std::cerr << "No path!" << std::endl;
                break;
            }
            face(th);
            stepForward();
            forwardPath.emplace_back(x_, y_, facing_);
            if (senseAllSidesAndCheckNew()) {
                recomputeDistances();
            }
        }

        // Wait 5 seconds at center before returning
        delay(5000);

        // Return to start and record return path
        returnToStart();
    }
}

//ADD THE CODE FOR THE BUTTON
void waitForButton() {
    while (!API::buttonPressed()) {
        // wait for manual trigger
    }
}

} // namespace Mouse

void actualRun() {
    using namespace Mouse;  
    Wire.begin();
    Serial.begin(115200);


  
    if (!myMPU.init()) {
        Serial.println("MPU9250 not found");
        while (1);
    }

    myMPU.setGyroRange(MPU9250_GYRO_RANGE_250);
    myMPU.setAccRange(MPU9250_ACC_RANGE_2G);
    myMPU.enableGyrDLPF();
    myMPU.setGyrDLPF(MPU9250_DLPF_6);  // low-pass filter
    myMPU.enableAccDLPF();
    myMPU.setAccDLPF(MPU9250_DLPF_6);

    delay(1000);
    myMPU.autoOffsets();  // important for stability

    returnPath.clear(); //Clearing Caches
    forwardPath.clear();
    x_ = y_ = 0; //Setting the directions
    facing_ = N;
    runFast = false;
    waitForButton(); // Wait before 1st run
    solve();         // First mapping run

    runFast = true;
    x_ = y_ = 0;
    facing_ = N;

    waitForButton(); // Wait before 2nd run
    solve();         // Fast run 1

    x_ = y_ = 0;
    facing_ = N;

    waitForButton(); // Wait before 3rd run
    solve();         // Fast run 2

    return;
}
