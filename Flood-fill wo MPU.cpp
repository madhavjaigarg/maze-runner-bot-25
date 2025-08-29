#include <utility>
#include <queue>
#include <stdint.h>
#include <Wire.h>
#include <vector>
#include <math.h>

#define touchSensor1 14
#define touchSensor2 15

// ----------------- MOTOR DRIVER PINS -----------------
#define leftForward 11
#define leftBack 12
#define rightForward 10
#define rightBack 13

void setMotorPWM (int left, int right){

    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);    
    
    if (left >= 0){
        analogWrite(leftForward, left);
        analogWrite(leftBack, 0); 
    } else {
        analogWrite(leftBack, abs(left));
        analogWrite(leftForward, 0);
    }

    if (right >= 0){
        analogWrite(rightForward, right); 
        analogWrite(rightBack, 0);         
    } else {
        analogWrite(rightBack, abs(right));
        analogWrite(rightForward, 0);
    }
    
}

//--------------ULTRASONIC SENSOR-----------
#define uslt 3
#define usle 4
#define usrt 1
#define usre 0
#define usft 2
#define usfe 6

long readProximity(int trigPin, int echoPin){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000);
    long distance = duration * 0.034 / 2;
    return distance;
    
}
namespace Mouse {

enum Heading {N=0, E=1, S=2, W=3};

// --- maze constants (unchanged) ---
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
static std::vector<std::pair<int,int>> goals;
static std::vector<std::tuple<int,int,Heading>> forwardPath;
static std::vector<std::tuple<int,int,Heading>> returnPath;
static bool runFast = false;

// PID
float kp = 1.2, ki = 0.0, kd = 0.05;
float previousError = 0, integral = 0;

bool senseRelative(Heading rel) {
    const int threshold = 100; //CHANGE THRESHOLD TUNE

    switch (rel) {
        case N: { // Front sensors
            return readProximity(usft, usfe) < threshold;
        }

        case W: // Left sensor
            return readProximity(uslt, usle) < threshold;
            break;

        case E: // Right sensor
            return readProximity(usrt, usre) < threshold;
            break;

        case S:
            return false; // No rear sensor
    }

    return false;
}


float computePID(float error) {
    integral += error;
    // prevent windup
    integral = constrain(integral, -100.0f, 100.0f); 

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

void turnLeft() {
    setMotorPWM(-120, 120);
    delay(400);  // tune this
    setMotorPWM(0, 0);
    facing_ = Heading((facing_ + 3) & 3);
}

void turnRight() {
    setMotorPWM(120, -120);
    delay(400);  // tune this
    setMotorPWM(0, 0);
    facing_ = Heading((facing_ + 1) & 3);
}

void stepBack() {
    setMotorPWM(-100,-100);
    delay(500); //TUNE
    setMotorPWM(0,0);
}
void face(Heading h){
    int dt=((int)h-(int)facing_)&3;
    if(dt==1) turnRight();
    else if(dt==3) turnLeft();
    else if(dt==2){turnRight();turnRight();}
}

void wallCenter() {
    const int threshold = 200;     // Max distance to treat as wall (mm, tune)
    const int targetWall = 120;    // Desired distance from wall (mm, tune)
    int leftVal  = readProximity(uslt, usle);
    int rightVal = readProximity(usrt, usre);

    float error = 0;

    // --- Error calculation ---
    if (leftVal < threshold && rightVal < threshold) {
        // Both walls present → balance between them
        error = (leftVal - rightVal);
    }
    else if (leftVal < threshold) {
        error = (leftVal - targetWall);
    }
    else if (rightVal < threshold) {
        error = (targetWall - rightVal);
    }
    else {
        error = 0;
    }
    // --- PID correction ---
    float correction = computePID(error);
    // --- Motor speeds ---
    int baseSpeed = 120;   // forward cruising speed, tune this
    int leftSpeed  = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;
    
    setMotorPWM(leftSpeed, rightSpeed);
}

void tripSensor(){
    if (digitalRead(touchSensor1) == HIGH || digitalRead(touchSensor2) == HIGH ) {
        Mouse::stepBack();
        Mouse::wallCenter();
        delay(1000); //TUNE
        setMotorPWM(0, 0);
    }
}

//CALIBRATE SERVO MOTOR
// ---- STEP FORWARD WITHOUT MPU ----
void stepForward() {
    unsigned long startTime = millis();
    unsigned long travelTime = 1000; // tune for one cell

    while (millis() - startTime < travelTime) {

        wallCenter();
        
        tripSensor();
    }

    setMotorPWM(0, 0);
    delay(10);

    if (facing_ == N) y_++;
    else if (facing_ == E) x_++;
    else if (facing_ == S) y_--;
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
        if(!chooseNextCell(tx,ty,th)){Serial.println("No path!"); break;}
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
        delay(2000); //TUNE

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
                Serial.println("No path!");
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

void waitForButton() {
    while (digitalRead(touchSensor1) == HIGH || digitalRead(touchSensor2) == HIGH) { }
    delay(10);
}

} // namespace Mouse


void actualRun() {
    using namespace Mouse;

    delay(1000);

    returnPath.clear();
    forwardPath.clear();
    x_ = y_ = 0;
    facing_ = N;
    runFast = false;

    waitForButton();
    solve();

    runFast = true; x_ = y_ = 0; facing_ = N;
    waitForButton();
    solve();

    runFast = true; x_ = y_ = 0; facing_ = N;
    waitForButton();
    solve();
}

void setup() {
    //leftsensor
    pinMode (uslt, OUTPUT);
    pinMode (usle, INPUT);
    //rightsensor
    pinMode (usrt, OUTPUT);
    pinMode (usre, INPUT);
    //frontsensor
    pinMode (usft, OUTPUT);
    pinMode (usfe, INPUT);
    
    pinMode(leftForward, OUTPUT);
    pinMode(leftBack, OUTPUT);
    pinMode(rightForward, OUTPUT);
    pinMode(rightBack, OUTPUT);
    pinMode(touchSensor1, INPUT);
    pinMode(touchSensor2, INPUT);
    
    Wire.begin();
    Serial.begin(115200);
    
}

void loop(){
    actualRun();
    Mouse::tripSensor();
}
