#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp" 
#include "Lidar.hpp"
#include <Wire.h>
#include "IMUOdometry.hpp"  
#include <MPU6050.h>
#include "OledDisplay.hpp"  

#define MOT1PWM 9
#define MOT1DIR 10
#define MOT2PWM 11
#define MOT2DIR 12

#define EN1A 2
#define EN1B 7
#define EN2A 3
#define EN2B 8

#define LIDAR 4

// --- GLOBAL VARIABLES ---
float ax_offset = 0.0;
float ay_offset = 0.0;
float gz_offset = 0.0;
int basePWM = 100;
int leftPWM = basePWM;
int rightPWM = basePWM;
const float wheelDiameterCM = 3.55;
const float countsPerRev = 1400;
const float cmPerCount = (PI * wheelDiameterCM) / countsPerRev; 
static float targetDistanceCM = 50;

// Task states
volatile bool objectDetected = false;
bool turnLeft = false;
bool turnRight = false;
bool forward = false;
bool reverse = false;
bool stop = true;

// Lidar pins
int LIDAR1 = A0;
int LIDAR2 = A1;
int LIDAR3 = A2;

// Objects
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::Encoder encoder1(EN1A, EN1B,1400);
mtrn3100::Encoder encoder2(EN2A, EN2B,1400);
mtrn3100::PIDController pid(1,0.1,0.05);
mtrn3100::Lidar leftLidar(LIDAR1,  0x30);
mtrn3100::Lidar frontLidar(LIDAR2,  0x31);
mtrn3100::Lidar rightLidar(LIDAR3,  0x32);

IMU imuOdom;
OledDisplay oled;

// Robot position
int robotX, robotY, heading;

// Maze cell struct
struct Cell {
    bool wallNorth;
    bool wallEast;
    bool wallSouth;
    bool wallWest;
    bool visited;
};

Cell maze[9][9]; // 9x9 maze

// --- HELPER FUNCTIONS ---
void lidarISR() { objectDetected = true; }

void encoder1ISR() { encoder1.readEncoder(); }
void encoder2ISR() { encoder2.readEncoder(); }

void encoderReset() { encoder1.count = 0; encoder2.count = 0; }

int normalizeHeading(int h) { 
    if (h < 0) h += 360; 
    if (h >= 360) h -= 360; 
    return h; 
}

void pauseBetweenMoves() {
    motor1.stop();
    motor2.stop();
    delay(200); // 200 ms pause between actions
}

// --- MOVEMENT FUNCTIONS ---
bool turnToYaw(float targetYaw, int tolerance=2, int maxSpeed=30) {
    imuOdom.update();
    float currentYaw, dummy1, dummy2;
    imuOdom.getOrientation(dummy1, dummy2, currentYaw);

    float error = targetYaw - currentYaw;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;

    static mtrn3100::PIDController yawPID(1.2, 0.0, 0.1);
    static unsigned long prevTime = millis();

    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0f;
    prevTime = now;

    float correction = yawPID.compute(0, -error, dt);

    if (abs(error) <= tolerance) {
        motor1.stop();
        motor2.stop();
        yawPID.reset();
        return true;
    }

    int pwm = constrain(abs(correction), 20, min(maxSpeed, abs(error) * 2));
    if (correction > 0) { motor1.forward(pwm); motor2.forward(pwm); }
    else { motor1.reverse(pwm); motor2.reverse(pwm); }

    return false;
}

void moveStraightWithHeadingCorrection(float targetHeading, float distanceCM) {
    encoder1.reset();
    encoder2.reset();
    long targetCounts = distanceCM / cmPerCount;

    static mtrn3100::PIDController headingPID(2.0, 0.0, 0.5);
    unsigned long prevTime = millis();

    while (true) {
        imuOdom.update();
        float currentYaw, dummy1, dummy2;
        imuOdom.getOrientation(dummy1, dummy2, currentYaw);

        float error = targetHeading - currentYaw;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        unsigned long now = millis();
        float dt = (now - prevTime) / 1000.0f;
        prevTime = now;

        float correction = headingPID.compute(0, error, dt);
        int pwmLeft = basePWM - correction;
        int pwmRight = basePWM + correction;

        pwmLeft = constrain(pwmLeft, 0, 255);
        pwmRight = constrain(pwmRight, 0, 255);

        motor1.forward(pwmLeft);
        motor2.reverse(pwmRight);

        long avgCount = (abs(encoder1.getCount()) + abs(encoder2.getCount())) / 2;
        if (avgCount >= targetCounts) { motor1.stop(); motor2.stop(); break; }
        delay(10);
    }
}

// Movement wrappers with pauses
void moveForward() {
    pauseBetweenMoves();
    moveStraightWithHeadingCorrection(heading, 19.0);
    pauseBetweenMoves();
}

void rotateLeft() {
    pauseBetweenMoves();
    turnToYaw(heading - 90, 2, 100);
    heading = normalizeHeading(heading - 90);
    pauseBetweenMoves();
}

void rotateRight() {
    pauseBetweenMoves();
    turnToYaw(heading + 90, 2, 100);
    heading = normalizeHeading(heading + 90);
    pauseBetweenMoves();
}

// --- MAZE LOGIC ---
void updateWalls() {
    int leftDist  = leftLidar.readDistanceAndTrigger();
    int rightDist = rightLidar.readDistanceAndTrigger();
    int frontDist = frontLidar.readDistanceAndTrigger();
    Cell &current = maze[robotX][robotY];

    if (heading == 90) { current.wallNorth = (frontDist!=-1); current.wallEast  = (rightDist!=-1); current.wallWest  = (leftDist!=-1); }
    else if (heading == 180) { current.wallEast  = (frontDist!=-1); current.wallSouth = (rightDist!=-1); current.wallNorth = (leftDist!=-1); }
    else if (heading == 270) { current.wallSouth = (frontDist!=-1); current.wallWest  = (rightDist!=-1); current.wallEast  = (leftDist!=-1); }
    else if (heading == 0) { current.wallWest  = (frontDist!=-1); current.wallNorth = (rightDist!=-1); current.wallSouth = (leftDist!=-1); }
}

bool canGoDirection(int dir) {
    dir = normalizeHeading(dir);
    Cell &c = maze[robotX][robotY];
    if (dir==90 && !c.wallNorth && robotY>0 && !maze[robotX][robotY-1].visited) return true;
    if (dir==180 && !c.wallEast && robotX<8 && !maze[robotX+1][robotY].visited) return true;
    if (dir==270 && !c.wallSouth && robotY<8 && !maze[robotX][robotY+1].visited) return true;
    if (dir==0 && !c.wallWest && robotX>0 && !maze[robotX-1][robotY].visited) return true;
    return false;
}

void stepForward() {
    if (heading == 90) robotY--;
    else if (heading == 180) robotX--;
    else if (heading == 270) robotY++;
    else if (heading == 0) robotX++;
    moveForward();
}

void maze_exploration(int startX, int startY, int startHeading) {
    for(int x=0;x<9;x++) for(int y=0;y<9;y++) maze[x][y] = {false,false,false,false,false};
    robotX=startX; robotY=startY; heading=startHeading;

    bool explorationDone=false;
    while(!explorationDone){
        maze[robotX][robotY].visited=true;
        updateWalls();

        if(canGoDirection(heading-90)) { rotateLeft(); stepForward(); }
        else if(canGoDirection(heading)) { stepForward(); }
        else if(canGoDirection(heading+90)) { rotateRight(); stepForward(); }
        else { rotateRight(); rotateRight(); stepForward(); }

        explorationDone = true;
        for(int x=0;x<9;x++){for(int y=0;y<9;y++){if(!maze[x][y].visited){explorationDone=false;break;}} if(!explorationDone) break;}
    }
}

// --- SETUP AND LOOP ---
void startLidars() {
    pinMode(LIDAR1, OUTPUT); pinMode(LIDAR2, OUTPUT); pinMode(LIDAR3, OUTPUT);
    digitalWrite(LIDAR1, LOW); digitalWrite(LIDAR2, LOW); digitalWrite(LIDAR3, LOW); delay(10);
    digitalWrite(LIDAR1,HIGH); delay(10); leftLidar.init(); delay(10);
    digitalWrite(LIDAR2,HIGH); delay(10); frontLidar.init(); delay(10);
    digitalWrite(LIDAR3,HIGH); delay(10); rightLidar.init(); delay(10);
    Serial.println("LIDARs initialized");
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    attachInterrupt(digitalPinToInterrupt(EN1A), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN2A), encoder2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LIDAR), lidarISR, RISING);

    encoder1.reset(); encoder2.reset();
    startLidars();
    imuOdom.begin();
}

void loop() {
    maze_exploration(2, 0, 90); // Start at (2,0) facing North
}
