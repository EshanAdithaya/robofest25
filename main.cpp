#include <Arduino.h>

// Pin Definitions
#define TRIGGER_FRONT 5
#define ECHO_FRONT 18
#define TRIGGER_LEFT 19
#define ECHO_LEFT 21
#define TRIGGER_RIGHT 22
#define ECHO_RIGHT 23

#define MOTOR_LEFT_FWD 25
#define MOTOR_LEFT_BCK 26
#define MOTOR_RIGHT_FWD 27
#define MOTOR_RIGHT_BCK 14
#define MOTOR_LEFT_EN 12
#define MOTOR_RIGHT_EN 13

#define ENCODER_LEFT_A 32
#define ENCODER_LEFT_B 33
#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 35

#define LED_SUCCESS 2
#define MODE_BUTTON 0  // Built-in button

// Maze Constants
#define MAZE_SIZE 16
#define CELL_SIZE 180  // 18cm in mm

// Maze representation - each cell stores wall info in bits: N E S W
uint8_t maze[MAZE_SIZE][MAZE_SIZE];
uint8_t distance[MAZE_SIZE][MAZE_SIZE];
bool isLearningMode = true;

// Current position
struct Position {
    uint8_t x = 0;
    uint8_t y = 0;
    uint8_t direction = 0;  // 0=N, 1=E, 2=S, 3=W
} currentPos;

// Function prototypes
void initHardware();
void readSensors();
void updateWalls();
void floodFill();
uint8_t getNextMove();
void move(uint8_t direction);
void learnMaze();
void findShortestPath();
void indicateSuccess();

volatile bool buttonPressed = false;

void IRAM_ATTR handleModeButton() {
    static unsigned long lastDebounceTime = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastDebounceTime > 200) {  // Debounce
        buttonPressed = true;
        lastDebounceTime = currentTime;
    }
}

void setup() {
    initHardware();
    attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), handleModeButton, FALLING);
}

void loop() {
    if (buttonPressed) {
        isLearningMode = !isLearningMode;
        buttonPressed = false;
        // Reset position if switching to learning mode
        if (isLearningMode) {
            currentPos = {0, 0, 0};
            memset(maze, 0, sizeof(maze));
            memset(distance, 0xFF, sizeof(distance));
        }
    }

    if (isLearningMode) {
        learnMaze();
    } else {
        findShortestPath();
    }
}

void initHardware() {
    // Initialize pins
    pinMode(TRIGGER_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIGGER_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIGGER_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);
    
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_BCK, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_BCK, OUTPUT);
    pinMode(MOTOR_LEFT_EN, OUTPUT);
    pinMode(MOTOR_RIGHT_EN, OUTPUT);
    
    pinMode(LED_SUCCESS, OUTPUT);
    pinMode(MODE_BUTTON, INPUT_PULLUP);

    // Motor PWM setup
    ledcSetup(0, 5000, 8);  // Channel 0, 5kHz, 8-bit
    ledcSetup(1, 5000, 8);  // Channel 1, 5kHz, 8-bit
    ledcAttachPin(MOTOR_LEFT_EN, 0);
    ledcAttachPin(MOTOR_RIGHT_EN, 1);
}

void learnMaze() {
    readSensors();
    updateWalls();
    floodFill();
    uint8_t nextMove = getNextMove();
    move(nextMove);
    
    // Check if reached goal (center of maze)
    if (currentPos.x == MAZE_SIZE/2 && currentPos.y == MAZE_SIZE/2) {
        indicateSuccess();
    }
}

void findShortestPath() {
    // A* implementation using stored maze data
    uint8_t nextMove = getOptimalMove();
    move(nextMove);
    
    if (currentPos.x == MAZE_SIZE/2 && currentPos.y == MAZE_SIZE/2) {
        indicateSuccess();
        delay(1000);  // Pause before returning
        // Return to start using optimal path
        returnToStart();
    }
}

void indicateSuccess() {
    for(int i = 0; i < 5; i++) {
        digitalWrite(LED_SUCCESS, HIGH);
        delay(100);
        digitalWrite(LED_SUCCESS, LOW);
        delay(100);
    }
}

// Implementations for sensor reading, wall detection, and movement control would follow...
// Adding core functionality first, will expand based on testing
// Sensor reading and motor control implementations

unsigned long readUltrasonic(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    return pulseIn(echoPin, HIGH, 30000) * 0.17;  // Distance in mm
}

struct WallReadings {
    bool front;
    bool left;
    bool right;
} walls;

void readSensors() {
    static const uint16_t WALL_THRESHOLD = 150;  // mm
    
    uint16_t frontDist = readUltrasonic(TRIGGER_FRONT, ECHO_FRONT);
    uint16_t leftDist = readUltrasonic(TRIGGER_LEFT, ECHO_LEFT);
    uint16_t rightDist = readUltrasonic(TRIGGER_RIGHT, ECHO_RIGHT);
    
    walls.front = frontDist < WALL_THRESHOLD;
    walls.left = leftDist < WALL_THRESHOLD;
    walls.right = rightDist < WALL_THRESHOLD;
}

void updateWalls() {
    uint8_t wallBits = 0;
    
    // Convert relative walls to absolute walls based on current direction
    switch(currentPos.direction) {
        case 0:  // North
            if(walls.front) wallBits |= 0b1000;
            if(walls.left) wallBits |= 0b0100;
            if(walls.right) wallBits |= 0b0001;
            break;
        case 1:  // East
            if(walls.front) wallBits |= 0b0001;
            if(walls.left) wallBits |= 0b1000;
            if(walls.right) wallBits |= 0b0010;
            break;
        case 2:  // South
            if(walls.front) wallBits |= 0b0010;
            if(walls.left) wallBits |= 0b0001;
            if(walls.right) wallBits |= 0b0100;
            break;
        case 3:  // West
            if(walls.front) wallBits |= 0b0100;
            if(walls.left) wallBits |= 0b0010;
            if(walls.right) wallBits |= 0b1000;
            break;
    }
    
    maze[currentPos.x][currentPos.y] = wallBits;
}

void setMotors(int16_t leftSpeed, int16_t rightSpeed) {
    // Speed range: -255 to 255
    
    // Left motor
    if(leftSpeed >= 0) {
        digitalWrite(MOTOR_LEFT_BCK, LOW);
        digitalWrite(MOTOR_LEFT_FWD, HIGH);
    } else {
        digitalWrite(MOTOR_LEFT_BCK, HIGH);
        digitalWrite(MOTOR_LEFT_FWD, LOW);
        leftSpeed = -leftSpeed;
    }
    
    // Right motor
    if(rightSpeed >= 0) {
        digitalWrite(MOTOR_RIGHT_BCK, LOW);
        digitalWrite(MOTOR_RIGHT_FWD, HIGH);
    } else {
        digitalWrite(MOTOR_RIGHT_BCK, HIGH);
        digitalWrite(MOTOR_RIGHT_FWD, LOW);
        rightSpeed = -rightSpeed;
    }
    
    ledcWrite(0, leftSpeed);
    ledcWrite(1, rightSpeed);
}

void move(uint8_t targetDirection) {
    // Calculate shortest turn
    int8_t turn = targetDirection - currentPos.direction;
    if(turn > 2) turn -= 4;
    if(turn < -2) turn += 4;
    
    // Execute turn
    if(turn != 0) {
        if(turn > 0) {
            setMotors(100, -100);  // Turn right
        } else {
            setMotors(-100, 100);  // Turn left
        }
        delay(abs(turn) * 250);  // Approximate time for 90-degree turn
        setMotors(0, 0);
    }
    
    // Move forward one cell
    setMotors(200, 200);
    delay(500);  // Time to move one cell
    setMotors(0, 0);
    
    // Update position
    currentPos.direction = targetDirection;
    switch(targetDirection) {
        case 0: currentPos.y++; break;  // North
        case 1: currentPos.x++; break;  // East
        case 2: currentPos.y--; break;  // South
        case 3: currentPos.x--; break;  // West
    }
}

// PID control for straight line movement and turning
void moveWithPID() {
    static float lastError = 0;
    static float integral = 0;
    const float Kp = 2.0;
    const float Ki = 0.1;
    const float Kd = 1.0;
    
    float error = walls.left - walls.right;  // Difference between side sensors
    integral = integral * 0.8 + error * 0.2;
    float derivative = error - lastError;
    
    float correction = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    
    int16_t baseSpeed = 200;
    setMotors(baseSpeed - correction, baseSpeed + correction);
}