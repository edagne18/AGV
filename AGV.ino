#include <NewPing.h>       // Includes the NewPing library for ultrasonic sensors.
#include <Servo.h>         // Includes the Servo library to control servo motors.
#include <AFMotor.h>       // Includes the Adafruit Motor Shield library to control DC motors.

// Definitions for ultrasonic sensor pins and maximum distance.
#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define MAX_DISTANCE 50

// Definitions for infrared sensor pins.
#define irLeft A0
#define irRight A1
#define irFront A5

// Definition for the digital pin connected to the reset button.
#define RESET_BUTTON_PIN 2 

// Create servo objects for ultrasonic sensor and forklift mechanism.
Servo servo;          
Servo forkliftServo;  
// Initialize ultrasonic sensor with defined pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Initialize four motors using Adafruit Motor Shield.
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// Global variables to store distances and object detection flag.
int leftDistance = 0;
int rightDistance = 0;
boolean object;

void setup() {
  delay(5000); // Delays the start of setup by 5 seconds.
  Serial.begin(9600); // Starts serial communication at 9600 baud rate.
  // Sets infrared sensor pins as inputs.
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  pinMode(irFront, INPUT);
  // Initialize the reset button pin as an input with pull-up resistor.
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP); 

  servo.attach(10); // Connects the ultrasonic sensor's servo to pin 10.
  forkliftServo.attach(9); // Connects the forklift servo to pin 9.
  // Initial positions for servos.
  servo.write(90);
  forkliftServo.write(0);

  // Sets an initial speed for all motors.
  motor1.setSpeed(90);
  motor2.setSpeed(90);
  motor3.setSpeed(90);
  motor4.setSpeed(90);
}

// Stops all motors and prints "Stopped" to the serial monitor.
void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  Serial.println("Stopped");
}

// Function to raise the forklift by decreasing the servo angle.
void raiseForklift() {
  int currentAngle = 45;
  int targetAngle = 0;
  forkliftServo.write(currentAngle);
  delay(15);
  
  while (currentAngle > targetAngle) {
    currentAngle--;
    forkliftServo.write(currentAngle);
    delay(15);
  }
  delay(50);
}

// Function to lower the forklift by increasing the servo angle.
void lowerForklift() {
  int currentAngle = 0;
  int targetAngle = 45;
  forkliftServo.write(currentAngle);
  delay(15);
  
  while (currentAngle < targetAngle) {
    currentAngle++;
    forkliftServo.write(currentAngle);
    delay(15);
  }
  delay(50);
}

// Moves the vehicle forward for a short distance and then stops.
void moveForwardShort() {
  motor1.setSpeed(85);
  motor2.setSpeed(85);
  motor3.setSpeed(85);
  motor4.setSpeed(85);
  
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(600);
  
  Stop();
}

// Moves the vehicle backward for a short distance and then stops.
void moveBackwardShort() {
  motor1.setSpeed(90);
  motor2.setSpeed(90);
  motor3.setSpeed(90);
  motor4.setSpeed(90);
  
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(600);
  
  Stop();
}

// Retrieves the distance measurement from the ultrasonic sensor.
int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = MAX_DISTANCE;
  }
  return cm;
}

// Rotates the ultrasonic sensor to the left to measure distance.
void lookLeft() {
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  servo.write(90);
  Serial.print("Left: ");
  Serial.println(leftDistance);
}

// Rotates the ultrasonic sensor to the right to measure distance.
void lookRight() {
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  servo.write(90);
  Serial.print("Right: ");
  Serial.println(rightDistance);
}

// Turns the vehicle around 180 degrees.
void turnAround() {
  Serial.println("Attempting to turn around");
  motor1.setSpeed(100);
  motor2.setSpeed(100);
  motor3.setSpeed(100);
  motor4.setSpeed(100);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(1200);

  Stop();
  Serial.println("Turn completed");
}

// Moves the vehicle left by reversing the right motors and forwarding the left motors.
void moveLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// Moves the vehicle right by reversing the left motors and forwarding the right motors.
void moveRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

// Moves the vehicle forward by running all motors forward.
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

// Moves the vehicle backward by running all motors backward, then stops.
void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  Serial.println("Backing Up");
  delay(300);
  Stop();
}

// Handles obstacle avoidance by stopping and choosing a direction based on sensor readings.
void objectAvoid() {
  int distance = getDistance();
  if (distance <= 20) {
    Stop();
    Serial.println("Obstacle Detected, Stopped");

    lookLeft();
    lookRight();
    delay(100);

    moveBackward();
    delay(500);
    Stop();

    if (rightDistance < 50 && leftDistance < 50) {
      turnAround();
    } else if (rightDistance >= leftDistance) {
      object = true;
      turn();  // Turn right
    } else {
      object = false;
      turn();  // Turn left
    }
    delay(100);
  } else {
    Serial.println("moveforward");
    moveForward();
  }
}

// Determines the turn direction based on object position and moves accordingly.
void turn() {
  if (object == false) {
    Serial.println("turn Right");
    moveLeft();
    delay(450);
    moveForward();
    delay(1000);
    moveRight();
    delay(500);
    if (digitalRead(irRight) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
  else {
    Serial.println("turn left");
    moveRight();
    delay (450);
    moveForward();
    delay (1000);
    moveLeft();
    delay (500);
    if (digitalRead(irLeft) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
}

// Control variable to enable or disable forklift operations.
bool isForkliftOperationEnabled = true; 

// Manages forklift operations based on sensor input and system state.
void handleForkliftOperation() {
  if (!isForkliftOperationEnabled) return; // Check if operation is enabled

  if (digitalRead(irFront) == 0) {
    Stop();
    lowerForklift(); // Lower the forklift first
    moveForwardShort(); // Move forward a little bit
    raiseForklift(); // Raise the forklift
    delay(1000); // Wait for a bit after operation
    isForkliftOperationEnabled = false; // Deactivate operation after one execution
  } else {
    checkAndHandleSensors(); // Handle sensors
  }
}

// Enum to define different system states.
enum SystemState {
  NORMAL_OPERATION,
  HALT_OPERATION,
  FORKLIFT_LOWERED
};

SystemState systemState = NORMAL_OPERATION; // Initial system state.

// Checks and handles sensor input to determine vehicle movement and system state.
void checkAndHandleSensors() {
  if (systemState == HALT_OPERATION) {
    return; // Early exit if system is halted
  }

  int irLeftState = digitalRead(irLeft);
    int irRightState = digitalRead(irRight);

  if (irLeftState == 0 && irRightState == 0) {
    moveForward();
  }
  else if (irLeftState == 1 && irRightState == 0) {
    moveLeft();
    delay(200);
    moveForward();
  }
  else if (irLeftState == 0 && irRightState == 1) {
    moveRight();
    delay(200);
    moveForward();
  }
  else if (irLeftState == 1 && irRightState == 1) {
    Stop();
    if (systemState != FORKLIFT_LOWERED) {
      lowerForklift();
      moveBackwardShort();
      systemState = HALT_OPERATION; // Update state to halt further operations
    }
  }

  if (systemState == NORMAL_OPERATION) {
    objectAvoid();
  }
}

// Main loop that handles system operations continuously.
void loop() {
  checkResetButton(); // Handle reset button press

  if (systemState == HALT_OPERATION) {
    Serial.println("System Halted."); // Debug message
    return; // Stop further actions in this loop iteration
  }

  // Normal operation
  checkAndHandleSensors();
  if (systemState == NORMAL_OPERATION) {
    handleForkliftOperation();
  }
}

// Checks the state of the reset button and resets system state if pressed.
void checkResetButton() {
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    isForkliftOperationEnabled = true;
    systemState = NORMAL_OPERATION; // Allow operations to continue
    delay(100); // Debounce delay
  }
}
