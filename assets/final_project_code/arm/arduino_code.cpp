#include <Servo.h>

// ---- Pose definitions (hardware angles) ----
const int startup_pose[5] = { 0, 0, 35, 0, 140 };
const int prepare_pose[5] = { 0, 45, 35, 15, 140};
const int grabbed_pose[5] = { 0, 45, 35, 15, 165};
const int holding_pose[5] = { 0, 0, 40, 20, 165};
const int overcup_pose[5] = { 145, 0, 0, 20, 165};
const int cupdrop_pose[5] = { 145, 0, 0, 20, 140};

// --- Servo Objects ---
Servo servo_base;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_gripper;

// --- Pins ---
const int PIN_BASE = 3;
const int PIN_2 = 5;
const int PIN_3 = 6;
const int PIN_4 = 9;
const int PIN_GRIPPER = 10;

// --- State variables (hardware servo angles) ---
int curr_base = startup_pose[0];
int curr_2 = startup_pose[1];
int curr_3 = startup_pose[2];
int curr_4 = startup_pose[3];
int curr_gripper = startup_pose[4];

// User input limits
const int USER_MIN = -45;
const int USER_MAX = 45;

// Hardware limits for arms
const int SERVO_MIN = 0;
const int SERVO_MAX = 45;

// Gripper limits
const int GRIP_MIN = 140;
const int GRIP_MAX = 180;

// ms per degree (smooth motion)
const int MOVE_DELAY = 60;

// Smooth movement
void moveServoSmooth(Servo &s, int &currentAngle, int targetAngle) {
  if (targetAngle > currentAngle) {
    for (int a = currentAngle; a <= targetAngle; a++) {
      s.write(a);
      delay(MOVE_DELAY);
    }
  } else {
    for (int a = currentAngle; a >= targetAngle; a--) {
      s.write(a);
      delay(MOVE_DELAY);
    }
  }
  currentAngle = targetAngle;
}

void moveToPose(const int pose[5]) {
  moveServoSmooth(servo_base,     curr_base,     pose[0]);
  moveServoSmooth(servo_2,        curr_2,        pose[1]);
  moveServoSmooth(servo_3,        curr_3,        pose[2]);
  moveServoSmooth(servo_4,        curr_4,        pose[3]);
  moveServoSmooth(servo_gripper,  curr_gripper,  pose[4]);
}

// same as above function, but one servo gets priority
void moveToPose(const int pose[5], int priorityIndex) {
  // Move the priority servo first
  switch (priorityIndex) {
    case 0: moveServoSmooth(servo_base,     curr_base,     pose[0]); break;
    case 1: moveServoSmooth(servo_2,        curr_2,        pose[1]); break;
    case 2: moveServoSmooth(servo_3,        curr_3,        pose[2]); break;
    case 3: moveServoSmooth(servo_4,        curr_4,        pose[3]); break;
    case 4: moveServoSmooth(servo_gripper,  curr_gripper,  pose[4]); break;
  }

  // Move the remaining servos (skip the priority one)
  if (priorityIndex != 0) moveServoSmooth(servo_base,     curr_base,     pose[0]);
  if (priorityIndex != 1) moveServoSmooth(servo_2,        curr_2,        pose[1]);
  if (priorityIndex != 2) moveServoSmooth(servo_3,        curr_3,        pose[2]);
  if (priorityIndex != 3) moveServoSmooth(servo_4,        curr_4,        pose[3]);
  if (priorityIndex != 4) moveServoSmooth(servo_gripper,  curr_gripper,  pose[4]);
}


// Convert user angle (-45..45) → safe servo angle (0..45)
int mapUserToServo(int userAngle) {
  userAngle = constrain(userAngle, USER_MIN, USER_MAX);
  int mapped = abs(userAngle);          // ALWAYS move toward zero position
  return constrain(mapped, SERVO_MIN, SERVO_MAX);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Commands: base -30   OR   s2 20   OR   grip 150");
  Serial.println("Arm angles accepted: -45 to 45   (mapped safely)");
  Serial.println("Gripper range: 140–180");

  servo_base.attach(PIN_BASE);
  servo_2.attach(PIN_2);
  servo_3.attach(PIN_3);
  servo_4.attach(PIN_4);
  servo_gripper.attach(PIN_GRIPPER);

  moveToPose(startup_pose);

  // servo_base.write(curr_base);
  // servo_2.write(curr_2);
  // servo_3.write(curr_3);
  // servo_4.write(curr_4);
  // servo_gripper.write(curr_gripper);

}

void loop() {
  if (Serial.available()) {
    // raspberry pi read
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    // old read for serial input
    int spaceIndex = input.indexOf(' ');
    int value = 0;
    String name = input;
    if (spaceIndex != -1) {
        name = input.substring(0, spaceIndex);
        value = input.substring(spaceIndex + 1).toInt();
    }
    name.trim();

    // String name = Serial.readStringUntil(' ');
    // name.trim();
    // int value   = Serial.readStringUntil('\n').toInt();

    // Full flow: from startup, grab the cube, raise, drop, return to startup
    if (input == "START") {
      Serial.println("Pi command: START");
      moveToPose(prepare_pose);
      moveToPose(grabbed_pose);
      moveToPose(holding_pose);
      moveToPose(overcup_pose, 2);
      moveToPose(cupdrop_pose);
      moveToPose(startup_pose); // return to startup pose

      return;
    }

    // --- Check for named poses ---
    if (name == "STARTUP") {
      Serial.println("Moving to startup pose");
      moveToPose(startup_pose);
    }
    else if (name == "PREPARE") {
      Serial.println("Moving to prepare pose");
      moveToPose(prepare_pose);
    }
    else if (name == "HOLD") {
      Serial.println("Moving to holding pose");
      moveToPose(holding_pose);
    }
    else if (name == "GRAB") {
      Serial.println("Shortcut for grip 165");
      int mapped = constrain(165, GRIP_MIN, GRIP_MAX);
      moveServoSmooth(servo_gripper, curr_gripper, mapped);
    }
    else if (name == "DROP") {
      Serial.println("Shortcut for grip 140");
      int mapped = constrain(140, GRIP_MIN, GRIP_MAX);
      moveServoSmooth(servo_gripper, curr_gripper, mapped);
    }
    else if (name == "BASE") {
        // base should have full range of motion, no need to constrain
        //int mapped = mapUserToServo(value);
        moveServoSmooth(servo_base, curr_base, value);
      }
      else if (name == "S2") {
        int mapped = mapUserToServo(value);
        moveServoSmooth(servo_2, curr_2, mapped);
      }
      else if (name == "S3") {
        int mapped = mapUserToServo(value);
        moveServoSmooth(servo_3, curr_3, mapped);
      }
      else if (name == "S4") {
        int mapped = mapUserToServo(value);
        moveServoSmooth(servo_4, curr_4, mapped);
      }
      else if (name == "GRIP") {
        int mapped = constrain(value, GRIP_MIN, GRIP_MAX);
        moveServoSmooth(servo_gripper, curr_gripper, mapped);
      }
      else {
        Serial.println("Unknown servo name.");
      }
      Serial.print("base=");
      Serial.print(curr_base);
      Serial.print("  s2=");
      Serial.print(curr_2);
      Serial.print("  s3=");
      Serial.print(curr_3);
      Serial.print("  s4=");
      Serial.print(curr_4);
      Serial.print("  grip=");
      Serial.println(curr_gripper);

    }
}
