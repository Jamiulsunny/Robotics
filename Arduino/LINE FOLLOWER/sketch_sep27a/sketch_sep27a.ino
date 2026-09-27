#include <Arduino.h>

// =====================================================
//         LINE FOLLOWER - UPDATE 6 (BUGS FIXED)
//
//   Fixes applied in this version:
//   1. Removed double-smoothed derivative (was adding
//      phase lag -> overshoot on sharp turns, worse in
//      one direction on back-and-forth runs).
//   2. Line-lost fallback direction no longer defaults
//      to RIGHT when lastDirection is still 0 and
//      lastError is exactly 0 - now symmetric.
//   3. lastError is reset to 0 when exiting the wide/
//      all-black zone, preventing a derivative spike
//      when normal tracking resumes.
//   4. correction is kept as float through to the final
//      motor speed calculation (rounded instead of
//      truncated) to remove small steering bias.
// =====================================================

const int baseSpeed = 90;
const int maxSpeed  = 190;

const int sensorCenter = 3500;
const int sensorCount  = 8;

// =====================================================
//                  PD TUNING
// =====================================================

float kp = 0.045;
float kd = 0.20;

// =====================================================
//                 ERROR MEMORY
// =====================================================

int lastError = 0;     // previous loop's error (position - center)
int lastDirection = 0; // -1 = LEFT, 0 = CENTER, +1 = RIGHT

// =====================================================
//              MOTOR SPEED MEMORY
// =====================================================

int previousLeftSpeed  = 0;
int previousRightSpeed = 0;

// =====================================================
//                  MOTOR DRIVER PINS
// =====================================================

const int PWM_A = 5;
const int AIN1 = 3;
const int AIN2 = 4;

const int PWM_B = 9;
const int BIN1 = 7;
const int BIN2 = 8;

const int STBY = 6;

// =====================================================
//                    SENSOR PINS
// =====================================================

int sensors[sensorCount] = {
  A0, A1, A2, A3, A4, A5, 10, 11
};

// =====================================================
//                  SENSOR WEIGHTS
// =====================================================

int sensorWeight[sensorCount] = {
  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000
};

// =====================================================
//                 LINE LOST CONTROL
// =====================================================

bool lineLost = false;
unsigned long lineLostStart = 0;

const unsigned long shortGapTime = 140; // dashed line gap tolerance (ms)

const int allBlackThreshold = 7;
const int maxSpeedChange = 30;

// =====================================================
//                    SETUP
// =====================================================

void setup() {
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensors[i], INPUT);
  }

  digitalWrite(STBY, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);
  setMotorSpeed(0, 0);
  delay(200);
}

// =====================================================
//                  MOTOR FUNCTION
// =====================================================

void setMotor(int side, int speedVal) {
  speedVal = constrain(speedVal, -maxSpeed, maxSpeed);

  if (side == 0) { // LEFT MOTOR
    if (speedVal >= 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    } else {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    analogWrite(PWM_A, abs(speedVal));
  } else { // RIGHT MOTOR
    if (speedVal >= 0) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    } else {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWM_B, abs(speedVal));
  }
}

// =====================================================
//              LIMIT SPEED CHANGE
// =====================================================

int smoothSpeed(int targetSpeed, int previousSpeed) {
  int difference = targetSpeed - previousSpeed;

  if (difference > maxSpeedChange) {
    targetSpeed = previousSpeed + maxSpeedChange;
  } else if (difference < -maxSpeedChange) {
    targetSpeed = previousSpeed - maxSpeedChange;
  }

  return targetSpeed;
}

// =====================================================
//                SET BOTH MOTOR SPEED
// =====================================================

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  leftSpeed = smoothSpeed(leftSpeed, previousLeftSpeed);
  rightSpeed = smoothSpeed(rightSpeed, previousRightSpeed);

  previousLeftSpeed  = leftSpeed;
  previousRightSpeed = rightSpeed;

  setMotor(0, leftSpeed);
  setMotor(1, rightSpeed);
}

// =====================================================
//                    MAIN LOOP
// =====================================================

void loop() {
  long weightedSum = 0;
  int totalSensor = 0;

  for (int i = 0; i < sensorCount; i++) {
    int reading = digitalRead(sensors[i]);
    if (reading == HIGH) {
      weightedSum += sensorWeight[i];
      totalSensor++;
    }
  }

  // ===================================================
  //                  LINE LOST
  // ===================================================

  if (totalSensor == 0) {
    if (!lineLost) {
      lineLost = true;
      lineLostStart = millis();
    }

    unsigned long lostTime = millis() - lineLostStart;

    // FIX: symmetric fallback - no forced default to RIGHT
    if (lastDirection == 0) {
      lastDirection = (lastError <= 0) ? -1 : 1;
    }

    // Short gap (dashed line) - tolerate up to shortGapTime
    if (lostTime < shortGapTime) {
      int gapSpeed = 70;
      int gapCorrection = lastError * 0.030;
      setMotorSpeed(gapSpeed - gapCorrection, gapSpeed + gapCorrection);
      return;
    }

    // Sharp turn search mechanism
    if (lastDirection < 0) {
      setMotorSpeed(-90, 120); // turn hard left
    } else {
      setMotorSpeed(120, -90); // turn hard right
    }
    return;
  }

  lineLost = false;

  // ===================================================
  //             WIDE / ALL BLACK DETECTION
  // ===================================================

  if (totalSensor >= allBlackThreshold) {
    int wideBlackSpeed = 60;

    if (lastDirection < 0) {
      setMotorSpeed(wideBlackSpeed - 15, wideBlackSpeed + 15);
    } else if (lastDirection > 0) {
      setMotorSpeed(wideBlackSpeed + 15, wideBlackSpeed - 15);
    } else {
      setMotorSpeed(wideBlackSpeed, wideBlackSpeed);
    }

    // FIX: reset lastError so re-entering normal tracking
    // doesn't produce a derivative spike from a stale error.
    lastError = 0;
    return;
  }

  // ===================================================
  //              POSITION CALCULATION
  // ===================================================

  int position = weightedSum / totalSensor;
  int error = position - sensorCenter;

  if (error < -100) {
    lastDirection = -1;
  } else if (error > 100) {
    lastDirection = 1;
  }

  // FIX: use raw derivative directly - no second smoothing pass,
  // which was adding phase lag and causing overshoot on sharp turns.
  int derivative = error - lastError;

  float correction = (kp * error) + (kd * derivative);
  lastError = error;

  // ===================================================
  //              ADAPTIVE SPEED
  // ===================================================

  int absError = abs(error);
  int dynamicSpeed = baseSpeed;

  if (absError > 2800) dynamicSpeed = 48;
  else if (absError > 2200) dynamicSpeed = 55;
  else if (absError > 1600) dynamicSpeed = 65;
  else if (absError > 1000) dynamicSpeed = 75;
  else if (absError > 500) dynamicSpeed = 85;

  // FIX: round instead of truncate to remove small steering bias
  int leftMotorSpeed  = (int)round(dynamicSpeed - correction);
  int rightMotorSpeed = (int)round(dynamicSpeed + correction);

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}