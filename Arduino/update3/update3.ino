#include <Arduino.h>

// =====================================================
//              LINE FOLLOWER - UPDATED 2
//              BLACK LINE DETECTION FIX
// =====================================================


// =====================================================
//                 LINE FOLLOWER SETTINGS
// =====================================================

const int baseSpeed = 95;
const int maxSpeed  = 200;

const int sensorCenter = 3500;
const int sensorCount  = 8;


// =====================================================
//                    PD TUNING
// =====================================================

float kp = 0.040;
float kd = 0.22;

int lastError = 0;

int lastDirection = 0;


// =====================================================
//                 MOTOR DRIVER PINS
// =====================================================

// LEFT MOTOR
const int PWM_A = 5;
const int AIN1 = 3;
const int AIN2 = 4;

// RIGHT MOTOR
const int PWM_B = 9;
const int BIN1 = 7;
const int BIN2 = 8;

// STANDBY
const int STBY = 6;


// =====================================================
//                    SENSOR PINS
// =====================================================

// D1 -> A0
// D2 -> A1
// D3 -> A2
// D4 -> A3
// D5 -> A4
// D6 -> A5
// D7 -> D10
// D8 -> D11

int sensors[sensorCount] = {
  A0, A1, A2, A3,
  A4, A5, 10, 11
};


// =====================================================
//                  SENSOR WEIGHTS
// =====================================================

int sensorWeight[sensorCount] = {
  0,
  1000,
  2000,
  3000,
  4000,
  5000,
  6000,
  7000
};


// =====================================================
//                 LINE LOST SETTINGS
// =====================================================

unsigned long lineLostStart = 0;

bool lineLost = false;

const unsigned long shortGapTime = 100;


// =====================================================
//                       SETUP
// =====================================================

void setup() {

  // Motor pins
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);


  // Sensor pins
  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensors[i], INPUT);
  }


  // Enable motor driver
  digitalWrite(STBY, HIGH);


  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);


  // Start delay
  delay(1000);


  // Stop motor
  setMotorSpeed(0, 0);
}


// =====================================================
//                    MOTOR FUNCTION
// =====================================================

void setMotor(int side, int speedVal) {

  speedVal = constrain(
    speedVal,
    -maxSpeed,
    maxSpeed
  );


  // ---------------------------------------------------
  // LEFT MOTOR
  // ---------------------------------------------------

  if (side == 0) {

    if (speedVal >= 0) {

      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);

    } else {

      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }

    analogWrite(
      PWM_A,
      abs(speedVal)
    );
  }


  // ---------------------------------------------------
  // RIGHT MOTOR
  // ---------------------------------------------------

  else {

    if (speedVal >= 0) {

      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);

    } else {

      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }

    analogWrite(
      PWM_B,
      abs(speedVal)
    );
  }
}


// =====================================================
//                SET BOTH MOTOR SPEED
// =====================================================

void setMotorSpeed(
  int leftSpeed,
  int rightSpeed
) {

  leftSpeed = constrain(
    leftSpeed,
    -maxSpeed,
    maxSpeed
  );

  rightSpeed = constrain(
    rightSpeed,
    -maxSpeed,
    maxSpeed
  );


  setMotor(0, leftSpeed);
  setMotor(1, rightSpeed);
}


// =====================================================
//                      MAIN LOOP
// =====================================================

void loop() {

  long weightedSum = 0;

  int totalSensor = 0;


  // ===================================================
  //                  READ SENSORS
  // ===================================================

  for (int i = 0; i < sensorCount; i++) {

    int reading = digitalRead(
      sensors[i]
    );


    // =================================================
    // IMPORTANT:
    //
    // YOUR SENSOR:
    //
    // WHITE = LOW
    // BLACK = HIGH
    //
    // Therefore BLACK detection = HIGH
    // =================================================

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


    unsigned long lostTime =
      millis() - lineLostStart;


    // -------------------------------------------------
    // SHORT WHITE GAP
    // -------------------------------------------------

    if (lostTime < shortGapTime) {

      int gapSpeed = 85;

      int gapCorrection =
        lastError * 0.035;


      int leftSpeed =
        gapSpeed - gapCorrection;

      int rightSpeed =
        gapSpeed + gapCorrection;


      setMotorSpeed(
        leftSpeed,
        rightSpeed
      );

      return;
    }


    // -------------------------------------------------
    // LONG LINE LOSS
    // -------------------------------------------------

    if (lastDirection < 0) {

      // Search LEFT
      setMotorSpeed(
        -75,
        120
      );

    }

    else if (lastDirection > 0) {

      // Search RIGHT
      setMotorSpeed(
        120,
        -75
      );

    }

    else {

      // Unknown
      setMotorSpeed(
        70,
        70
      );
    }

    return;
  }


  // ===================================================
  //             BLACK LINE FOUND
  // ===================================================

  lineLost = false;


  // ===================================================
  //                POSITION CALCULATION
  // ===================================================

  int position =
    weightedSum / totalSensor;


  int error =
    position - sensorCenter;


  // ===================================================
  //                UPDATE DIRECTION
  // ===================================================

  if (error < -250) {

    lastDirection = -1;
  }

  else if (error > 250) {

    lastDirection = 1;
  }


  // ===================================================
  //                    PD CONTROL
  // ===================================================

  int derivative =
    error - lastError;


  float correction =
    (kp * error) +
    (kd * derivative);


  lastError = error;


  // ===================================================
  //              ADAPTIVE SPEED
  // ===================================================

  int absError =
    abs(error);


  int dynamicSpeed =
    baseSpeed;


  // ---------------------------------------------------
  // VERY SHARP TURN
  // ---------------------------------------------------

  if (absError > 2600) {

    dynamicSpeed = 55;
  }


  // ---------------------------------------------------
  // SHARP TURN
  // ---------------------------------------------------

  else if (absError > 1900) {

    dynamicSpeed = 65;
  }


  // ---------------------------------------------------
  // MEDIUM TURN
  // ---------------------------------------------------

  else if (absError > 1200) {

    dynamicSpeed = 78;
  }


  // ---------------------------------------------------
  // SMALL TURN
  // ---------------------------------------------------

  else if (absError > 600) {

    dynamicSpeed = 88;
  }


  // ===================================================
  //                  MOTOR OUTPUT
  // ===================================================

  int leftMotorSpeed =
    dynamicSpeed - correction;


  int rightMotorSpeed =
    dynamicSpeed + correction;


  // ===================================================
  //                  LIMIT OUTPUT
  // ===================================================

  leftMotorSpeed =
    constrain(
      leftMotorSpeed,
      -maxSpeed,
      maxSpeed
    );


  rightMotorSpeed =
    constrain(
      rightMotorSpeed,
      -maxSpeed,
      maxSpeed
    );


  // ===================================================
  //                  MOTOR CONTROL
  // ===================================================

  setMotorSpeed(
    leftMotorSpeed,
    rightMotorSpeed
  );
}