#include <Wire.h>
#include <math.h>

// =====================================================
// MPU6050
// =====================================================

#define MPU_ADDR 0x68

#define SDA_PIN 18
#define SCL_PIN 19


// =====================================================
// TB6612FNG MOTOR DRIVER
// =====================================================

#define STBY_PIN 13

#define AIN1_PIN 21
#define AIN2_PIN 17
#define PWMA_PIN 25

#define BIN1_PIN 27
#define BIN2_PIN 14
#define PWMB_PIN 26


const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;


// =====================================================
// ENCODERS
// =====================================================

// Left encoder

#define ENC_LEFT_A 34
#define ENC_LEFT_B 35

// Right encoder

#define ENC_RIGHT_A 32
#define ENC_RIGHT_B 33


// =====================================================
// ENCODER COUNTERS
// =====================================================

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;


// =====================================================
// ENCODER DIRECTION
// =====================================================

const int ENC_LEFT_SIGN = 1;
const int ENC_RIGHT_SIGN = 1;


// =====================================================
// ENCODER VELOCITY
// =====================================================

float leftVelocity = 0.0;
float rightVelocity = 0.0;

long previousLeftCount = 0;
long previousRightCount = 0;

const unsigned long VELOCITY_INTERVAL_MS = 20;

unsigned long previousVelocityTime = 0;


// =====================================================
// SENSOR VALUES
// =====================================================

float gyroX = 0.0;
float gyroXOffset = 0.0;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float accelAngle = 0.0;
float gyroAngle = 0.0;

float currentAngle = 0.0;


// =====================================================
// FILTER TIMING
// =====================================================

unsigned long previousTime = 0;


// =====================================================
// COMPLEMENTARY FILTER
// =====================================================

const float TAU = 0.75;


// =====================================================
// OUTER VELOCITY PI CONTROLLER
// =====================================================

float targetVelocity = 0.0;

float velocityKp = 0.02;
float velocityKi = 0.01;

float velocityError = 0.0;

float velocityIntegral = 0.0;


// =====================================================
// VELOCITY INTEGRAL PROTECTION
// =====================================================

const float VELOCITY_INTEGRAL_LIMIT = 5000.0;


// =====================================================
// MAXIMUM VELOCITY ANGLE COMMAND
// =====================================================

const float MAX_VELOCITY_ANGLE_COMMAND = 8.0;


// =====================================================
// VELOCITY LOOP GATING
// =====================================================
//
// The velocity loop is only active when the robot is
// reasonably close to its balance angle.
//
// Stage 7:
// ±10° → velocity loop disabled.
//
// =====================================================

const float VELOCITY_LOOP_ANGLE_ERROR_LIMIT = 10.0;

bool velocityLoopEnabled = false;


// =====================================================
// STAGE 8 - FALL DETECTION
// =====================================================
//
// Fall detection is intentionally separate from velocity
// loop gating.
//
// Stage 7:
// ±10° → velocity loop OFF
//
// Stage 8:
// ±25° → robot considered FALLEN
//
// =====================================================

const float FALL_ANGLE_LIMIT = 25.0;


// =====================================================
// FALL RECOVERY ANGLE
// =====================================================
//
// Once the robot has been declared fallen, it must return
// closer to its balance angle before the controllers are
// allowed to restart.
//
// This prevents the robot from immediately restarting
// while it is still significantly tilted.
//
// =====================================================

const float FALL_RECOVERY_ANGLE_LIMIT = 10.0;


// =====================================================
// ROBOT FALL STATE
// =====================================================

bool robotFallen = false;


// =====================================================
// VELOCITY CONTROL TIMING
// =====================================================

unsigned long previousVelocityControlTime = 0;


// =====================================================
// ANGLE PID CONTROLLER
// =====================================================

float baseTargetAngle = 0.0;

float targetAngle = 0.0;

float Kp = 19.0;
float Ki = 0.1;
float Kd = 0.8;


// =====================================================
// PID STATE
// =====================================================

float error = 0.0;

float previousError = 0.0;

float previousAngle = 0.0;

float errorSum = 0.0;


// =====================================================
// PID LIMITS
// =====================================================

const float ERROR_SUM_LIMIT = 300.0;

const float MOTOR_POWER_LIMIT = 255.0;


// =====================================================
// WRITE MPU6050 REGISTER
// =====================================================

void writeRegister(byte reg, byte value)
{
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}


// =====================================================
// READ MPU6050
// =====================================================

bool readMPU(float &gx, float &ax, float &ay, float &az)
{
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom(MPU_ADDR, 14) != 14)
    return false;


  int16_t rawAccelX =
      (Wire.read() << 8) | Wire.read();

  int16_t rawAccelY =
      (Wire.read() << 8) | Wire.read();

  int16_t rawAccelZ =
      (Wire.read() << 8) | Wire.read();


  // Temperature

  Wire.read();
  Wire.read();


  int16_t rawGyroX =
      (Wire.read() << 8) | Wire.read();


  // Remaining gyro axes

  Wire.read();
  Wire.read();
  Wire.read();
  Wire.read();


  // Accelerometer configured for ±2g

  ax = rawAccelX / 16384.0;
  ay = rawAccelY / 16384.0;
  az = rawAccelZ / 16384.0;


  // Gyroscope configured for ±250 °/s

  gx = rawGyroX / 131.0;


  return true;
}


// =====================================================
// CALCULATE ACCELEROMETER TILT ANGLE
// =====================================================

float calculateAccelAngle(float ay, float az)
{
  return atan2(ay, az) * 180.0 / PI;
}


// =====================================================
// GYRO CALIBRATION
// =====================================================

void calibrateGyro()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("       GYRO CALIBRATION");
  Serial.println("================================");
  Serial.println();

  Serial.println("Keep the robot COMPLETELY STILL.");
  Serial.println("Calibration starts in 3 seconds...");

  delay(3000);


  const int samples = 1000;

  float sum = 0.0;

  int validSamples = 0;


  Serial.println("Calibrating...");


  for (int i = 0; i < samples; i++)
  {
    float gx;
    float ax;
    float ay;
    float az;


    if (readMPU(gx, ax, ay, az))
    {
      sum += gx;
      validSamples++;
    }


    delay(2);
  }


  if (validSamples == 0)
  {
    Serial.println("ERROR: MPU6050 NOT RESPONDING!");

    while (true)
    {
      delay(100);
    }
  }


  gyroXOffset = sum / validSamples;


  Serial.println();
  Serial.println("Calibration complete.");

  Serial.print("Valid samples: ");
  Serial.println(validSamples);

  Serial.print("Gyro X offset: ");
  Serial.print(gyroXOffset, 3);
  Serial.println(" °/s");

  Serial.println();
}


// =====================================================
// LEFT ENCODER ISR
// =====================================================

void IRAM_ATTR leftEncoderISR()
{
  bool channelB =
      digitalRead(ENC_LEFT_B);

  if (channelB)
  {
    leftEncoderCount -= ENC_LEFT_SIGN;
  }
  else
  {
    leftEncoderCount += ENC_LEFT_SIGN;
  }
}


// =====================================================
// RIGHT ENCODER ISR
// =====================================================

void IRAM_ATTR rightEncoderISR()
{
  bool channelB =
      digitalRead(ENC_RIGHT_B);

  if (channelB)
  {
    rightEncoderCount -= ENC_RIGHT_SIGN;
  }
  else
  {
    rightEncoderCount += ENC_RIGHT_SIGN;
  }
}


// =====================================================
// ENCODER SETUP
// =====================================================

void setupEncoders()
{
  pinMode(
      ENC_LEFT_A,
      INPUT
  );

  pinMode(
      ENC_LEFT_B,
      INPUT
  );

  pinMode(
      ENC_RIGHT_A,
      INPUT
  );

  pinMode(
      ENC_RIGHT_B,
      INPUT
  );


  attachInterrupt(
      digitalPinToInterrupt(ENC_LEFT_A),
      leftEncoderISR,
      RISING
  );


  attachInterrupt(
      digitalPinToInterrupt(ENC_RIGHT_A),
      rightEncoderISR,
      RISING
  );


  Serial.println("Encoder feedback initialized.");

  Serial.print("Left encoder A: GPIO ");
  Serial.println(ENC_LEFT_A);

  Serial.print("Left encoder B: GPIO ");
  Serial.println(ENC_LEFT_B);

  Serial.print("Right encoder A: GPIO ");
  Serial.println(ENC_RIGHT_A);

  Serial.print("Right encoder B: GPIO ");
  Serial.println(ENC_RIGHT_B);

  Serial.println();
}


// =====================================================
// READ ENCODER COUNTS
// =====================================================

void readEncoderCounts(
    long &leftCount,
    long &rightCount)
{
  noInterrupts();

  leftCount =
      leftEncoderCount;

  rightCount =
      rightEncoderCount;

  interrupts();
}


// =====================================================
// CALCULATE WHEEL VELOCITY
// =====================================================

void updateWheelVelocity()
{
  unsigned long now =
      millis();


  unsigned long elapsed =
      now - previousVelocityTime;


  if (elapsed < VELOCITY_INTERVAL_MS)
    return;


  long leftCount;
  long rightCount;


  readEncoderCounts(
      leftCount,
      rightCount
  );


  long deltaLeft =
      leftCount - previousLeftCount;

  long deltaRight =
      rightCount - previousRightCount;


  float dt =
      elapsed / 1000.0;


  if (dt > 0.0)
  {
    leftVelocity =
        deltaLeft / dt;

    rightVelocity =
        deltaRight / dt;
  }


  previousLeftCount =
      leftCount;

  previousRightCount =
      rightCount;

  previousVelocityTime =
      now;
}


// =====================================================
// CALCULATE AVERAGE WHEEL VELOCITY
// =====================================================

float calculateAverageVelocity()
{
  return (leftVelocity + rightVelocity) / 2.0;
}


// =====================================================
// RESET VELOCITY PI
// =====================================================

void resetVelocityPI()
{
  velocityError = 0.0;

  velocityIntegral = 0.0;

  velocityLoopEnabled = false;

  targetAngle =
      baseTargetAngle;
}


// =====================================================
// OUTER VELOCITY PI CONTROLLER
// =====================================================

void updateVelocityPI(float dt)
{
  if (dt <= 0.0)
    return;


  // ---------------------------------------------------
  // DO NOT RUN VELOCITY CONTROL WHILE FALLEN
  // ---------------------------------------------------

  if (robotFallen)
  {
    velocityLoopEnabled = false;

    velocityError = 0.0;

    velocityIntegral = 0.0;

    targetAngle =
        baseTargetAngle;

    return;
  }


  // ---------------------------------------------------
  // MEASURED VELOCITY
  // ---------------------------------------------------

  float measuredVelocity =
      calculateAverageVelocity();


  // ---------------------------------------------------
  // VELOCITY ERROR
  // ---------------------------------------------------

  velocityError =
      targetVelocity - measuredVelocity;


  // ---------------------------------------------------
  // VELOCITY LOOP GATING
  // ---------------------------------------------------

  float angleErrorFromBalance =
      currentAngle - baseTargetAngle;


  if (fabs(angleErrorFromBalance) >
      VELOCITY_LOOP_ANGLE_ERROR_LIMIT)
  {
    velocityLoopEnabled = false;

    velocityIntegral = 0.0;

    targetAngle =
        baseTargetAngle;

    return;
  }


  velocityLoopEnabled = true;


  // ---------------------------------------------------
  // PROPORTIONAL TERM
  // ---------------------------------------------------

  float P =
      velocityKp * velocityError;


  // ---------------------------------------------------
  // PROVISIONAL INTEGRAL
  // ---------------------------------------------------

  float proposedIntegral =
      velocityIntegral
      + velocityError * dt;


  proposedIntegral =
      constrain(
          proposedIntegral,
          -VELOCITY_INTEGRAL_LIMIT,
          VELOCITY_INTEGRAL_LIMIT
      );


  float I =
      velocityKi * proposedIntegral;


  // ---------------------------------------------------
  // PROVISIONAL VELOCITY OUTPUT
  // ---------------------------------------------------

  float velocityAngleCommand =
      P + I;


  // ---------------------------------------------------
  // LIMIT VELOCITY ANGLE COMMAND
  // ---------------------------------------------------

  float limitedVelocityAngleCommand =
      constrain(
          velocityAngleCommand,
          -MAX_VELOCITY_ANGLE_COMMAND,
          MAX_VELOCITY_ANGLE_COMMAND
      );


  // ---------------------------------------------------
  // INTEGRAL ANTI-WINDUP
  // ---------------------------------------------------

  bool outputSaturated =
      (velocityAngleCommand !=
       limitedVelocityAngleCommand);


  bool errorReducesSaturation =
      false;


  if (outputSaturated)
  {
    if (velocityAngleCommand >
        MAX_VELOCITY_ANGLE_COMMAND)
    {
      if (velocityError < 0.0)
      {
        errorReducesSaturation = true;
      }
    }

    else if (velocityAngleCommand <
             -MAX_VELOCITY_ANGLE_COMMAND)
    {
      if (velocityError > 0.0)
      {
        errorReducesSaturation = true;
      }
    }
  }


  // ---------------------------------------------------
  // ACCEPT OR REJECT INTEGRAL UPDATE
  // ---------------------------------------------------

  if (!outputSaturated ||
      errorReducesSaturation)
  {
    velocityIntegral =
        proposedIntegral;
  }


  // ---------------------------------------------------
  // RECALCULATE INTEGRAL TERM
  // ---------------------------------------------------

  I =
      velocityKi * velocityIntegral;


  // ---------------------------------------------------
  // FINAL VELOCITY CONTROLLER OUTPUT
  // ---------------------------------------------------

  velocityAngleCommand =
      P + I;


  velocityAngleCommand =
      constrain(
          velocityAngleCommand,
          -MAX_VELOCITY_ANGLE_COMMAND,
          MAX_VELOCITY_ANGLE_COMMAND
      );


  // ---------------------------------------------------
  // COMMAND INNER ANGLE LOOP
  // ---------------------------------------------------

  targetAngle =
      baseTargetAngle
      + velocityAngleCommand;
}


// =====================================================
// MOTOR SETUP
// =====================================================

void setupMotors()
{
  pinMode(STBY_PIN, OUTPUT);

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);


  digitalWrite(STBY_PIN, HIGH);


  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PWMA_PIN, 0);
  ledcAttachPin(PWMB_PIN, 1);


  ledcWrite(0, 0);
  ledcWrite(1, 0);
}


// =====================================================
// MOTOR A
// =====================================================

void driveMotorA(int speed)
{
  speed =
      constrain(
          speed,
          -255,
          255
      );


  if (speed > 0)
  {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  }

  else if (speed < 0)
  {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  }

  else
  {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
  }


  ledcWrite(
      0,
      abs(speed)
  );
}


// =====================================================
// MOTOR B
// =====================================================

void driveMotorB(int speed)
{
  speed =
      constrain(
          speed,
          -255,
          255
      );


  if (speed > 0)
  {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  }

  else if (speed < 0)
  {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }

  else
  {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
  }


  ledcWrite(
      1,
      abs(speed)
  );
}


// =====================================================
// DRIVE BOTH MOTORS
// =====================================================

void driveMotors(float power)
{
  int pwm =
      (int)power;


  pwm =
      constrain(
          pwm,
          -255,
          255
      );


  driveMotorA(pwm);

  driveMotorB(-pwm);
}


// =====================================================
// RESET ANGLE PID
// =====================================================

void resetPID()
{
  error = 0.0;

  previousError = 0.0;

  previousAngle =
      currentAngle;

  errorSum = 0.0;
}


// =====================================================
// ANGLE PID
// =====================================================

float calculateAnglePID(float dt)
{
  // ---------------------------------------------------
  // DO NOT RUN ANGLE PID WHILE FALLEN
  // ---------------------------------------------------

  if (robotFallen)
  {
    error = 0.0;

    return 0.0;
  }


  // ---------------------------------------------------
  // ERROR
  // ---------------------------------------------------

  error =
      targetAngle - currentAngle;


  // ---------------------------------------------------
  // INTEGRAL
  // ---------------------------------------------------

  errorSum +=
      error * dt;


  errorSum =
      constrain(
          errorSum,
          -ERROR_SUM_LIMIT,
          ERROR_SUM_LIMIT
      );


  // ---------------------------------------------------
  // ANGULAR RATE
  // ---------------------------------------------------

  float angleRate = 0.0;


  if (dt > 0.0)
  {
    angleRate =
        (currentAngle - previousAngle)
        / dt;
  }


  // ---------------------------------------------------
  // PID TERMS
  // ---------------------------------------------------

  float P =
      Kp * error;

  float I =
      Ki * errorSum;

  float D =
      -Kd * angleRate;


  // ---------------------------------------------------
  // TOTAL OUTPUT
  // ---------------------------------------------------

  float output =
      P + I + D;


  output =
      constrain(
          output,
          -MOTOR_POWER_LIMIT,
          MOTOR_POWER_LIMIT
      );


  return output;
}


// =====================================================
// FALL DETECTION
// =====================================================
//
// Returns true when the robot is sufficiently far from
// its physical balance angle.
//
// =====================================================

bool detectFall()
{
  float angleErrorFromBalance =
      currentAngle - baseTargetAngle;


  return (
      fabs(angleErrorFromBalance)
      >= FALL_ANGLE_LIMIT
  );
}


// =====================================================
// ENTER FALLEN STATE
// =====================================================
//
// This function performs the complete controller
// shutdown/reset sequence.
//
// =====================================================

void enterFallenState()
{
  robotFallen = true;

  // Immediately stop the motors.

  driveMotors(0);


  // Reset inner angle controller.

  resetPID();


  // Reset outer velocity controller.

  resetVelocityPI();


  // Force neutral target.

  targetAngle =
      baseTargetAngle;


  Serial.println();
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("          ROBOT FALL DETECTED");
  Serial.println("          MOTORS DISABLED");
  Serial.println("          CONTROLLERS RESET");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println();
}


// =====================================================
// FALL RECOVERY
// =====================================================
//
// The robot must return within the recovery angle before
// normal control is allowed to restart.
//
// =====================================================

void checkFallRecovery()
{
  if (!robotFallen)
    return;


  float angleErrorFromBalance =
      currentAngle - baseTargetAngle;


  if (fabs(angleErrorFromBalance) <=
      FALL_RECOVERY_ANGLE_LIMIT)
  {
    robotFallen = false;


    // Reset both controllers again immediately before
    // returning to normal operation.

    resetPID();

    resetVelocityPI();


    targetAngle =
        baseTargetAngle;


    Serial.println();
    Serial.println("========================================");
    Serial.println("       ROBOT RECOVERY DETECTED");
    Serial.println("       CONTROLLERS RESET");
    Serial.println("       NORMAL CONTROL ENABLED");
    Serial.println("========================================");
    Serial.println();
  }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(500);


  // ---------------------------------------------------
  // I2C
  // ---------------------------------------------------

  Wire.begin(
      SDA_PIN,
      SCL_PIN
  );

  Wire.setClock(400000);

  delay(100);


  // ---------------------------------------------------
  // MPU6050 CONFIGURATION
  // ---------------------------------------------------

  writeRegister(
      0x6B,
      0x00
  );

  delay(100);


  writeRegister(
      0x1C,
      0x00
  );

  delay(100);


  writeRegister(
      0x1B,
      0x00
  );

  delay(100);


  writeRegister(
      0x1A,
      0x01
  );

  delay(100);


  // ---------------------------------------------------
  // GYRO CALIBRATION
  // ---------------------------------------------------

  calibrateGyro();


  // ---------------------------------------------------
  // INITIAL ATTITUDE
  // ---------------------------------------------------

  float gx;
  float ax;
  float ay;
  float az;


  if (readMPU(
          gx,
          ax,
          ay,
          az))
  {
    accelAngle =
        calculateAccelAngle(
            ay,
            az
        );


    gyroAngle =
        accelAngle;


    currentAngle =
        accelAngle;


    previousAngle =
        currentAngle;
  }

  else
  {
    Serial.println(
        "ERROR: Initial MPU6050 read failed!"
    );


    while (true)
    {
      delay(100);
    }
  }


  // ---------------------------------------------------
  // INITIAL TIMING
  // ---------------------------------------------------

  previousTime =
      micros();

  previousVelocityTime =
      millis();

  previousVelocityControlTime =
      millis();


  // ---------------------------------------------------
  // MOTOR SETUP
  // ---------------------------------------------------

  setupMotors();

  driveMotors(0);


  // ---------------------------------------------------
  // ENCODER SETUP
  // ---------------------------------------------------

  setupEncoders();


  // ---------------------------------------------------
  // CONTROLLER INITIALIZATION
  // ---------------------------------------------------

  baseTargetAngle =
      currentAngle;

  targetAngle =
      baseTargetAngle;


  robotFallen =
      false;


  resetPID();

  resetVelocityPI();


  // ---------------------------------------------------
  // STAGE 8 MESSAGE
  // ---------------------------------------------------

  Serial.println("==============================================");
  Serial.println(" STAGE 8 - FALL DETECTION + RESET LOGIC");
  Serial.println("==============================================");
  Serial.println();

  Serial.println(
      "Gyro calibration active."
  );

  Serial.println(
      "Complementary filter active."
  );

  Serial.println(
      "Inner angle PID active."
  );

  Serial.println(
      "Wheel encoder feedback active."
  );

  Serial.println(
      "Outer velocity PI active."
  );

  Serial.println(
      "Velocity-loop gating active."
  );

  Serial.println(
      "Velocity integral anti-windup active."
  );

  Serial.println(
      "Fall detection active."
  );

  Serial.println(
      "Controller reset on fall active."
  );

  Serial.println();

  Serial.print("Angle Kp: ");
  Serial.println(Kp);

  Serial.print("Angle Ki: ");
  Serial.println(Ki);

  Serial.print("Angle Kd: ");
  Serial.println(Kd);

  Serial.print("Velocity Kp: ");
  Serial.println(velocityKp);

  Serial.print("Velocity Ki: ");
  Serial.println(velocityKi);

  Serial.print("Velocity integral limit: ");
  Serial.println(VELOCITY_INTEGRAL_LIMIT);

  Serial.print("Maximum velocity angle command: ");
  Serial.print(MAX_VELOCITY_ANGLE_COMMAND);
  Serial.println(" °");

  Serial.print("Velocity loop angle limit: ");
  Serial.print(VELOCITY_LOOP_ANGLE_ERROR_LIMIT);
  Serial.println(" °");

  Serial.print("Fall angle limit: ");
  Serial.print(FALL_ANGLE_LIMIT);
  Serial.println(" °");

  Serial.print("Fall recovery angle limit: ");
  Serial.print(FALL_RECOVERY_ANGLE_LIMIT);
  Serial.println(" °");

  Serial.print("Target velocity: ");
  Serial.print(targetVelocity, 2);
  Serial.println(" cnt/s");

  Serial.print("Base target angle: ");
  Serial.print(baseTargetAngle, 2);
  Serial.println(" °");

  Serial.println();
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // TIMING
  // ---------------------------------------------------

  unsigned long currentTime =
      micros();


  float dt =
      (currentTime - previousTime)
      / 1000000.0;


  previousTime =
      currentTime;


  if (dt <= 0.0 || dt > 0.1)
  {
    return;
  }


  // ---------------------------------------------------
  // READ MPU6050
  // ---------------------------------------------------

  float gx;
  float ax;
  float ay;
  float az;


  if (!readMPU(
          gx,
          ax,
          ay,
          az))
  {
    Serial.println(
        "MPU6050 read error!"
    );


    driveMotors(0);

    resetPID();

    resetVelocityPI();

    delay(100);

    return;
  }


  // ---------------------------------------------------
  // STORE SENSOR VALUES
  // ---------------------------------------------------

  gyroX =
      gx;

  accelX =
      ax;

  accelY =
      ay;

  accelZ =
      az;


  // ---------------------------------------------------
  // APPLY GYRO CALIBRATION
  // ---------------------------------------------------

  float calibratedGyroX =
      gyroX - gyroXOffset;


  // ---------------------------------------------------
  // ACCELEROMETER ANGLE
  // ---------------------------------------------------

  accelAngle =
      calculateAccelAngle(
          accelY,
          accelZ
      );


  // ---------------------------------------------------
  // GYROSCOPE ANGLE
  // ---------------------------------------------------

  gyroAngle +=
      calibratedGyroX * dt;


  // ---------------------------------------------------
  // COMPLEMENTARY FILTER
  // ---------------------------------------------------

  float alpha =
      TAU / (TAU + dt);


  currentAngle =
      alpha *
          (
              currentAngle
              +
              calibratedGyroX * dt
          )
      +
      (1.0 - alpha)
          * accelAngle;


  // ---------------------------------------------------
  // UPDATE WHEEL VELOCITY
  // ---------------------------------------------------

  updateWheelVelocity();


  // ---------------------------------------------------
  // FALL DETECTION
  // ---------------------------------------------------
  //
  // Only enter the fallen state if the robot has crossed
  // the Stage 8 fall threshold.
  //

  if (!robotFallen)
  {
    if (detectFall())
    {
      enterFallenState();
    }
  }


  // ---------------------------------------------------
  // FALL RECOVERY
  // ---------------------------------------------------

  checkFallRecovery();


  // ---------------------------------------------------
  // UPDATE VELOCITY PI
  // ---------------------------------------------------

  updateVelocityPI(dt);


  // ---------------------------------------------------
  // ANGLE PID
  // ---------------------------------------------------

  float motorPower =
      calculateAnglePID(dt);


  // ---------------------------------------------------
  // UPDATE PID STATE
  // ---------------------------------------------------

  previousAngle =
      currentAngle;

  previousError =
      error;


  // ---------------------------------------------------
  // MOTOR COMMAND
  // ---------------------------------------------------
  //
  // The fallen-state check is repeated here so that
  // entering the fallen state always results in zero
  // motor output.
  //

  if (robotFallen)
  {
    driveMotors(0);
  }
  else
  {
    driveMotors(
        motorPower
    );
  }


  // ---------------------------------------------------
  // SERIAL MONITORING
  // ---------------------------------------------------

  float measuredVelocity =
      calculateAverageVelocity();


  Serial.print("Angle: ");
  Serial.print(currentAngle, 2);

  Serial.print(" ° | Target Angle: ");
  Serial.print(targetAngle, 2);

  Serial.print(" ° | Vel Target: ");
  Serial.print(targetVelocity, 2);

  Serial.print(" | Vel: ");
  Serial.print(measuredVelocity, 2);

  Serial.print(" cnt/s | Vel Error: ");
  Serial.print(velocityError, 2);

  Serial.print(" | Vel I: ");
  Serial.print(velocityIntegral, 2);

  Serial.print(" | Vel Loop: ");
  Serial.print(
      velocityLoopEnabled
      ? "ON"
      : "OFF"
  );

  Serial.print(" | Fall State: ");
  Serial.print(
      robotFallen
      ? "FALLEN"
      : "SAFE"
  );

  Serial.print(" | Angle Error: ");
  Serial.print(error, 2);

  Serial.print(" | Power: ");
  Serial.print(motorPower, 2);

  Serial.print(" | L Count: ");
  Serial.print(leftEncoderCount);

  Serial.print(" | R Count: ");
  Serial.print(rightEncoderCount);

  Serial.print(" | L Vel: ");
  Serial.print(leftVelocity, 2);

  Serial.print(" | R Vel: ");
  Serial.print(rightVelocity, 2);

  Serial.print(" | dt: ");
  Serial.print(dt * 1000.0, 2);

  Serial.println(" ms");


  // ---------------------------------------------------
  // SMALL LOOP DELAY
  // ---------------------------------------------------

  delay(5);
}