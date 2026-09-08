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
//
// volatile is required because these variables are
// modified inside interrupt service routines.
//

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;


// =====================================================
// ENCODER DIRECTION
// =====================================================
//
// Change these later if the physical encoder direction
// is opposite to the desired sign.
//

const int ENC_LEFT_SIGN = 1;
const int ENC_RIGHT_SIGN = 1;


// =====================================================
// ENCODER VELOCITY
// =====================================================

float leftVelocity = 0.0;
float rightVelocity = 0.0;

long previousLeftCount = 0;
long previousRightCount = 0;


// Velocity measurement interval

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
// ANGLE PID CONTROLLER
// =====================================================

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
//
// Velocity here is measured in:
//
//     encoder counts / second
//
// We intentionally use encoder counts/s at Stage 5.
// Physical wheel RPM can be added later once the exact
// encoder counts-per-revolution specification is verified.
//

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


  // Same LEDC API used in Stage 4.

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
// RESET PID
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
  // ERROR

  error =
      targetAngle - currentAngle;


  // INTEGRAL

  errorSum +=
      error * dt;


  errorSum =
      constrain(
          errorSum,
          -ERROR_SUM_LIMIT,
          ERROR_SUM_LIMIT
      );


  // ANGULAR RATE

  float angleRate = 0.0;


  if (dt > 0.0)
  {
    angleRate =
        (currentAngle - previousAngle)
        / dt;
  }


  // PID TERMS

  float P =
      Kp * error;

  float I =
      Ki * errorSum;

  float D =
      -Kd * angleRate;


  // TOTAL OUTPUT

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
  // PID INITIALIZATION
  // ---------------------------------------------------

  resetPID();


  // ---------------------------------------------------
  // STAGE 5 MESSAGE
  // ---------------------------------------------------

  Serial.println("================================");
  Serial.println("    STAGE 5 - ENCODER FEEDBACK");
  Serial.println("================================");
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

  Serial.println();

  Serial.print("Kp: ");
  Serial.println(Kp);

  Serial.print("Ki: ");
  Serial.println(Ki);

  Serial.print("Kd: ");
  Serial.println(Kd);

  Serial.print("Target angle: ");
  Serial.print(targetAngle, 2);
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

  driveMotors(
      motorPower
  );


  // ---------------------------------------------------
  // UPDATE WHEEL VELOCITY
  // ---------------------------------------------------

  updateWheelVelocity();


  // ---------------------------------------------------
  // SERIAL MONITORING
  // ---------------------------------------------------

  Serial.print("Angle: ");
  Serial.print(currentAngle, 2);

  Serial.print(" ° | Error: ");
  Serial.print(error, 2);

  Serial.print(" | Power: ");
  Serial.print(motorPower, 2);

  Serial.print(" | L Count: ");
  Serial.print(leftEncoderCount);

  Serial.print(" | R Count: ");
  Serial.print(rightEncoderCount);

  Serial.print(" | L Vel: ");
  Serial.print(leftVelocity, 2);

  Serial.print(" cnt/s | R Vel: ");
  Serial.print(rightVelocity, 2);

  Serial.print(" cnt/s | dt: ");
  Serial.print(dt * 1000.0, 2);

  Serial.println(" ms");


  // Small loop delay

  delay(5);
}