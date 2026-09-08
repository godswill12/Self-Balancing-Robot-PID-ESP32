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
// SENSOR VALUES
// =====================================================

float gyroX = 0.0;
float gyroXOffset = 0.0;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float accelAngle = 0.0;


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

  int16_t rawAccelX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelZ = (Wire.read() << 8) | Wire.read();

  // Temperature
  Wire.read();
  Wire.read();

  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();

  // Remaining gyro axes
  Wire.read();
  Wire.read();
  Wire.read();
  Wire.read();

  ax = rawAccelX / 16384.0;
  ay = rawAccelY / 16384.0;
  az = rawAccelZ / 16384.0;

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
// MOTOR CONTROL
// =====================================================

void setupMotors()
{
  pinMode(STBY_PIN, OUTPUT);

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);

  // PlatformIO / older ESP32 Arduino Core API
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PWMA_PIN, 0);
  ledcAttachPin(PWMB_PIN, 1);

  ledcWrite(0, 0);
  ledcWrite(1, 0);
}


void driveMotorA(int speed)
{
  speed = constrain(speed, -255, 255);

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

  ledcWrite(0, abs(speed));
}


void driveMotorB(int speed)
{
  speed = constrain(speed, -255, 255);

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

  ledcWrite(1, abs(speed));
}


void driveMotors(float power)
{
  int pwm = (int)power;

  driveMotorA(pwm);
  driveMotorB(-pwm);
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

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  delay(100);

  // ---------------------------------------------------
  // MPU6050 CONFIGURATION
  // ---------------------------------------------------

  writeRegister(0x6B, 0x00);   // Wake up MPU6050
  delay(100);

  writeRegister(0x1C, 0x00);   // Accelerometer ±2g
  delay(100);

  writeRegister(0x1B, 0x00);   // Gyroscope ±250 °/s
  delay(100);

  writeRegister(0x1A, 0x01);   // DLPF configuration
  delay(100);

  // ---------------------------------------------------
  // GYRO CALIBRATION
  // ---------------------------------------------------

  calibrateGyro();

  // ---------------------------------------------------
  // INITIAL ACCELEROMETER ANGLE
  // ---------------------------------------------------

  float gx;
  float ax;
  float ay;
  float az;

  if (readMPU(gx, ax, ay, az))
  {
    accelAngle = calculateAccelAngle(ay, az);
  }

  // ---------------------------------------------------
  // MOTORS
  // ---------------------------------------------------

  setupMotors();

  driveMotors(0);

  Serial.println("================================");
  Serial.println("       STAGE 2 - SENSOR TEST");
  Serial.println("================================");
  Serial.println();

  Serial.println("Gyro calibration + accelerometer");
  Serial.println("tilt estimation active.");
  Serial.println();

  Serial.print("Initial accel angle: ");
  Serial.print(accelAngle, 2);
  Serial.println(" °");

  Serial.println();
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  float gx;
  float ax;
  float ay;
  float az;

  // ---------------------------------------------------
  // READ MPU6050
  // ---------------------------------------------------

  if (!readMPU(gx, ax, ay, az))
  {
    Serial.println("MPU6050 read error!");

    driveMotors(0);

    delay(100);

    return;
  }

  // ---------------------------------------------------
  // STORE SENSOR VALUES
  // ---------------------------------------------------

  gyroX = gx;

  accelX = ax;
  accelY = ay;
  accelZ = az;

  // ---------------------------------------------------
  // APPLY GYRO CALIBRATION
  // ---------------------------------------------------

  float calibratedGyroX = gyroX - gyroXOffset;

  // ---------------------------------------------------
  // CALCULATE ACCELEROMETER TILT ANGLE
  // ---------------------------------------------------

  accelAngle = calculateAccelAngle(accelY, accelZ);

  // ---------------------------------------------------
  // SERIAL OUTPUT
  // ---------------------------------------------------

  Serial.print("Raw Gyro X: ");
  Serial.print(gyroX, 2);

  Serial.print(" °/s | Calibrated Gyro X: ");
  Serial.print(calibratedGyroX, 2);

  Serial.print(" °/s | Accel Angle: ");
  Serial.print(accelAngle, 2);

  Serial.print(" ° | Accel: ");
  Serial.print(accelX, 2);
  Serial.print(", ");
  Serial.print(accelY, 2);
  Serial.print(", ");
  Serial.print(accelZ, 2);

  Serial.println();

  // Motors remain stopped in Stage 2
  driveMotors(0);

  delay(50);
}