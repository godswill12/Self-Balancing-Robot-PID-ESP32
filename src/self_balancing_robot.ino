#include <Wire.h>

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
// MPU6050 REGISTER WRITE
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

    // Skip temperature
    Wire.read();
    Wire.read();

    int16_t rawGyroX = (Wire.read() << 8) | Wire.read();

    // Skip remaining gyro axes
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


// =====================================================
// MOTOR B
// =====================================================

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


// =====================================================
// DRIVE BOTH MOTORS
// =====================================================

void driveMotors(int speed)
{
    driveMotorA(speed);

    // Right motor is physically mounted in the opposite
    // orientation, so its direction is inverted.
    driveMotorB(-speed);
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(500);

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    delay(100);

    // Wake MPU6050
    writeRegister(0x6B, 0x00);

    // Accelerometer ±2g
    writeRegister(0x1C, 0x00);

    // Gyroscope ±250 °/s
    writeRegister(0x1B, 0x00);

    // Configure digital low-pass filter
    writeRegister(0x1A, 0x01);

    setupMotors();

    Serial.println("================================");
    Serial.println("ESP32 SELF-BALANCING ROBOT");
    Serial.println("Hardware Baseline");
    Serial.println("================================");
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

    if (readMPU(gx, ax, ay, az))
    {
        Serial.print("Accel: ");
        Serial.print(ax, 2);
        Serial.print(" g, ");

        Serial.print(ay, 2);
        Serial.print(" g, ");

        Serial.print(az, 2);
        Serial.print(" g | Gyro X: ");
        Serial.print(gx, 2);
        Serial.println(" deg/s");
    }
    else
    {
        Serial.println("MPU6050 read error!");
    }

    // Simple motor test
    driveMotors(60);

    delay(1000);

    driveMotors(0);

    delay(1000);
}