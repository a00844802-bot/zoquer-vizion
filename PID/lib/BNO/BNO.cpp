#include "Arduino.h"
#include "Bno.h"
#include "cmath"

// https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/

BNO055::BNO055()
{
    yaw_ = 0;
    target_angle_ = 0;
    difference_angle_ = 0;
}

// Start retrieving data from the BNO055 sensor, disable the external crystal in case errors occur
void BNO055::InitializeBNO()
{

    Serial.println("Initializing BNO055...");
    Serial.println("BNO I2C Address: 0x28");
    Serial.println("Teensy 4.1 SDA: pin 17 (1), SCL: pin 16 (0)");
    
    if (!bno_.begin(OPERATION_MODE_IMUPLUS))
    {
        Serial.println("ERROR: BNO055 not detected at address 0x28!");
        Serial.println("Check:");
        Serial.println("  - I2C wiring (SDA pin 17, SCL pin 16)");
        Serial.println("  - Pull-up resistors on SDA/SCL");
        Serial.println("  - BNO055 address (should be 0x28)");
        while (1);
    }
    delay(1000);
    bno_.setExtCrystalUse(true);
    Serial.println("BNO055 initialized successfully!");
}

// Retrieve the yaw angle (x) from the BNO055 sensor using quaternion data from the euler vector
void BNO055::GetBNOData()
{
    imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw_ = euler.x();
    if (yaw_ > 180)
    {
        yaw_ = -1 * (360 - yaw_);
    }

}

double BNO055::GetYaw()
{
    return yaw_;
}

double BNO055::GetRoll()
{
    imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
    roll_ = euler.z();
    return roll_;
}

double BNO055::GetPitch()
{
    imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
    pitch_ = euler.y();
    return pitch_;
}
double BNO055::analize_error(double setpoint, double current_yaw) {
    double error = setpoint - current_yaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return -error; // Checar si aplica el valor negativo aqui
  }

  


void BNO055::SetTarget(double target) {
    target_angle_ = target;

    // Normalizar de uña
    if (target_angle_ > 180) target_angle_ -= 360;
    if (target_angle_ < -180) target_angle_ += 360;

}

double BNO055::GetError() {
    double error = target_angle_ - yaw_;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    return error;
}