#ifndef Bno_h
#define Bno_h

#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"

class BNO055
{
private:
    double yaw_;
    double roll_;
    double pitch_;
    double target_angle_;
    double difference_angle_;
    Adafruit_BNO055 bno_{55, 0x28, &Wire};

public:
    BNO055();
    void InitializeBNO();
    void GetBNOData();
    double GetYaw();
    double GetRoll();
    double GetPitch();
    void SetYaw(double yaw);

    //Intentar despues de que el BNO jale si que si
    void SetTarget(double target);
    double GetError();
};

#endif