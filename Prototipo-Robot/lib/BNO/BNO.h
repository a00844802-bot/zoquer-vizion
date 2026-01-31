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
    double raw_yaw_;
    double roll_;
    double pitch_;
    double target_angle_;
    double difference_angle_;
    Adafruit_BNO055 bno_{55, 0x28, &Wire};

public:
    BNO055();
    void InitializeBNO();
    double NormalizeAngle(double angle);
    void GetBNOData();
    double GetYaw();
    double GetRoll();
    double GetPitch();
    void SetYaw(double yaw);
    
    // Funciones de setpoint y error
    void SetTarget(double target);
    double GetError();
    
    // NUEVAS FUNCIONES PARA TELEMETRÍA
    double GetTarget();
    double GetRawYaw();
    void PlotAngles();  // Graficar ángulos del BNO
};

#endif