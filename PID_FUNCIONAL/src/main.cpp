#include "motores.h"
#include "constantes.h"
#include "BNO.h"
#include "PID.h"
//#include "Photo.h"

float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
unsigned long previous_millis = 0;
const long interval = 20;

int setpoint = 0;
int translation_angle = 0;
int adjust_angle = 0;

BNO055 my_bno;
//Photo photo;

PID pid(2, 0.001, 0.1, 300);

Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2
);

void setup() {
    Serial.begin(115200);
    motors.InitializeMotors();
    my_bno.InitializeBNO();
    //photo.InitializeADS();
}

void loop() {

    my_bno.GetBNOData();

    double yaw = my_bno.GetYaw();
    double error = my_bno.GetError();
    double pid_out = -pid.Calculate(error);

    // DEBUG PID + BNO
    Serial.print("Yaw:");
    Serial.print(yaw);
    Serial.print(",");

    Serial.print("Error:");
    Serial.print(error);
    Serial.print(",");

    Serial.print("PID:");
    Serial.println(pid_out);

    motors.MoveMotorsImu(0, 0, pid_out);

}
