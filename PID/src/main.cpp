
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

PID pid(1.5, 0 , 0, 100);

Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

void setup() {
  Serial.begin(115200);
  motors.InitializeMotors();
  my_bno.InitializeBNO();
  //photo.InitializeADS();
}

void loop() {
  my_bno.GetBNOData();
  double current_yaw = my_bno.GetYaw();
  double current_pitch = my_bno.GetPitch();
  double current_roll = my_bno.GetRoll();
  Serial.print(" Yaw: ");
  Serial.print(current_yaw);
  Serial.print(" Pitch: ");
  Serial.print(current_pitch);
  Serial.print(" Roll: ");
  Serial.println(current_roll);
  delay(100);

  double error = my_bno.analize_error(setpoint, current_yaw);
  double correction = pid.Calculate(0, error);
  double speed_w = correction;
  if (speed_w != 0){
  Serial.print("Angulo del BNO: ");
  Serial.println(current_yaw);
  Serial.print("Velocidad corregida: ");
  Serial.println(speed_w);
  motors.MoveMotorsImu(0, 0 , speed_w);
  } 
}
//Solo para el commit desde ubunto si que si gente andamos de vuelta el la play jajjaa

