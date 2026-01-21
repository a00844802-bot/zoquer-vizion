#include <Arduino.h>
#include <cmath>
#include "motores.h"
#include "BNO.h"
#include "PID.h"
#include "constantes.h"

//Incluir variables globales
BNO055 bno;


Motors motorss(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);

//Declaración de Funciones
void check_4Line(void);

void process_serialD(void);//Procesar los datos seriales

void process_serialD2(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  bno.InitializeBNO();
  motorss.InitializeMotors();
}
void loop() {
  

}
