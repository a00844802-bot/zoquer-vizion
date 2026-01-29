#include "motores.h"
#include <Arduino.h>
#include "constantes.h"
#include <cmath>

Motors::Motors(uint8_t pwm_pin1, uint8_t in1_1, uint8_t in2_1,
               uint8_t pwm_pin2, uint8_t in1_2, uint8_t in2_2,
               uint8_t pwm_pin3, uint8_t in1_3, uint8_t in2_3,
               uint8_t pwm_pin4, uint8_t in1_4, uint8_t in2_4)
: motor1(pwm_pin1, in1_1, in2_1),
  motor2(pwm_pin2, in1_2, in2_2),
  motor3(pwm_pin3, in1_3, in2_3),
  motor4(pwm_pin4, in1_4, in2_4)
{}

void Motors::InitializeMotors()
{
    Serial.println("Inicializando motores...");
    motor1.InitializeMotor();
    motor2.InitializeMotor();
    motor3.InitializeMotor();
    motor4.InitializeMotor();
}

void Motors::SetAllSpeeds(uint8_t speed)
{
    motor1.SetSpeed(speed);
    motor2.SetSpeed(speed);
    motor3.SetSpeed(speed);
    motor4.SetSpeed(speed);
}

void Motors::GetAllSpeeds()
{
    // OJO: ahora ya no existe GetSpeed() (porque era confuso).
    // Si quieres imprimir algo útil, imprime el pin PWM:
    Serial.print("Motor 1 PWM pin: ");
    Serial.println(motor1.GetPwmPin());
    Serial.print("Motor 2 PWM pin: ");
    Serial.println(motor2.GetPwmPin());
    Serial.print("Motor 3 PWM pin: ");
    Serial.println(motor3.GetPwmPin());
    Serial.print("Motor 4 PWM pin: ");
    Serial.println(motor4.GetPwmPin());
}

void Motors::StopMotors()
{
    motor1.StopMotor();
    motor2.StopMotor();
    motor3.StopMotor();
    motor4.StopMotor();
}

void Motors::MoveForward()
{
    motor1.MoveForward();
    motor2.MoveForward();
    motor3.MoveBackward();
    motor4.MoveBackward();
}

void Motors::MoveRight()
{
    motor1.MoveForward();
    motor2.MoveBackward();
    motor3.MoveBackward();
    motor4.MoveBackward();
}

void Motors::MoveLeft()
{
    motor1.MoveBackward();
    motor2.MoveForward();
    motor3.MoveForward();
    motor4.MoveForward();
}

void Motors::MoveBackward()
{
    motor1.MoveBackward();
    motor2.MoveBackward();
    motor3.MoveForward();
    motor4.MoveForward();
}

void Motors::MoveMotor1() { motor1.MoveForward(); }
void Motors::MoveMotor2() { motor2.MoveForward(); }
void Motors::MoveMotor3() { motor3.MoveForward(); }
void Motors::MoveMotor4() { motor4.MoveForward(); }

void Motors::MoveMotors(int degree, uint8_t speed)
{
    float m1 = cos(((45  + degree) * PI / 180));
    float m2 = cos(((135 + degree) * PI / 180));
    float m3 = cos(((225 + degree) * PI / 180));
    float m4 = cos(((315 + degree) * PI / 180));

    int speedA = abs(int(m1 * speed));
    int speedB = abs(int(m2 * speed));
    int speedC = abs(int(m3 * speed));
    int speedD = abs(int(m4 * speed));

    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);
    speedC = constrain(speedC, 0, 255);
    speedD = constrain(speedD, 0, 255);

    // Puedes usar analogWrite con el pin:
    analogWrite(motor1.GetPwmPin(), speedA);
    analogWrite(motor2.GetPwmPin(), speedB);
    analogWrite(motor3.GetPwmPin(), speedC);
    analogWrite(motor4.GetPwmPin(), speedD);

    // Dirección según signo
    (m1 >= 0) ? motor1.MoveForward() : motor1.MoveBackward();
    (m2 >= 0) ? motor2.MoveForward() : motor2.MoveBackward();
    (m3 >= 0) ? motor3.MoveForward() : motor3.MoveBackward();
    (m4 >= 0) ? motor4.MoveForward() : motor4.MoveBackward();
}

void Motors::MoveMotorsImu(double degree, uint8_t speed, double w)
{

    float t1 = cos((315 + degree) * PI / 180) * speed;
    float t2 = cos(( 45 + degree) * PI / 180) * speed;
    float t3 = cos((135 + degree) * PI / 180) * speed;
    float t4 = cos((225 + degree) * PI / 180) * speed;


    float m1 = t1 + w;   // M1 +
    float m2 = t2 + w;   // M2 -
    float m3 = t3 + w;   // M3 +
    float m4 = t4 + w;   // M4 -


    int speedA = constrain(abs((int)m1), 0, 255);
    int speedB = constrain(abs((int)m2), 0, 255);
    int speedC = constrain(abs((int)m3), 0, 255);
    int speedD = constrain(abs((int)m4), 0, 255);

    motor1.SetSpeed(speedA);
    motor2.SetSpeed(speedB);
    motor3.SetSpeed(speedC);
    motor4.SetSpeed(speedD);

    (m1 >= 0) ? motor1.MoveForward() : motor1.MoveBackward();
    (m2 >= 0) ? motor2.MoveForward() : motor2.MoveBackward();
    (m3 >= 0) ? motor3.MoveForward() : motor3.MoveBackward();
    (m4 >= 0) ? motor4.MoveForward() : motor4.MoveBackward();
}
