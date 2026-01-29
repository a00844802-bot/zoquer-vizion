#include "motor.h"
#include <Arduino.h>


Motor::Motor(uint8_t pwm_pin, uint8_t in1, uint8_t in2)
{
    pwm_pin_ = pwm_pin;
    in1_ = in1;
    in2_ = in2;
}

void Motor::InitializeMotor()
{
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
    pinMode(pwm_pin_, OUTPUT);
}

void Motor::SetSpeed(uint8_t speed)
{
    analogWrite(pwm_pin_, speed);
}

void Motor::MoveForward()
{
    digitalWrite(in1_, HIGH);
    digitalWrite(in2_, LOW);
}

void Motor::MoveBackward()
{
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, HIGH);
}

void Motor::StopMotor()
{
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
}

uint8_t Motor::GetPwmPin()
{
    return pwm_pin_;
}
