#include "PID.h"
#include <cmath>

PID::PID(double kp, double ki, double kd, double max_output)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_output_ = max_output;
    max_integral_ = 100.0;
    last_error_ = 0.0;
    sum_error_  = 0.0;
    last_time_  = millis();
    has_last_   = false;
    last_output_ = 0.0;
    
    // Inicializar componentes
    last_P_ = 0.0;
    last_I_ = 0.0;
    last_D_ = 0.0;
}

double PID::clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double PID::Calculate(double error)
{
    unsigned long now = millis();
    double dt = (now - last_time_) / 1000.0;
    
    if (dt <= 0.0 || dt > 1.0) {
        last_time_ = now;
        return last_output_;
    }
    
    // Proporcional
    double P = kp_ * error;
    last_P_ = P;  // Guardar para telemetría
    
    // Integral
    sum_error_ += error * dt;
    sum_error_ = clamp(sum_error_, -max_integral_, max_integral_);
    double I = ki_ * sum_error_;
    last_I_ = I;  // Guardar para telemetría
    
    // Derivativa
    double D = 0.0;
    if (has_last_) {
        D = kd_ * ((error - last_error_) / dt);
    }
    last_D_ = D;  // Guardar para telemetría
    
    double output = P + I + D;
    output = clamp(output, -max_output_, max_output_);
    
    last_error_ = error;
    last_time_ = now;
    has_last_ = true;
    last_output_ = output;
    
    return output;
}

void PID::Reset()
{
    sum_error_ = 0.0;
    last_error_ = 0.0;
    has_last_ = false;
    last_time_ = millis();
    last_P_ = 0.0;
    last_I_ = 0.0;
    last_D_ = 0.0;
}

void PID::SetTunings(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::SetMaxOutput(double max_output)
{
    max_output_ = max_output;
}

void PID::SetMaxIntegral(double max_integral)
{
    max_integral_ = max_integral;
}

double PID::GetIntegral()
{
    return sum_error_;
}

// NUEVAS FUNCIONES PARA TELEMETRÍA
double PID::GetProportional()
{
    return last_P_;
}

double PID::GetDerivative()
{
    return last_D_;
}

double PID::GetLastOutput()
{
    return last_output_;
}

double PID::GetLastError()
{
    return last_error_;
}

// Función para graficar en el plotter de PlatformIO
void PID::PlotPID(double error)
{
    // El formato correcto para el plotter es: >variable:valor
    Serial.print(">Error:");
    Serial.println(error);
    
    Serial.print(">P:");
    Serial.println(last_P_);
    
    Serial.print(">I:");
    Serial.println(last_I_);
    
    Serial.print(">D:");
    Serial.println(last_D_);
    
    Serial.print(">Output:");
    Serial.println(last_output_);
    
    Serial.print(">Setpoint:");
    Serial.println(0.0);  // Para ver la referencia en 0
}