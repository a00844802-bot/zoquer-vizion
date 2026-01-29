#include "PID.h"
#include "Arduino.h"
#include <cmath>

PID::PID(double kp, double ki, double kd, double max_output)
    : kp_(kp), 
      ki_(ki), 
      kd_(kd), 
      max_output_(max_output),
      max_integral_(100.0),  // Límite para prevenir windup excesivo
      last_error_(0), 
      sum_error_(0), 
      last_time_(millis()) {}

double PID::clamp(double x, double lo, double hi){
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double PID::Calculate(double error)
{
    const double I_DEADBAND = 1.5; // grados - no integrar si error es muy pequeño
    
    unsigned long current_time = millis();
    double delta_time = (current_time - last_time_) / 1000.0; // Convertir a segundos
    
    // Evitar divisiones por cero o tiempos negativos
    if (delta_time <= 0 || delta_time > 1.0) {
        last_time_ = current_time;
        last_error_ = error;
        return 0;
    }
    
    // ===== TÉRMINO PROPORCIONAL =====
    double proportional = kp_ * error;
    
    // ===== TÉRMINO INTEGRAL (con anti-windup preventivo) =====
    double integral = 0;
    
    // Solo integrar si el error está fuera de la deadband
    if (fabs(error) > I_DEADBAND) {
        // Calcular incremento de integral (método trapezoidal)
        double integral_increment = ((error + last_error_) / 2.0) * delta_time;
        double new_sum = sum_error_ + integral_increment;
        
        // ANTI-WINDUP PREVENTIVO: Verificar si la integral causaría saturación
        double test_output = proportional + (ki_ * new_sum);
        
        // Solo integrar si NO estamos saturados O si el error está reduciendo la saturación
        bool would_saturate = (fabs(test_output) > max_output_);
        bool error_reducing_saturation = ((test_output > 0 && error < 0) || 
                                         (test_output < 0 && error > 0));
        
        if (!would_saturate || error_reducing_saturation) {
            sum_error_ = new_sum;
            // Limitar la integral acumulada
            sum_error_ = clamp(sum_error_, -max_integral_, max_integral_);
        }
        // Si estamos saturados y el error empuja en la misma dirección, NO integramos
    }
    
    integral = ki_ * sum_error_;
    
    // ===== TÉRMINO DERIVATIVO =====
    double delta_error = (error - last_error_) / delta_time;
    double derivative = kd_ * delta_error;
    
    // ===== SALIDA TOTAL =====
    double output = proportional + integral + derivative;
    output = clamp(output, -max_output_, max_output_);
    
    // Actualizar estados
    last_error_ = error;
    last_time_ = current_time;
    
    return output;
}

void PID::Reset()
{
    sum_error_ = 0;
    last_error_ = 0;
    last_time_ = millis();
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
