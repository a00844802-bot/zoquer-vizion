#include "PID.h"
#include "Arduino.h"
#include <cmath>

PID::PID(double kp, double ki, double kd, double max_output) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_output_ = max_output;
    max_integral_ = 100.0;
    last_error_ = 0;    
    sum_error_ = 0;
    last_time_ = millis();
    last_output_ = 0;
}

double PID::clamp(double x, double lo, double hi){ //Limitar Hi y Lo
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double PID::Calculate(double error)
{
    const double I_DEADBAND = 1.5; // Zona muerta para la integral

    unsigned long current_time = millis();

    if (current_time - last_time_ >= 20) // Mínimo dt de 20 ms
    {
        double delta_time = (current_time - last_time_) / 1000.0;

        if (delta_time <= 0 || delta_time > 1.0) {
            last_time_ = current_time;
            last_error_ = error;
            return last_output_; //Regresa el last ourput si no hay un dt válido
        }

        //Término proporcional
        double proportional = kp_ * error;

        //Término integral
        double integral = 0;

        if (fabs(error) > I_DEADBAND) { //Valor absoluto y comprobar si está fuera de la deadband
            
            double integral_increment = ((error + last_error_) / 2.0) * delta_time;//Posible incremento)Trapezoide
            double new_sum = sum_error_ + integral_increment; //Posible nueva suma

            double test_output = proportional + (ki_ * new_sum);//Output de prueba

            bool would_saturate = (fabs(test_output) > max_output_);//Comprobar si la saluda estaría saturada
            bool error_reducing_saturation =
                ((test_output > 0 && error < 0) || (test_output < 0 && error > 0)); //Comprobar si el error reduciría la saturación

            if (!would_saturate || error_reducing_saturation) { //Si no hay saturación o el error reduciría la saturación
                sum_error_ = new_sum;
                sum_error_ = clamp(sum_error_, -max_integral_, max_integral_);
            }
        }

        integral = ki_ * sum_error_;

        //Término derivativo
        double delta_error = (error - last_error_) / delta_time;
        double derivative = kd_ * delta_error;

        // Output final
        double output = proportional + integral + derivative;
        output = clamp(output, -max_output_, max_output_);

        // Actualizar variables
        last_error_ = error;
        last_time_ = current_time;

        return output;
    }
    return 0;
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
