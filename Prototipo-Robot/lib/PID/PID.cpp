#include "PID.h"
#include "Arduino.h"

PID::PID(double kp, double ki, double kd, double max_error)
    : kp_(kp), ki_(ki), kd_(kd), max_error_(max_error), last_error_(0), sum_error_(0), last_time_(millis()) {}

// Note that the sampling time for our PID controller is 20ms
double PID::Calculate(double error)
{
    unsigned long current_time = millis();
    if (current_time - last_time_ >= 20)
    {
        double delta_time = (current_time - last_time_)/1000.0; // Convert to seconds
        sum_error_ += ((error + last_error_) / 2.0) * delta_time;
        double delta_error = (error - last_error_) / delta_time;


        if (sum_error_ > max_error_)
        {
            sum_error_ = max_error_;
        }
        else if (sum_error_ < -max_error_)
        {
            sum_error_ = -max_error_;
        }
        double proportional = kp_ * error;
        double integral = ki_ * sum_error_;
        double derivative = kd_ * delta_error;
        double output = proportional + integral + derivative;
        last_error_ = error;
        last_time_ = current_time;
        return output;
    }  
    return 0;
}