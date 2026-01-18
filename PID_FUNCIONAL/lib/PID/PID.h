#ifndef PID_h
#define PID_h
#include "Arduino.h"

class PID {
    public:
        PID(double kp, double ki, double kd, double max_error);
        double Calculate(double error);

    private:
        double kp_;
        double ki_;
        double kd_;
        double max_error_;
        double last_error_;
        double sum_error_;
        unsigned long current_time_;
        unsigned long last_time_;
};

#endif