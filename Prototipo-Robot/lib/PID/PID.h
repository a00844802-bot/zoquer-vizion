#ifndef PID_h
#define PID_h

#include <Arduino.h>

class PID {
public:
    PID(double kp, double ki, double kd, double max_output);

    // Calcula salida del PID a partir del error (usa dt con millis()).
    double Calculate(double error);

    // Reinicia integral, last_error y tiempo.
    void Reset();

    // Cambiar ganancias
    void SetTunings(double kp, double ki, double kd);

    // Límites
    void SetMaxOutput(double max_output);
    void SetMaxIntegral(double max_integral);

    // Debug/telemetría
    double GetIntegral();

private:
    // Helper interno
    static double clamp(double x, double lo, double hi);

    // Ganancias del controlador
    double kp_;
    double ki_;
    double kd_;

    // Límites
    double max_output_;     // Límite de salida (|u| <= max_output_)
    double max_integral_;   // Límite de integral (|I| <= max_integral_)

    // Estado interno
    double last_error_;        // Error del ciclo anterior
    double sum_error_;         // Integral acumulada
    unsigned long last_time_;  // millis() del último cálculo
    bool has_last_;            // para manejar el primer ciclo sin dt/derivada basura
    double last_output_;       // Última salida calculada
};

#endif
