#ifndef PID_h
#define PID_h
#include "Arduino.h"

class PID {
public:
    /**
     * Constructor del controlador PID
     * @param kp Ganancia proporcional
     * @param ki Ganancia integral
     * @param kd Ganancia derivativa
     * @param max_output Límite máximo de salida (±max_output)
     */
    PID(double kp, double ki, double kd, double max_output);
    
    /**
     * Calcula la salida del PID basado en el error actual
     * @param error Error actual (setpoint - medición)
     * @return Salida del controlador PID
     */
    double Calculate(double error);
    
    /**
     * Reinicia el estado interno del PID (integral y error previo)
     */
    void Reset();
    
    /**
     * Actualiza las ganancias del PID en tiempo de ejecución
     * @param kp Nueva ganancia proporcional
     * @param ki Nueva ganancia integral
     * @param kd Nueva ganancia derivativa
     */
    void SetTunings(double kp, double ki, double kd);
    
    /**
     * Establece el límite máximo de salida
     * @param max_output Nuevo límite máximo
     */
    void SetMaxOutput(double max_output);
    
    /**
     * Establece el límite máximo de la integral acumulada
     * @param max_integral Nuevo límite para anti-windup
     */
    void SetMaxIntegral(double max_integral);
    
    /**
     * Obtiene el valor actual de la integral acumulada (para debug)
     * @return Valor de sum_error_
     */
    double GetIntegral();
    
    /**
     * Función auxiliar para limitar valores
     */
    double clamp(double x, double lo, double hi);

private:
    // Ganancias del controlador
    double kp_;
    double ki_;
    double kd_;
    
    // Límites
    double max_output_;     // Límite de salida
    double max_integral_;   // Límite de integral para anti-windup
    
    // Estado interno
    double last_error_;     // Error del ciclo anterior
    double sum_error_;      // Integral acumulada
    unsigned long last_time_; // Timestamp del último cálculo
};

#endif