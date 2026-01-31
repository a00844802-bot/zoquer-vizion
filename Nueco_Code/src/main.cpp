#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"

float setpoint = 0;
double current_yaw = 0;

BNO055 bno;
PID pid(2, 0.001, 0.1, 300);

Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2
);

const uint8_t FORWARD_SPEED = 90;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║   TEST PID - AVANCE EN LÍNEA RECTA     ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    motors.InitializeMotors();
    Serial.println("✓ Motores inicializados");
    
    bno.InitializeBNO();
    Serial.println("✓ BNO055 inicializado");
    
    delay(2000);
    
    bno.GetBNOData();
    setpoint = bno.GetYaw();
    
    Serial.println("\n════════════════════════════════════════");
    Serial.print("✓ Ángulo objetivo guardado: ");
    Serial.print(setpoint, 2);
    Serial.println("°");
    Serial.println("════════════════════════════════════════\n");
    Serial.println("El robot avanzará en línea recta");
    Serial.println("manteniendo siempre este ángulo.\n");
    Serial.println("Iniciando en 3 segundos...\n");
    
    delay(3000);
    
    Serial.println("¡INICIANDO!\n");
    Serial.println("   YAW    | SETPOINT |  ERROR  | SPEED_W | ESTADO");
    Serial.println("──────────────────────────────────────────────────");
}

void loop() {
    
    bno.GetBNOData();
    current_yaw = bno.GetYaw();
    
    double error = bno.analize_error(setpoint, current_yaw);
    double speed_w = pid.Calculate(setpoint, error);
    
    motors.MoveMotorsImu(0, FORWARD_SPEED, speed_w);
    
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        
        Serial.print("  ");
        if (current_yaw >= 0 && current_yaw < 100) Serial.print(" ");
        if (current_yaw >= 0 && current_yaw < 10) Serial.print(" ");
        Serial.print(current_yaw, 1);
        Serial.print("° ");
        
        Serial.print("|   ");
        if (setpoint >= 0 && setpoint < 100) Serial.print(" ");
        if (setpoint >= 0 && setpoint < 10) Serial.print(" ");
        Serial.print(setpoint, 1);
        Serial.print("° ");
        
        Serial.print("|  ");
        if (error >= 0) Serial.print(" ");
        if (abs(error) < 100) Serial.print(" ");
        if (abs(error) < 10) Serial.print(" ");
        Serial.print(error, 1);
        Serial.print("° ");
        
        Serial.print("|  ");
        if (speed_w >= 0) Serial.print(" ");
        if (abs(speed_w) < 100) Serial.print(" ");
        if (abs(speed_w) < 10) Serial.print(" ");
        Serial.print(speed_w, 1);
        Serial.print(" ");
        
        Serial.print("| ");
        if (abs(error) < 2.0) {
            Serial.println("✓ RECTO");
        } else if (error > 0) {
            Serial.println("↻ Corrigiendo derecha");
        } else {
            Serial.println("↺ Corrigiendo izquierda");
        }
    }
    
    delay(10);
}