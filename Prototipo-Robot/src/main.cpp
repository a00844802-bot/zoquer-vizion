#include <Arduino.h>
#include <cmath>
#include "motores.h"
#include "BNO.h"
#include "PID.h"
#include "constantes.h"

BNO055 bno;
PID pid(2.5, 0.01, 0.125, 120.0);

const uint8_t Speed = 120; //La velocidad base del robot
float setpoint = 0.0f; //Ángulo objetivo
double current_yaw = 0.0; //Angulo actual
double last_speed_w = 0.0;//No se utiliza xd

// Datos cámara ESPEJO
float ball_distance = 0, ball_angle = 0;
float goal_distance = 0, goal_angle = 0;
float own_distance = 0, own_angle = 0;
bool open_ball_seen = false, goal_seen = false, own_seen = false;

Motors motorss(
  MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
  MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
  MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
  MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2
);

// Buffers serial
String serial1_line;
String serial2_line;

//Procesar vision ESPEJO
void processSerial2(const String& line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);
  if (parsed == 6) {
    ball_distance = dist;
    ball_angle = ang;
    goal_distance = g_dist;
    goal_angle = g_ang;
    own_distance = o_dist;
    own_angle = o_ang;
    open_ball_seen = (fabsf(dist) > 1e-3f);
    goal_seen = (fabsf(g_dist) > 1e-3f);
    own_seen = (fabsf(o_dist) > 1e-3f);
  }
}

//Leer las lineas seriales
void readSerialLines() {
  /*
  //cámara frontal por Serial1 - COMENTADO
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      //process_serialF(serial1_line);
      serial1_line = "";
    } else {
      serial1_line += c;
      if (serial1_line.length() > 120) serial1_line = "";
    }
  }
  */
  
  //Camara del espejo por Serial2
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      processSerial2(serial2_line);
      serial2_line = "";
    } else {
      serial2_line += c;
      if (serial2_line.length() > 80) serial2_line = "";
    }
  }
}

void setup() {
  Serial.begin(115200);
  //Serial1.begin(115200);
  Serial2.begin(115200);
  
  bno.InitializeBNO();
  motorss.InitializeMotors();
  
  delay(300);
  bno.GetBNOData();
  setpoint = (float)bno.GetYaw();
  bno.SetTarget(setpoint);
  delay(300);
}

void loop() {
  // Leer los datos de las camaras
  readSerialLines();
  
  // Obtener Yaw del BNO
  bno.GetBNOData();
  current_yaw = bno.GetYaw();
  
  // Obtener el error
  double error = bno.GetError();
  
  // Calcular la corrección del PID
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, -180, 180);
  
  // ===== GRAFICACIÓN PARA EL PLOTTER =====
  // Variables del PID
  Serial.print(">Error:");
  Serial.println(error);
  
  Serial.print(">Output:");
  Serial.println(speed_w);
  
  Serial.print(">P:");
  Serial.println(pid.GetProportional());
  
  Serial.print(">I:");
  Serial.println(pid.GetIntegral());
  
  Serial.print(">D:");
  Serial.println(pid.GetDerivative());
  
  // Variables del BNO
  Serial.print(">Yaw:");
  Serial.println(current_yaw);
  
  Serial.print(">Target:");
  Serial.println(bno.GetTarget());
  
  Serial.print(">Setpoint:");
  Serial.println(0.0);  // Línea de referencia
  
  // Estado de la pelota (opcional)
  Serial.print(">BallSeen:");
  Serial.println(open_ball_seen ? 1 : 0);
  
  
  // Si detecta la pelota avanza(Con PID)
  if (open_ball_seen) {
    float ang = -ball_angle;
    if (fabsf(ang) < 7.0f) ang = 0.0f;
    ang = constrain(ang, -90.0f, 90.0f);
    motorss.MoveMotorsImu((int)ang, Speed, speed_w);
  } else {
    // Sino el robot solo se acomoda
    motorss.MoveMotorsImu(0, 0, speed_w);
  }
  
  delay(20);
}