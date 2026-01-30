#include <Arduino.h>
#include <cmath>
#include "motores.h"
#include "BNO.h"
#include "PID.h"
#include "constantes.h"

BNO055 bno;
PID pid(2.5, 0.01, 0.125, 120.0); 

const uint8_t Speed = 120; //La velocidad dbase del robot
float setpoint = 0.0f; //Ángulo objetivo
double current_yaw = 0.0; //Angulo actual
double last_speed_w = 0.0;//No se utiliza xd

// Datos cámara frontal
float ball_distance = 0, ball_angle = 0; 
float goal_distance = 0, goal_angle = 0;
float own_distance  = 0, own_angle  = 0;

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

//Procesar vision frontal
void process_serialF(const String& line) {
  float dist, ang, g_dist, g_ang, o_dist, o_ang;
  int parsed = sscanf(line.c_str(), "%f %f %f %f %f %f",
                      &dist, &ang, &g_dist, &g_ang, &o_dist, &o_ang);

  if (parsed == 6) {
    ball_distance = dist;  ball_angle = ang;
    goal_distance = g_dist; goal_angle = g_ang;
    own_distance  = o_dist; own_angle  = o_ang;

    open_ball_seen = (fabsf(dist) > 1e-3f);
    goal_seen      = (fabsf(g_dist) > 1e-3f);
    own_seen       = (fabsf(o_dist) > 1e-3f);
  }
}


void processSerial1(const String& line) {
  (void)line;
}
//Leer las lineas serialees
void readSerialLines() {

  //cámara frontal por Serial1
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      process_serialF(serial1_line); 
      serial1_line = "";
    } else {
      serial1_line += c;
      if (serial1_line.length() > 120) serial1_line = "";
    }
  }

  //Camara del espejo por Serial2
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;

    if (c == '\n') {
      processSerial1(serial2_line);    // 👈 antes era Serial1
      serial2_line = "";
    } else {
      serial2_line += c;
      if (serial2_line.length() > 80) serial2_line = "";
    }
  }
}

//void DebugSerial(){
 // Serial.print("Ball_distance: ");
  //Serial.print(ball_distance);
  //Serial.print(",");
  //Serial.print("Ball_angle: ");
  //Serial.print(ball_angle);
  //Serial.print("Current_yaw:");
  //Serial.println(current_yaw);
//}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
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
  //Leer los datos de las camaras
  readSerialLines();
  //DebugSerial();

  //Obtener Yaw del BNO
  bno.GetBNOData();
  current_yaw = bno.GetYaw();
  //Serial.print("Yaw_angle: ");
  //Serial.println(current_yaw);

  //Obtener el error
  double error = bno.GetError();
  Serial.print("Error: ");
  Serial.println(error);
  //Calcular la corrección del PID
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, -180, 180);

  //Si detecta la pelota avanza(Con PID)
  if (open_ball_seen) {
    float ang = -ball_angle;
    if (fabsf(ang) < 7.0f) ang = 0.0f;
    ang = constrain(ang, -90.0f, 90.0f);

    motorss.MoveMotorsImu((int)ang, Speed, speed_w);
  } else {
    //Serial.print("Speed_w: ");
    //Serial.println(speed_w);

    //Sino el robot solo se acomoda
    motorss.MoveMotorsImu(0, 0, speed_w);
  }
  delay(20);
}
