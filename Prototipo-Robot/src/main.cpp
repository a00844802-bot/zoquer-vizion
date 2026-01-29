#include <Arduino.h>
#include <cmath>
#include "motores.h"
#include "BNO.h"
#include "PID.h"
#include "constantes.h"

BNO055 bno;
PID pid(1, 0.0001, 0.1, 300);

const uint8_t Speed = 90;
float setpoint = 0.0f;
double current_yaw = 0.0;

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
//Leer las lineas seriales
void readSerialLines() {

  // AHORA: visión frontal por Serial1
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      process_serialF(serial1_line);   // 👈 antes era Serial2
      serial1_line = "";
    } else {
      serial1_line += c;
      if (serial1_line.length() > 120) serial1_line = "";
    }
  }

  // AHORA: la otra cámara (o nada) por Serial2
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

void DebugSerial(){
  Serial.print("Ball_distance: ");
  Serial.print(ball_distance);
  Serial.print(",");
  Serial.print("Ball_angle: ");
  Serial.print(ball_angle);
  Serial.print("Current_yaw:");
  Serial.println(current_yaw);
}

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
  // 1) actualizar visión
  readSerialLines();
  DebugSerial();

  // 2) IMU
  bno.GetBNOData();
  current_yaw = bno.GetYaw();

  // 3) PID: mantener rumbo
  double error = bno.GetError();
  double speed_w = pid.Calculate(error);
  speed_w = constrain(speed_w, -180, 180);

  // 4) Mover hacia pelota manteniendo yaw
  if (open_ball_seen) {
    float ang = -ball_angle;
    if (fabsf(ang) < 7.0f) ang = 0.0f;
    ang = constrain(ang, -90.0f, 90.0f);

    motorss.MoveMotorsImu((int)ang, Speed, 0);
  } else {
    motorss.MoveMotorsImu(0, 0 , speed_w);
  }

}
