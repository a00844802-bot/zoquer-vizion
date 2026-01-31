#include "motores.h"
#include "constantes.h"
#include "Bno.h"
#include "PID.h"
<<<<<<< HEAD

float setpoint = 0;
double current_yaw = 0;

BNO055 bno;
PID pid(2, 0.001, 0.1, 300);
=======
//#include "PhotoMux.h"
#include <cmath>
//Incluir la libreria del servo para el Kicker

float bno_angle = 0;
unsigned long start_millis;
unsigned long current_millis;
unsigned long previous_millis = 0;
float setpoint = 0;
float translation_angle = 0;
float adjust_angle = 0;
float angle_degrees = 0;
float ponderated_ball = 0;
float ponderated_goal = 0;
float ball_angle = 0;
float goal_angle = 0;
float ball_distance = 0;
float last_distance = 0;
float last_angle = 0;
float distance = 0;
float goal_distance = 0;
float distance_pixels = 0;
float differential_ball = 0;
float differential_goal = 0;
bool open_ball_seen = false;
bool goal_seen = false;
bool pixy_seen = false;
const int BUFFER_SIZE = 50;
char buffer[BUFFER_SIZE];
/*const int servo_min = 1000;
const int servo_mid = 1500;
const int servo_max = 2000;*/
int time_shoot = 2000;
uint8_t front[2] = {A8, A9};
uint8_t right[4] = {A3, A12, A13, A14};
uint8_t left[4] = {A6, A15, A16, A17};
uint8_t back[4] = {A0, A1, A2, A7};

const float DRIBBLER_CAPTURE_DISTANCE = 40.0;
const float DRIBBLER_CAPTURE_ANGLE = 15.0;
const float BALL_CAPTURED_DISTANCE = 25.0;

enum DribblerState {
    DRIBBLER_OFF,
    DRIBBLER_APPROACHING,
    DRIBBLER_CAPTURING,
    DRIBBLER_CAPTURED
};

DribblerState dribbler_state = DRIBBLER_OFF;

BNO055 bno;
Servo dribbler;
//Photo photo;

PID pid(4, 0.01, 0.6, 500); //0.6, 0.01, 0.6, 200
>>>>>>> 2c90f6da0140f911420e69b269a672a848f4b0e5

Motors motors(
    MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
    MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
    MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
<<<<<<< HEAD
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
=======
    MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2);
    
//PhotoSensors sensors(front, left, right, back);

void updateDribblerState() {
    if (!open_ball_seen) {
        dribbler_state = DRIBBLER_OFF;
        return;
    }
    
    if (ball_distance <= BALL_CAPTURED_DISTANCE && abs(ball_angle) < 5.0) {
        dribbler_state = DRIBBLER_CAPTURED;
    }
    else if (ball_distance <= DRIBBLER_CAPTURE_DISTANCE && 
             abs(ball_angle) <= DRIBBLER_CAPTURE_ANGLE) {
        dribbler_state = DRIBBLER_CAPTURING;
    }
    else {
        dribbler_state = DRIBBLER_APPROACHING;
    }
}

void controlDribbler() {
    switch (dribbler_state) {
        case DRIBBLER_OFF:
            dribbler.writeMicroseconds(servo_min);
            break;
            
        case DRIBBLER_APPROACHING:
            dribbler.writeMicroseconds(servo_min + 100);
            break;
            
        case DRIBBLER_CAPTURING:
            dribbler.writeMicroseconds(servo_mid);
            break;
            
        case DRIBBLER_CAPTURED:
            dribbler.writeMicroseconds(servo_max);
            break;
    }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  dribbler.attach(6);
  dribbler.writeMicroseconds(servo_min);
  motors.InitializeMotors();
  bno.InitializeBNO(); 
/* 
  sensors.setThreshold(FRONT, 600);
  sensors.setThreshold(LEFT,  580);
  sensors.setThreshold(RIGHT, 590);
  sensors.setThreshold(BACK,  610);
*/
  delay(1000);
}

void loop() {

  static unsigned long lastBnoUpdate = 0;
  if (millis() - lastBnoUpdate >= 5) {
    lastBnoUpdate = millis();
    bno.GetBNOData();
  }

  if (Serial1.available()) {
    int bytesRead = Serial1.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[bytesRead] = '\0';

    char* token = strtok(buffer, " ");
    if (token != nullptr) {
      ball_distance = atof(token);
      token = strtok(NULL, " ");
      if (token != nullptr) {
        ball_angle = atof(token);
      }
    }

    open_ball_seen = (ball_distance != 0 || ball_angle != 0);
  }

  updateDribblerState();

  if (open_ball_seen) {
    bno.SetTarget(bno.GetYaw() + ball_angle);
  } else {
    bno.SetTarget(bno.GetYaw());
  }
  
  double error = bno.GetError();
  double speed_w = pid.Calculate(0, error);
  double speed_ball = 100;

  if (open_ball_seen) {
    if (dribbler_state == DRIBBLER_CAPTURED) {
      speed_ball = 70;
    }
    motors.MoveMotorsImu(-ball_angle, speed_ball, speed_w);
  } else {
    motors.MoveMotorsImu(0, 0, speed_w);
    if (abs(speed_w) < 2) {
      motors.SetAllSpeeds(80);
      motors.MoveBackward();
    }
  }

  controlDribbler();
>>>>>>> 2c90f6da0140f911420e69b269a672a848f4b0e5
}