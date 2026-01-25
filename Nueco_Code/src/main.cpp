#include <Arduino.h>
#include "PhotoMux.h"
#include "motores.h"
#include "constantes.h"

// =======================
// CONFIG
// =======================

unsigned long lineDetectedTime = 0;
const unsigned long reverseTime = 300;
bool reversing = false;

// =======================
// MOTORES
// =======================

Motors motors(
  MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,
  MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2,
  MOTOR3_PWM, MOTOR3_IN1, MOTOR3_IN2,
  MOTOR4_PWM, MOTOR4_IN1, MOTOR4_IN2
);

// =======================
// PHOTOMUX
// =======================

PhotoMux::Sensor front[5] = {
  {1,0}, {1,1}, {1,2}, {1,3}, {1,4}
};

PhotoMux sensors(selectPins, muxPins);

// =======================
// SETUP
// =======================

void setup() {
  Serial.begin(115200);

  motors.InitializeMotors();

  sensors.begin();
  analogReadResolution(12);
  sensors.configureSide(FRONT, front, 5);
  sensors.setThreshold(FRONT, 2100);

  delay(500);
}

// =======================
// LOOP
// =======================

void loop() {

  // 🔹 DETECCIÓN LIMPIA DE LÍNEA
  bool lineDetected = sensors.isLineDetected(FRONT);

  // 🔹 IMPRIMIR SOLO ESTADO
  // 1 = LINEA
  // 0 = NO LINEA
  Serial.println(lineDetected ? 1 : 0);

  // =======================
  // CONTROL DE REVERSA
  // =======================

  if (lineDetected && !reversing) {
    reversing = true;
    lineDetectedTime = millis();
    motors.SetAllSpeeds(130);
    motors.MoveBackward();
  }

  if (reversing) {
    motors.MoveBackward();

    if (millis() - lineDetectedTime >= reverseTime) {
      reversing = false;
      motors.StopMotors();
    }
  }
}
