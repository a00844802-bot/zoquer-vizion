#include "PhotoMux.h"
#include "constantes.h"
#include <Arduino.h>

// ════════════════════════════════════════
//  TUS SENSORES (Copia de tu código)
// ════════════════════════════════════════

PhotoMux::Sensor front[7] = {
  {1, 0}, 
  {1, 1}, 
  {1, 2}, 
  {1, 3}, 
  {1, 4}, 
  {1, 5}, 
  {1, 6}
};



PhotoMux sensors(selectPins, muxPins);

void setup() {
  Serial.begin(115200);
  delay(5000);
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   TEST PHOTOMUX - TUS SENSORES         ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  sensors.begin();
  analogReadResolution(12);
  
  sensors.configureSide(FRONT, front, 5);
  
  delay(1000);
}

void loop() {
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║       LECTURA COMPLETA - TODOS         ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  sensors.lecturaDebug(FRONT);
  

}