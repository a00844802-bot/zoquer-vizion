#include <Arduino.h>

// ════════════════════════════════════════
//  CONFIGURA LOS PINES DE TUS 5 SENSORES
// ════════════════════════════════════════

const uint8_t SENSOR_PINS[5] = {
  A0,   // Sensor 1
  A1,   // Sensor 2
  A2,   // Sensor 3
  A3,   // Sensor 4
  A4    // Sensor 5
};

const char* SENSOR_NAMES[5] = {
  "FRONT",
  "LEFT",
  "RIGHT",
  "BACK",
  "CENTER"
};

// ════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   TEST 5 FOTORRESISTORES - PLACA       ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  analogReadResolution(12);  // 12 bits (0-4095)
  
  Serial.println("✓ Sistema inicializado\n");
  Serial.println("Sensores configurados:");
  for (int i = 0; i < 5; i++) {
    Serial.print("  ");
    Serial.print(i + 1);
    Serial.print(". ");
    Serial.print(SENSOR_NAMES[i]);
    Serial.print(" → Pin A");
    Serial.println(SENSOR_PINS[i] - A0);
  }
  
  Serial.println("\n════════════════════════════════════════\n");
  delay(1000);
}

void loop() {
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║          LECTURA DE SENSORES           ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  for (int i = 0; i < 5; i++) {
    int valor = analogRead(SENSOR_PINS[i]);
    
    Serial.print("┌─── ");
    Serial.print(SENSOR_NAMES[i]);
    Serial.print(" (A");
    Serial.print(SENSOR_PINS[i] - A0);
    Serial.println(") ───┐");
    
    Serial.print("│ Valor: ");
    if (valor < 1000) Serial.print(" ");
    if (valor < 100) Serial.print(" ");
    if (valor < 10) Serial.print(" ");
    Serial.print(valor);
    Serial.print(" │ ");
    
    int barras = map(valor, 0, 4095, 0, 20);
    for (int j = 0; j < barras; j++) {
      Serial.print("█");
    }
    
    if (valor > 3000) {
      Serial.print(" ⚠️ MUY ALTA");
    } else if (valor > 2000) {
      Serial.print(" ⚡ ALTA (línea)");
    } else if (valor > 1000) {
      Serial.print(" • Media");
    } else if (valor < 200) {
      Serial.print(" ✗ Baja");
    }
    
    Serial.println("\n└──────────────────────┘\n");
  }
  
  Serial.println("════════════════════════════════════════");
  Serial.println("  Próxima lectura en 1 segundo...");
  Serial.println("════════════════════════════════════════\n\n");
  
  delay(1000);
}