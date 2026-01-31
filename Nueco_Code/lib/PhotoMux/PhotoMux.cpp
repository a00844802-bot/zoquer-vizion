#include "PhotoMux.h"
#include <Arduino.h>

PhotoMux::PhotoMux(const uint8_t selectPins[3], const uint8_t muxPins[4]) {
    memcpy(_selectPins, selectPins, 3);
    memcpy(_muxPins, muxPins, 4);

    thresholds[FRONT] = 2400; //400
    thresholds[LEFT] = 1500; //410
    thresholds[RIGHT] = 2600;
    thresholds[BACK] = 2930;

    frontSensors = nullptr;
    leftSensors = nullptr;
    rightSensors = nullptr;
    backSensors = nullptr;

    frontCount = leftCount = rightCount = backCount = 0;
}

void PhotoMux::begin() {
    for (int i = 0; i < 3; i++) {
        pinMode(_selectPins[i], OUTPUT);
    }
}

void PhotoMux::configureSide(Side side, const Sensor* sensors, uint8_t count) {
    Sensor* ptr = new Sensor[count];
    memcpy(ptr, sensors, count * sizeof(Sensor));

    switch (side) {
        case FRONT: frontSensors = ptr; frontCount = count; break;
        case LEFT:  leftSensors = ptr;  leftCount = count;  break;
        case RIGHT: rightSensors = ptr; rightCount = count; break;
        case BACK:  backSensors = ptr;  backCount = count;  break;
    }
}

void PhotoMux::setThreshold(Side side, int threshold) {
    thresholds[side] = threshold;
}

void PhotoMux::selectChannel(uint8_t channel) {
    digitalWrite(_selectPins[0], channel & 0x01);
    digitalWrite(_selectPins[1], (channel >> 1) & 0x01);
    digitalWrite(_selectPins[2], (channel >> 2) & 0x01);
    delayMicroseconds(5); // Estabilización
}

int PhotoMux::readSensor(uint8_t muxIndex, uint8_t channel) {
    selectChannel(channel);
    return analogRead(_muxPins[muxIndex]);
}

float PhotoMux::readAverage(const Sensor* sensors, uint8_t size) {
    int sum = 0;
    for (uint8_t i = 0; i < size; i++) {
        sum += readSensor(sensors[i].muxIndex, sensors[i].channel);
    }
    return sum / float(size);
}

float PhotoMux::getAverage(Side side) {
    switch (side) {
        case FRONT: return readAverage(frontSensors, frontCount);
        case LEFT:  return readAverage(leftSensors, leftCount);
        case RIGHT: return readAverage(rightSensors, rightCount);
        case BACK:  return readAverage(backSensors, backCount);
        default:    return 0;
    }
}

float PhotoMux::getRawAverage(Side side) {
    return getAverage(side);  // Podrías hacer un filtro más crudo si lo deseas
}

bool PhotoMux::isLineDetected(Side side) {
    return getAverage(side) > thresholds[side];
}

/*void PhotoMux::lecturaDebug(Side side) {
    const char* sideNames[] = {"FRONT", "LEFT", "RIGHT", "BACK"};
    
    Sensor* sensors = nullptr;
    uint8_t count = 0;
    
    switch (side) {
        case FRONT: sensors = frontSensors; count = frontCount; break;
        case LEFT:  sensors = leftSensors;  count = leftCount;  break;
        case RIGHT: sensors = rightSensors; count = rightCount; break;
        case BACK:  sensors = backSensors;  count = backCount;  break;
    }
    
    if (sensors == nullptr || count == 0) {
        Serial.print(sideNames[side]);
        Serial.println(": No configurado");
        return;
    }
    
    Serial.print("┌─── ");
    Serial.print(sideNames[side]);
    Serial.println(" ───┐");
    
    for (uint8_t i = 0; i < count; i++) {
        int valor = readSensor(sensors[i].muxIndex, sensors[i].channel);
        
        Serial.print("│ S");
        Serial.print(i);
        Serial.print(" [MUX");
        Serial.print(sensors[i].muxIndex);
        Serial.print("-C");
        Serial.print(sensors[i].channel);
        Serial.print("]: ");
        Serial.print(valor);
        
        if (valor > thresholds[side]) {
            Serial.print(" ✓");
        }
        Serial.println();
    }
    
    float promedio = getAverage(side);
    Serial.print("│ Promedio: ");
    Serial.print(promedio, 1);
    Serial.print(" | Umbral: ");
    Serial.print(thresholds[side]);
    
    if (promedio > thresholds[side]) {
        Serial.println(" → LÍNEA!");
    } else {
        Serial.println(" → OK");
    }
    
    Serial.println("└────────────┘\n");
}*/