#include "RobotStepper.h"

RobotStepper::RobotStepper(uint8_t stepPin, uint8_t dirPin, bool inverted) {
    _stepPin = stepPin;
    _dirPin = dirPin;
    _inverted = inverted;
    _lastToggleTime = 0;
    _halfPeriod = 0;
    _stepState = false;
    _currentSpeedHz = 0;
}

void RobotStepper::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
}

void RobotStepper::setSpeedHz(float speedHz) {
    _currentSpeedHz = speedHz;
    
    // 1. Determinar la dirección según el signo
    if (speedHz > 0) {
        digitalWrite(_dirPin, _inverted ? LOW : HIGH);
    } else if (speedHz < 0) {
        digitalWrite(_dirPin, _inverted ? HIGH : LOW);
    }
    
    // 2. Calcular el semi-periodo de la onda cuadrada
    float absSpeed = abs(speedHz);
    
    if (absSpeed < 1.0) {
        // Si la velocidad es casi cero, apagamos el temporizador virtual
        _halfPeriod = 0; 
    } else {
        // Periodo T = 1 / Frecuencia (en segundos)
        // En microsegundos: T_us = 1,000,000 / Frecuencia
        // Como es una onda simétrica, el pin cambia cada T/2
        _halfPeriod = (unsigned long)(500000.0 / absSpeed);
    }
}

void RobotStepper::update() {
    // Si la velocidad es 0, no hacemos nada
    if (_halfPeriod == 0) return;
    
    // Obtener el tiempo actual en microsegundos (alta precisión)
    unsigned long currentTime = micros();
    
    // Verificar si ya transcurrió el semi-periodo requerido
    if (currentTime - _lastToggleTime >= _halfPeriod) {
        _lastToggleTime = currentTime;
        _stepState = !_stepState; // Invertir el estado lógico
        digitalWrite(_stepPin, _stepState); // Inyectar al pin físico
    }
}

void RobotStepper::stop() {
    _halfPeriod = 0;
    _currentSpeedHz = 0;
    digitalWrite(_stepPin, LOW);
}