#ifndef ROBOT_STEPPER_H
#define ROBOT_STEPPER_H

#include <Arduino.h>

class RobotStepper {
private:
    uint8_t _stepPin;
    uint8_t _dirPin;
    bool _inverted;
    
    unsigned long _lastToggleTime; // Almacena el último microsegundo de cambio
    unsigned long _halfPeriod;     // Tiempo en microsegundos que el pin debe estar en HIGH o LOW
    bool _stepState;               // Estado actual del pin STEP (HIGH/LOW)
    float _currentSpeedHz;         // Velocidad actual en Pasos por Segundo

public:
    // Constructor de la clase
    RobotStepper(uint8_t stepPin, uint8_t dirPin, bool inverted = false);
    
    // Inicializa los pines físicos
    void begin();
    
    // Configura la velocidad en Pasos por Segundo (Hz). Puede ser positiva o negativa.
    void setSpeedHz(float speedHz);
    
    // Ejecuta el toggle del pin STEP de forma no bloqueante. Debe llamarse en cada vuelta del loop.
    void update();
    
    // Detiene el motor inmediatamente
    void stop();
};

#endif