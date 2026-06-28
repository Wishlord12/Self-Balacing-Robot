/* ==========================================================================
 * PROYECTO: Control Péndulo Invertido - Optimización de Ciclo de Reloj
 * MICROCONTROLADOR: ESP32-S3N16R8
 * LIBRERÍA PERSONALIZADA: RobotStepper (Liberada del cuello de botella)
 * MODIFICACIÓN: PID Estándar Completo con Auto-Guardado y Menú en Vivo
 * ========================================================================== */

/*============================================================================

 * DESCRIPCIÓN GENERAL:
 * Este código implementa un sistema de control para un robot de péndulo invertido
 * utilizando un ESP32-S3. Se emplea un sensor MPU6050 para medir el ángulo y la
 * velocidad angular, y se utiliza un algoritmo PID para mantener el equilibrio.
 * 
 * CARACTERÍSTICAS PRINCIPALES:
 * - Control PID con ajuste en tiempo real a través del monitor serie.
 * - Guardado automático de los parámetros PID en la memoria flash del ESP32.
 * - Función de recalibración del MPU6050 sin necesidad de reiniciar el sistema.
 * - Sistema de seguridad que detiene los motores si el ángulo excede los 45 grados.
 * - Indicadores LED para mostrar el estado del sistema (equilibrio, alerta, etc.).
 *
 * NOTAS:
 * - Asegúrese de no mover el robot durante la calibración del MPU6050.
 * - Los motores pueden ser habilitados o deshabilitados mediante comandos en el monitor serie.
 *
 *============================================================================*/

 // Librerías necesarias para el funcionamiento del sistema
#include <Arduino.h> // Librería principal de Arduino para ESP32
#include <Wire.h> // Librería para comunicación I2C
#include <MPU6050_light.h> // Librería para el sensor MPU6050
#include <Adafruit_NeoPixel.h> // Librería para control de LEDs NeoPixel
#include <PID_v1.h> // Librería para control PID
#include <Preferences.h> // Librería para almacenamiento en memoria flash del ESP32
#include "RobotStepper.h" // Librería personalizada para control de motores paso a paso

// Definición de pines y parámetros del sistema
#define I2C_SDA_PIN 1
#define I2C_SCL_PIN 2
#define PIN_LED_RGB 48 
#define NUM_PIXELS 1   
#define DIR_IZQ 6
#define STEP_IZQ 5
#define DIR_DER 7
#define STEP_DER 15
#define EN_PIN 17
#define MS3_PIN 3
#define MS2_PIN 10   
#define MS1_PIN 11   

// Instancias de los motores paso a paso utilizando la librería RobotStepper
RobotStepper motorIzq(STEP_IZQ, DIR_IZQ, false);
RobotStepper motorDer(STEP_DER, DIR_DER, false); 
float MAX_SPEED = 26000.0; // Velocidad límite en Hz

// Instancia del LED interno para indicar el estado del sistema
Adafruit_NeoPixel led_interno(NUM_PIXELS, PIN_LED_RGB, NEO_GRB + NEO_KHZ800);

// Instancia del sensor MPU6050 para medir ángulo y velocidad angular
MPU6050 mpu(Wire);

// Variables globales
bool motoresHabilitados = true;
float smoothedAngleX = 0;
float smoothedGyroX = 0;
float alpha = 0.1; 
double Setpoint, Input, OutputLibreria, OutputFinal = 0; // Variables para el control PID
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 100; 
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 5; // 5ms = 200Hz de muestreo estable
double Kp = 40.0; // Coeficiente proporcional inicial
double Ki = 2.0;  // Coeficiente integral inicial
double Kd = 0.6;  // Coeficiente derivativo inicial
float zonaMuerta = 0.; //Zona muerta para evitar oscilaciones menores a este ángulo

// Instancia del PID (el parámetro Kd ahora opera internamente en la librería)
PID myPID(&Input, &OutputLibreria, &Setpoint, Kp, Ki, Kd, DIRECT);

// Instancia para el control de Preferences
Preferences memoriaPID;


// Función para apagar/encender los motores de forma segura
void alternarMotores() {
    motoresHabilitados = !motoresHabilitados; // Invierte el estado actual
    
    Serial.println("\n=================================================");
    if (motoresHabilitados) {
        Serial.println(">>> MOTORES ENCENDIDOS (Sistema Activo) <<<");
        // No forzamos el LOW aquí porque el loop() lo hará si el ángulo es correcto
    } else {
        Serial.println(">>> MOTORES APAGADOS (Sistema Pausado) <<<");
        // Detenemos físicamente todo
        OutputFinal = 0;
        motorIzq.stop();
        motorDer.stop();
        digitalWrite(EN_PIN, HIGH); // Quita el torque (rueda libre)
        myPID.SetMode(MANUAL);      // Pausa el PID para que no acumule errores
    }
    Serial.println("=================================================\n");
}
// Función para guardar los valores en la memoria flash cuando tú lo decidas
void guardarNuevosValores(double nuevoKp, double nuevoKi, double nuevoKd) {
    memoriaPID.begin("RobotMem", false); 
    memoriaPID.putDouble("Kp", nuevoKp);
    memoriaPID.putDouble("Ki", nuevoKi);
    memoriaPID.putDouble("Kd", nuevoKd);
    memoriaPID.end();
    
    Kp = nuevoKp;
    Ki = nuevoKi;
    Kd = nuevoKd;
    
    myPID.SetTunings(Kp, Ki, Kd); // Sincroniza la librería
    
    Serial.println("\n=================================================");
    Serial.println(">>> ¡Valores guardados permanentemente en Flash! <<<");
    Serial.println("=================================================");
}
// Función para reiniciar y recalibrar el MPU6050 en caliente
void recalibrarMPU() {
    Serial.println("\n=================================================");
    Serial.println(">>> RECALIBRANDO MPU6050... ¡NO MOVER EL ROBOT! <<<");
    
    // 1. Apagar motores para evitar vibraciones
    motorIzq.stop();
    motorDer.stop();
    digitalWrite(EN_PIN, HIGH); 
    
    // 2. Pausar el PID
    myPID.SetMode(MANUAL);
    
    // 3. Pequeña pausa para estabilizar físicamente el robot
    delay(1000); 
    
    // 4. Recalcular offsets (Toma unos segundos)
    mpu.calcOffsets(true, true); 
    
    // 5. Reiniciar las variables del filtro y del control
    smoothedAngleX = 0;
    smoothedGyroX = 0;
    Input = 0;
    OutputFinal = 0;
    
    Serial.println(">>> ¡Calibración exitosa! Retomando control... <<<");
    Serial.println("=================================================\n");
}

void setup() {
    setCpuFrequencyMhz(240); 
    Serial.begin(460800); 
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); 
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);

    // Configuración física fija a 1/4 de paso
    digitalWrite(MS1_PIN, HIGH); 
    digitalWrite(MS2_PIN, LOW); 
    digitalWrite(MS3_PIN, HIGH); 

    motorIzq.begin();
    motorDer.begin();

    led_interno.begin();
    led_interno.clear();
    led_interno.show();

    Serial.println("\n=====================================");
    Serial.println("   ROBOT OPTIMIZADO - PID ESTÁNDAR   ");
    Serial.println("=====================================");

    // Leer los coeficientes PID desde la memoria del ESP32-S3
    memoriaPID.begin("RobotMem", false);
    Kp = memoriaPID.getDouble("Kp", 40.0);
    Ki = memoriaPID.getDouble("Ki", 2.0);
    Kd = memoriaPID.getDouble("Kd", 0.6);
    memoriaPID.end();

    Serial.print("Valores cargados -> Kp: "); Serial.print(Kp);
    Serial.print(" | Ki: "); Serial.print(Ki);
    Serial.print(" | Kd: "); Serial.println(Kd);

    // Inicializar I2C y forzarlo a modo RÁPIDO (400 kHz)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); 

    byte status = mpu.begin();
    if (status != 0) {
        Serial.println("¡ERROR! MPU6050 no encontrado.");
        while (1); 
    }

    Serial.println("Calibrando sensor... ¡NO MOVER!");
    delay(5000);
    mpu.calcOffsets(true, true); 
    mpu.update();
    Setpoint = 28.2; //Hay que buscar un 28.2 de offset para que el robot quede en equilibrio
    myPID.SetTunings(Kp, Ki, Kd); // Asegura aplicar las constantes leídas
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(controlInterval); 
    myPID.SetOutputLimits(-MAX_SPEED, MAX_SPEED); 
}

void loop() {
    // ======================================================================
    // INTERRUPCIÓN VIRTUAL: GESTIÓN DE PASOS DE ALTA VELOCIDAD
    // ======================================================================
    motorIzq.update();
    motorDer.update();

    unsigned long currentMillis = millis();

   // ======================================================================
    // LAZO DE CONTROL CONTROLADO (Cada 5 milisegundos)
    // ======================================================================
    if (currentMillis - lastControlTime >= controlInterval) {
        lastControlTime = currentMillis;

        mpu.update();
        float rawAngle = mpu.getAngleX();
        smoothedAngleX = (alpha * rawAngle + (1 - alpha) * smoothedAngleX);
        
        float rawGyro = mpu.getGyroX();
        smoothedGyroX = alpha * rawGyro + (1 - alpha) * smoothedGyroX;

        float valorAbsolutoAngulo = abs(smoothedAngleX);

        // ESTRUCTURA DE CONTROL CORREGIDA: If -> Else If -> Else
        if (!motoresHabilitados) {
            // 0. ESTADO DE PAUSA MANUAL (Ignora el péndulo)
            OutputFinal = 0;
            motorIzq.stop();
            motorDer.stop();
            digitalWrite(EN_PIN, HIGH); // APAGAR MOTORES
            if (myPID.GetMode() == AUTOMATIC) myPID.SetMode(MANUAL);
        }
        else if (valorAbsolutoAngulo > 45.0) {
            // 1. ESTADO DE EMERGENCIA (Caída libre)
            OutputFinal = 0;
            motorIzq.stop();
            motorDer.stop();
            digitalWrite(EN_PIN, HIGH); // APAGAR MOTORES (Libera el torque por seguridad)
            myPID.SetMode(MANUAL);      // Pausar PID para que no acumule basura
        } 
        else if (valorAbsolutoAngulo <= zonaMuerta) {
            // 2. ZONA MUERTA (Equilibrio perfecto)
            OutputFinal = 0;
            motorIzq.stop();
            motorDer.stop();
            digitalWrite(EN_PIN, HIGH);  // MANTENER ENCENDIDO (Torque de retención activo)
            // Opcional: podrías poner myPID.SetMode(MANUAL) aquí si notas que el Ki acumula error
        } 
        else {
            // 3. ESTADO DE BALANCEO ACTIVO
            if (myPID.GetMode() == MANUAL) myPID.SetMode(AUTOMATIC); // Reactivar PID si estaba pausado
            digitalWrite(EN_PIN, LOW);  // Asegurar que los motores tienen fuerza
            
            Input = (double)smoothedAngleX;
            myPID.Compute(); 
            
            OutputFinal = OutputLibreria;
            OutputFinal = constrain(OutputFinal, -MAX_SPEED, MAX_SPEED);
            
            motorIzq.setSpeedHz(-OutputFinal);
            motorDer.setSpeedHz(OutputFinal);
        }
    }

    // ======================================================================
    // TELEMETRÍA (Cada 100 milisegundos)
    // ======================================================================
    if (currentMillis - lastUpdateTime >= updateInterval) {
        lastUpdateTime = currentMillis;
        
        float valorAbsolutoAngulo = abs(smoothedAngleX);
        if (valorAbsolutoAngulo > 30) {
            led_interno.setPixelColor(0, led_interno.Color(200, 0, 0)); 
        } else if (valorAbsolutoAngulo <= 1) {
            led_interno.setPixelColor(0, led_interno.Color(0, 200, 0)); 
        } else {
            led_interno.setPixelColor(0, led_interno.Color(255, 255, 0)); 
        }
        led_interno.show();
        
        // Impresión detallada para sintonización visual
        Serial.print("Angulo:"); Serial.print(smoothedAngleX);
        Serial.print(",Kp:"); Serial.print(Kp);
        Serial.print(",Ki:"); Serial.print(Ki);
        Serial.print(",Kd:"); Serial.print(Kd);
        Serial.print(",Hz_PID:"); Serial.print(OutputFinal);
        Serial.print(",Setpoint:"); Serial.println(Setpoint);
    }

    // ======================================================================
    // MENÚ DE SINTONIZACIÓN EN VIVO (Vía Monitor Serie)
    // ======================================================================
    if (Serial.available() > 0) {
        char comando = Serial.read();       // Lee la acción (P, I, D, o S)
        double valor = Serial.parseFloat(); // Lee el valor flotante asociado
        
        if (comando == 'P' || comando == 'p') {
            Kp = valor;
            myPID.SetTunings(Kp, Ki, Kd); 
            Serial.print(">>> Kp modificado a: "); Serial.println(Kp);
        }
        else if (comando == 'I' || comando == 'i') {
            Ki = valor;
            myPID.SetTunings(Kp, Ki, Kd); 
            Serial.print(">>> Ki modificado a: "); Serial.println(Ki);
        }
        else if (comando == 'D' || comando == 'd') {
            Kd = valor;
            myPID.SetTunings(Kp, Ki, Kd); 
            Serial.print(">>> Kd modificado a: "); Serial.println(Kd);
        }
        else if (comando == 'S' || comando == 's') {
            guardarNuevosValores(Kp, Ki, Kd); // Guarda la configuración en la flash
        }
        else if (comando == 'R' || comando == 'r') {
            recalibrarMPU(); 
        }
        // NUEVO COMANDO PARA APAGAR/ENCENDER MOTORES
        else if (comando == 'M' || comando == 'm') {
            alternarMotores(); 
        }
        // Vaciar cualquier residuo del buffer serial (ej. saltos de línea)
        while(Serial.available() > 0) { Serial.read(); }
    }
}