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
#include <Fuzzy.h> // Librería para control difuso (Fuzzy Logic) - Opcional

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
const float RAMP_RATE = 0.01; // grados que cambia el offset por ciclo (5ms) -> empieza bajo y sube si querés más aceleración

// Instancia del PID (el parámetro Kd ahora opera internamente en la librería)
PID myPID(&Input, &OutputLibreria, &Setpoint, Kp, Ki, Kd, DIRECT);
double kd_gyro = 16.50; //punto de partida = tu Kd actual, luego se resintoniza 
// Instancia para el control de Preferences
Preferences memoriaPID;

// Función para inicializar el sistema y configurar los parámetros iniciales
// Pines y variables del sensor IR
const int pinSensorIR = 4;  
volatile bool senalDetectada = false;
volatile unsigned long ultimoPulsoTiempo = 0;
const unsigned long tiempoEspera = 100; // Tolerancia de pérdida de señal

const float SETPOINT_BASE = 28; 
float offsetMovimiento = 0.0;     // Modificador directo del ángulo
const float MAX_OFFSET = 0.6;     // Grados de inclinación para avanzar

// Variables para el movimiento por pulsos ("Caminar y Frenar")
unsigned long tiempoFaseMovimiento = 0;
bool enFaseDeAvance = true;
const unsigned long TIEMPO_AVANCE = 400; // ms que se inclina
const unsigned long TIEMPO_FRENO = 0;  // ms que se endereza para recuperar

// Configuracion del control remoto
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <RemoteXY.h>
#define REMOTEXY_BLUETOOTH_NAME "Self Balancing Robot"
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 105 bytes V19 
  { 255,4,0,51,0,98,0,19,0,0,0,115,101,120,0,24,2,106,200,200,
  84,1,1,5,0,1,36,74,31,31,87,23,16,16,0,2,31,85,112,0,
  1,35,117,32,32,86,48,17,17,0,2,31,68,111,119,110,0,67,1,10,
  104,26,45,3,99,11,100,2,26,51,1,5,96,31,31,70,35,16,16,0,
  2,31,76,101,102,116,0,1,68,97,31,31,105,35,16,16,0,2,31,82,
  105,103,104,116,0 };
  
// Estructura Pelua para las variables de la interfaz
struct {
    // variables de entrada (botones)
  uint8_t up; // =1 si el botón está presionado, si no =0
  uint8_t down; 
  uint8_t Left; 
  uint8_t Right; 

    // variables de salida (texto)
  char Estado_Envio[51]; // string UTF8 de hasta 50 caracteres

    // otras variables
  uint8_t connect_flag;  // =1 si está conectado por Bluetooth, si no =0
} RemoteXY;
#pragma pack(pop)

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

//Funcion de interrupción para el sensor IR
void IRAM_ATTR funcionInterrupcion() {
  senalDetectada = true;
  ultimoPulsoTiempo = millis(); 
}

void setup() {
    setCpuFrequencyMhz(240); 
    Serial.begin(460800); 
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); 
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);
    pinMode(pinSensorIR, INPUT_PULLUP); // Configura el pin del sensor IR 
    attachInterrupt(digitalPinToInterrupt(pinSensorIR), funcionInterrupcion, FALLING); // Configura la interrupción para el sensor IR
    RemoteXY_Init();

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
    Setpoint = SETPOINT_BASE + offsetMovimiento; //Hay que buscar un 28.2 de offset para que el robot quede en equilibrio
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
        RemoteXY_Handler();

        mpu.update();
        float rawAngle = mpu.getAngleX();
        smoothedAngleX = (alpha * rawAngle + (1 - alpha) * smoothedAngleX);
        
        float rawGyro = mpu.getGyroX();
        smoothedGyroX = alpha * rawGyro + (1 - alpha) * smoothedGyroX;

        float valorAbsolutoAngulo = abs(smoothedAngleX);

        // =========================================================
        // ---> 2. LÓGICA DE MOVIMIENTO INFRARROJO / REMOTEXY <---
        // =========================================================
        bool senalActiva = false;
        float direccion = 0.0;

        // Evaluamos si hay alguna orden de movimiento
        if (RemoteXY.up == 1) {
            senalActiva = true;
            direccion = MAX_OFFSET; // Avanzar (0.35)
        } 
        else if (RemoteXY.down == 1) {
            senalActiva = true;
            direccion = -MAX_OFFSET - 0.2; // Retroceder (-0.35)
        } 
        else if (senalDetectada) {
            senalActiva = true;
            direccion = MAX_OFFSET; // El sensor IR también hace avanzar
            
            // Si se pierde la señal IR por más del tiempo de espera
            if (millis() - ultimoPulsoTiempo > tiempoEspera) {
                senalDetectada = false; 
                senalActiva = false;
            }
        }

        // =========================================================
        // ---> APLICACIÓN DE LOS PULSOS PARA NO CAERSE <---
        // =========================================================
        if (senalActiva) {
            // ¡Variable faltante declarada aquí!
            unsigned long tiempoTranscurrido = millis() - tiempoFaseMovimiento;

            if (enFaseDeAvance) {
                // FASE 1: Se inclina en la dirección solicitada (Adelante o Atrás)
                offsetMovimiento = direccion; 
                
                if (tiempoTranscurrido >= TIEMPO_AVANCE) {
                    enFaseDeAvance = false;
                    tiempoFaseMovimiento = millis(); // Cambia a frenar
                }
            } 
            else {
                // FASE 2: Vuelve al centro directo para recuperar equilibrio
                offsetMovimiento = 0.0; 
                
                if (tiempoTranscurrido >= TIEMPO_FRENO) {
                    enFaseDeAvance = true;
                    tiempoFaseMovimiento = millis(); // Cambia a avanzar
                }
            }
        } 
        else {
            // No hay señal de la app ni del sensor: Reposo total inmediato
            offsetMovimiento = 0.0;
            enFaseDeAvance = true; // Prepara para que el próximo pulso empiece avanzando
            tiempoFaseMovimiento = millis();
        }

        // =========================================================
        // ---> AQUÍ SE CALCULA EL SETPOINT Y EL ERROR <---
        // =========================================================
        Setpoint = SETPOINT_BASE + offsetMovimiento;
        float errorAbsoluto = abs(smoothedAngleX - Setpoint);
        snprintf(RemoteXY.Estado_Envio, sizeof(RemoteXY.Estado_Envio), "Setpoint: %.2f", Setpoint);

        // ESTRUCTURA DE CONTROL CORREGIDA: If -> Else If -> Else
        if (!motoresHabilitados) {
            // 0. ESTADO DE PAUSA MANUAL (Ignora el péndulo)
            OutputFinal = 0;
            motorIzq.stop();
            motorDer.stop();
            digitalWrite(EN_PIN, HIGH); // APAGAR MOTORES
            if (myPID.GetMode() == AUTOMATIC) myPID.SetMode(MANUAL);
        }
        else if (valorAbsolutoAngulo > 65.0) {
            // 1. ESTADO DE EMERGENCIA (Caída libre)
            OutputFinal = 0;
            motorIzq.stop();
            motorDer.stop();
            digitalWrite(EN_PIN, HIGH); 
            myPID.SetMode(MANUAL);      
        } 
        else if (valorAbsolutoAngulo <= zonaMuerta) {
            // 2. ZONA MUERTA (Equilibrio perfecto)
            OutputFinal = 0;
            motorIzq.stop();
            motorDer.stop();
            digitalWrite(EN_PIN, HIGH);  
        } 
        else {
            // 3. ESTADO DE BALANCEO ACTIVO
            if (myPID.GetMode() == MANUAL) myPID.SetMode(AUTOMATIC); 
            digitalWrite(EN_PIN, LOW);  
            
            Input = (double)smoothedAngleX;
            myPID.Compute(); 
            
            OutputFinal = OutputLibreria - kd_gyro * smoothedGyroX; // el signo se ajusta probando;
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
        
        float errorAbsoluto = abs(smoothedAngleX - Setpoint);
        if (errorAbsoluto > 15.0) {
            led_interno.setPixelColor(0, led_interno.Color(200, 0, 0)); // Rojo
        } else if (errorAbsoluto <= 2.0) {
            led_interno.setPixelColor(0, led_interno.Color(0, 200, 0)); // Verde
        } else {
            led_interno.setPixelColor(0, led_interno.Color(255, 255, 0)); // Amarillo
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