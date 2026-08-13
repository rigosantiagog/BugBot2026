#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
//  COMUNICACIÓN — UART — Pines de comunicacion
// ============================================================

constexpr int RX_PIN = 26;
constexpr int TX_PIN = 25; 
extern HardwareSerial Enlace;

// ============================================================
//  MULTIPLEXOR — Pines de multiplexor
// ============================================================

constexpr int pinS0 = 19; 
constexpr int pinS1 = 18; 
constexpr int pinS2 = 17; 
constexpr int pinS3 = 16; 
constexpr int pinSIG = 4; 

constexpr int totalSensores = 16; 
constexpr float GRADOS_POR_SENSOR = 22.5; 

// ============================================================
//  ULTRASONICOS — Pines de ultrasonicos
// ============================================================

constexpr int TRIG_F = 13; // Frente
constexpr int ECHO_F = 12;

constexpr int TRIG_B = 14; // Atrás
constexpr int ECHO_B = 27;

constexpr int TRIG_L = 32; // Izquierda
constexpr int ECHO_L = 33;

constexpr int TRIG_R = 21; // Derecha
constexpr int ECHO_R = 26;

// ============================================================
//  NAVEGACION — ULTRASONICOS
// ============================================================

extern volatile int distFrente;
extern volatile int distAtras;
extern volatile int distIzq;
extern volatile int distDer;

//Indica los fotorreceptores actualmente encendidos
extern volatile bool activo[16]; 

extern volatile float angulo;

#endif
