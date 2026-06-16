/**
 * @file config.h (anillo)
 * @brief Configuraciones para la ESP32 del anillo: pines, UART, multiplexor y ultrasonidos.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
//  COMUNICACIÓN UART (hacia la ESP32 de motores)
// ============================================================

constexpr int RX_PIN = 26;   // No se usa en transmisión (solo por completitud)
constexpr int TX_PIN = 25;   // Pin de transmisión
extern HardwareSerial Enlace;

// ============================================================
//  MULTIPLEXOR CD74HC4067
// ============================================================

constexpr int pinS0 = 19;   // Selector bit 0
constexpr int pinS1 = 18;   // Selector bit 1
constexpr int pinS2 = 17;   // Selector bit 2
constexpr int pinS3 = 16;   // Selector bit 3
constexpr int pinSIG = 4;   // Señal común (salida del multiplexor)

constexpr int totalSensores = 16;
constexpr float GRADOS_POR_SENSOR = 22.5;   // 360° / 16

// ============================================================
//  ULTRASONIDOS HC-SR04
// ============================================================

constexpr int TRIG_F = 13; // Frontal
constexpr int ECHO_F = 12;
constexpr int TRIG_B = 14; // Trasero
constexpr int ECHO_B = 27;
constexpr int TRIG_L = 32; // Izquierdo
constexpr int ECHO_L = 33;
constexpr int TRIG_R = 21; // Derecho
constexpr int ECHO_R = 26;

// ============================================================
//  VARIABLES COMPARTIDAS
// ============================================================

extern volatile int distFrente;
extern volatile int distAtras;
extern volatile int distIzq;
extern volatile int distDer;

extern volatile bool activo[16];   // Estado de cada sensor IR
extern volatile float angulo;      // Ángulo calculado de la pelota

#endif