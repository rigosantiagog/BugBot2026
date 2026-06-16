/**
 * @file config.h
 * @brief Configuraciones globales, pines, constantes y declaración de variables externas para la ESP32 de motores.
 * 
 * Este archivo contiene todas las definiciones que son comunes a varios módulos:
 * - Pines de los motores (TB6612FNG).
 * - Parámetros de velocidad, tolerancias y tiempos.
 * - Variables compartidas (volátiles) para comunicación entre archivos.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
//  PARÁMETROS DE JUEGO
// ============================================================

constexpr float FRENTE_ANILLO = 0.0f;        // Ángulo del chasis que apunta al frente de ataque (grados)
constexpr float TOL_APUNTADO = 40.0f;        // Tolerancia de encuadre antes de avanzar hacia la pelota (grados)
constexpr unsigned long T_QUIETO_MS = 4000;  // Duración del bloqueo inicial para estabilizar el MPU (ms)
constexpr int N_CAPTURA = 3;                 // Mínimo de sensores IR activos para confirmar posesión

constexpr float YAW_PORTERIA = 0.0f;   // Rumbo de la portería rival según el norte calibrado (grados)
constexpr float TOL_PORTERIA = 15.0f;  // Tolerancia de rumbo aceptable al avanzar a portería (grados)

// ============================================================
//  PINES DE CONTROL DE MOTORES (TB6612FNG)
// ============================================================

constexpr uint8_t STBY = 4;  // Habilitación global de los drivers (LOW = bloqueado)

// Motor Frontal Izquierdo (FL)
constexpr uint8_t FL_PWM = 25;  // Velocidad
constexpr uint8_t FL_IN1 = 26;  // Dirección 1
constexpr uint8_t FL_IN2 = 27;  // Dirección 2

// Motor Trasero Izquierdo (RL)
constexpr uint8_t RL_PWM = 33;  // Velocidad
constexpr uint8_t RL_IN1 = 32;  // Dirección 1
constexpr uint8_t RL_IN2 = 14;  // Dirección 2

// Motor Frontal Derecho (FR)
constexpr uint8_t FR_PWM = 13;  // Velocidad
constexpr uint8_t FR_IN1 = 23;  // Dirección 1
constexpr uint8_t FR_IN2 = 2;   // Dirección 2

// Motor Trasero Derecho (RR)
constexpr uint8_t RR_PWM = 19;  // Velocidad
constexpr uint8_t RR_IN1 = 18;  // Dirección 1
constexpr uint8_t RR_IN2 = 5;   // Dirección 2

// Inversión de giro para cada motor (true si el motor gira al revés de lo esperado)
constexpr bool INVERTIR_FL = false;
constexpr bool INVERTIR_FR = false;
constexpr bool INVERTIR_RL = false;
constexpr bool INVERTIR_RR = true;   // Ajustar según montaje físico

constexpr uint32_t PWM_FREQ = 20000;  // Frecuencia PWM (Hz) — 20 kHz evita ruido audible
constexpr uint8_t PWM_RES = 8;        // Resolución PWM — 8 bits (0-255)

// ============================================================
//  PARÁMETROS DE VELOCIDAD
// ============================================================

constexpr int PWM_MAX = 182;     // Límite absoluto de PWM para proteger los motores Faulhaber
constexpr int VEL_AVANCE = 120;  // Potencia de ataque directo a la pelota
constexpr int VEL_GIRO = 85;     // Potencia máxima de rotación sobre el eje propio
constexpr int VEL_BUSCAR = 80;   // Potencia durante el patrón de búsqueda
constexpr int RAMPA_PASO = 4;    // Incremento/decremento de PWM por ciclo de loop (aceleración suave)

constexpr unsigned long T_CONFIRMA_MS = 500;  // Tiempo mínimo de señal continua antes de atacar (ms)

// ============================================================
//  ULTRASONIDOS — Distancia de frenado para remate
// ============================================================

constexpr int DIST_FRENADO_CM = 20;   // Distancia a la portería para ejecutar frenado brusco (cm)

// ============================================================
//  CONTROL DE GIRO EN REGRESANDO (portería)
// ============================================================

constexpr float K_GIRO = 1.5f;               // Ganancia proporcional para el giro (velocidad = K * error)
constexpr int GIRO_MIN_PWM = 30;             // Velocidad mínima de giro para vencer la fricción
constexpr unsigned long T_GIRO_TIMEOUT = 3000; // Tiempo máximo girando sin alinear (ms)

// ============================================================
//  VARIABLES MUTABLES (definidas en config.cpp)
// ============================================================

extern volatile int velAvanceActual;          // Velocidad actual de la rampa de aceleración
extern volatile unsigned long tSenalEstable;  // Marca de tiempo desde que la pelota entró al cono frontal

// ============================================================
//  COMUNICACIÓN UART (Anillo IR → ESP32) - PROTOCOLO BINARIO
// ============================================================

constexpr uint8_t RX_PIN = 34;  // Pin de recepción (UART2 RX)
constexpr uint8_t TX_PIN = 17;  // Pin de transmisión (no usado, solo por definición)

extern HardwareSerial Enlace;   // Objeto Serial2

// Variables decodificadas de la trama binaria
extern volatile float anguloIR;            // Ángulo de la pelota (grados)
extern volatile int estadoIR;              // 1 = pelota detectada, 0 = ausente
extern volatile int nIR;                   // Número de sensores IR activos (0-16)
extern volatile unsigned long ultimoDato;  // Tiempo del último paquete válido (ms)

// ============================================================
//  NAVEGACIÓN — GIROSCOPIO MPU6500
// ============================================================

constexpr uint8_t MPU_ADDR = 0x68;  // Dirección I2C del MPU6500

extern volatile float yaw;            // Rumbo acumulado (grados)
extern volatile float gyroZoffset;    // Offset del eje Z (calculado en calibrarGyro)
extern volatile unsigned long tPrev;  // Tiempo del ciclo anterior (microsegundos)

// ============================================================
//  NAVEGACION — ULTRASONICOS (datos recibidos por UART)
// ============================================================

extern volatile int distFrente;   // Distancia frontal (cm)
extern volatile int distAtras;    // Distancia trasera (cm)
extern volatile int distIzq;      // Distancia izquierda (cm)
extern volatile int distDer;      // Distancia derecha (cm)

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================

enum EstadoRobot {
  ESPERANDO_PELOTA,  // Bloqueo inicial
  BUSCANDO,          // Búsqueda activa
  PERSIGUIENDO,      // Persiguiendo la pelota
  FRENANDO,          // Frenado tras captura
  REGRESANDO         // Yendo a portería
};

// Variables de estado (definidas en config.cpp)
extern volatile EstadoRobot estadoActual;

extern volatile unsigned long tFrenoIniciado;    // Momento de inicio del frenado (ms)
extern volatile unsigned long tUltimaVezPelota;  // Última vez que se vio la pelota (ms)

extern volatile int pasoBusqueda;            // Paso actual del patrón de búsqueda (0-7)
extern volatile unsigned long tBusqueda;     // Temporizador del patrón de búsqueda (ms)
extern volatile bool pelotaPerdidaReciente;  // Bandera de pérdida reciente

extern String recepVecinos;   // Cadena con los estados de los 16 sensores (para debug)

#endif
