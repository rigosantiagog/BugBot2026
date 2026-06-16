/**
 * @file config.cpp
 * @brief Definición de variables globales compartidas entre módulos de la ESP32 de motores.
 * 
 * Aquí se instancian las variables declaradas como extern en config.h.
 * Incluye variables de velocidad, UART, giroscopio, ultrasonidos y máquina de estados.
 */

#include "config.h"

// ============================================================
//  PARÁMETROS DE VELOCIDAD
// ============================================================

volatile int velAvanceActual = 0;          // Velocidad actual de la rampa (0 = detenido)
volatile unsigned long tSenalEstable = 0;  // Tiempo desde que la señal entró en el cono frontal

// ============================================================
//  COMUNICACIÓN UART
// ============================================================

HardwareSerial Enlace(2);   // Instancia del puerto Serial2 (UART2)

volatile float anguloIR = -1.0f;  // -1.0 indica "sin dato"
volatile int estadoIR = 0;        // 0 = no detecta, 1 = detecta
volatile int nIR = 0;             // Contador de sensores activos
volatile unsigned long ultimoDato = 0; // Último timestamp de trama válida

// ============================================================
//  NAVEGACIÓN — GIROSCOPIO
// ============================================================

volatile float yaw = 0.0f;          // Rumbo acumulado
volatile float gyroZoffset = 0.0f;  // Offset calibrado
volatile unsigned long tPrev = 0;   // Tiempo anterior (micros)

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================

volatile EstadoRobot estadoActual = ESPERANDO_PELOTA;

volatile unsigned long tFrenoIniciado = 0;
volatile unsigned long tUltimaVezPelota = 0;

volatile int pasoBusqueda = 0;
volatile unsigned long tBusqueda = 0;
volatile bool pelotaPerdidaReciente = false;

// ============================================================
//  ULTRASONICOS (datos recibidos)
// ============================================================

volatile int distFrente = 999;
volatile int distAtras = 999;
volatile int distIzq = 999;
volatile int distDer = 999;

String recepVecinos;   // Para debug