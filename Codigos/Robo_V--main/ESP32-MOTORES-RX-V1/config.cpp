#include "config.h"

// ============================================================
//  PARÁMETROS DE VELOCIDAD — variables mutables
// ============================================================

volatile int velAvanceActual = 0;          // Arranca en 0; la rampa lo lleva hasta VEL_AVANCE
volatile unsigned long tSenalEstable = 0;  // Se resetea cada vez que el encuadre se rompe

// ============================================================
//  COMUNICACIÓN UART — objeto y variables mutables
// ============================================================

HardwareSerial Enlace(2);   // Instancia del puerto Serial 2 por hardware nativo de la ESP32
char uartBuffer[64];        // Buffer de recepción de tramas; se pisa con cada mensaje nuevo
volatile int bufIndex = 0;  // Índice de escritura dentro del buffer

volatile float anguloIR = -1.0f;  // -1.0 indica que aún no se ha recibido ningún dato
volatile int estadoIR = 0;
volatile int nIR = 0;
volatile unsigned long ultimoDato = 0;

// ============================================================
//  NAVEGACIÓN — variables mutables
// ============================================================

volatile float yaw = 0.0f;          // El norte se fija en 0 al terminar la calibración
volatile float gyroZoffset = 0.0f;  // Se sobreescribe por calibrarGyro() en el setup
volatile unsigned long tPrev = 0;

// ============================================================
//  MÁQUINA DE ESTADOS — variables mutables
// ============================================================

volatile EstadoRobot estadoActual = ESPERANDO_PELOTA;  // Estado inicial al encender

volatile unsigned long tFrenoIniciado = 0;
volatile unsigned long tUltimaVezPelota = 0;

volatile int pasoBusqueda = 0;
volatile unsigned long tBusqueda = 0;
volatile bool pelotaPerdidaReciente = false;

// ============================================================
//  NAVEGACION — ULTRASONICOS
// ============================================================

volatile int distFrente = 999;
volatile int distAtras = 999;
volatile int distIzq = 999;
volatile int distDer = 999;

String recepVecinos;
