#include "config.h"

HardwareSerial Enlace(2);

// ============================================================
//  NAVEGACION — ULTRASONICOS — variables mutables
// ============================================================

volatile int distFrente = 999;
volatile int distAtras  = 999;
volatile int distIzq    = 999;
volatile int distDer    = 999;

volatile float angulo = -1.0;

volatile bool activo[16]; 
