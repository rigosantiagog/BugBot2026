/**
 * @file config.cpp (anillo)
 * @brief Definición de variables globales del anillo.
 */

#include "config.h"

HardwareSerial Enlace(2);   // Puerto Serial2

volatile int distFrente = 999;
volatile int distAtras  = 999;
volatile int distIzq    = 999;
volatile int distDer    = 999;

volatile float angulo = -1.0;

volatile bool activo[16];