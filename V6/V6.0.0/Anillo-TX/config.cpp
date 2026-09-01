#include "config.h"

HardwareSerial Enlace(2);

volatile int distFrente = 999;
volatile int distAtras  = 999;
volatile int distIzq    = 999;
volatile int distDer    = 999;

volatile bool activo[16] = {false};
volatile float angulo = -1.0f;
volatile float yaw    = 0.0f;
volatile float robotX = 0.0f;
volatile float robotY = 0.0f;