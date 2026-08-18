#include "config.h"

HardwareSerial Enlace(2);

volatile int distFrente = 999;
volatile int distAtras = 999;
volatile int distIzq = 999;
volatile int distDer = 999;

volatile float angulo = -1.0;
volatile bool activo[16];

// Ultimo yaw recibido desde la ESP32 de motores (0.0 hasta que llegue el primer dato valido)
volatile float yaw = 0.0;
