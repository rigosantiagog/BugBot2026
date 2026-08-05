#ifndef FUNC_ULTRASONICOS_H
#define FUNC_ULTRASONICOS_H

#include "config.h"

int medirDistanciaInterrupt(int trig, SensorEcho& sensor); // Mide un sensor HC-SR04 por interrupción (con timeout de 30ms)
void TareaUltrasonicos(void * pvParameters);                // Tarea FreeRTOS (Core 0): mide los 4 HC-SR04 en bucle continuo
void iniciarUltrasonicos();                                  // Lanza TareaUltrasonicos en el Core 0
void inicializarPinesUltrasonidos();                         // Configura pinMode de TRIG/ECHO y crea el semáforo binario

#endif
