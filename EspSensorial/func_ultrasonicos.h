#ifndef FUNC_ULTRASONICOS_H
#define FUNC_ULTRASONICOS_H

#include "config.h"

int medirDistanciaInterrupt(int trig, SensorEcho& sensor);
void TareaUltrasonicos(void * pvParameters);
void iniciarUltrasonicos();
void inicializarPinesUltrasonidos();

#endif
