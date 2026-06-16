/**
 * @file func_ultrasonicos.h
 * @brief Declaraciones para la medición con ultrasonidos HC-SR04.
 */

#ifndef FUNC_ULTRASONICOS_H
#define FUNC_ULTRASONICOS_H

int medirDistancia(int pinTrig, int pinEcho);
void TareaUltrasonicos(void * pvParameters);
void iniciarUltrasonicos();   // Lanza la tarea en el Núcleo 0

#endif