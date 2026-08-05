#ifndef FUNC_GIRO_H
#define FUNC_GIRO_H

#include <Arduino.h>

void mpuW(uint8_t r, uint8_t v);      // Escribe un byte en un registro del MPU
uint8_t mpuR(uint8_t r);              // Lee un byte de un registro del MPU
int16_t mpuGz();                      // Lee la velocidad angular en el eje Z (2 bytes)
bool mpuInit();                       // Inicializa el MPU y configura la escala ±250°/s
void calibrarGyro();                  // Calcula el offset del giroscopio (2000 muestras)
void actualizarRumbo();               // Integra la velocidad para actualizar yaw
float errorAngular(float angulo);     // Reduce un ángulo al rango [-180, 180]

#endif