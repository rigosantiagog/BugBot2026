#ifndef FUNC_GIRO_H
#define FUNC_GIRO_H

#include <Arduino.h>

/* ===================== FUNCIONES DEL SENSOR MPU I2C =========================== */
void mpuW(uint8_t r, uint8_t v);           // Función para escribir en el MPU
int16_t mpuGz();                           // Función para extraer datos físicos del MPU
bool mpuInit();                            // Función de arranque del giroscopio
void calibrarGyro();                       // Rutina de autocalibración térmica
void actualizarRumbo();                    // Rutina que se llama en cada milisegundo del loop
float errorAngular(float angulo);  // Rutina de la ruta más corta

#endif