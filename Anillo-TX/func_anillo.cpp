// func_anillo.cpp
#include <math.h>
#include <Arduino.h>
#include "config.h"
#include "func_multiplexor.h"

/**
 * @brief Encuentra la cadena más larga de sensores activos consecutivos y calcula el ángulo centroide.
 * @return Número de sensores en la cadena más larga (0 si no hay).
 */
int ubicarPelota(){
  int mejorInicio = -1, mejorLargo = 0;
  int iniActual = -1, largoActual = 0;

  for (int k = 0; k < totalSensores * 2; k++) {
    int i = k % totalSensores;
    if (activo[i]) {
      if (largoActual == 0) iniActual = i;
      largoActual++;
      if (largoActual > mejorLargo) {
        mejorLargo = largoActual;
        mejorInicio = iniActual;
      }
    } else {
      largoActual = 0;
    }
  }
  if (mejorLargo > totalSensores) mejorLargo = totalSensores;

  angulo = -1.0;
  if (mejorLargo > 0) {
    float sx = 0, sy = 0;
    for (int j = 0; j < mejorLargo; j++) {
      int idx = (mejorInicio + j) % totalSensores;
      float a = radians(idx * GRADOS_POR_SENSOR);
      sx += cos(a);
      sy += sin(a);
    }
    angulo = degrees(atan2(sy, sx));
    if (angulo < 0) angulo += 360.0;
  }
  return mejorLargo;
}

void fotorreceptoresActivos(int& totalActivos){
  totalActivos = 0;
  for (int i = 0; i < totalSensores; i++) {
    seleccionarCanal(i);
    delayMicroseconds(100);
    activo[i] = (digitalRead(pinSIG) == LOW);
    if (activo[i]) totalActivos++;
  }
}