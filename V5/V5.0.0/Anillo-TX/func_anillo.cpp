#include <math.h>
#include <Arduino.h>
#include "config.h"
#include "func_multiplexor.h"
#include "func_anillo.h"
#include "debug_wifi.h"

// =========================================================================
// DETECCIÓN DE PELOTA Y CÁLCULO DEL ÁNGULO
// =========================================================================
int ubicarPelota() {
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

  angulo = -1.0f;
  if (mejorLargo > 0) {
    float sx = 0, sy = 0;
    for (int j = 0; j < mejorLargo; j++) {
      int idx = (mejorInicio + j) % totalSensores;
      float a = radians(idx * GRADOS_POR_SENSOR);
      sx += cos(a);
      sy += sin(a);
    }
    angulo = degrees(atan2(sy, sx));
    if (angulo < 0) angulo += 360.0f;
  }
  return mejorLargo;
}

// =========================================================================
// LECTURA CRUDA DE LOS 16 SENSORES IR
// =========================================================================
void fotorreceptoresActivos(int& totalActivos) {
  totalActivos = 0;
  for (int i = 0; i < totalSensores; i++) {
    seleccionarCanal(i);
    delayMicroseconds(100);
    activo[i] = (digitalRead(pinSIG) == LOW);
    if (activo[i]) totalActivos++;
  }
}

// =========================================================================
// FILTRO DE PERSISTENCIA (DEBOUNCE) PARA EL ANILLO IR
// =========================================================================
void aplicarFiltroPersistenciaIR(int& totalActivos) {
  static uint8_t confianza[16] = {0};

  totalActivos = 0;
  for (int i = 0; i < totalSensores; i++) {
    if (activo[i]) {
      if (confianza[i] < IR_CONFIANZA_MAX) confianza[i]++;
    } else {
      if (confianza[i] > 0) confianza[i]--;
    }
    activo[i] = (confianza[i] >= IR_UMBRAL_ACTIVO);
    if (activo[i]) totalActivos++;
  }
}

// =========================================================================
// IMPRESIÓN DEL BITMAP COMO ARREGLO [0/1]
// =========================================================================
void imprimirArregloSensores(uint16_t bitmap) {
  char buffer[64];
  int pos = 0;
  pos += snprintf(&buffer[pos], sizeof(buffer) - pos, "BMP:[");
  for (int i = 0; i < totalSensores; i++) {
    uint8_t bit = (bitmap >> i) & 0x01;
    pos += snprintf(&buffer[pos], sizeof(buffer) - pos, "%d%s", bit, (i < totalSensores - 1) ? "," : "");
  }
  snprintf(&buffer[pos], sizeof(buffer) - pos, "]\n");
  Debug.print(buffer);
}

// =========================================================================
// TRADUCCIÓN DEL ÁNGULO A CARDINAL
// =========================================================================
const char* obtenerOrientacionCardinal(float anguloGrados) {
  if (anguloGrados < 0) return "-";
  static const char* nombres[8] = { "N", "NE", "E", "SE", "S", "SO", "O", "NO" };
  int sector = (int)((anguloGrados + (GRADOS_POR_SENSOR / 2.0f)) / 45.0f) % 8;
  return nombres[sector];
}