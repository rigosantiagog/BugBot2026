// func_anillo.cpp
#include <math.h>
#include <Arduino.h>
#include "config.h"
#include "func_multiplexor.h"
#include "func_anillo.h"
#include "debug_wifi.h"

/**
 * @brief Encuentra la cadena más larga de sensores activos consecutivos y calcula el ángulo centroide.
 * @return Número de sensores en la cadena más larga (0 si no hay).
 */
int ubicarPelota(){
  int mejorInicio = -1, mejorLargo = 0;
  int iniActual = -1, largoActual = 0;

  // Recorre el anillo dos veces para considerar el cruce por 0
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
  if (mejorLargo > totalSensores) mejorLargo = totalSensores; // Límite de seguridad

  angulo = -1.0;   // Por defecto sin ángulo

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

/**
 * @brief Lee los 16 sensores a través del multiplexor y actualiza el arreglo activo[].
 * @param totalActivos Referencia para devolver el número de sensores activos.
 */
void fotorreceptoresActivos(int& totalActivos){
  totalActivos = 0;
  for (int i = 0; i < totalSensores; i++) {
    seleccionarCanal(i);
    delayMicroseconds(100);                    // Tiempo para estabilizar la señal
    // El TSSP58038 entrega LOW cuando detecta la pelota (salida activa baja)
    activo[i] = (digitalRead(pinSIG) == LOW);
    if (activo[i]) totalActivos++;
  }
}

/**
 * @brief Imprime el bitmap de 16 sensores IR como un arreglo binario [0/1] en el debug
 *        (Serial USB + WiFi, ver debug_wifi.h).
 *
 *        Mapeo de posiciones (coincide con el orden usado en ubicarPelota() y con el
 *        armado de bitmapIR en el .ino, donde bit i = sensor i):
 *          índice 0  -> sensor 0  ->   0.0°
 *          índice 1  -> sensor 1  ->  22.5°
 *          índice 2  -> sensor 2  ->  45.0°
 *          ...
 *          índice 15 -> sensor 15 -> 337.5°
 *        (ángulo del sensor = índice * GRADOS_POR_SENSOR)
 *
 * @param bitmap Valor de 16 bits donde cada bit indica si el sensor correspondiente detecta la pelota.
 */
void imprimirArregloSensores(uint16_t bitmap) {
  Debug.print("BMP:[");
  for (int i = 0; i < totalSensores; i++) {
    // Extraemos el bit i-ésimo (sensor i): 1 si detecta la pelota, 0 si no
    uint8_t bit = (bitmap >> i) & 0x01;
    Debug.print(bit);
    if (i < totalSensores - 1) Debug.print(",");   // Separador entre elementos, excepto el último
  }
  Debug.println("]");
}
