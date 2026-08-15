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
  // Se arma todo el texto en un buffer local y se manda de UN SOLO jalon con Debug.print().
  // Antes se hacia un Debug.print() por cada digito/coma (33 escrituras): cada una viajaba
  // como un paquete TCP separado y puerto_anillo.py le pone su propio timestamp a cada
  // recv(), asi que el arreglo se veia partido en varias lineas en el .log. Con un solo
  // print() ya viaja como una sola linea.
  char buffer[64];   // "BMP:[" + 16 digitos + 15 comas + "]\n" + margen, sobra espacio de sobra
  int pos = 0;
  pos += snprintf(&buffer[pos], sizeof(buffer) - pos, "BMP:[");
  for (int i = 0; i < totalSensores; i++) {
    // Extraemos el bit i-ésimo (sensor i): 1 si detecta la pelota, 0 si no
    uint8_t bit = (bitmap >> i) & 0x01;
    pos += snprintf(&buffer[pos], sizeof(buffer) - pos, "%d%s", bit, (i < totalSensores - 1) ? "," : "");
  }
  snprintf(&buffer[pos], sizeof(buffer) - pos, "]\n");
  Debug.print(buffer);
}

/**
 * @brief Filtro de persistencia (debounce) para los 16 sensores IR.
 *
 *        Cada sensor tiene un contador de "confianza" (0..IR_CONFIANZA_MAX):
 *          - Si la lectura CRUDA de activo[i] es true, el contador sube (sin pasar el techo).
 *          - Si es false, el contador baja (sin bajar de 0).
 *        El sensor se reporta como activo, YA FILTRADO, solo si su confianza llego
 *        a IR_UMBRAL_ACTIVO o mas.
 *
 *        Efecto practico: un destello de ruido de 1 sola vuelta de loop en un sensor
 *        aislado sube su confianza a 1 nada mas (no alcanza IR_UMBRAL_ACTIVO=2), asi que
 *        NO llega a disparar un angulo erratico. Y una pelota real que estaba encendiendo
 *        un sensor no se pierde por 1-2 vueltas de loop sin lectura activa, porque la
 *        confianza baja de a poco (con IR_CONFIANZA_MAX=4 y umbral=2, hacen falta 3
 *        vueltas seguidas sin lectura para que el sensor deje de reportarse activo).
 *
 *        IMPORTANTE: debe llamarse justo despues de fotorreceptoresActivos() y ANTES de
 *        ubicarPelota(), ya que modifica activo[] en el lugar (y por lo tanto tambien
 *        afecta bitmapIR, que se calcula a partir de activo[]).
 *
 * @param totalActivos Referencia: se sobreescribe con el conteo de sensores activos YA FILTRADO.
 */
void aplicarFiltroPersistenciaIR(int& totalActivos) {
  static uint8_t confianza[16] = {0};   // Persiste entre llamadas (una por sensor)

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

/**
 * @brief Traduce un angulo (0-360 grados) a una de las 8 direcciones cardinales/
 *        intercardinales, para que el debug se lea de un vistazo sin tener que hacer
 *        la cuenta mental de grados a direccion.
 *
 *        Cada direccion cubre un sector de 45 grados (2 sensores), centrado en el
 *        multiplo de 45 mas cercano. Como el sensor 0 (0°) y el sensor 15 (337.5°) estan
 *        justo en el norte del anillo, el sector "N" queda exactamente entre ambos:
 *        [337.5°, 360°) U [0°, 22.5°).
 *
 * @param anguloGrados Angulo en grados (0-360). Si es negativo (-1.0, sin deteccion), devuelve "-".
 * @return Puntero a cadena constante: "N", "NE", "E", "SE", "S", "SO", "O", "NO", o "-".
 */
const char* obtenerOrientacionCardinal(float anguloGrados) {
  if (anguloGrados < 0) return "-";   // -1.0 = sin pelota detectada (ver ubicarPelota())

  static const char* nombres[8] = { "N", "NE", "E", "SE", "S", "SO", "O", "NO" };
  // Se suma medio sector (22.5) antes de dividir entre 45 para que el sector quede
  // CENTRADO en cada multiplo de 45 (y no empezando en el), luego %8 cierra el circulo.
  int sector = (int)((anguloGrados + (GRADOS_POR_SENSOR / 2.0f)) / 45.0f) % 8;
  return nombres[sector];
}
