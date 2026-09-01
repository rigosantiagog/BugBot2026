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
// LECTURA CRUDA DE LOS 16 SENSORES IR (con sobremuestreo por canal)
// =========================================================================
// La pelota NO emite luz IR de forma continua: manda ráfagas cortas (~200us)
// separadas por silencios (periodo total ~833us, duty cycle ~24%). Si solo
// se toma UNA lectura por canal (como antes), es muy fácil "caer" justo en
// el silencio y perder la pelota aunque esté justo enfrente. Por eso aquí se
// toman varias lecturas rápidas por canal (IR_MUESTRAS_POR_CANAL) y basta
// con que UNA sola vea la ráfaga para marcar el sensor como activo en esta
// pasada — esto ataca el problema en el punto de lectura, en vez de
// intentar compensarlo después solo con el filtro de persistencia.
void fotorreceptoresActivos(int& totalActivos) {
  totalActivos = 0;                                        // Reinicia el contador de sensores activos antes de recorrer el anillo

  for (int i = 0; i < totalSensores; i++) {                 // Recorre los 16 canales del multiplexor uno por uno
    seleccionarCanal(i);                                    // Pone las líneas S0-S3 para conectar el sensor "i" a SIG
    delayMicroseconds(20);                                  // Espera breve para que el MUX termine de conmutar (propagación)

    bool detectado = false;                                 // Bandera: true si ALGUNA muestra ve la ráfaga IR en este canal

    for (uint8_t m = 0; m < IR_MUESTRAS_POR_CANAL; m++) {    // Toma varias lecturas rápidas dentro de la misma selección de canal
      if (digitalRead(pinSIG) == LOW) {                      // El TSSP58038 pone su salida en LOW cuando detecta la portadora IR
        detectado = true;                                    // Con que UNA muestra caiga dentro de la ráfaga ya cuenta como detección
      }
      delayMicroseconds(IR_INTERVALO_MUESTRA_US);            // Espera antes de la siguiente muestra, repartiéndolas en el tiempo
    }

    activo[i] = detectado;                                  // Guarda el resultado crudo (SIN filtrar) de este sensor
    if (activo[i]) totalActivos++;                          // Si detectó la pelota, suma al total de activos crudos
  }
}

// =========================================================================
// FILTRO DE PERSISTENCIA (DEBOUNCE) PARA EL ANILLO IR
// =========================================================================
// Cada sensor tiene un contador de "confianza" (0..IR_CONFIANZA_MAX). Un
// acierto crudo lo sube de inmediato (para no perder detecciones reales),
// pero la confianza solo baja después de varios FALLOS SEGUIDOS
// (IR_FALLOS_PARA_DECAER), no con cada fallo aislado. Así, el parpadeo
// normal de la señal (por el duty cycle bajo de la pelota) ya no tira la
// confianza a cero de inmediato, pero el ruido o los reflejos que no se
// sostienen en el tiempo sí terminan decayendo y se descartan.
void aplicarFiltroPersistenciaIR(int& totalActivos) {
  static uint8_t confianza[16]      = {0};                  // Confianza acumulada por sensor (persiste entre llamadas)
  static uint8_t fallosSeguidos[16] = {0};                  // Racha de fallos crudos consecutivos por sensor

  totalActivos = 0;                                          // Reinicia el conteo de sensores activos YA filtrados

  for (int i = 0; i < totalSensores; i++) {                  // Recorre los 16 sensores del anillo
    if (activo[i]) {                                         // Si la lectura cruda de este ciclo fue positiva
      fallosSeguidos[i] = 0;                                 // Se corta cualquier racha de fallos que llevara
      if (confianza[i] < IR_CONFIANZA_MAX) {                 // Mientras no se llegue al techo del contador
        confianza[i]++;                                      // Sube la confianza de inmediato (respuesta rápida a la señal real)
      }
    } else {                                                  // Si la lectura cruda de este ciclo fue negativa
      fallosSeguidos[i]++;                                   // Aumenta la racha de fallos seguidos de este sensor
      if (fallosSeguidos[i] >= IR_FALLOS_PARA_DECAER) {       // Solo castiga la confianza tras varios fallos seguidos
        if (confianza[i] > 0) confianza[i]--;                 // Resta un punto de confianza (decaimiento lento, no inmediato)
        fallosSeguidos[i] = 0;                                // Reinicia la racha para volver a contar desde cero
      }
    }

    activo[i] = (confianza[i] >= IR_UMBRAL_ACTIVO);           // El sensor se considera activo si supera el umbral filtrado
    if (activo[i]) totalActivos++;                            // Cuenta cuántos sensores quedaron activos tras el filtro
  }

  // ---- Salida de depuración por el monitor serie (limitada en frecuencia) ----
  static unsigned long ultimoDebug = 0;                       // Guarda el millis() del último print para no saturar el monitor
  if (millis() - ultimoDebug >= IR_DEBUG_INTERVALO_MS) {       // Solo imprime cada IR_DEBUG_INTERVALO_MS milisegundos
    ultimoDebug = millis();                                    // Actualiza la marca de tiempo del último print

    Debug.print("[IR-FILTRO] Activos: ");                      // Etiqueta de la línea de depuración (va también por USB, ver debug_wifi)
    Debug.print(totalActivos);                                 // Cuántos sensores quedaron activos tras el filtro
    Debug.print("  Confianza: [");                             // Abre el arreglo de confianza por sensor
    for (int i = 0; i < totalSensores; i++) {                   // Recorre los 16 sensores para imprimir su confianza
      Debug.print(confianza[i]);                                // Imprime el valor de confianza del sensor i
      if (i < totalSensores - 1) Debug.print(',');              // Separador entre valores, menos después del último
    }
    Debug.println("]");                                         // Cierra el arreglo y salta de línea
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
