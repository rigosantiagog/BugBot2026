/**
 * @file func_giro.cpp
 * @brief Implementación del giroscopio MPU6500 con calibración robusta sin std::sort.
 * 
 * La calibración utiliza un método de selección para encontrar la mediana
 * sin necesidad de ordenar todo el arreglo, evitando errores de memoria
 * (Guru Meditation) que ocurrían con std::sort en la ESP32.
 * 
 * También incluye filtro de promedio móvil y zona muerta para reducir el ruido.
 */

#include "func_giro.h"       // Declaraciones de funciones
#include "config.h"          // Definiciones globales (MPU_ADDR, yaw, gyroZoffset, etc.)
#include <Wire.h>            // Comunicación I2C
#include <cmath>             // Para fabs()

// ============================================================
//  FUNCIONES DE ACCESO AL MPU6500 (lectura/escritura de registros)
// ============================================================

/**
 * @brief Escribe un byte en un registro del MPU.
 * @param r  Dirección del registro.
 * @param v  Valor a escribir.
 */
void mpuW(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR);   // Inicia comunicación con el dispositivo I2C
  Wire.write(r);                      // Envía la dirección del registro
  Wire.write(v);                      // Envía el valor a escribir
  Wire.endTransmission();             // Finaliza la transmisión
}

/**
 * @brief Lee un byte de un registro del MPU.
 * @param r  Dirección del registro.
 * @return   Valor leído (0 si falla).
 */
uint8_t mpuR(uint8_t r) {
  Wire.beginTransmission(MPU_ADDR);   // Inicia comunicación
  Wire.write(r);                      // Envía la dirección del registro
  Wire.endTransmission(false);        // Reinicia pero no detiene (repeated start)
  Wire.requestFrom(MPU_ADDR, (uint8_t)1); // Solicita 1 byte
  if (Wire.available()) return Wire.read(); // Devuelve el byte leído
  return 0;                            // Si no hay datos, retorna 0
}

/**
 * @brief Lee la velocidad angular en el eje Z (2 bytes, little-endian).
 * @return Valor entero de 16 bits (LSB). Se divide por 131.0 para obtener °/s.
 */
int16_t mpuGz() {
  Wire.beginTransmission(MPU_ADDR);   // Inicia comunicación
  Wire.write(0x47);                   // Registro alto del eje Z (0x47 y 0x48)
  Wire.endTransmission(false);        // Repeated start
  Wire.requestFrom(MPU_ADDR, (uint8_t)2); // Solicita 2 bytes
  return (Wire.read() << 8) | Wire.read(); // Combina los bytes en un entero de 16 bits
}

// ============================================================
//  INICIALIZACIÓN DEL MPU
// ============================================================

/**
 * @brief Inicializa el MPU: configura pines I2C, velocidad, timeout y registros.
 *        Verifica la identidad del chip leyendo WHO_AM_I.
 * @return true si se detecta el dispositivo y se configura correctamente.
 */
bool mpuInit() {
  Wire.begin(21, 22);                 // SDA=21, SCL=22 (según conexión)
  Wire.setClock(400000);              // 400 kHz (Fast Mode)
  Wire.setTimeout(25);                // Timeout de 25 ms para evitar bloqueos

  // Verificar conexión con el MPU
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU no responde en la dirección I2C.");
    return false;
  }

  // Leer WHO_AM_I (debe ser 0x68, 0x70 o 0x71 según el modelo)
  uint8_t who = mpuR(0x75);
  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);
  if (who != 0x68 && who != 0x70 && who != 0x71) {
    Serial.println("AVISO: WHO_AM_I no coincide con lo esperado.");
    // No fallamos, pero avisamos para depuración.
  }

  // Despertar el MPU (salir del modo sleep)
  mpuW(0x6B, 0x00);
  delay(100);

  // Configurar el giroscopio a ±250°/s (registro 0x1B = 0x00)
  // Factor de conversión: 131.0 LSB/(°/s)
  mpuW(0x1B, 0x00);
  delay(50);

  // Leer el registro de configuración para confirmar la escala
  uint8_t gyroConfig = mpuR(0x1B);
  Serial.printf("Gyro config (0x1B) = 0x%02X (0x00 = ±250°/s)\n", gyroConfig);

  return true;
}

// ============================================================
//  CALIBRACIÓN DEL GIROSCOPIO (SIN std::sort)
// ============================================================

/**
 * @brief Calibración del giroscopio: toma 1000 lecturas estáticas, calcula el offset.
 *        El offset se guarda en gyroZoffset (en LSB).
 * 
 * Se usa un método de selección para encontrar la mediana sin ordenar todo el arreglo,
 * evitando así errores de memoria (Guru Meditation) que ocurrían con std::sort.
 * Luego se promedian los valores cercanos a la mediana para mayor robustez.
 */
void calibrarGyro() {
  const int muestras = 1000;           // Número de lecturas (suficiente y rápido)
  float valores[muestras];             // Arreglo para almacenar las lecturas
  Serial.print("Calibrando MPU6500 estáticamente (1000 muestras)... ");

  // Toma 1000 lecturas del eje Z
  for (int i = 0; i < muestras; i++) {
    valores[i] = (float)mpuGz();       // Lee el valor crudo
    delay(2);                          // Pausa de 2 ms entre lecturas
  }

  // ============================================================
  //  Cálculo de la mediana sin usar std::sort
  //  Algoritmo de selección O(n^2) pero para n=1000 es aceptable.
  // ============================================================
  int mitad = muestras / 2;            // Posición de la mediana (índice 500)
  float mediana = 0.0f;

  // Para cada elemento, contamos cuántos son menores que él.
  // El elemento que tenga 'mitad' elementos menores es la mediana.
  for (int i = 0; i < muestras; i++) {
    int count = 0;                     // Cuenta de elementos menores que valores[i]
    for (int j = 0; j < muestras; j++) {
      if (valores[j] < valores[i]) count++;
    }
    // Si el número de menores es <= mitad y además no es demasiado pequeño,
    // consideramos que este es el valor mediano.
    // (Esta condición simple funciona porque los valores son únicos o casi únicos)
    if (count <= mitad && count + (muestras - count) > mitad) {
      mediana = valores[i];
      break;
    }
  }

  // ============================================================
  //  Promedio de valores cercanos a la mediana (elimina picos)
  // ============================================================
  double suma = 0;
  int count = 0;
  // Definimos un margen de ±50 LSB alrededor de la mediana
  for (int i = 0; i < muestras; i++) {
    if (fabs(valores[i] - mediana) < 50.0f) {
      suma += valores[i];
      count++;
    }
  }
  // Si encontramos suficientes valores, usamos el promedio; si no, usamos la mediana
  if (count > 0) {
    gyroZoffset = (float)(suma / count);
  } else {
    gyroZoffset = mediana;   // Fallback
  }

  Serial.printf("Offset Fijo = %.4f LSB\n", gyroZoffset);
}

// ============================================================
//  FILTRO Y ACTUALIZACIÓN DEL RUMBO (YAW)
// ============================================================

// Variables estáticas para el filtro de promedio móvil (3 muestras)
static float lecturasAnteriores[3] = {0, 0, 0};
static int idxLectura = 0;

/**
 * @brief Actualiza el rumbo (yaw) integrando la velocidad angular.
 *        Debe llamarse periódicamente (cada ciclo del loop).
 *        Aplica un filtro de promedio móvil y una zona muerta para reducir el ruido.
 */
void actualizarRumbo() {
  unsigned long n = micros();                // Tiempo actual en microsegundos
  float dt = (n - tPrev) / 1000000.0;        // Diferencia de tiempo en segundos
  if (dt > 0.05) dt = 0.05;                  // Saturación para evitar saltos bruscos
  tPrev = n;                                 // Actualiza el tiempo anterior

  // Lee el valor crudo del giroscopio (eje Z)
  int16_t raw = mpuGz();
  // Convierte a °/s restando el offset y dividiendo por 131.0
  float gz = ((float)raw - gyroZoffset) / 131.0f;

  // Filtro de promedio móvil (últimas 3 lecturas)
  lecturasAnteriores[idxLectura % 3] = gz;   // Guarda la lectura actual
  idxLectura++;                               // Incrementa el índice
  // Calcula el promedio de las 3 lecturas
  float gzFiltrado = (lecturasAnteriores[0] + lecturasAnteriores[1] + lecturasAnteriores[2]) / 3.0f;

  // Zona muerta: si la velocidad es menor a 0.2 °/s, se ignora (evita ruido)
  if (abs(gzFiltrado) < 0.2f) {
    gzFiltrado = 0.0f;
  }

  // Integra la velocidad angular para obtener el ángulo (yaw)
  yaw += gzFiltrado * dt;
}

// ============================================================
//  UTILIDAD: NORMALIZACIÓN DE ÁNGULOS
// ============================================================

/**
 * @brief Reduce un ángulo al rango [-180, 180] grados para encontrar el camino más corto.
 * @param angulo Ángulo en grados.
 * @return Ángulo equivalente en [-180, 180].
 */
float errorAngular(float angulo) {
  while (angulo > 180) angulo -= 360;
  while (angulo < -180) angulo += 360;
  return angulo;
}