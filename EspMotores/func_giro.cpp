/**
 * @file func_giro.cpp
 * @brief Implementación de las funciones del giroscopio MPU6500.
 * 
 * Utiliza la librería Wire para comunicación I2C.
 * Las variables globales gyroZoffset y yaw se definen en config.cpp.
 * 
 * El MPU6500 tiene los mismos registros que el MPU6050:
 * - WHO_AM_I = 0x70 (MPU6500), 0x68 (MPU6050), 0x71 (MPU9250).
 * - Registro 0x1B: configuración del giroscopio (0x00 = ±250°/s, factor 131.0).
 * - Registro 0x6B: gestión de energía (0x00 = despertar).
 */

#include "func_giro.h"
#include "config.h"
#include <Wire.h>

/**
 * @brief Escribe un byte en un registro del MPU.
 * @param r  Dirección del registro.
 * @param v  Valor a escribir.
 */
void mpuW(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

/**
 * @brief Lee un byte de un registro del MPU.
 * @param r  Dirección del registro.
 * @return   Valor leído (0 si falla).
 */
uint8_t mpuR(uint8_t r) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(r);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

/**
 * @brief Lee la velocidad angular en el eje Z (2 bytes, little-endian).
 * @return Valor entero de 16 bits (LSB). Se debe dividir por 131.0 para obtener °/s.
 */
int16_t mpuGz() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                     // Registro alto del eje Z (0x47 y 0x48)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}

/**
 * @brief Inicializa el MPU: configura pines I2C, velocidad, timeout y registros.
 *        Verifica la identidad del chip leyendo WHO_AM_I.
 * @return true si se detecta el dispositivo y se configura correctamente.
 */
bool mpuInit() {
  Wire.begin(21, 22);                   // SDA=21, SCL=22 (según conexión)
  Wire.setClock(400000);                // 400 kHz (Fast Mode)
  Wire.setTimeout(25);                  // Timeout de 25 ms

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

  // Despertar el MPU (salir del sleep)
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

/**
 * @brief Calibración del giroscopio: toma 1000 lecturas estáticas, calcula el offset.
 *        El offset se guarda en gyroZoffset (en LSB). Se descartan los valores
 *        extremos para mayor robustez.
 */
void calibrarGyro() {
  const int muestras = 1000;
  double acum = 0; // Alta precisión matemática en acumulación espacial

  Serial.print("Calibrando MPU6500 estáticamente... ");
  for (int i = 0; i < muestras; i++) {
    acum += mpuGz();
    delay(3);
  }

  // Se extrae la media aritmética real flotante
  gyroZoffset = (float)(acum / (double)muestras);       
  Serial.printf("Offset Fijo = %.4f LSB\n", gyroZoffset);
}

/**
 * @brief Actualiza el rumbo (yaw) integrando la velocidad angular.
 *        Debe llamarse periódicamente (cada ciclo del loop).
 *        Si el valor de velocidad es muy pequeño (< 0.2 °/s), se ignora para evitar ruido.
 */
void actualizarRumbo() {
  unsigned long n = micros();
  float dt = (n - tPrev) / 1000000.0;
  if (dt > 0.05) dt = 0.05; // Limitar dt drásticamente ante variaciones del bucle
  tPrev = n;

  int16_t raw = mpuGz();
  float gz = ((float)raw - gyroZoffset) / 131.0f; 

  // Zona muerta adaptativa a ruidos del chasis mecánico/eléctrico
  if (abs(gz) < 0.65f) {
    gz = 0.0f;
  }

  yaw += gz * dt;
}

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
