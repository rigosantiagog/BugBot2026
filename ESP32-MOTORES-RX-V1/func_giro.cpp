/**
 * @file func_giro.cpp
 * @brief Implementación de las funciones del giroscopio MPU6500.
 * 
 * Utiliza la librería Wire para comunicación I2C.
 * Las variables globales gyroZoffset y yaw se definen en config.cpp.
 */

#include "func_giro.h"
#include "config.h"
#include <Wire.h>

/**
 * @brief Escribe un valor en un registro del MPU.
 */
void mpuW(uint8_t r, uint8_t v) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

/**
 * @brief Lee los dos bytes del eje Z (velocidad angular).
 * @return Valor entero de 16 bits (sin procesar).
 */
int16_t mpuGz() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);               // Registro donde empieza el eje Z
  Wire.endTransmission(false);    // Repeated start
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  return (Wire.read() << 8) | Wire.read();
}

/**
 * @brief Inicializa el MPU: configura pines I2C, velocidad, timeout y registros.
 * @return true si se detecta el dispositivo, false en caso contrario.
 */
bool mpuInit() {
  Wire.begin(21, 22);              // SDA=21, SCL=22 (según conexión)
  Wire.setClock(400000);
  Wire.setTimeout(25);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuW(0x6B, 0x00);               // Despierta el MPU (salir del sleep)
  delay(100);
  mpuW(0x1B, 0x00);               // Escala ±250 °/s
  delay(50);
  return true;
}

/**
 * @brief Calcula el offset del giroscopio promediando 500 lecturas estáticas.
 *        Guarda el resultado en la variable global gyroZoffset.
 */
void calibrarGyro() {
  long s = 0;
  for (int i = 0; i < 500; i++) {
    s += mpuGz();
    delay(3);
  }
  gyroZoffset = (float)s / 500.0;
}

/**
 * @brief Actualiza el rumbo (yaw) integrando la velocidad angular.
 *        Debe llamarse periódicamente (en cada ciclo del loop).
 */
void actualizarRumbo() {
  unsigned long n = micros();
  float dt = (n - tPrev) / 1000000.0;   // Tiempo en segundos
  tPrev = n;
  float gz = (mpuGz() - gyroZoffset) / 131.0; // Conversión a °/s (escala ±250)
  yaw += gz * dt;
}

/**
 * @brief Normaliza un ángulo al rango [-180, 180] para encontrar el camino más corto.
 */
float errorAngular(float angulo) {
  while (angulo > 180) angulo -= 360;
  while (angulo < -180) angulo += 360;
  return angulo;
}