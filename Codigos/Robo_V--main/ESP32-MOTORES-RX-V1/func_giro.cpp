#include "func_giro.h"
#include "config.h"
#include <Wire.h>

/* ===================== FUNCIONES DEL SENSOR MPU I2C =========================== */
void mpuW(uint8_t r, uint8_t v) {    // Función para escribir en el MPU
  Wire.beginTransmission(MPU_ADDR);  // Llama al dispositivo MPU por la red I2C
  Wire.write(r);                     // Señala la posición de memoria interna a cambiar
  Wire.write(v);                     // Escribe el nuevo valor
  Wire.endTransmission();            // Cierra el paquete de datos
}

int16_t mpuGz() {                           // Función para extraer datos físicos del MPU
  Wire.beginTransmission(MPU_ADDR);         // Llama al MPU
  Wire.write(0x47);                         // Pide los datos de la rotación en el Eje Z
  Wire.endTransmission(false);              // Pausa la línea sin cerrarla
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);   // Exige que el MPU le devuelva 2 bytes de datos
  return (Wire.read() << 8) | Wire.read();  // Une los dos bytes y los devuelve a la matemática
}

bool mpuInit() {                                  // Función de arranque del giroscopio
  Wire.begin(21, 22);                             // Enciende los pines SDA y SCL de la ESP32
  Wire.setClock(400000);                          // Establece el protocolo a modo ultra rápido (Fast Mode)
  Wire.setTimeout(25);                            // Ordena a la ESP32 no bloquearse si los cables fallan o sufren ruido de motores
  Wire.beginTransmission(MPU_ADDR);               // Pregunta si hay alguien en la dirección del MPU
  if (Wire.endTransmission() != 0) return false;  // Si nadie responde, reporta falla
  mpuW(0x6B, 0x00);
  delay(100);  // Apaga el modo dormido del MPU
  mpuW(0x1B, 0x00);
  delay(50);    // Configura la precisión máxima del giroscopio
  return true;  // Reporta éxito
}

void calibrarGyro() {              // Rutina de autocalibración térmica
  long s = 0;                      // Acumulador muy grande
  for (int i = 0; i < 500; i++) {  // Toma 500 fotografías estáticas
    s += mpuGz();                  // Suma el valor erróneo del sensor
    delay(3);                      // Pequeña pausa entre fotos
  }
  gyroZoffset = (float)s / 500.0;  // Saca el promedio del error y lo guarda como compensación
}

void actualizarRumbo() {                       // Rutina que se llama en cada milisegundo del loop
  unsigned long n = micros();                  // Lee el reloj atómico interno en microsegundos
  float dt = (n - tPrev) / 1000000.0;          // Calcula cuántas fracciones de segundo pasaron desde la última vez
  tPrev = n;                                   // Actualiza la marca temporal
  float gz = (mpuGz() - gyroZoffset) / 131.0;  // Extrae el grado/segundo puro aplicando la compensación calibrada
  yaw += gz * dt;                              // Integra el cálculo sumándolo al rumbo absoluto general
}

float errorAngular(float angulo) {  // Rutina de la ruta más corta
  while (angulo > 180) angulo -= 360;                 // Si le pide dar casi una vuelta entera a la derecha, mejor gira a la izquierda
  while (angulo < -180) angulo += 360;                // Aplica lo mismo pero para la izquierda
  return angulo;                                 // Devuelve los grados exactos que se deben mover
}