/**
 * @file Anillo-TX.ino
 * @brief ESP32 del anillo IR: lee 16 sensores TSSP58038, calcula el centroide de la pelota,
 *        mide distancias con 4 ultrasonidos HC-SR04 y envía una trama binaria por UART.
 * 
 * El protocolo de envío es binario con cabecera 0xAA 0x55, 16 bytes de datos y checksum XOR.
 */

#include <Arduino.h> 
#include <string.h>
#include "config.h"
#include "func_ultrasonicos.h"
#include "func_multiplexor.h"
#include "func_anillo.h"

/**
 * @brief Configuración inicial: UART, pines del multiplexor, lanzamiento de la tarea de ultrasonidos.
 */
void setup() {
  Serial.begin(115200);                          // Puerto serie para debug local
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN); // UART para enviar a la ESP32 de motores

  // Configura pines del multiplexor CD74HC4067
  pinMode(pinS0, OUTPUT); pinMode(pinS1, OUTPUT);
  pinMode(pinS2, OUTPUT); pinMode(pinS3, OUTPUT);
  pinMode(pinSIG, INPUT_PULLUP);                 // Señal común (salida del multiplexor)

  // Inicia la tarea de lectura de ultrasonidos en el Núcleo 0 (FreeRTOS)
  iniciarUltrasonicos();

  Serial.println("\n=== ESP32 ANILLO (TX) v11 (Binario) Listo ===");
}

/**
 * @brief Bucle principal: lee sensores, calcula ángulo y distancias, empaqueta y envía.
 */
void loop() {
  int totalActivos = 0;                          // Contador de sensores IR activos

  fotorreceptoresActivos(totalActivos);         // Lee los 16 sensores y actualiza activo[]

  int mejorLargo = ubicarPelota();              // Encuentra la cadena más larga de activos

  uint8_t estado = (mejorLargo > 0) ? 1 : 0;    // Estado: 1 si hay pelota, 0 si no

  // Construye el mapa de bits de los 16 sensores (bit i = 1 si activo[i] es true)
  uint16_t bitmapIR = 0;
  for (int i = 0; i < 16; i++) {
    if (activo[i]) bitmapIR |= (1 << i);
  }

  // --- Empaquetado de la trama binaria (16 bytes de datos) ---
  uint8_t data[16];
  int idx = 0;

  // 1. Ángulo (float, 4 bytes, little-endian)
  memcpy(&data[idx], (const void*)&angulo, sizeof(float));
  idx += 4;

  // 2. Estado (uint8_t)
  data[idx++] = estado;

  // 3. Número de activos (uint8_t)
  data[idx++] = (uint8_t)totalActivos;

  // 4. Distancias (uint16_t, 2 bytes cada una)
  uint16_t f = (uint16_t)distFrente;
  uint16_t b = (uint16_t)distAtras;
  uint16_t l = (uint16_t)distIzq;
  uint16_t r = (uint16_t)distDer;
  memcpy(&data[idx], (const void*) &f, sizeof(uint16_t)); idx += 2;
  memcpy(&data[idx], (const void*) &b, sizeof(uint16_t)); idx += 2;
  memcpy(&data[idx], (const void*) &l, sizeof(uint16_t)); idx += 2;
  memcpy(&data[idx], (const void*) &r, sizeof(uint16_t)); idx += 2;

  // 5. Mapa de bits de sensores (uint16_t)
  memcpy(&data[idx], &bitmapIR, sizeof(uint16_t)); idx += 2; // Debe ser 16

  // --- Cálculo del checksum XOR sobre los 16 bytes de datos ---
  uint8_t checksum = 0;
  for (int i = 0; i < 16; i++) {
    checksum ^= data[i];
  }

  // --- Envío por UART ---
  Enlace.write(0xAA);        // Cabecera 1
  Enlace.write(0x55);        // Cabecera 2
  Enlace.write(data, 16);    // Datos
  Enlace.write(checksum);    // Checksum

  // Debug local (monitor serie)
  Serial.printf("TX Bin -> A:%.1f C:%d N:%d RADAR[F:%d B:%d L:%d R:%d] BMP:0x%04X ",
                angulo, estado, totalActivos, distFrente, distAtras, distIzq, distDer, bitmapIR);
  Serial.print("[ ");
  for ( int i = 0 ; i < 16 ; i++ ){
    Serial.print(activo[i]);
    Serial.print(" , ");
  }
  Serial.println(" ] ");

  delay(50);   // Frecuencia de 20 Hz
}