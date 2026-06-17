/**
 * @file Anillo-TX.ino
 * @brief ESP32 del anillo: lee 16 sensores IR, ultrasonidos, envía trama binaria.
 *        Incluye OTA.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "func_ultrasonicos.h"
#include "func_multiplexor.h"
#include "func_anillo.h"

// Credenciales WiFi (punto de acceso)
const char* ssid = "ESP32_DEBUG";
const char* password = "12345678";

void setup() {
  Serial.begin(115200);
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(pinS0, OUTPUT); pinMode(pinS1, OUTPUT);
  pinMode(pinS2, OUTPUT); pinMode(pinS3, OUTPUT);
  pinMode(pinSIG, INPUT_PULLUP);

  // Inicializar pines de ultrasonidos
  inicializarPinesUltrasonidos();
  iniciarUltrasonicos();

  // --- WiFi y OTA ---
  WiFi.softAP(ssid, password);
  Serial.print("IP anillo: ");
  Serial.println(WiFi.softAPIP());

  ArduinoOTA.setHostname("BugBot-Anillo");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.onStart([]() {
    Serial.println("Iniciando OTA en anillo...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA anillo finalizada.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progreso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error OTA anillo[%u]\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA listo en el anillo.");

  Serial.println("\n=== ANILLO v11 (OTA + Filtro) Listo ===");
}

void loop() {
  ArduinoOTA.handle();   // Atender OTA

  int totalActivos = 0;
  fotorreceptoresActivos(totalActivos);
  int mejorLargo = ubicarPelota();
  uint8_t estado = (mejorLargo > 0) ? 1 : 0;

  uint16_t bitmapIR = 0;
  for (int i = 0; i < 16; i++) {
    if (activo[i]) bitmapIR |= (1 << i);
  }

  // Empaquetado binario (16 bytes)
  uint8_t data[16];
  int idx = 0;
  memcpy(&data[idx], (const void*)&angulo, sizeof(float)); idx += 4;
  data[idx++] = estado;
  data[idx++] = (uint8_t)totalActivos;
  uint16_t f = (uint16_t)distFrente;
  uint16_t b = (uint16_t)distAtras;
  uint16_t l = (uint16_t)distIzq;
  uint16_t r = (uint16_t)distDer;
  memcpy(&data[idx], &f, sizeof(uint16_t)); idx += 2;
  memcpy(&data[idx], &b, sizeof(uint16_t)); idx += 2;
  memcpy(&data[idx], &l, sizeof(uint16_t)); idx += 2;
  memcpy(&data[idx], &r, sizeof(uint16_t)); idx += 2;
  memcpy(&data[idx], &bitmapIR, sizeof(uint16_t)); idx += 2;

  uint8_t checksum = 0;
  for (int i = 0; i < 16; i++) checksum ^= data[i];

  Enlace.write(0xAA);
  Enlace.write(0x55);
  Enlace.write(data, 16);
  Enlace.write(checksum);

  // Debug local
  Serial.printf("TX -> A:%.1f C:%d N:%d RADAR[F:%d B:%d L:%d R:%d] BMP:0x%04X\n",
                angulo, estado, totalActivos, distFrente, distAtras, distIzq, distDer, bitmapIR);

  delay(50);
}