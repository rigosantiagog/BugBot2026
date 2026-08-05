/**
 * @file func_comunicacion.cpp
 * @brief Comunicación UART con la ESP32 del anillo.
 * 
 * Recibe la trama binaria (estructura TramaData) y envía el yaw actual como respuesta.
 * El checksum se calcula sobre todos los bytes de la trama (TAM_TRAMA = sizeof(TramaData)).
 * CORREGIDO: ahora el XOR se acumula antes de guardar el byte, para que todos los bytes
 * de datos entren al checksum (anteriormente el último byte se escapaba).
 */
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include "config.h"
#include "func_comunicacion.h"

// ----- Lectura de la trama del anillo -----
void leerUART() {
  static const uint8_t TAM_TRAMA = sizeof(TramaData); // Tamaño esperado de la trama (24 bytes)
  static uint8_t state = 0;             // Estado de la máquina de recepción
  static uint8_t buffer[sizeof(TramaData)];
  static uint8_t idx = 0;
  static uint8_t chkCalculado = 0;
  static unsigned long tTimeout = 0;    // Timeout para evitar bloqueos

  // Si pasan más de 50 ms sin datos, reinicia la máquina
  if (state != 0 && (millis() - tTimeout > 50)) {
    state = 0;
    idx = 0;
  }

  while (Enlace.available()) {
    uint8_t b = Enlace.read();          // Lee un byte
    tTimeout = millis();                // Actualiza el timeout

    switch (state) {
      case 0: // Buscando el primer byte de cabecera (0xAA)
        if (b == 0xAA) state = 1;
        break;
      case 1: // Buscando el segundo byte de cabecera (0x55)
        if (b == 0x55) {
          state = 2;                    // Cabecera confirmada, empezamos a leer datos
          idx = 0;
          chkCalculado = 0;
        } else {
          state = 0;                    // Si no es 0x55, reinicia
        }
        break;
      case 2: // Leyendo los TAM_TRAMA bytes de datos
        chkCalculado ^= b;              // XOR del byte (CORREGIDO: se calcula antes de guardar)
        buffer[idx] = b;
        idx++;
        if (idx >= TAM_TRAMA) {
          state = 3;                    // Ya tenemos todos los datos, esperamos el checksum
        }
        break;
      case 3: // Leyendo el checksum
        if (b == chkCalculado) {
          // ====================================================
          // Desempaquetar la trama
          // ====================================================
          TramaData tramaRecibida;
          memcpy(&tramaRecibida, buffer, sizeof(TramaData));

          anguloIR    = tramaRecibida.angulo;
          estadoIR    = tramaRecibida.estado;
          nIR         = tramaRecibida.totalActivos;
          distFrente  = tramaRecibida.distFrente;
          distAtras   = tramaRecibida.distAtras;
          distIzq     = tramaRecibida.distIzq;
          distDer     = tramaRecibida.distDer;
          ultimoDato  = millis();

          // Reconstruir la cadena de vecinos para debug
          recepVecinos = "";
          for (int i = 0; i < 16; i++) {
            if (i > 0) recepVecinos += ",";
            recepVecinos += (tramaRecibida.bitmapIR & (1 << i)) ? '1' : '0';
          }
          posicionRobotX = tramaRecibida.posX;
          posicionRobotY = tramaRecibida.posY;

          // ====================================================
          // RESPUESTA: Enviar el yaw actual al anillo
          // ====================================================
          uint8_t headerYaw[2] = {0x55, 0xAA}; // Cabecera invertida
          float yawActual = (float)yaw;        // Yaw del giroscopio
          uint8_t chkYaw = 0;
          uint8_t* bytePointer = (uint8_t*)&yawActual;

          // Calcula el checksum del float (4 bytes)
          for (int i = 0; i < 4; i++) chkYaw ^= bytePointer[i];

          // Envía la respuesta
          Enlace.write(headerYaw, 2);
          Enlace.write(bytePointer, 4);
          Enlace.write(chkYaw);
        }
        // Reinicia la máquina para buscar la siguiente trama
        state = 0;
        idx = 0;
        break;
    }
  }
}

// ----- Inicialización de OTA -----
void arduino_OTA() {
  ArduinoOTA.setHostname("BugBot-Motores");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Iniciando OTA: " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA finalizada.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progreso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error OTA[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Autenticación fallida");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Error al iniciar");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Error de conexión");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Error al recibir");
    else if (error == OTA_END_ERROR) Serial.println("Error al finalizar");
  });
  ArduinoOTA.begin();
  Serial.println("OTA listo.");
}

// ----- Envío de logs al servidor remoto (opcional) -----
void debugLog(const String& texto) {
  if (client.connected()) client.println(texto);
}