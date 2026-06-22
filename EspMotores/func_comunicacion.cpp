#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include "config.h"
#include "func_comunicacion.h"
//UART
void leerUART() {
  static uint8_t state = 0;
  static uint8_t buffer[24]; // Sigue siendo de 24 bytes (el tamaño de TramaData)
  static uint8_t idx = 0;
  static uint8_t chkCalculado = 0;
  static unsigned long tTimeout = 0;

  // Timeout de seguridad de 50ms por si se corta la comunicación
  if (state != 0 && (millis() - tTimeout > 50)) {
    state = 0;
    idx = 0;
  }

  while (Enlace.available()) {
    uint8_t b = Enlace.read();
    tTimeout = millis();

    switch (state) {
      case 0:
        if (b == 0xAA) state = 1;
        break;
      case 1:
        if (b == 0x55) {
          state = 2;
          idx = 0;
          chkCalculado = 0;
        } else {
          state = 0;
        }
        break;
      case 2:
        buffer[idx++] = b;
        if (idx < 24) {
          chkCalculado ^= b;
        } else {
          state = 3;
        }
        break;
      case 3:
        if (b == chkCalculado) {
          // ====================================================
          // OPTIMIZACIÓN: Desempaquetar todo en una sola copia
          // ====================================================
          TramaData tramaRecibida;
          memcpy(&tramaRecibida, buffer, sizeof(TramaData));

          // Asignar los datos directamente a tus variables globales de juego
          anguloIR    = tramaRecibida.angulo;
          estadoIR    = tramaRecibida.estado;
          nIR         = tramaRecibida.totalActivos;
          distFrente  = tramaRecibida.distFrente;
          distAtras   = tramaRecibida.distAtras;
          distIzq     = tramaRecibida.distIzq;
          distDer     = tramaRecibida.distDer;
          ultimoDato  = millis();

          // Reconstruir tu string de vecinos para el debug visual
          recepVecinos = "";
          for (int i = 0; i < 16; i++) {
            if (i > 0) recepVecinos += ",";
            recepVecinos += (tramaRecibida.bitmapIR & (1 << i)) ? '1' : '0';
          }
          posicionRobotX = tramaRecibida.posX;
          posicionRobotY = tramaRecibida.posY;
          

          // ====================================================
          // RESPUESTA RÁPIDA: Enviar el Yaw del MPU al Anillo
          // ====================================================
          uint8_t headerYaw[2] = {0x55, 0xAA}; // Cabecera de telemetría invertida
          float yawActual = (float)yaw;        // Ángulo actual del giroscopio
          uint8_t chkYaw = 0;
          
          // Apuntamos directamente a los bytes del float en RAM (Cero memcpy al enviar)
          uint8_t* bytePointer = (uint8_t*)&yawActual;

          // Calcular Checksum del float sobre sus 4 bytes reales
          for(int i = 0; i < 4; i++) chkYaw ^= bytePointer[i];

          // Mandar ráfaga síncrona inmediata por el cable
          Enlace.write(headerYaw, 2);
          Enlace.write(bytePointer, 4);
          Enlace.write(chkYaw);
        }
        state = 0;
        idx = 0;
        break;
    }
  }
}

//OTA
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
    Serial.println("OTA listo. Conéctate a la red ESP32_DEBUG para actualizar.");
}

//Debug
void debugLog(const String& texto) {
  if (client.connected()) client.println(texto);
}
