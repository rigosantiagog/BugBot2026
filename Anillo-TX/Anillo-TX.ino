/**
 * @file Anillo-TX.ino
 * @brief ESP32 del anillo: lee 16 sensores IR, ultrasonidos, envía trama binaria.
 *        Incluye OTA y ajuste de orientación del anillo.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "func_ultrasonicos.h"
#include "func_multiplexor.h"
#include "func_anillo.h"
#include "func_comunicacion.h"

// Credenciales WiFi (punto de acceso)
const char* ssid = "ESP32_DEBUG_ANILLO";
const char* password = "12345678";

WiFiClient client;
const char* ipPC = "192.168.4.2";
const int puerto = 5000;

void wifi_setup() {
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
  Serial.println("\n=== ANILLO v11 (OTA + Estructuras + Offset) Listo ===");
}

void debugLog(const String& texto) {
  if (client.connected()) client.println(texto);
}

void setup() {
  Serial.begin(115200);
  Enlace.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(pinS0, OUTPUT); pinMode(pinS1, OUTPUT);
  pinMode(pinS2, OUTPUT); pinMode(pinS3, OUTPUT);
  pinMode(pinSIG, INPUT_PULLUP);

  iniciarUltrasonicos();
  //wifi_setup();

  // Conexión opcional a servidor de logs (no bloqueante)
  /*Serial.println("Buscando servidor de logs (opcional)...");
  while (!client.connect(ipPC, puerto)) {
    Serial.println("Reintentando servidor logs...");
  }
  if (client.connected()) {
    Serial.println("Conectado a servidor logs.");
  } else {
    Serial.println("AVISO: sin servidor de logs. Continuando sin debug remoto.");
  }*/
}

void loop() {
  ArduinoOTA.handle();

  // Leer el anillo de 16 sensores IR
  int totalActivos = 0;
  fotorreceptoresActivos(totalActivos);

  // Calcular ángulo de la pelota (cadena más larga)
  int mejorLargo = ubicarPelota();
  uint8_t estado = (mejorLargo > 0) ? 1 : 0;

  // Enviar la trama completa a motores (el ángulo se ajusta con OFFSET_FRENTE)
  enviarTramaMotores(angulo, estado, totalActivos,
                     distFrente, distAtras, distIzq, distDer,
                     robotX, robotY);

  // Recibir la respuesta de motores (yaw)
  recibirYaw();

  // Debug local
  Serial.printf("TX -> A:%.1f C:%d N:%d RADAR[F:%d B:%d L:%d R:%d] BMP:0x%04X\n",
                angulo, estado, totalActivos,
                distFrente, distAtras, distIzq, distDer,
                obtenerBitmapIR());

  // Debug remoto (cada 100 ms)
  static unsigned long tiempo = 0;
  if (millis() - tiempo > 100) {
    tiempo = millis();
    debugLog(
      "Tiempo: " + String(millis()) + "\n" +
      "Memoria RAM: " + String(ESP.getFreeHeap()) + "\n" +
      "Angulo: " + String(angulo) + "\n" +
      "Estado: " + String(estado) + "\n" +
      "N Activos: " + String(totalActivos) + "\n" +
      "DistF: " + String(distFrente) + "\n" +
      "DistB: " + String(distAtras) +"\n" + 
      "DistL: " + String(distIzq) + "\n" +
      "DistR: " + String(distDer) + "\n" +
      "BitmapIR: " + String(obtenerBitmapIR()) + "\n" +
      "Coord X: " + String(robotX) + "\n" +
      "Coord Y: " + String(robotY) + "\n"
    );
  }

  //delay(10);
}