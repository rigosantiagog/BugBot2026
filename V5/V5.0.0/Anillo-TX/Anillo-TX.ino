/**
 * @file Anillo-TX.ino
 * @brief ESP32 del anillo: lee 16 sensores IR (con filtro), ultrasonidos (con interrupciones),
 *        calcula coordenadas absolutas y envía trama a la ESP32 de motores.
 *        Incluye OTA y debug WiFi hacia puerto_anillo.py.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "func_ultrasonicos.h"
#include "func_multiplexor.h"
#include "func_anillo.h"
#include "func_comunicacion.h"
#include "debug_wifi.h"

// Credenciales del punto de acceso
const char* ssid     = "ESP32_DEBUG_ANILLO";
const char* password = "12345678";

void setup() {
  Serial.begin(115200);
  Enlace.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Pines del multiplexor y señal IR
  pinMode(pinS0, OUTPUT); pinMode(pinS1, OUTPUT);
  pinMode(pinS2, OUTPUT); pinMode(pinS3, OUTPUT);
  pinMode(pinSIG, INPUT_PULLUP);

  // Lanzar tarea de ultrasonidos (Core 0)
  iniciarUltrasonicos();

  // ===== WiFi + OTA =====
  WiFi.softAP(ssid, password);
  Serial.print("IP anillo: ");
  Serial.println(WiFi.softAPIP());

  iniciarDebugWiFi();   // Solo prepara, la conexión se hará en el loop

  ArduinoOTA.setHostname("BugBot-Anillo");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.onStart([]() {
    Debug.println("Iniciando OTA en anillo...");
  });
  ArduinoOTA.onEnd([]() {
    Debug.println("\nOTA anillo finalizada.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Debug.printf("Progreso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Debug.printf("Error OTA anillo[%u]\n", error);
  });
  ArduinoOTA.begin();
  Debug.println("OTA listo en el anillo.");

  Debug.println("\n=== ANILLO UNIFICADO v1.0 ===");
}

void loop() {
  ArduinoOTA.handle();
  atenderDebugWiFi();          // Reintenta la conexión al servidor de logs

  recibirYaw();                // Procesa el yaw entrante (no bloqueante)

  // ---- Lectura y filtrado del anillo IR ----
  int totalActivos = 0;
  fotorreceptoresActivos(totalActivos);             // Lectura cruda
  aplicarFiltroPersistenciaIR(totalActivos);        // Filtra activo[] y actualiza totalActivos
  int mejorLargo = ubicarPelota();                  // Calcula angulo y devuelve largo de cadena
  uint8_t estado = (mejorLargo > 0) ? 1 : 0;

  // ---- Envío de la trama a motores ----
  uint16_t bitmapIR = obtenerBitmapIR();            // Usa activo[] ya filtrado
  enviarTramaMotores(angulo, estado, totalActivos,
                     distFrente, distAtras,
                     distIzq, distDer,
                     robotX, robotY);

  // ---- Debug local (cada 500 ms) ----
  static unsigned long tiempo = 0;
  String DebugInfo = "";
  if (millis() - tiempo > 500) {
    tiempo = millis();

    const char* orientacion = obtenerOrientacionCardinal(angulo);

    DebugInfo = 
    "Tiempo: " + String(millis()) + "\n" +
    "RAM: " + String(ESP.getFreeHeap()) + ", Flash: " + String(ESP.getFlashChipSize()) + "\n" +
    "Angulo: " + String(angulo) + "°" + String(orientacion) + "\n" +
    "Yaw IMU: " + String(yaw) + "\n" +
    "Estado: " + String(estado) + ", Activos: " + String(totalActivos) + "\n" +
    "Dist: Frente = " + String(distFrente) + " Atras = " + String(distAtras) + "\n" + 
    "Dist: Izquierda = " + String(distIzq) + " Derecha = " + String(distDer) + "\n" +
    "Coord: X = " + String(robotX) + ", Y = " + String(robotY) + "\n";
    Debug.print(DebugInfo);
    imprimirArregloSensores(bitmapIR);
  }

  delay(10);
}