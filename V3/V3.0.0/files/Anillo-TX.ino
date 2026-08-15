/**
 * @file Anillo-TX.ino
 * @brief ESP32 del anillo: lee 16 sensores IR, ultrasonidos, y usa func_comunicacion
 *        para armar/enviar la trama hacia la ESP32 de motores y recibir el yaw de vuelta.
 *        Incluye OTA y debug remoto por WiFi (Serial USB + WiFi al mismo tiempo).
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

// Credenciales WiFi (punto de acceso)
const char* ssid = "ESP32_DEBUG_ANILLO";
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

  // --- Debug remoto por WiFi: se arranca justo despues del softAP, ya con la red activa ---
  iniciarDebugWiFi();

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

  Debug.println("\n=== ANILLO v12 (OTA + Filtro + Debug WiFi + Comunicacion) Listo ===");
}

void loop() {
  ArduinoOTA.handle();     // Atender OTA
  atenderDebugWiFi();      // Mantiene/reintenta la conexion hacia el servidor de logs (puerto_anillo.py)

  recibirYaw();            // Procesa cualquier yaw entrante desde la ESP32 de motores (no bloqueante)

  int totalActivos = 0;
  fotorreceptoresActivos(totalActivos);       // Lectura cruda de los 16 sensores
  aplicarFiltroPersistenciaIR(totalActivos);  // Filtra activo[] en el lugar (debounce) y recalcula totalActivos
  int mejorLargo = ubicarPelota();
  uint8_t estado = (mejorLargo > 0) ? 1 : 0;

  uint16_t bitmapIR = obtenerBitmapIR();   // Mismo bitmap que arma enviarTramaMotores() por dentro

  // TODO: aun no se calcula la posicion del robot en la cancha (localizacion).
  // Se envia en 0.0 mientras se implementa; no afecta al resto de la trama.
  float robotX = 0.0f;
  float robotY = 0.0f;

  // Arma y envia la trama completa (24 bytes) hacia la ESP32 de motores
  enviarTramaMotores(angulo, estado, totalActivos,
                      (uint16_t)distFrente, (uint16_t)distAtras,
                      (uint16_t)distIzq, (uint16_t)distDer,
                      robotX, robotY);

  
  
  static unsigned long tiempo = 0;
  if(millis() - tiempo > 500) {
    tiempo = millis();
    // Debug local (Serial USB + WiFi al mismo tiempo)
    const char* orientacion = obtenerOrientacionCardinal(angulo);   // "N","NE","E","SE","S","SO","O","NO" o "-"

    // OJO: antes esto se armaba con String + String(x) + "\n" + ... (14 concatenaciones).
    // Cada "+" de String crea un objeto temporal nuevo en el heap; en esta prueba se vio
    // la RAM libre caer de ~204,000 a 2,047 bytes en un solo ciclo (se recupero sola, pero
    // es un riesgo real de quedarse sin memoria y reiniciar solo en medio de un partido).
    // Con snprintf() a un buffer fijo en el stack no se toca el heap para nada.
    char debugInfo[320];
    snprintf(debugInfo, sizeof(debugInfo),
        "Tiempo: %lu\n"
        "Memoria RAM: %u\n"
        "Memoria FLASH: %u\n"
        "Angulo: %.2f\n"
        "Orientacion: %s\n"
        "Estado: %d\n"
        "N Activos: %d\n"
        "Distancia Frente: %d\n"
        "Distancia Atras: %d\n"
        "Distancia Izquierda: %d\n"
        "Distancia Derecha: %d\n"
        "Mejor largo: %d\n"
        "Coord X: %.2f\n"
        "Coord Y: %.2f\n",
        millis(), ESP.getFreeHeap(), ESP.getFlashChipSize(),
        angulo, orientacion, estado, totalActivos,
        distFrente, distAtras, distIzq, distDer,
        mejorLargo, robotX, robotY);

    Debug.print(debugInfo);
    imprimirArregloSensores(bitmapIR);   // Imprime "BMP:[0,0,1,...]" con salto de linea incluido
  }


  delay(50);
}
