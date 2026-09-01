#include "debug_wifi.h"
#include "config.h"
#include <cstdarg>
#include <cstdio>

static WiFiClient clienteLogs;
static unsigned long ultimoIntento = 0;
constexpr unsigned long INTERVALO_RECONEXION = 3000;
constexpr int32_t TIMEOUT_CONEXION_MS = 500;

DebugStream Debug;

// =========================================================================
// IMPLEMENTACIÓN DE DebugStream
// =========================================================================
size_t DebugStream::write(uint8_t caracter) {
  Serial.write(caracter);
  if (clienteLogs.connected()) {
    clienteLogs.write(caracter);
  }
  return 1;
}

size_t DebugStream::write(const uint8_t *buffer, size_t tamano) {
  Serial.write(buffer, tamano);
  if (clienteLogs.connected()) {
    clienteLogs.write(buffer, tamano);
  }
  return tamano;
}

int DebugStream::printf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[256];
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len > 0) {
    this->print(buffer);
  }
  return len;
}

// =========================================================================
// FUNCIONES PARA EL MANEJO DE LA CONEXIÓN AL SERVIDOR DE LOGS
// =========================================================================
void iniciarDebugWiFi() {
  Serial.printf("Debug WiFi: intentando conectar a %s:%d\n",
                IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI);
}

void atenderDebugWiFi() {
  if (clienteLogs.connected()) return;

  if (millis() - ultimoIntento < INTERVALO_RECONEXION) return;
  ultimoIntento = millis();

  Serial.println("Reintentando servidor logs...");
  if (clienteLogs.connect(IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI, TIMEOUT_CONEXION_MS)) {
    Serial.println("Conectado a servidor logs.");
  }
}