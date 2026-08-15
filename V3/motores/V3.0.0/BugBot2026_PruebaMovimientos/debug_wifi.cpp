// debug_wifi.cpp
#include "debug_wifi.h"
#include "config_robot.h"

// Cliente TCP que se conecta HACIA el servidor de logs (puerto_anillo.py) que corre
// en la laptop/PC. El ESP32 es quien inicia la conexion, no al reves.
static WiFiClient clienteLogs;

static unsigned long ultimoIntento = 0;                  // millis() del ultimo intento de conexion
constexpr unsigned long INTERVALO_RECONEXION = 3000;      // Reintenta cada 3 s si no esta conectado
constexpr int32_t TIMEOUT_CONEXION_MS = 500;               // Maximo que puede "congelarse" el loop al intentar conectar

DebugStream Debug;   // Instancia global usada en todo el proyecto en lugar de Serial directamente

/**
 * @brief Escribe un solo byte tanto al Serial (USB) como al servidor de logs, si hay conexion.
 */
size_t DebugStream::write(uint8_t caracter) {
  Serial.write(caracter);                     // El USB siempre recibe el debug, este o no conectado el WiFi
  if (clienteLogs.connected()) {
    clienteLogs.write(caracter);              // Se replica hacia puerto_anillo.py si ya hay conexion
  }
  return 1;
}

/**
 * @brief Escribe un bloque de bytes tanto al Serial como al servidor de logs (mas eficiente que byte a byte).
 */
size_t DebugStream::write(const uint8_t *buffer, size_t tamano) {
  Serial.write(buffer, tamano);
  if (clienteLogs.connected()) {
    clienteLogs.write(buffer, tamano);
  }
  return tamano;
}

/**
 * @brief Solo deja listo el mensaje inicial; el primer intento de conexion real ocurre
 *        en atenderDebugWiFi() dentro del loop, para no bloquear el arranque si
 *        puerto_anillo.py todavia no esta corriendo en la laptop.
 */
void iniciarDebugWiFi() {
  Serial.printf("Debug WiFi: se intentara conectar a %s:%d (puerto_anillo.py)\n",
                IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI);
}

/**
 * @brief Mantiene la conexion hacia el servidor de logs: si ya esta conectado no hace nada;
 *        si no, reintenta cada INTERVALO_RECONEXION ms (con un timeout corto para no
 *        congelar el loop por mucho tiempo si el servidor no esta disponible).
 *        Debe llamarse en cada vuelta del loop().
 */
void atenderDebugWiFi() {
  if (clienteLogs.connected()) return;   // Ya conectado, nada que hacer

  if (millis() - ultimoIntento < INTERVALO_RECONEXION) return;   // Aun no toca reintentar
  ultimoIntento = millis();


  while(!clienteLogs.connected()){

    Serial.println("Reintentando servidor logs...");
  if (clienteLogs.connect(IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI, TIMEOUT_CONEXION_MS)) {
    Serial.println("Conectado a servidor logs.");
  }
  // Si connect() devuelve false, simplemente se reintenta en el siguiente ciclo

  }
  
}
