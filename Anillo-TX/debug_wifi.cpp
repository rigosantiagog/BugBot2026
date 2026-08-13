// debug_wifi.cpp
#include "debug_wifi.h"
#include "config.h"

// Servidor TCP de depuracion: escucha en el puerto PUERTO_DEBUG_WIFI (definido en config.h)
static WiFiServer servidorDebug(PUERTO_DEBUG_WIFI);
static WiFiClient clienteDebug;   // Un solo cliente de debug a la vez, para no mezclar dos monitores

DebugStream Debug;   // Instancia global usada en todo el proyecto en lugar de Serial directamente

/**
 * @brief Escribe un solo byte tanto al Serial (USB) como al cliente WiFi conectado, si existe.
 */
size_t DebugStream::write(uint8_t caracter) {
  Serial.write(caracter);                         // El USB siempre recibe el debug, si esta conectado
  if (clienteDebug && clienteDebug.connected()) {
    clienteDebug.write(caracter);                  // Se replica por WiFi si hay un monitor conectado
  }
  return 1;
}

/**
 * @brief Escribe un bloque de bytes tanto al Serial como al cliente WiFi (mas eficiente que byte a byte).
 */
size_t DebugStream::write(const uint8_t *buffer, size_t tamano) {
  Serial.write(buffer, tamano);
  if (clienteDebug && clienteDebug.connected()) {
    clienteDebug.write(buffer, tamano);
  }
  return tamano;
}

/**
 * @brief Arranca el servidor TCP de depuracion. Debe llamarse despues de WiFi.softAP(),
 *        ya que necesita que la red WiFi este activa para poder escuchar conexiones.
 */
void iniciarDebugWiFi() {
  servidorDebug.begin();
  servidorDebug.setNoDelay(true);   // Envia cada linea de inmediato, sin esperar a llenar el buffer TCP
  Serial.printf("Debug WiFi listo -> conectate por telnet a %s puerto %d\n",
                WiFi.softAPIP().toString().c_str(), PUERTO_DEBUG_WIFI);
}

/**
 * @brief Revisa si llego un cliente nuevo (lo acepta si no hay otro activo) y libera
 *        al cliente actual si ya se desconecto. Debe llamarse en cada vuelta del loop().
 */
void atenderDebugWiFi() {
  // Si hay una conexion entrante y no tenemos un cliente activo, la tomamos
  if (servidorDebug.hasClient()) {
    if (!clienteDebug || !clienteDebug.connected()) {
      if (clienteDebug) clienteDebug.stop();        // Limpia restos de una conexion previa
      clienteDebug = servidorDebug.available();
      Serial.println("Cliente de debug WiFi conectado.");
    } else {
      // Ya hay un cliente activo: se rechaza el nuevo para no mezclar dos monitores a la vez
      WiFiClient rechazado = servidorDebug.available();
      rechazado.stop();
    }
  }

  // Si el cliente que teniamos se desconecto, lo liberamos
  if (clienteDebug && !clienteDebug.connected()) {
    clienteDebug.stop();
    Serial.println("Cliente de debug WiFi desconectado.");
  }
}
