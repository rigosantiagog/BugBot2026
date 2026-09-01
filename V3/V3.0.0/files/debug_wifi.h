// debug_wifi.h
#ifndef DEBUG_WIFI_H
#define DEBUG_WIFI_H

#include <Arduino.h>
#include <WiFi.h>
#include <Print.h>

// DebugStream: imprime siempre por Serial (USB) y, ademas, hacia el servidor de logs
// (puerto_anillo.py) si el ESP32 ya logro conectarse a el como cliente TCP.
// Al heredar de Print, print()/println()/printf() funcionan exactamente igual que con Serial;
// en el codigo solo hay que cambiar "Serial." por "Debug." y listo.
class DebugStream : public Print {
  public:
    size_t write(uint8_t caracter) override;                      // Escribe un solo byte
    size_t write(const uint8_t *buffer, size_t tamano) override;   // Escribe un bloque (mas eficiente)
};

extern DebugStream Debug;   // Instancia global: usar Debug.print / Debug.println / Debug.printf

void iniciarDebugWiFi();    // Prepara el cliente de debug (llamar UNA VEZ en setup, despues de WiFi.softAP)
void atenderDebugWiFi();    // Mantiene la conexion al servidor de logs: conecta/reconecta (llamar en cada loop)

#endif
