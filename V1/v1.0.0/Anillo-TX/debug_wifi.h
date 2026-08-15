// debug_wifi.h
#ifndef DEBUG_WIFI_H
#define DEBUG_WIFI_H

#include <Arduino.h>
#include <WiFi.h>
#include <Print.h>

// DebugStream: imprime siempre por Serial (USB) y, ademas, por WiFi si hay un cliente conectado.
// Al heredar de Print, print()/println()/printf() funcionan exactamente igual que con Serial;
// en el codigo solo hay que cambiar "Serial." por "Debug." y listo.
class DebugStream : public Print {
  public:
    size_t write(uint8_t caracter) override;                      // Escribe un solo byte
    size_t write(const uint8_t *buffer, size_t tamano) override;   // Escribe un bloque (mas eficiente)
};

extern DebugStream Debug;   // Instancia global: usar Debug.print / Debug.println / Debug.printf

void iniciarDebugWiFi();    // Arranca el servidor TCP de debug (llamar UNA VEZ en setup, despues de WiFi.softAP)
void atenderDebugWiFi();    // Acepta clientes nuevos y libera los que se desconectaron (llamar en cada loop)

#endif
