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

void iniciarDebugWiFi();    // Prepara el cliente de debug (llamar UNA VEZ en setup, antes de esperarConexionDebugWiFi)

// Espera de forma ACOTADA (maximo timeoutMs) a que se conecte el servidor de logs,
// reintentando cada TIMEOUT_CONEXION_MS. Pensada para llamarse UNA VEZ en setup(),
// ANTES de inicializar motores/giroscopio, para asegurar que el log ya este
// grabando antes de que el robot se mueva. A diferencia de un while() sin limite,
// esta SIEMPRE regresa (conectado o no) despues de timeoutMs como maximo, para
// que el robot pueda arrancar aunque la laptop/puerto_anillo.py no este disponible.
// Devuelve true si logro conectar dentro del tiempo, false si se agoto el timeout.
bool esperarConexionDebugWiFi(unsigned long timeoutMs);

void atenderDebugWiFi();    // Mantiene la conexion al servidor de logs: UN reintento no bloqueante por llamada (llamar en cada loop)

#endif
