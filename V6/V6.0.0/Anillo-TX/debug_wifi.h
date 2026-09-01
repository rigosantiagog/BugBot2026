#ifndef DEBUG_WIFI_H
#define DEBUG_WIFI_H

#include <Arduino.h>
#include <WiFi.h>
#include <Print.h>

class DebugStream : public Print {
  public:
    size_t write(uint8_t caracter) override;
    size_t write(const uint8_t *buffer, size_t tamano) override;
    int printf(const char *format, ...) __attribute__((format(printf, 2, 3)));
};

extern DebugStream Debug;

void iniciarDebugWiFi();
void atenderDebugWiFi();

#endif