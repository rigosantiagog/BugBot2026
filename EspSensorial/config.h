#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// UART
constexpr int RX_PIN = 26;
constexpr int TX_PIN = 25;
extern HardwareSerial Enlace;

struct __attribute__((__packed__)) TramaData {
  float angulo;
  uint8_t estado;
  uint8_t totalActivos;
  uint16_t distFrente;
  uint16_t distAtras;
  uint16_t distIzq;
  uint16_t distDer;
  uint16_t bitmapIR;
  float posX;
  float posY;
};

// Multiplexor
constexpr int pinS0 = 19, pinS1 = 18, pinS2 = 17, pinS3 = 16;
constexpr int pinSIG = 4;
constexpr int totalSensores = 16;
constexpr float GRADOS_POR_SENSOR = 22.5;

// Ultrasonidos
constexpr int TRIG_F = 13, ECHO_F = 12;
constexpr int TRIG_B = 14, ECHO_B = 27;
constexpr int TRIG_L = 32, ECHO_L = 33;
constexpr int TRIG_R = 21, ECHO_R = 26;

// Filtro
constexpr int FILTRO_ULTRASONIDOS = 5;

// Variables compartidas
extern volatile int distFrente, distAtras, distIzq, distDer;
extern volatile bool activo[16];
extern volatile float angulo;
extern volatile float yaw;

// Estructura para manejar la interrupción de cada sensor
struct SensorEcho {
  uint8_t pin;
  volatile unsigned long t_inicio;
  volatile unsigned long t_fin;
  volatile bool listo;
};

extern SensorEcho sensoresEcho[4];
extern SemaphoreHandle_t semaforoEcho;
extern volatile float robotX;
extern volatile float robotY;

#endif
