#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// UART2 hacia la ESP32 de Motores (enlace bidireccional)
// CORREGIDO: RX_PIN estaba en GPIO26, mismo pin que ECHO_R. Se reubicó a GPIO35 (solo entrada).
constexpr int RX_PIN = 35; // Recibe el yaw que regresa la ESP32 de motores
constexpr int TX_PIN = 25; // Envía la trama de sensores hacia la ESP32 de motores
extern HardwareSerial Enlace;

// Estructura de la trama binaria que se envía a la ESP32 de motores.
// Debe ser IDÉNTICA en ambos lados.
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

// Multiplexor CD74HC4067: selecciona 1 de 16 sensores IR TSSP58038.
// EN del multiplexor va cableado físicamente a GND.
constexpr int pinS0 = 19, pinS1 = 18, pinS2 = 17, pinS3 = 16;
constexpr int pinSIG = 4;
constexpr int totalSensores = 16;
constexpr float GRADOS_POR_SENSOR = 22.5; // 360° / 16

// Sensores ultrasónicos HC-SR04 (4 unidades)
constexpr int TRIG_F = 13, ECHO_F = 12;
constexpr int TRIG_B = 14, ECHO_B = 27;
constexpr int TRIG_L = 32, ECHO_L = 33;
constexpr int TRIG_R = 21, ECHO_R = 26;

// Filtro de promedio móvil para ultrasonidos
constexpr int FILTRO_ULTRASONIDOS = 5;

// Variables compartidas (entre tareas)
extern volatile int distFrente, distAtras, distIzq, distDer;
extern volatile bool activo[16];
extern volatile float angulo;
extern volatile float yaw;

// Estructura para manejar la interrupción de cada sensor de eco
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

// Constante para ajustar la orientación del anillo IR.
// Si el sensor 0 está apuntando al frente del robot, dejar 0.
// Si está apuntando atrás, poner 180.
// Puedes ajustarlo según tu montaje físico.
constexpr float OFFSET_FRENTE = 0.0f; 

#endif