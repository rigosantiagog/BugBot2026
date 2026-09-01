#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =========================================================================
// UART2 hacia la ESP32 de Motores (enlace bidireccional)
// =========================================================================
constexpr int RX_PIN = 35;   // Recibe el yaw (GPIO35 solo entrada)
constexpr int TX_PIN = 25;   // Envía la trama de sensores
extern HardwareSerial Enlace;

// =========================================================================
// Estructura de la trama que se envía a motores (DEBE SER IDÉNTICA EN AMBOS)
// =========================================================================
struct __attribute__((__packed__)) TramaData {
  float    angulo;        // Ángulo hacia la pelota (ajustado con OFFSET_FRENTE)
  uint8_t  estado;        // 1 si hay pelota, 0 si no
  uint8_t  totalActivos;  // Nº de sensores IR activos (filtrados)
  uint16_t distFrente;    // cm
  uint16_t distAtras;     // cm
  uint16_t distIzq;       // cm
  uint16_t distDer;       // cm
  uint16_t bitmapIR;      // bit i = 1 si sensor i detecta pelota
  float    posX;          // Coordenada X estimada (cm) o -999 si no válida
  float    posY;          // Coordenada Y estimada (cm) o -999 si no válida
};

// =========================================================================
// Multiplexor CD74HC4067
// =========================================================================
constexpr int pinS0 = 19, pinS1 = 18, pinS2 = 17, pinS3 = 16;
constexpr int pinSIG = 4;
constexpr int totalSensores = 16;
constexpr float GRADOS_POR_SENSOR = 22.5f;   // 360/16

// =========================================================================
// Sensores ultrasónicos HC‑SR04
// =========================================================================
constexpr int TRIG_F = 13, ECHO_F = 12;
constexpr int TRIG_B = 14, ECHO_B = 27;
constexpr int TRIG_L = 32, ECHO_L = 33;
constexpr int TRIG_R = 21, ECHO_R = 26;

constexpr int FILTRO_ULTRASONIDOS = 5;   // Promedio móvil

// =========================================================================
// Lectura del anillo IR: sobremuestreo por canal + filtro de persistencia
// =========================================================================
// NOTA DE CALIBRACIÓN — leer antes de tocar estos valores:
// La pelota oficial NO emite luz IR de forma continua: manda ráfagas de
// ~200us separadas por silencios, con un periodo total de ~833us (duty
// cycle ~24%). Eso significa que es NORMAL que un sensor "parpadee" aunque
// la pelota esté justo enfrente. Estas constantes existen para absorber ese
// parpadeo sin perder la detección real:
//
//   - Si con la pelota quieta y de frente el robot casi no la detecta:
//     sube IR_MUESTRAS_POR_CANAL, o baja IR_UMBRAL_ACTIVO.
//   - Si en cambio se activan sensores "fantasma" sin la pelota cerca
//     (reflejos, ruido eléctrico): sube IR_FALLOS_PARA_DECAER, o baja
//     IR_MUESTRAS_POR_CANAL / IR_INTERVALO_MUESTRA_US.
//
// Usa el print "[IR-FILTRO]" del monitor serie para ver en vivo el efecto
// de cada cambio antes de subir al robot a la cancha.

constexpr uint8_t  IR_MUESTRAS_POR_CANAL   = 3;    // Nº de lecturas rápidas por sensor en cada pasada del MUX
constexpr uint16_t IR_INTERVALO_MUESTRA_US = 40;   // Separación (us) entre esas lecturas dentro del mismo canal
constexpr uint8_t  IR_CONFIANZA_MAX        = 8;    // Techo del contador de confianza por sensor
constexpr uint8_t  IR_UMBRAL_ACTIVO        = 2;    // Confianza mínima para considerar el sensor activo
constexpr uint8_t  IR_FALLOS_PARA_DECAER   = 2;    // Fallos crudos SEGUIDOS antes de restar confianza
constexpr unsigned long IR_DEBUG_INTERVALO_MS = 250; // Cada cuánto se imprime el estado del filtro por el monitor serie

// =========================================================================
// Debug remoto por WiFi (conexión al servidor puerto_anillo.py)
// =========================================================================
constexpr char IP_SERVIDOR_LOGS[]   = "192.168.4.2";
constexpr uint16_t PUERTO_DEBUG_WIFI = 5000;

// =========================================================================
// Ajuste de orientación del anillo (offset en grados)
// =========================================================================
constexpr float OFFSET_FRENTE = 0.0f;   // Calibrar según montaje

// =========================================================================
// Variables compartidas entre tareas
// =========================================================================
extern volatile int distFrente, distAtras, distIzq, distDer;
extern volatile bool activo[16];
extern volatile float angulo;
extern volatile float yaw;          // Último yaw recibido (grados)
extern volatile float robotX;       // Coordenada calculada
extern volatile float robotY;

// Estructura auxiliar para la interrupción de los ultrasonidos
struct SensorEcho {
  uint8_t pin;
  volatile unsigned long t_inicio;
  volatile unsigned long t_fin;
  volatile bool listo;
};
extern SensorEcho sensoresEcho[4];
extern SemaphoreHandle_t semaforoEcho;

#endif
