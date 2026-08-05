#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================
//  PARÁMETROS DE JUEGO
// ============================================================

constexpr float FRENTE_ANILLO = 0.0f;          // Offset del anillo (0 si sensor 0 apunta al frente)
constexpr float TOL_APUNTADO = 40.0f;          // Grados de tolerancia para considerar la pelota encuadrada
constexpr unsigned long T_QUIETO_MS = 4000;    // Tiempo de espera post-calibración (ms)
constexpr int N_CAPTURA = 4;                   // Mínimo de sensores IR activos para captura (aumentado de 3)
constexpr float YAW_PORTERIA = 0.0f;           // Rumbo hacia la portería rival (grados, fijado en calibración)
constexpr float TOL_PORTERIA = 15.0f;          // Tolerancia angular para considerar el robot alineado

// ============================================================
//  PINES DE CONTROL DE MOTORES (TB6612FNG x2)
// ============================================================

constexpr uint8_t STBY = 4;                    // Habilitación global (LOW = motores apagados)
constexpr uint8_t FL_PWM = 25, FL_IN1 = 26, FL_IN2 = 27; // Motor frontal izquierdo
constexpr uint8_t RL_PWM = 33, RL_IN1 = 32, RL_IN2 = 14; // Motor trasero izquierdo
constexpr uint8_t FR_PWM = 13, FR_IN1 = 23, FR_IN2 = 2;  // Motor frontal derecho
constexpr uint8_t RR_PWM = 19, RR_IN1 = 18, RR_IN2 = 5;  // Motor trasero derecho

// Inversión de giro (true si la rueda gira al revés de lo esperado)
constexpr bool INVERTIR_FL = false, INVERTIR_FR = false, INVERTIR_RL = false, INVERTIR_RR = true;

constexpr uint32_t PWM_FREQ = 20000;           // 20 kHz (por encima del rango audible)
constexpr uint8_t PWM_RES = 8;                 // 8 bits de resolución (0-255)

// ============================================================
//  PARÁMETROS DE VELOCIDAD
// ============================================================

constexpr int PWM_MAX = 182;                   // Límite máximo de PWM (seguridad)
constexpr int VEL_AVANCE = 120;                // Velocidad de avance al perseguir o regresar
constexpr int VEL_GIRO = 85;                   // Velocidad máxima de giro en sitio
constexpr int VEL_BUSCAR = 80;                 // Velocidad en el patrón de búsqueda
constexpr int RAMPA_PASO = 4;                  // Incremento/decremento de PWM por ciclo (aceleración suave)
constexpr unsigned long T_CONFIRMA_MS = 500;   // Tiempo de estabilidad antes de avanzar

// ============================================================
//  ULTRASONIDOS – Distancia de frenado para remate
// ============================================================

constexpr int DIST_FRENADO_CM = 20;            // Distancia a la que el robot frena en portería (cm)

// ============================================================
//  CONTROL DE GIRO EN REGRESANDO (Control PI mejorado)
// ============================================================

constexpr float K_GIRO = 2.8f;                 // Ganancia proporcional (más agresivo)
constexpr float K_I_GIRO = 0.15f;              // Ganancia integral (elimina error residual)
constexpr float SATURACION_I = 30.0f;          // Límite del término integral (evita saturación)
constexpr int GIRO_MIN_PWM = 15;               // PWM mínimo para vencer la fricción
constexpr unsigned long T_GIRO_TIMEOUT = 3000; // Tiempo máximo de giro sin progreso (ms)

// ============================================================
//  NUEVOS: TIMEOUTS PARA DETECTAR ATASCOS EN REGRESANDO
// ============================================================

constexpr unsigned long T_AVANCE_SIN_PROGRESO = 3000; // Tiempo sin que distFrente disminuya (ms)
constexpr unsigned long T_YAW_SIN_CAMBIO = 3000;      // Tiempo sin que el Yaw cambie (ms)

// ============================================================
//  VARIABLES MUTABLES (definidas en config.cpp)
// ============================================================

extern volatile int velAvanceActual;           // Velocidad actual con rampa
extern volatile unsigned long tSenalEstable;   // Tiempo de señal estable
extern volatile float Correccion;              // Corrección angular para avanzar

// ============================================================
//  COMUNICACIÓN UART (Anillo → Motores) – RX=34, TX=17
// ============================================================

constexpr uint8_t RX_PIN = 34;                 // Pin de recepción UART2 (GPIO34, solo entrada)
constexpr uint8_t TX_PIN = 17;                 // Pin de transmisión UART2 (GPIO17)
extern HardwareSerial Enlace;

// Variables decodificadas de la trama del anillo
extern volatile float anguloIR;                // Ángulo de la pelota (grados)
extern volatile int estadoIR;                  // 1 = pelota detectada, 0 = ausente
extern volatile int nIR;                       // Número de sensores IR activos
extern volatile unsigned long ultimoDato;      // Timestamp de la última trama válida (ms)
extern volatile int distFrente, distAtras, distIzq, distDer; // Distancias ultrasónicas (cm)

// Estructura de la trama recibida (debe coincidir con la del anillo)
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

// ============================================================
//  NAVEGACIÓN – GIROSCOPIO MPU6500 (I2C)
// ============================================================

constexpr uint8_t MPU_ADDR = 0x68;             // Dirección I2C del MPU6500
extern volatile float yaw;                     // Rumbo acumulado (grados)
extern volatile float gyroZoffset;             // Offset del giroscopio (LSB)
extern volatile unsigned long tPrev;           // Tiempo anterior para integración (microsegundos)

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================

enum EstadoRobot {
  ESPERANDO_PELOTA,  // Bloqueo inicial
  BUSCANDO,          // Patrón de búsqueda
  PERSIGUIENDO,      // Persiguiendo la pelota
  FRENANDO,          // Frenado breve tras captura
  REGRESANDO         // Yendo a portería
};

extern volatile EstadoRobot estadoActual;
extern volatile unsigned long tFrenoIniciado;   // Momento de inicio del frenado (ms)
extern volatile unsigned long tUltimaVezPelota; // Última vez que se vio la pelota (ms)
extern volatile int pasoBusqueda;               // Paso actual del patrón de búsqueda (0-7)
extern volatile unsigned long tBusqueda;        // Temporizador del paso de búsqueda (ms)
extern volatile bool pelotaPerdidaReciente;     // Indica que la pelota se acaba de perder
extern String recepVecinos;                     // String con el estado de los 16 sensores (debug)

// ============================================================
//  LOCALIZACIÓN (recibida por UART, no se usa directamente)
// ============================================================

extern volatile float posicionRobotX;
extern volatile float posicionRobotY;

// ============================================================
//  WiFi / DEBUG REMOTO
// ============================================================

extern WebServer server;
extern WiFiClient client;
constexpr char* ssid = "ESP32_DEBUG_MOTORES";
constexpr char* password = "12345678";
constexpr char* ipPC = "192.168.4.2";
constexpr int puerto = 5000;
#endif