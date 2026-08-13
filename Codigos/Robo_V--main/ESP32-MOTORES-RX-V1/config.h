
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
//  PARÁMETROS DE JUEGO
// ============================================================

constexpr float FRENTE_ANILLO = 0.0f;        // Ángulo del chasis que apunta al frente de ataque (grados)
constexpr float TOL_APUNTADO = 40.0f;        // Tolerancia de encuadre antes de avanzar hacia la pelota (grados)
constexpr unsigned long T_QUIETO_MS = 4000;  // Duración del bloqueo inicial para estabilizar el MPU (ms)
constexpr int N_CAPTURA = 3;                 // Mínimo de sensores IR activos para confirmar posesión

constexpr float YAW_PORTERIA = 0.0f;   // Rumbo de la portería rival según el norte calibrado (grados)
constexpr float TOL_PORTERIA = 15.0f;  // Tolerancia de rumbo aceptable al avanzar a portería (grados)

// ============================================================
//  PINES DE CONTROL DE MOTORES
// ============================================================

constexpr uint8_t STBY = 4;  // Habilitación global de los drivers TB6612FNG (LOW = bloqueado)

constexpr uint8_t FL_PWM = 25;  // Velocidad   — Motor Frontal Izquierdo
constexpr uint8_t FL_IN1 = 26;  // Dirección 1 — Motor Frontal Izquierdo
constexpr uint8_t FL_IN2 = 27;  // Dirección 2 — Motor Frontal Izquierdo

constexpr uint8_t RL_PWM = 33;  // Velocidad   — Motor Trasero Izquierdo
constexpr uint8_t RL_IN1 = 32;  // Dirección 1 — Motor Trasero Izquierdo
constexpr uint8_t RL_IN2 = 14;  // Dirección 2 — Motor Trasero Izquierdo

constexpr uint8_t FR_PWM = 13;  // Velocidad   — Motor Frontal Derecho
constexpr uint8_t FR_IN1 = 23;  // Dirección 1 — Motor Frontal Derecho
constexpr uint8_t FR_IN2 = 2;   // Dirección 2 — Motor Frontal Derecho

constexpr uint8_t RR_PWM = 19;  // Velocidad   — Motor Trasero Derecho
constexpr uint8_t RR_IN1 = 18;  // Dirección 1 — Motor Trasero Derecho
constexpr uint8_t RR_IN2 = 5;   // Dirección 2 — Motor Trasero Derecho

constexpr bool INVERTIR_FL = false;  // Inversión de giro — Frontal Izquierdo
constexpr bool INVERTIR_FR = false;  // Inversión de giro — Frontal Derecho
constexpr bool INVERTIR_RL = false;  // Inversión de giro — Trasero Izquierdo
constexpr bool INVERTIR_RR = true;   // Inversión de giro — Trasero Derecho

constexpr uint32_t PWM_FREQ = 20000;  // Frecuencia PWM (Hz) — 20 kHz evita ruido audible
constexpr uint8_t PWM_RES = 8;        // Resolución PWM — 8 bits (0–255)

// ============================================================
//  PARÁMETROS DE VELOCIDAD
// ============================================================

constexpr int PWM_MAX = 182;     // Límite absoluto de PWM para proteger los motores Faulhaber
constexpr int VEL_AVANCE = 120;  // Potencia de ataque directo a la pelota
constexpr int VEL_GIRO = 85;     // Potencia de rotación sobre el eje propio
constexpr int VEL_BUSCAR = 80;   // Potencia durante el patrón de búsqueda
constexpr int RAMPA_PASO = 4;    // Incremento de PWM por ciclo de loop (aceleración suave)

constexpr unsigned long T_CONFIRMA_MS = 500;  // Tiempo mínimo de señal continua antes de atacar (ms)

// Variables mutables de velocidad — definidas en config.cpp
extern volatile int velAvanceActual;          // Valor actual de la rampa de aceleración
extern volatile unsigned long tSenalEstable;  // Marca de tiempo desde que la pelota entró al cono frontal

// ============================================================
//  COMUNICACIÓN UART (Anillo IR → ESP32)
// ============================================================

constexpr uint8_t RX_PIN = 34;  // Pin de entrada de datos desde el anillo IR
constexpr uint8_t TX_PIN = 17;  // Pin de salida serial (sin uso físico actualmente)

// Objetos y variables mutables de UART — definidos en config.cpp
extern HardwareSerial Enlace;  // Puerto Serial 2 por hardware
extern char uartBuffer[64];    // Buffer de recepción de tramas de texto
extern volatile int bufIndex;  // Posición actual de escritura dentro del buffer

extern volatile float anguloIR;            // Ángulo de la pelota recibido por UART (grados); -1.0 = sin dato
extern volatile int estadoIR;              // Presencia de pelota: 1 = detectada, 0 = ausente
extern volatile int nIR;                   // Cantidad de sensores IR activos en la trama recibida
extern volatile unsigned long ultimoDato;  // Tiempo del último paquete válido recibido (ms)

// ============================================================
//  NAVEGACIÓN — GIROSCOPIO MPU
// ============================================================

constexpr uint8_t MPU_ADDR = 0x68;  // Dirección I2C del MPU6050

// Variables mutables de navegación — definidas en config.cpp
extern volatile float yaw;            // Rumbo acumulado del robot en la cancha (grados)
extern volatile float gyroZoffset;    // Compensación de drift del eje Z (se calcula en calibrarGyro)
extern volatile unsigned long tPrev;  // Marca de tiempo del ciclo anterior para integración de ángulo

// ============================================================
//  NAVEGACION — ULTRASONICOS
// ============================================================

constexpr int DIST_SEGURIDAD_CM = 25;  // Distancia límite a la pared para frenar el carro (Ajustable)
extern volatile int distFrente;
extern volatile int distAtras;
extern volatile int distIzq;
extern volatile int distDer;

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================

enum EstadoRobot {
  ESPERANDO_PELOTA,  // Bloqueo inicial hasta recibir señal de arranque
  BUSCANDO,          // Patrón exploratorio cuando se pierde la pelota
  PERSIGUIENDO,      // Encuadre y ataque hacia la pelota detectada
  FRENANDO,          // Frenado electromagnético tras capturar la pelota
  REGRESANDO         // Avance a la portería rival con la pelota controlada
};

// Variables mutables de estado — definidas en config.cpp
extern volatile EstadoRobot estadoActual;

extern volatile unsigned long tFrenoIniciado;    // Marca de inicio del frenado de protección (ms)
extern volatile unsigned long tUltimaVezPelota;  // Último instante en que se detectó la pelota (ms)

extern volatile int pasoBusqueda;            // Paso actual del patrón de búsqueda (0–7)
extern volatile unsigned long tBusqueda;     // Cronómetro interno del patrón de búsqueda (ms)
extern volatile bool pelotaPerdidaReciente;  // true si la pelota se perdió en el ciclo anterior

extern String recepVecinos;

#endif
