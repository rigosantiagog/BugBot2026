#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>


// Parámetros de juego
constexpr float FRENTE_ANILLO = 0.0f;
constexpr float TOL_APUNTADO = 40.0f;
constexpr unsigned long T_QUIETO_MS = 4000;
constexpr int N_CAPTURA = 3;
constexpr float YAW_PORTERIA = 0.0f;
constexpr float TOL_PORTERIA = 15.0f;

// Pines motores (TB6612FNG)
constexpr uint8_t STBY = 4;
constexpr uint8_t FL_PWM = 25, FL_IN1 = 26, FL_IN2 = 27;
constexpr uint8_t RL_PWM = 33, RL_IN1 = 32, RL_IN2 = 14;
constexpr uint8_t FR_PWM = 13, FR_IN1 = 23, FR_IN2 = 2;
constexpr uint8_t RR_PWM = 19, RR_IN1 = 18, RR_IN2 = 5;
constexpr bool INVERTIR_FL = false, INVERTIR_FR = false, INVERTIR_RL = false, INVERTIR_RR = true;
constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t PWM_RES = 8;

// Velocidades
constexpr int PWM_MAX = 182;
constexpr int VEL_AVANCE = 120;
constexpr int VEL_GIRO = 85;
constexpr int VEL_BUSCAR = 80;
constexpr int RAMPA_PASO = 4;
constexpr unsigned long T_CONFIRMA_MS = 500;

// Ultrasonidos (distancia de frenado)
constexpr int DIST_FRENADO_CM = 20;

// Control de giro en REGRESANDO
constexpr float K_GIRO = 2.5f;
constexpr int GIRO_MIN_PWM = 20;
constexpr unsigned long T_GIRO_TIMEOUT = 3000;

// Variables externas
extern volatile int velAvanceActual;
extern volatile unsigned long tSenalEstable;
extern volatile float Correccion;

// UART
constexpr uint8_t RX_PIN = 34;
constexpr uint8_t TX_PIN = 17;
extern HardwareSerial Enlace;
extern volatile float anguloIR;
extern volatile int estadoIR, nIR;
extern volatile unsigned long ultimoDato;
extern volatile int distFrente, distAtras, distIzq, distDer;
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

// MPU6500
constexpr uint8_t MPU_ADDR = 0x68;
extern volatile float yaw, gyroZoffset;
extern volatile unsigned long tPrev;

// Máquina de estados
enum EstadoRobot { ESPERANDO_PELOTA, BUSCANDO, PERSIGUIENDO, FRENANDO, REGRESANDO };
extern volatile EstadoRobot estadoActual;
extern volatile unsigned long tFrenoIniciado, tUltimaVezPelota;
extern volatile int pasoBusqueda;
extern volatile unsigned long tBusqueda;
extern volatile bool pelotaPerdidaReciente;
extern String recepVecinos;

extern volatile float Correccion; ///Correccion de velocidad debug

//LOCALIZACION
extern volatile float distSeguridad;
extern volatile float posicionRobotX; //RECIBIDOS DEL UART tanto X como Y
extern volatile float posicionRobotY;

//WIFI

extern WebServer server;
// Cliente TCP para logs (opcional)
extern WiFiClient client;
constexpr char* ssid = "ESP32_DEBUG_MOTORES";
constexpr char* password = "12345678";

constexpr char* ipPC = "192.168.4.2";
constexpr int puerto = 5000;


#endif
