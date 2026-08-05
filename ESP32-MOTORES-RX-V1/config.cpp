#include "config.h"

// --- Control de motores ---
volatile int velAvanceActual = 0;        // Velocidad actual (0 = detenido)
volatile unsigned long tSenalEstable = 0;
volatile float Correccion = 0.0f;

// --- UART2 ---
HardwareSerial Enlace(2);                // Puerto serie UART2
volatile float anguloIR = -1.0f;         // -1 = sin dato
volatile int estadoIR = 0;
volatile int nIR = 0;
volatile unsigned long ultimoDato = 0;

// --- Giroscopio ---
volatile float yaw = 0.0f;
volatile float gyroZoffset = 0.0f;
volatile unsigned long tPrev = 0;

// --- Máquina de estados ---
volatile EstadoRobot estadoActual = ESPERANDO_PELOTA;
volatile unsigned long tFrenoIniciado = 0;
volatile unsigned long tUltimaVezPelota = 0;
volatile int pasoBusqueda = 0;
volatile unsigned long tBusqueda = 0;
volatile bool pelotaPerdidaReciente = false;

// --- Distancias recibidas ---
volatile int distFrente = 999;
volatile int distAtras = 999;
volatile int distIzq = 999;
volatile int distDer = 999;
String recepVecinos;

// --- Localización ---
volatile float posicionRobotX = 0.0f;
volatile float posicionRobotY = 0.0f;

// --- WiFi ---
WebServer server(80);
WiFiClient client;