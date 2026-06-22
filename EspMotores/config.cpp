#include "config.h"

volatile int velAvanceActual = 0;
volatile unsigned long tSenalEstable = 0;
volatile float Correccion = 0.0f;

HardwareSerial Enlace(2);
volatile float anguloIR = -1.0f;
volatile int estadoIR = 0;
volatile int nIR = 0;
volatile unsigned long ultimoDato = 0;

volatile float yaw = 0.0f;
volatile float gyroZoffset = 0.0f;
volatile unsigned long tPrev = 0;

volatile EstadoRobot estadoActual = ESPERANDO_PELOTA;
volatile unsigned long tFrenoIniciado = 0;
volatile unsigned long tUltimaVezPelota = 0;
volatile int pasoBusqueda = 0;
volatile unsigned long tBusqueda = 0;
volatile bool pelotaPerdidaReciente = false;

volatile int distFrente = 999;
volatile int distAtras = 999;
volatile int distIzq = 999;
volatile int distDer = 999;
String recepVecinos;

//Localizacion
extern volatile float distSeguridad = 30.0f;
volatile float posicionRobotX = 0.0f; //RECIBIDOS DEL UART tanto X como Y
volatile float posicionRobotY = 0.0f;

WebServer server(80);
WiFiClient client;
