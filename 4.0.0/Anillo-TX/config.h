

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// UART
constexpr int RX_PIN = 26;
constexpr int TX_PIN = 25;
extern HardwareSerial Enlace;

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

// Filtro de persistencia (debounce) del anillo IR: cada sensor tiene un contador de
// "confianza" que sube al leerse activo y baja al leerse inactivo (nunca de golpe).
// Con esto: (1) un destello de ruido de 1 sola vuelta de loop en un sensor aislado ya
// NO alcanza a disparar un angulo erratico (hace falta que se repita), y (2) un sensor
// que estaba encendido no se pierde por 1-2 vueltas de loop en las que no se leyo activo
// (comun por el desfase entre los 38kHz nominales del TSSP58038 y los 40kHz reales de
// la pelota oficial). Ver aplicarFiltroPersistenciaIR() en func_anillo.cpp.
constexpr uint8_t IR_CONFIANZA_MAX = 4;   // Techo del contador de confianza por sensor
constexpr uint8_t IR_UMBRAL_ACTIVO = 2;   // Confianza minima para considerar un sensor activo ya filtrado

// Depuracion remota por WiFi: el ESP32 se conecta COMO CLIENTE al servidor de logs
// (puerto_anillo.py, corriendo en tu laptop/PC) y le envia todo lo que normalmente
// ves por Serial. IP_SERVIDOR_LOGS es la IP de esa laptop dentro de la red del ESP32.
//
// Como el ESP32 sigue siendo el punto de acceso (WiFi.softAP), tu laptop al conectarse
// a "ESP32_DEBUG_ANILLO" normalmente recibe la IP 192.168.4.2 (el ESP32 se queda con
// la 192.168.4.1). Si tu equipo toma otra IP, ajustala aqui.
constexpr char IP_SERVIDOR_LOGS[] = "192.168.4.2";
constexpr uint16_t PUERTO_DEBUG_WIFI = 5000;   // Debe coincidir con PORT en puerto_anillo.py

// Comunicacion con la ESP32 de motores
extern volatile float yaw;             // Ultimo yaw recibido desde la ESP32 de motores (grados), via recibirYaw()

// OFFSET_FRENTE: ajusta el angulo calculado por el anillo para que 0 grados
// coincida con el frente FISICO del robot (depende de como quedo montado el anillo IR).
// TODO: calibrar con el valor real; se deja en 0.0 (sin ajuste) para que compile mientras tanto.
constexpr float OFFSET_FRENTE = 0.0f;

// Variables compartidas
extern volatile int distFrente, distAtras, distIzq, distDer;
extern volatile bool activo[16];
extern volatile float angulo;

#endif
