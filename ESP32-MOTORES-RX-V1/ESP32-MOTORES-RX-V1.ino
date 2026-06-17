/**
 * @file ESP32-MOTORES-RX-V1.ino
 * @brief Control principal del robot: máquina de estados, MPU6500, motores y UART binaria.
 *        Incluye servidor web de debug y actualización OTA.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>          // Librería para OTA
#include "func_giro.h"
#include "func_motor.h"
#include "config.h"

#define MODO 1   // 1 = competencia, 0 = calibración

// Credenciales WiFi (punto de acceso)
const char* ssid = "ESP32_DEBUG";
const char* password = "12345678";

WebServer server(80);
String debugInfo = "Iniciando...\n";

// Cliente TCP para logs (opcional)
WiFiClient client;
const char* ipPC = "192.168.4.2";
const int puerto = 5000;
int kk = 0;

void handleRoot() {
  server.send(200, "text/plain", debugInfo);
}

/* ===================== LECTURA UART BINARIA =========================== */
void leerUART() {
  static uint8_t state = 0;
  static uint8_t buffer[16];
  static uint8_t idx = 0;
  static uint8_t chkCalculado = 0;
  static unsigned long tTimeout = 0;

  if (state != 0 && (millis() - tTimeout > 50)) {
    state = 0;
    idx = 0;
  }

  while (Enlace.available()) {
    uint8_t b = Enlace.read();
    tTimeout = millis();

    switch (state) {
      case 0:
        if (b == 0xAA) state = 1;
        break;
      case 1:
        if (b == 0x55) {
          state = 2;
          idx = 0;
          chkCalculado = 0;
        } else {
          state = 0;
        }
        break;
      case 2:
        buffer[idx++] = b;
        if (idx < 16) {
          chkCalculado ^= b;
        } else {
          state = 3;
        }
        break;
      case 3:
        if (b == chkCalculado) {
          memcpy((void*)&anguloIR, &buffer[0], sizeof(float));
          estadoIR = buffer[4];
          nIR = buffer[5];
          memcpy((void*)&distFrente, &buffer[6], sizeof(uint16_t));
          memcpy((void*)&distAtras, &buffer[8], sizeof(uint16_t));
          memcpy((void*)&distIzq, &buffer[10], sizeof(uint16_t));
          memcpy((void*)&distDer, &buffer[12], sizeof(uint16_t));
          uint16_t bitmapIR;
          memcpy(&bitmapIR, &buffer[14], sizeof(uint16_t));
          ultimoDato = millis();

          recepVecinos = "";
          for (int i = 0; i < 16; i++) {
            if (i > 0) recepVecinos += ",";
            recepVecinos += (bitmapIR & (1 << i)) ? '1' : '0';
          }
        }
        state = 0;
        idx = 0;
        break;
    }
  }
}

/* ===================== SETUP ================================== */
void setup() {
  Serial.begin(115200);
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(300);
  Serial.println("\n=== ESP32 MOTORES v10 (OTA + IR corregido) ===");

  // Motores
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  int pd[] = { FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2 };
  for (int p : pd) pinMode(p, OUTPUT);
  pwmInit(FL_PWM);
  pwmInit(FR_PWM);
  pwmInit(RL_PWM);
  pwmInit(RR_PWM);
  frenar();
  digitalWrite(STBY, HIGH);

  // MPU6500
  if (mpuInit()) Serial.println("MPU6500 OK.");
  else Serial.println("ERROR: MPU6500 no responde.");

  Serial.println(">> Calibrando norte estático. NO mover.");
  calibrarGyro();
  yaw = 0.0;
  tPrev = micros();

  // Diagnóstico 5s
  Serial.println(">> DIAGNÓSTICO: Gira el robot lentamente (5s)");
  unsigned long tDiag = millis();
  while (millis() - tDiag < 5000) {
    int16_t raw = mpuGz();
    float gz = (raw - gyroZoffset) / 131.0;
    Serial.printf("Raw=%6d  Offset=%.1f  Gz=%6.2f °/s\n", raw, gyroZoffset, gz);
    delay(100);
  }
  Serial.println(">> Fin diagnóstico.");

  unsigned long t0 = millis();
  while (millis() - t0 < T_QUIETO_MS) {
    actualizarRumbo();
    frenar();
    delay(5);
  }
  Serial.println(">> Norte fijado. Esperando pelota.");
  estadoActual = ESPERANDO_PELOTA;

  // WiFi y servidor web
  WiFi.softAP(ssid, password);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  server.on("/", handleRoot);
  server.begin();
  debugInfo += "Servidor web iniciado\n";

  // --- CONFIGURACIÓN OTA ---
  ArduinoOTA.setHostname("BugBot-Motores");
  ArduinoOTA.setPassword("12345678");
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Iniciando OTA: " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA finalizada.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progreso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error OTA[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Autenticación fallida");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Error al iniciar");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Error de conexión");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Error al recibir");
    else if (error == OTA_END_ERROR) Serial.println("Error al finalizar");
  });
  ArduinoOTA.begin();
  Serial.println("OTA listo. Conéctate a la red ESP32_DEBUG para actualizar.");

  // Conexión opcional a servidor de logs (comentar si no se usa)
  
  while (!client.connect(ipPC, puerto)) {
    Serial.println("Reintentando servidor logs...");
    delay(1000);
  }
  Serial.println("Conectado a servidor logs.");
  
}

void debugLog(const String& texto) {
  if (client.connected()) client.println(texto);
}

/* ===================== LOOP ================================== */
void loop() {
  server.handleClient();
  ArduinoOTA.handle();   // Necesario para OTA

  actualizarRumbo();
  leerUART();

  bool haySenal = (estadoIR == 1) && (millis() - ultimoDato < 400);
  if (haySenal) {
    tUltimaVezPelota = millis();
    pelotaPerdidaReciente = false;
  }

#if MODO == 0
  // Modo calibración
  frenar();
  static unsigned long t = 0;
  if (millis() - t > 250) {
    t = millis();
    if (haySenal) Serial.printf("Ángulo = %.1f\n", anguloIR);
    else Serial.println("Sin señal");
  }
#else
  float errApunte = haySenal ? errorAngular(anguloIR) : 0;
  bool pelotaPegada = haySenal && (abs(errApunte) <= TOL_APUNTADO) && (nIR >= N_CAPTURA); // CORREGIDO

  // Máquina de estados
  if (estadoActual == ESPERANDO_PELOTA) {
    if (haySenal) estadoActual = PERSIGUIENDO;
  } else if (pelotaPegada && estadoActual != REGRESANDO && estadoActual != FRENANDO) {
    estadoActual = FRENANDO;
    tFrenoIniciado = millis();
  } else if (estadoActual == REGRESANDO) {
    if (millis() - tUltimaVezPelota > 1000) {
      estadoActual = BUSCANDO;
      pelotaPerdidaReciente = true;
      pasoBusqueda = 0;
      tBusqueda = millis();
    }
  } else if (estadoActual != FRENANDO && estadoActual != REGRESANDO) {
    if (haySenal) estadoActual = PERSIGUIENDO;
    else {
      if (!pelotaPerdidaReciente) {
        pelotaPerdidaReciente = true;
        pasoBusqueda = 0;
        tBusqueda = millis();
      }
      estadoActual = BUSCANDO;
    }
  }

  const char* nombre = "?";

  switch (estadoActual) {
    case ESPERANDO_PELOTA:
      frenar();
      nombre = "BLOQUEADO";
      break;
    case BUSCANDO:
      // Patrón de búsqueda (idéntico al original)
      if (pasoBusqueda == 0) {
        frenar();
        nombre = "BUSQ:Pausa1s";
        if (millis() - tBusqueda >= 1000) { pasoBusqueda = 1; tBusqueda = millis(); }
      } else if (pasoBusqueda == 1) {
        avanzarSuave(VEL_BUSCAR, 0);
        nombre = "BUSQ:Avance";
        if (millis() - tBusqueda >= 600) { pasoBusqueda = 2; tBusqueda = millis(); }
      } else if (pasoBusqueda == 2) {
        frenar();
        nombre = "BUSQ:Pausa";
        if (millis() - tBusqueda >= 300) { pasoBusqueda = 3; tBusqueda = millis(); }
      } else if (pasoBusqueda == 3) {
        girarEnSitio(VEL_GIRO);
        nombre = "BUSQ:GiroDer";
        if (millis() - tBusqueda >= 400) { pasoBusqueda = 4; tBusqueda = millis(); }
      } else if (pasoBusqueda == 4) {
        avanzarSuave(VEL_BUSCAR, 0);
        nombre = "BUSQ:DiagDer";
        if (millis() - tBusqueda >= 600) { pasoBusqueda = 5; tBusqueda = millis(); }
      } else if (pasoBusqueda == 5) {
        frenar();
        nombre = "BUSQ:Pausa";
        if (millis() - tBusqueda >= 300) { pasoBusqueda = 6; tBusqueda = millis(); }
      } else if (pasoBusqueda == 6) {
        girarEnSitio(-VEL_GIRO);
        nombre = "BUSQ:GiroIzq";
        if (millis() - tBusqueda >= 800) { pasoBusqueda = 7; tBusqueda = millis(); }
      } else if (pasoBusqueda == 7) {
        avanzarSuave(VEL_BUSCAR, 0);
        nombre = "BUSQ:DiagIzq";
        if (millis() - tBusqueda >= 600) { pasoBusqueda = 0; tBusqueda = millis(); }
      }
      break;
    case PERSIGUIENDO:
      if (abs(errApunte) > TOL_APUNTADO) {
        int sentido = (errApunte > 0) ? VEL_GIRO : -VEL_GIRO;
        girarEnSitio(sentido);
        tSenalEstable = 0;
        nombre = "PERS:ENCUADRANDO";
      } else {
        if (tSenalEstable == 0) tSenalEstable = millis();
        if (millis() - tSenalEstable < T_CONFIRMA_MS) {
          frenar();
          nombre = "PERS:FILTRO";
        } else {
          int corr = constrain((int)(errApunte * 2.0), -30, 30);
          Correccion = corr;
          avanzarSuave(VEL_AVANCE, corr);
          nombre = "PERS:ATACANDO";
        }
      }
      break;
    case FRENANDO:
      frenar();
      nombre = "FRENANDO";
      if (millis() - tFrenoIniciado > 250) estadoActual = REGRESANDO;
      break;
    case REGRESANDO:
      {
        static unsigned long tInicioGiro = 0;
        static float errYawAnterior = 999.0f;
        float errYaw = errorAngular(yaw);
        if (abs(errYaw) > TOL_PORTERIA) {
          int pwmGiro = constrain((int)(abs(errYaw) * K_GIRO), GIRO_MIN_PWM, VEL_GIRO);
          int sentidoYaw = (errYaw < 0) ? pwmGiro : -pwmGiro;
          girarEnSitio(sentidoYaw);
          nombre = "REGR:GIRANDO";
          if (abs(errYaw) < abs(errYawAnterior) - 2.0f) tInicioGiro = millis();
          errYawAnterior = errYaw;
          if (millis() - tInicioGiro > T_GIRO_TIMEOUT) {
            estadoActual = BUSCANDO;
            pelotaPerdidaReciente = true;
            pasoBusqueda = 0;
            tBusqueda = millis();
            nombre = "REGR:TIMEOUT";
            break;
          }
        } else {
          if (distFrente < DIST_FRENADO_CM) {
            frenar();
            nombre = "REGR:REMATE";
          } else {
            int corr = constrain((int)(errYaw * -2.0), -30, 30);
            Correccion = corr;
            avanzarSuave(VEL_AVANCE, corr);
            nombre = "REGR:AVANZANDO";
          }
          tInicioGiro = millis();
          errYawAnterior = 999.0f;
        }
        break;
      }
  }

  // Reporte por serial cada 250 ms
  static unsigned long t = 0;
  if (millis() - t > 250) {
    t = millis();
    float errYaw = errorAngular(yaw);
    Serial.printf("%s | A=%.1f S=%d N=%d ErrAp=%.1f ErrY=%.1f Yaw=%.1f V:%s\n",
                  nombre, anguloIR, haySenal, nIR, errApunte, errYaw, yaw, recepVecinos);
  }

  // Debug web cada 1s
  static unsigned long tiempo = 0;
  if (millis() - tiempo > 1000) {
    tiempo = millis();
    float errYaw = errorAngular(yaw);
    debugInfo =
      "Tiempo: " + String(millis()) + "\n" +
      "Estado: " + String(estadoActual) + "\n" +
      "Accion: " + nombre + "\n" +
      "AnguloIR: " + String(anguloIR) + "\n" +
      "Receptores: " + String(nIR) + "\n" +
      "Yaw: " + String(yaw) + "\n" +
      "ErrApunte: " + String(errApunte) + "\n" +
      "ErrYaw: " + String(errYaw) + "\n" +
      "Señal: " + String(haySenal ? "Si" : "No") + "\n" +
      "Vecinos: " + recepVecinos + "\n" +
      "DistF: " + String(distFrente) + " cm\n" +
      "DistB: " + String(distAtras) + " cm\n" +
      "DistL: " + String(distIzq) + " cm\n" +
      "DistR: " + String(distDer) + " cm\n" +
      "Vel: " + String(velAvanceActual) + "\n" +
      "Corr: " + String(Correccion) + "\n";
  }

  // Envío a servidor TCP opcional
  kk++;
  if (kk == 10) {
    debugLog(debugInfo);
    kk = 0;
  }

#endif
  delay(25);
}