/**
 * @file ESP32-MOTORES-RX-V1.ino
 * @brief Control principal del robot: máquina de estados, MPU6500, motores y UART binaria.
 *        Incluye servidor web de debug y actualización OTA.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h> 
#include "func_giro.h"
#include "func_motor.h"
#include "config.h"
#include "func_comunicacion.h"

#define MODO 1   // 1 = competencia, 0 = calibración

/* ===================== SETUP ================================== */
void setup() {
  Serial.begin(115200);
  Enlace.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(300);
  Serial.println("\n=== ESP32 MOTORES v10 (OTA + IR corregido) ===");
  
  //Motores
  inicializarMotores();
  
  // WiFi y servidor web
  WiFi.softAP(ssid, password);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  // Conexión opcional a servidor de logs (comentar si no se usa)
  
  while (!client.connect(ipPC, puerto)) {
    Serial.println("Reintentando servidor logs...");
    delay(1000);
  }

  Serial.println("Conectado a servidor logs.");
  // MPU6500
  if (mpuInit()) Serial.println("MPU6500 OK.");
  else Serial.println("ERROR: MPU6500 no responde.");

  Serial.println(">> Calibrando norte estático. NO mover.");
  calibrarGyro();
  yaw = 0.0f;
  tPrev = micros();
  Serial.println(">> Fin diagnóstico.");

  unsigned long t0 = millis();
  while (millis() - t0 < T_QUIETO_MS) {
    actualizarRumbo();
    frenar();
    delay(5);
  }
  Serial.println(">> Norte fijado. Esperando pelota.");
  

  arduino_OTA();
  
  estadoActual = ESPERANDO_PELOTA;
}

/* ===================== LOOP ================================== */
void loop() {
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
  if (millis() - tiempo > 100) {
    tiempo = millis();
    float errYaw = errorAngular(yaw);
    debugLog(
      "Tiempo: " + String(millis()) + "\n" +
      "Memoria RAM: " + String(ESP.getFreeHeap()) + "\n" +
      "Memoria FLASH: " + String(ESP.getFlashChipSize()) + "\n" +
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
      "Corr: " + String(Correccion) + "\n");
  }

#endif
  delay(10);
}
