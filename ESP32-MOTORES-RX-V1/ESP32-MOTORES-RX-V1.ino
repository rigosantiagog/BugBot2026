/**
 * @file ESP32-MOTORES-RX-V1.ino
 * @brief Control principal: máquina de estados, MPU6500, motores y UART.
 * 
 * Mejoras en esta versión:
 * - Control PI en el giro para alineación precisa.
 * - Timeouts en REGRESANDO para detectar atascos (por Yaw estancado y por distancia frontal sin cambio).
 * - N_CAPTURA = 4 y filtro temporal de 300 ms para captura estable.
 * - Comentarios detallados en cada línea.
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

#define MODO 1   // 1 = modo competencia, 0 = calibración (motores detenidos)

void setup() {
  // Inicialización de periféricos
  Serial.begin(115200);                              // Monitor serie para depuración
  Enlace.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // UART2 hacia el anillo
  delay(300);
  Serial.println("\n=== ESP32 MOTORES v12 (Timeouts en REGRESANDO) ===");

  inicializarMotores();                              // Configura pines y PWM de los motores

  // --- WiFi ---
  /*WiFi.softAP(ssid, password);                       // Crea el Access Point
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // Conexión opcional a servidor de logs (no bloqueante)
  Serial.println("Buscando servidor de logs (opcional)...");
  unsigned long tLogInicio = millis();
  while (!client.connect(ipPC, puerto)) {
  }
  if (client.connected()) {
    Serial.println("Conectado a servidor logs.");
  } else {
    Serial.println("AVISO: sin servidor de logs. Continuando sin debug remoto.");
  }*/

  // --- Giroscopio MPU6500 ---
  if (mpuInit()) {
    Serial.println("MPU6500 OK.");
  } else {
    Serial.println("ERROR: MPU6500 no responde.");
  }

  // Calibración estática (2000 muestras, ver func_giro.cpp)
  Serial.println(">> Calibrando norte estático. NO mover el robot.");
  calibrarGyro();                                    // Calcula gyroZoffset
  yaw = 0.0f;                                        // Establece el norte en 0°
  tPrev = micros();                                  // Inicializa el contador de tiempo

  // Periodo de quietud post-calibración (para estabilizar el sistema)
  unsigned long t0 = millis();
  while (millis() - t0 < T_QUIETO_MS) {
    actualizarRumbo();                               // Sigue integrando el giroscopio
    frenar();                                        // Mantiene los motores frenados
    delay(5);
  }
  Serial.println(">> Norte fijado. Esperando pelota.");

  arduino_OTA();                                     // Inicia el servicio OTA

  estadoActual = ESPERANDO_PELOTA;                   // Arranca la máquina de estados
}

void loop() {
  ArduinoOTA.handle();                               // Permite actualizaciones inalámbricas

  actualizarRumbo();                                 // Actualiza el rumbo (yaw) integrando el giroscopio
  leerUART();                                        // Lee la trama del anillo y actualiza variables

  // Determina si hay señal de pelota: estadoIR == 1 y trama reciente (< 400 ms)
  bool haySenal = (estadoIR == 1) && (millis() - ultimoDato < 400);
  if (haySenal) {
    tUltimaVezPelota = millis();                     // Actualiza el tiempo de última detección
    pelotaPerdidaReciente = false;                   // Reinicia la bandera de pérdida
  }

#if MODO == 0
  // Modo calibración: solo imprime el ángulo IR sin mover los motores
  frenar();
  static unsigned long t = 0;
  if (millis() - t > 250) {
    t = millis();
    if (haySenal) Serial.printf("Ángulo = %.1f\n", anguloIR);
    else Serial.println("Sin señal");
  }
#else
  // ============================================================
  //  MODO COMPETENCIA
  // ============================================================

  // 1. Cálculo del error de apunte hacia la pelota
  float errApunte = haySenal ? errorAngular(anguloIR) : 0;

  // 2. Condición de captura con filtro temporal (300 ms de estabilidad)
  bool condicionCaptura = haySenal && (abs(errApunte) <= TOL_APUNTADO) && (nIR >= N_CAPTURA);
  static unsigned long tPelotaPegada = 0;            // Momento en que se cumple la condición
  static bool estadoPrevio = false;
  if (condicionCaptura) {
    if (!estadoPrevio) {
      tPelotaPegada = millis();                      // Inicia el cronómetro
      estadoPrevio = true;
    }
  } else {
    estadoPrevio = false;                            // Se perdió la condición
  }
  bool pelotaPegada = condicionCaptura && (millis() - tPelotaPegada >= 300);

  // ============================================================
  //  MÁQUINA DE ESTADOS – TRANSICIONES
  // ============================================================
  if (estadoActual == ESPERANDO_PELOTA) {
    if (haySenal) estadoActual = PERSIGUIENDO;       // Primera detección: perseguir
  } else if (pelotaPegada && estadoActual != REGRESANDO && estadoActual != FRENANDO) {
    estadoActual = FRENANDO;                          // Captura confirmada: frenar brevemente
    tFrenoIniciado = millis();
  } else if (estadoActual == REGRESANDO) {
    // Si se pierde la pelota durante el regreso por más de 1s, abortar y buscar
    if (millis() - tUltimaVezPelota > 1000) {
      estadoActual = BUSCANDO;
      pelotaPerdidaReciente = true;
      pasoBusqueda = 0;
      tBusqueda = millis();
    }
  } else if (estadoActual != FRENANDO && estadoActual != REGRESANDO) {
    // Estados PERSIGUIENDO o BUSCANDO: si hay señal, persigue; si no, busca
    if (haySenal) {
      estadoActual = PERSIGUIENDO;
    } else {
      if (!pelotaPerdidaReciente) {
        pelotaPerdidaReciente = true;
        pasoBusqueda = 0;
        tBusqueda = millis();
      }
      estadoActual = BUSCANDO;
    }
  }

  const char* nombre = "?";                          // Etiqueta de depuración

  // ============================================================
  //  EJECUCIÓN DE ACCIONES SEGÚN EL ESTADO
  // ============================================================
  switch (estadoActual) {

    case ESPERANDO_PELOTA:
      frenar();                                       // Robot quieto
      nombre = "BLOQUEADO";
      break;

    case BUSCANDO:
      // Patrón de búsqueda en 8 pasos (pausa, avance, pausa, giro, etc.)
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
      // Si el error angular es grande, girar en sitio para encuadrar
      if (abs(errApunte) > TOL_APUNTADO) {
        int sentido = (errApunte > 0) ? VEL_GIRO : -VEL_GIRO;
        girarEnSitio(sentido);
        tSenalEstable = 0;                             // Reinicia el filtro de estabilidad
        nombre = "PERS:ENCUADRANDO";
      } else {
        // Error pequeño: esperar T_CONFIRMA_MS para confirmar señal estable
        if (tSenalEstable == 0) tSenalEstable = millis();
        if (millis() - tSenalEstable < T_CONFIRMA_MS) {
          frenar();
          nombre = "PERS:FILTRO";
        } else {
          int corr = constrain((int)(errApunte * 2.0), -30, 30); // Corrección para avanzar recto
          Correccion = corr;
          avanzarSuave(VEL_AVANCE, corr);
          nombre = "PERS:ATACANDO";
        }
      }
      break;

    case FRENANDO:
      frenar();                                          // Detención inmediata
      nombre = "FRENANDO";
      if (millis() - tFrenoIniciado > 250) {
        estadoActual = REGRESANDO;                       // Tras 250 ms, ir a portería
      }
      break;

    case REGRESANDO:
      {
        // ============================================================
        //  REGRESO A PORTERÍA CON CONTROL PI + TIMEOUTS DE ATASCO
        // ============================================================
        static unsigned long tInicioGiro = 0;            // Para timeout de giro
        static float errYawAnterior = 999.0f;            // Para detectar progreso en giro
        static unsigned long tUltimoCambioYaw = millis(); // Detecta si el yaw se estanca
        static float integralYaw = 0.0f;                 // Término integral del controlador
        static unsigned long tUltimaDistF = 0;           // Última vez que se actualizó distFrente
        static float distFAnterior = 999.0f;             // Distancia frontal anterior

        float errYaw = errorAngular(yaw);                // Error de orientación respecto al norte

        // ----- Detección de atasco por Yaw estancado (no cambia en T_YAW_SIN_CAMBIO) -----
        if (millis() - tUltimoCambioYaw > T_YAW_SIN_CAMBIO && abs(errYaw) > TOL_PORTERIA) {
          // Si el yaw no ha variado y no estamos alineados, forzar salida a búsqueda
          estadoActual = BUSCANDO;
          pelotaPerdidaReciente = true;
          pasoBusqueda = 0;
          tBusqueda = millis();
          nombre = "REGR:TIMEOUT_YAW";
          break;
        }

        // ----- Detección de atasco por distancia frontal sin progreso (no disminuye) -----
        if (distFrente != 999 && distFrente < distFAnterior - 2.0f) {
          // Si la distancia frontal disminuye (avanza), reiniciamos el cronómetro
          tUltimaDistF = millis();
        }
        distFAnterior = distFrente;
        if (millis() - tUltimaDistF > T_AVANCE_SIN_PROGRESO && distFrente > DIST_FRENADO_CM + 5) {
          // Si no avanza durante T_AVANCE_SIN_PROGRESO y no está en zona de remate, forzar búsqueda
          estadoActual = BUSCANDO;
          pelotaPerdidaReciente = true;
          pasoBusqueda = 0;
          tBusqueda = millis();
          nombre = "REGR:TIMEOUT_AVANCE";
          break;
        }

        if (abs(errYaw) > TOL_PORTERIA) {
          // ---- Controlador PI para el giro ----
          float pTerm = abs(errYaw) * K_GIRO;
          // Integral (acumulada solo si el error es pequeño para evitar sobrepaso)
          if (abs(errYaw) < 30.0f) {
            integralYaw += errYaw * K_I_GIRO;
            integralYaw = constrain(integralYaw, -SATURACION_I, SATURACION_I);
          } else {
            integralYaw = 0.0f;                          // Reinicia integral si el error es grande
          }
          int pwmGiro = (int)(pTerm + integralYaw);
          // Limitar entre GIRO_MIN_PWM y VEL_GIRO
          pwmGiro = constrain(pwmGiro, GIRO_MIN_PWM, VEL_GIRO);
          int sentidoYaw = (errYaw < 0) ? pwmGiro : -pwmGiro;
          girarEnSitio(sentidoYaw);
          nombre = "REGR:GIRANDO";

          // Detectar progreso en el giro (error disminuye al menos 1°)
          if (abs(errYaw) < abs(errYawAnterior) - 1.0f) {
            tInicioGiro = millis();                      // Reinicia el timeout de giro
            tUltimoCambioYaw = millis();                 // Actualiza el timestamp de cambio
          }
          errYawAnterior = errYaw;

          // Timeout de giro sin progreso (T_GIRO_TIMEOUT)
          if (millis() - tInicioGiro > T_GIRO_TIMEOUT) {
            estadoActual = BUSCANDO;
            pelotaPerdidaReciente = true;
            pasoBusqueda = 0;
            tBusqueda = millis();
            nombre = "REGR:TIMEOUT_GIRO";
            break;
          }
        } else {
          // ---- Ya alineado con la portería ----
          // Frenar si la distancia frontal es menor que DIST_FRENADO_CM
          if (distFrente < DIST_FRENADO_CM && distFrente != 999) {
            frenar();
            nombre = "REGR:REMATE";
          } else {
            // Avanzar con corrección para mantener la línea recta
            int corr = constrain((int)(errYaw * -2.0), -30, 30);
            Correccion = corr;
            avanzarSuave(VEL_AVANCE, corr);
            nombre = "REGR:AVANZANDO";
          }
          // Reiniciar temporizadores y término integral
          tInicioGiro = millis();
          errYawAnterior = 999.0f;
          tUltimoCambioYaw = millis();
          integralYaw = 0.0f;                              // Reinicia la integral al estar alineado
        }
        break;
      }
  } // fin switch

  // ============================================================
  //  REPORTE POR SERIAL (cada 250 ms)
  // ============================================================
  static unsigned long t = 0;
  if (millis() - t > 250) {
    t = millis();
    float errYaw = errorAngular(yaw);
    Serial.printf("%s | A=%.1f S=%d N=%d ErrAp=%.1f ErrY=%.1f Yaw=%.1f V:%s\n",
                  nombre, anguloIR, haySenal, nIR, errApunte, errYaw, yaw, recepVecinos);
  }

  // ============================================================
  //  DEBUG WEB (cada 100 ms)
  // ============================================================
  static unsigned long tiempo = 0;
  if (millis() - tiempo > 100) {
    tiempo = millis();
    float errYaw = errorAngular(yaw);
    debugLog(
      "Tiempo: " + String(millis()) + "\n" +
      "Memoria RAM: " + String(ESP.getFreeHeap()) + "\n" +
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
      "Corr: " + String(Correccion) + "\n"
    );
  }

#endif
  //delay(10);   // Pequeña pausa para evitar saturar el procesador
}