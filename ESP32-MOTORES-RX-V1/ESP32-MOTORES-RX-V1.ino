/**
 * @file ESP32-MOTORES-RX-V1.ino
 * @brief Control principal del robot: máquina de estados, giroscopio, motores y comunicación UART binaria.
 * 
 * Este archivo contiene el setup y el loop principal. Se encarga de:
 * - Inicializar periféricos (motores, giroscopio, WiFi).
 * - Leer la trama binaria del anillo (IR + ultrasonidos) mediante UART.
 * - Ejecutar la máquina de estados (ESPERANDO_PELOTA, BUSCANDO, PERSIGUIENDO, FRENANDO, REGRESANDO).
 * - Controlar los motores según el estado activo.
 * - Proporcionar información de debug por WiFi y monitor serie.
 * 
 * Versión: v9 (Protocolo Binario + Control P en Giro + Timeout)
 */

#include <Arduino.h>          // Funciones básicas de Arduino (pinMode, digitalWrite, etc.)
#include <Wire.h>             // Comunicación I2C para el giroscopio MPU6500
#include "func_giro.h"        // Funciones del giroscopio (calibración, lectura, actualización de rumbo)
#include "func_motor.h"       // Control de motores (avance, giro, detención suave y brusca)
#include "config.h"           // Configuraciones globales (pines, constantes, variables externas)
#include <WiFi.h>             // Conexión WiFi para debug
#include <WebServer.h>        // Servidor web para mostrar información en tiempo real

#define MODO 1                // 1 = modo competencia (motores activos), 0 = modo calibración (solo diagnóstico)

// Credenciales de la red WiFi que el robot crea como punto de acceso
const char* ssid = "ESP32_DEBUG";
const char* password = "12345678";

WebServer server(80);         // Servidor web en el puerto 80

String debugInfo = "Iniciando...\n";   // Cadena que se mostrará en la página web de debug

/**
 * @brief Manejador de la raíz del servidor web: envía la información de debug como texto plano.
 */
void handleRoot() {
  server.send(200, "text/plain", debugInfo);
}

/* ===================== CEREBRO UART (PROTOCOLO BINARIO) =========================== */

/**
 * @brief Lee la trama binaria enviada por la ESP32 del anillo.
 *        Protocolo: [0xAA][0x55][16 bytes de datos][XOR checksum].
 *        Decodifica y almacena en variables globales.
 * 
 * La trama de 16 bytes contiene:
 *   - Bytes 0-3:  float  anguloIR (little-endian)
 *   - Byte 4:     uint8_t estadoIR (1 = pelota detectada)
 *   - Byte 5:     uint8_t nIR (número de sensores activos)
 *   - Bytes 6-7:  uint16_t distFrente (cm)
 *   - Bytes 8-9:  uint16_t distAtras
 *   - Bytes 10-11: uint16_t distIzq
 *   - Bytes 12-13: uint16_t distDer
 *   - Bytes 14-15: uint16_t bitmapIR (bits 0-15 corresponden a sensores 0-15)
 */
void leerUART() {
  static uint8_t state = 0;           // Estado de la máquina de recepción (0=Busca CAB1, 1=Busca CAB2, 2=Lee datos, 3=Lee checksum)
  static uint8_t buffer[16];          // Buffer temporal para los 16 bytes de datos
  static uint8_t idx = 0;             // Índice de escritura en el buffer
  static uint8_t chkCalculado = 0;    // Checksum calculado sobre la marcha (XOR de los 16 bytes)
  static unsigned long tTimeout = 0;  // Temporizador para evitar bloqueos en la recepción

  // Si estamos en medio de una recepción y pasaron más de 50ms sin datos, reiniciamos para no colgarnos
  if (state != 0 && (millis() - tTimeout > 50)) {
    state = 0;
    idx = 0;
  }

  while (Enlace.available()) {        // Mientras haya datos en el buffer de entrada
    uint8_t b = Enlace.read();        // Lee un byte de la UART
    tTimeout = millis();              // Actualiza el timestamp del timeout

    switch (state) {
      case 0: // Buscando el primer byte de la cabecera (0xAA)
        if (b == 0xAA) state = 1;
        break;

      case 1: // Buscando el segundo byte de la cabecera (0x55)
        if (b == 0x55) {
          state = 2;                  // Cabecera válida, empezamos a leer datos
          idx = 0;
          chkCalculado = 0;
        } else {
          state = 0;                  // Si no es 0x55, reinicia la búsqueda
        }
        break;

      case 2: // Leyendo los 16 bytes de datos
        buffer[idx++] = b;
        if (idx < 16) {
          chkCalculado ^= b;          // Acumula el XOR de los datos (excepto el checksum)
        } else {
          // Ya leímos los 16 bytes, el siguiente byte será el checksum
          state = 3;
        }
        break;

      case 3: // Leyendo el checksum
        if (b == chkCalculado) {      // Verifica que el checksum coincida
          // --- Desempaquetado de la trama binaria ---
          // Bytes 0-3: float anguloIR (little-endian)
          // Se usa casting a (void*) para eliminar el calificador volatile y evitar error de compilación
          memcpy((void*)&anguloIR, &buffer[0], sizeof(float));
          // Byte 4: estadoIR
          estadoIR = buffer[4];
          // Byte 5: nIR
          nIR = buffer[5];
          // Bytes 6-7: uint16_t distFrente
          memcpy((void*)&distFrente, &buffer[6], sizeof(uint16_t));
          // Bytes 8-9: distAtras
          memcpy((void*)&distAtras, &buffer[8], sizeof(uint16_t));
          // Bytes 10-11: distIzq
          memcpy((void*)&distIzq, &buffer[10], sizeof(uint16_t));
          // Bytes 12-13: distDer
          memcpy((void*)&distDer, &buffer[12], sizeof(uint16_t));
          // Bytes 14-15: uint16_t bitmapIR (16 sensores en 16 bits)
          uint16_t bitmapIR;
          memcpy(&bitmapIR, &buffer[14], sizeof(uint16_t));

          // Actualiza el timestamp de último dato válido
          ultimoDato = millis();

          // --- Genera la cadena de debug "recepVecinos" a partir del bitmap ---
          recepVecinos = "";
          for (int i = 0; i < 16; i++) {
            if (i > 0) recepVecinos += ",";
            recepVecinos += (bitmapIR & (1 << i)) ? '1' : '0';
          }
        }
        // Reinicia la máquina de estados para buscar la siguiente trama
        state = 0;
        idx = 0;
        break;
    }
  }
}

/* ===================== SETUP DEL PARTIDO ================================== */

/**
 * @brief Configuración inicial que se ejecuta una sola vez al encender el robot.
 *        Inicializa pines, motores, giroscopio, calibración, WiFi y servidor web.
 */
void setup() {
  Serial.begin(115200);                             // Inicia comunicación serial con la PC para debug
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);  // Inicia UART para recibir datos del anillo (baudrate 38400)
  delay(300);                                       // Pequeña pausa para que los periféricos se estabilicen
  Serial.println("\n=== ESP32 MOTORES v9 (Protocolo Binario) ===");

  pinMode(STBY, OUTPUT);                            // Configura el pin STBY (habilitación de drivers) como salida
  digitalWrite(STBY, LOW);                          // Inicialmente deshabilita los drivers (seguro activado)

  // Lista de pines de dirección de los motores (IN1, IN2 de cada driver)
  int pd[] = { FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2 };
  for (int p : pd) pinMode(p, OUTPUT);              // Configura todos como salidas

  // Inicializa los canales PWM para cada motor
  pwmInit(FL_PWM);
  pwmInit(FR_PWM);
  pwmInit(RL_PWM);
  pwmInit(RR_PWM);
  frenar();                                         // Asegura que las ruedas estén detenidas (frenado brusco inicial)
  digitalWrite(STBY, HIGH);                         // Habilita los drivers (quita el seguro)

  // Inicializa el giroscopio MPU6500 por I2C
  if (mpuInit()) Serial.println("MPU OK.");
  else Serial.println("AVISO: MPU no responde.");   // Si falla, continúa pero sin brújula

  // Calibración del giroscopio: toma 500 muestras estáticas para obtener el offset
  Serial.println(">> BLOQUEO: Calibrando norte estático. NO mover el carro.");
  calibrarGyro();                                   // Calcula gyroZoffset
  yaw = 0.0;                                        // Fija el norte (0 grados) en la orientación actual
  tPrev = micros();                                 // Inicia el contador de tiempo para integración de rumbo
  unsigned long t0 = millis();                      // Marca de tiempo para el bloqueo de 4 segundos

  while (millis() - t0 < T_QUIETO_MS) {             // Espera T_QUIETO_MS (4000 ms) para estabilizar
    actualizarRumbo();                              // Actualiza el rumbo integrando el giroscopio
    frenar();                                       // Mantiene el robot frenado durante la calibración
    delay(5);                                       // Pequeña pausa para no saturar el procesador
  }

  Serial.println(">> Norte fijado con éxito. Entrando en Modo de Espera de Pelota.");
  estadoActual = ESPERANDO_PELOTA;                  // Estado inicial: esperando señal de arranque

  // --- Configuración del punto de acceso WiFi para debug ---
  WiFi.softAP(ssid, password);                      // Crea red WiFi con las credenciales dadas
  Serial.print("IP del ESP32: ");
  Serial.println(WiFi.softAPIP());                  // Muestra la IP asignada al punto de acceso
  server.on("/", handleRoot);                       // Asocia la raíz con el manejador handleRoot
  server.begin();                                   // Inicia el servidor web
  debugInfo += "Servidor iniciado\n";               // Agrega mensaje al debug
}

/* ===================== LOOP DEL PARTIDO ================================== */

/**
 * @brief Bucle principal que se ejecuta continuamente. 
 *        Actualiza el rumbo, lee la UART, maneja la máquina de estados y controla los motores.
 */
void loop() {
  server.handleClient();                            // Atiende peticiones del servidor web de debug
  actualizarRumbo();                                // Actualiza el rumbo (yaw) integrando el giroscopio
  leerUART();                                       // Lee y procesa la trama binaria del anillo

  // Determina si hay señal de pelota: estadoIR == 1 y el último dato no tiene más de 400 ms de antigüedad
  bool haySenal = (estadoIR == 1) && (millis() - ultimoDato < 400);

  if (haySenal) {                                   // Si se detecta la pelota
    tUltimaVezPelota = millis();                    // Actualiza el tiempo de la última detección
    pelotaPerdidaReciente = false;                  // Marca que la pelota no se ha perdido recientemente
  }

#if MODO == 0
  /* --- MODO CALIBRACIÓN VISUAL (Puro Diagnóstico) --- */
  frenar();                                         // Motores detenidos
  static unsigned long t = 0;
  if (millis() - t > 250) {
    t = millis();
    if (haySenal) Serial.printf(">> Ángulo detectado = %.1f\n", anguloIR);
    else Serial.println(">> Sin señal IR");
  }
#else
  /* --- MODO COMPETENCIA DESTRUCTORA --- */
  // Calcula el error de apunte (diferencia entre el ángulo de la pelota y el frente del robot)
  float errApunte = haySenal ? errorAngular(anguloIR) : 0;

  // Condición de pelota "pegada": hay señal, el error de apunte está dentro de la tolerancia y hay suficientes sensores activos
  bool pelotaPegada = haySenal && (abs(errApunte) <= TOL_APUNTADO) && (nIR >= N_CAPTURA);

  /* --- CEREBRO SUPERIOR: CAMBIO DE ESTADOS Y BANDERAS --- */
  // Máquina de estados: transiciones basadas en condiciones
  if (estadoActual == ESPERANDO_PELOTA) {           // Si está en espera de inicio
    if (haySenal) estadoActual = PERSIGUIENDO;      // Al ver la pelota, comienza a perseguirla
  } else if (pelotaPegada && estadoActual != REGRESANDO && estadoActual != FRENANDO) {
    // Si la pelota está firmemente atrapada y no estamos yendo a portería ni frenando, inicia frenado
    estadoActual = FRENANDO;
    tFrenoIniciado = millis();
  } else if (estadoActual == REGRESANDO) {          // Si estamos yendo a portería
    if (millis() - tUltimaVezPelota > 1000) {       // Si perdemos la pelota por más de 1 segundo
      estadoActual = BUSCANDO;                      // Abortar misión y buscar de nuevo
      pelotaPerdidaReciente = true;
      pasoBusqueda = 0;                             // Reiniciar patrón de búsqueda
      tBusqueda = millis();
    }
  } else if (estadoActual != FRENANDO && estadoActual != REGRESANDO) {
    // Para estados que no son frenado ni regreso (es decir, BUSCANDO o PERSIGUIENDO)
    if (haySenal) {
      estadoActual = PERSIGUIENDO;                  // Si ve la pelota, persigue
    } else {
      if (!pelotaPerdidaReciente) {                 // Si acaba de perderla
        pelotaPerdidaReciente = true;
        pasoBusqueda = 0;                           // Inicia búsqueda desde el principio
        tBusqueda = millis();
      }
      estadoActual = BUSCANDO;                      // Cambia a estado de búsqueda
    }
  }

  const char* nombre = "?";                         // Cadena para el diagnóstico (nombre del estado actual)

  /* --- CEREBRO FÍSICO: CONTROL DE ACTUADORES SEGÚN EL ESTADO --- */
  switch (estadoActual) {

    case ESPERANDO_PELOTA:                          // Estado de bloqueo inicial
      frenar();                                     // Frenado brusco (motores apagados)
      nombre = "BLOQUEADO (Esperando arranque)";
      break;

    case BUSCANDO:                                  // Patrón de búsqueda sectorial
      if (pasoBusqueda == 0) {
        frenar();                                   // Detiene el robot
        nombre = "BUSQUEDA: Pausa 1s (Escuchando ecos)";
        if (millis() - tBusqueda >= 1000) {
          pasoBusqueda = 1;
          tBusqueda = millis();
        }
      } else if (pasoBusqueda == 1) {
        avanzarSuave(VEL_BUSCAR, 0);                // Avanza hacia adelante
        nombre = "BUSQUEDA: Avance de Patrulla";
        if (millis() - tBusqueda >= 600) {
          pasoBusqueda = 2;
          tBusqueda = millis();
        }
      } else if (pasoBusqueda == 2) {
        frenar();                                   // Pausa para mirar
        nombre = "BUSQUEDA: Pausa Visual";
        if (millis() - tBusqueda >= 300) {
          pasoBusqueda = 3;
          tBusqueda = millis();
        }
      } else if (pasoBusqueda == 3) {
        girarEnSitio(VEL_GIRO);                     // Gira a la derecha
        nombre = "BUSQUEDA: Escaneo Derecha";
        if (millis() - tBusqueda >= 400) {
          pasoBusqueda = 4;
          tBusqueda = millis();
        }
      } else if (pasoBusqueda == 4) {
        avanzarSuave(VEL_BUSCAR, 0);                // Avanza diagonal derecha
        nombre = "BUSQUEDA: Avance Diagonal Der";
        if (millis() - tBusqueda >= 600) {
          pasoBusqueda = 5;
          tBusqueda = millis();
        }
      } else if (pasoBusqueda == 5) {
        frenar();
        nombre = "BUSQUEDA: Pausa Visual";
        if (millis() - tBusqueda >= 300) {
          pasoBusqueda = 6;
          tBusqueda = millis();
        }
      } else if (pasoBusqueda == 6) {
        girarEnSitio(-VEL_GIRO);                    // Gira a la izquierda (doble tiempo para abarcar más)
        nombre = "BUSQUEDA: Escaneo Izquierda";
        if (millis() - tBusqueda >= 800) {
          pasoBusqueda = 7;
          tBusqueda = millis();
        }
      } else if (pasoBusqueda == 7) {
        avanzarSuave(VEL_BUSCAR, 0);
        nombre = "BUSQUEDA: Avance Diagonal Izq";
        if (millis() - tBusqueda >= 600) {
          pasoBusqueda = 0;                         // Reinicia el patrón
          tBusqueda = millis();
        }
      }
      break;

    case PERSIGUIENDO:                              // Persecución y ataque a la pelota
      if (abs(errApunte) > TOL_APUNTADO) {          // Si no está encuadrado, gira
        int sentido = (errApunte > 0) ? VEL_GIRO : -VEL_GIRO;
        girarEnSitio(sentido);
        tSenalEstable = 0;                          // Reinicia el contador de señal estable
        nombre = "PERSIGUE -> ENCUADRANDO";
      } else {                                      // Está dentro del cono frontal
        if (tSenalEstable == 0) tSenalEstable = millis(); // Inicia el cronómetro de confirmación
        if (millis() - tSenalEstable < T_CONFIRMA_MS) {   // Filtro de señal para evitar falsos positivos
          frenar();                                 // No se mueve hasta confirmar
          nombre = "FILTRO VISUAL (Confirmando luz)";
        } else {
          int corr = constrain((int)(errApunte * 2.0), -30, 30); // Corrección angular para avanzar recto
          avanzarSuave(VEL_AVANCE, corr);           // Avanza hacia la pelota con corrección
          nombre = "ATACANDO PELOTA A FONDO";
        }
      }
      break;

    case FRENANDO:                                  // Frenado electromagnético tras capturar la pelota
      frenar();                                     // Detención inmediata
      nombre = "FRENADO ELECTROMAGNÉTICO";
      if (millis() - tFrenoIniciado > 250) {        // Espera 250 ms para estabilizar
        estadoActual = REGRESANDO;                  // Luego va a portería
      }
      break;

    case REGRESANDO:                                // Avance hacia la portería rival
      {
        // Variables estáticas para control de giro progresivo y timeout
        static unsigned long tInicioGiro = 0;       // Tiempo en que se inició el giro actual
        static float errYawAnterior = 999.0f;       // Para detectar si el error disminuye

        float errYaw = errorAngular(yaw);           // Calcula el error de orientación respecto al norte (portería)

        if (abs(errYaw) > TOL_PORTERIA) {           // Si no está alineado, gira con control proporcional
          // Control proporcional: velocidad = K * |error|, limitada entre GIRO_MIN_PWM y VEL_GIRO
          int pwmGiro = constrain((int)(abs(errYaw) * K_GIRO), GIRO_MIN_PWM, VEL_GIRO);
          int sentidoYaw = (errYaw < 0) ? pwmGiro : -pwmGiro; // Positivo = derecha, negativo = izquierda
          girarEnSitio(sentidoYaw);
          nombre = "PORTERIA -> GIRANDO BRÚJULA";

          // Control de timeout: si el error no disminuye en T_GIRO_TIMEOUT, abortar
          if (abs(errYaw) < abs(errYawAnterior) - 2.0f) { // El error está mejorando (al menos 2°)
            tInicioGiro = millis();                // Reinicia el cronómetro
          }
          errYawAnterior = errYaw;

          if (millis() - tInicioGiro > T_GIRO_TIMEOUT) {
            // Lleva mucho tiempo girando sin progreso → pasar a búsqueda
            estadoActual = BUSCANDO;
            pelotaPerdidaReciente = true;
            pasoBusqueda = 0;
            tBusqueda = millis();
            nombre = "TIMEOUT GIRO -> BUSCANDO";
            break;  // Sale del case para evitar ejecutar más código
          }
        } else {
          // Está alineado: comprueba distancia frontal para remate
          if (distFrente < DIST_FRENADO_CM) {       // Si la pared (portería) está a menos de DIST_FRENADO_CM
            frenar();                               // Frenado brusco para impulsar la pelota
            nombre = "PORTERIA -> REMATE (FRENADO SECO)";
          } else {
            int corr = constrain((int)(errYaw * -2.0), -30, 30); // Corrección para mantener línea recta
            avanzarSuave(VEL_AVANCE, corr);         // Avanza suavemente hacia la portería
            nombre = "PORTERIA -> AVANZANDO";
          }
          // Reinicia variables de control de giro al estar alineado
          tInicioGiro = millis();
          errYawAnterior = 999.0f;
        }
        break;
      }
  }

  /* --- REPORTE CONSTANTE A LA COMPUTADORA (serial) --- */
  static unsigned long t = 0;
  if (millis() - t > 250) {                         // Imprime cada 250 ms
    t = millis();
    // Se añade errYaw al reporte para facilitar la depuración del giro
    float errYaw = errorAngular(yaw);               // Solo para el reporte, se recalcula
    Serial.printf("%s | AnguloIR=%.1f Senal=%d N_Sens=%d ErrAp=%.1f ErrYaw=%.1f GyroYaw=%.1f Veci:%s\n",
                  nombre, anguloIR, haySenal, nIR, errApunte, errYaw, yaw, recepVecinos);
  }

  /* --- ACTUALIZACIÓN DEL DEBUG WIFI (cada 1 segundo) --- */
  static unsigned long tiempo = 0;
  if (millis() - tiempo > 1000) {
    tiempo = millis();
    float errYaw = errorAngular(yaw);               // Para mostrar también en el debug web
    debugInfo =
      "Tiempo: " + String(millis()) + "\n" +
      "Memoria libre: " + String(ESP.getFreeHeap()) + "\n" +
      "Estado del robot: " + String(estadoActual) + "\n" +
      "Accion: " + nombre + "\n" +
      "Angulo: " + String(anguloIR) + "\n" +
      "Receptores Activos: " + String(nIR) + "\n" +
      "Yaw: " + String(yaw) + "\n" +
      "Error de apunte: " + String(errApunte) + "\n" +
      "Error de orientacion (porteria): " + String(errYaw) + "\n" +
      "Hay señal? " + String(haySenal ? "Si" : "No") + "\n" +
      "Receptores Vecinos: " + recepVecinos + "\n" +
      "DistFrente: " + String(distFrente) + " cm\n" +
      "DistAtras: " + String(distAtras) + " cm\n" +
      "DistIzq: " + String(distIzq) + " cm\n" +
      "DistDer: " + String(distDer) + " cm\n";
  }

#endif
  delay(25);   // Pequeña pausa para dar tiempo a otras tareas del sistema
}
