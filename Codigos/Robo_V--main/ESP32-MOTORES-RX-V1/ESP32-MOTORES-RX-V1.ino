#include <Arduino.h>  // Incluye las rutinas principales de la ESP32
#include <Wire.h>     // Incluye la librería para leer el giroscopio por I2C
#include "func_giro.h"
#include "func_motor.h"
#include "config.h"
#include <WiFi.h>
#include <WebServer.h>
#define MODO 1

const char* ssid = "ESP32_DEBUG";
const char* password = "12345678";

WebServer server(80);

String debugInfo = "Iniciando...\n";

void handleRoot() {
  server.send(200, "text/plain", debugInfo);
}

/* ===================== CEREBRO UART =========================== */
void leerUART() {
  while (Enlace.available()) {
    char c = Enlace.read();
    if (c == '\n') {
      uartBuffer[bufIndex] = '\0';
      float a;
      int c_st, n, df, db, dl, dr;
      char rV[16];
      // Descompone la nueva trama con las distancias F B L R
      if (
      sscanf(uartBuffer, "A:%f C:%d N:%d F:%d B:%d L:%d R:%d ", &a, &c_st, &n, &df, &db, &dl, &dr) >= 7) {

        char* ptr = strstr(uartBuffer, "R:");
        if (ptr != NULL) {
          // saltar "R:" y el número de distDer
          ptr = strchr(ptr, ' ');
          if (ptr != NULL) {
            ptr++;  // ahora apunta al primer booleano

            for (int i = 0; i < 16; i++) {

              rV[i] = atoi(ptr);
              // avanzar hasta la siguiente coma
              ptr = strchr(ptr, ',');
              if (ptr != NULL)
                ptr++;
            }
          }
        }

        anguloIR = a;
        estadoIR = c_st;
        nIR = n;
        distFrente = df;
        distAtras = db;
        distIzq = dl;
        distDer = dr;  // Asigna distancias del radar
        recepVecinos = rV;
        ultimoDato = millis();
      }
      bufIndex = 0;
    } else if (c != '\r' && bufIndex < 127) {
      uartBuffer[bufIndex++] = c;
    }
  }
}

/* ===================== SETUP DEL PARTIDO ================================== */
void setup() {                                                      // Función que corre una sola vez al encender el switch
  Serial.begin(115200);                                             // Enciende el reporte hacia la laptop
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);                  // Enciende el oído del robot a 38400 baudios
  delay(300);                                                       // Pausa existencial para que la circuitería despierte
  Serial.println("\n=== ESP32 MOTORES v8 (Búsqueda Táctica) ===");  // Reporte

  pinMode(STBY, OUTPUT);    // Declara el pin de seguridad de motores como salida de voltaje
  digitalWrite(STBY, LOW);  // Baja el voltaje a cero para que los motores no puedan girar (Seguro activado)

  int pd[] = { FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2 };  // Lista de pines de dirección
  for (int p : pd) pinMode(p, OUTPUT);                                            // Bucle rápido que declara los 8 pines como salidas

  pwmInit(FL_PWM);
  pwmInit(FR_PWM);
  pwmInit(RL_PWM);
  pwmInit(RR_PWM);           // Inicia los 4 canales de pulsos eléctricos
  
  detener();                 // Garantiza por software que el pulso sea 0
  digitalWrite(STBY, HIGH);  // Quita el seguro, el robot ya tiene fuerza en las llantas

  if (mpuInit()) Serial.println("MPU OK.");        // Reporta si la brújula existe
  else Serial.println("AVISO: MPU no responde.");  // Alarma si hay cables desconectados

  Serial.println(">> BLOQUEO: Calibrando norte estático. NO mover el carro.");
  calibrarGyro();               // Toma las 500 muestras estáticas de calibración
  yaw = 0.0;                    // Fija ese punto como el Norte Oficial (0 grados)
  tPrev = micros();             // Inicia el reloj de integración
  unsigned long t0 = millis();  // Inicia un cronómetro para los 4 segundos

  while (millis() - t0 < T_QUIETO_MS) {  // Queda atrapado aquí durante 4000 milisegundos
    actualizarRumbo();                   // Mantiene la matemática corriendo para limpiar ruidos iniciales
    detener();                           // Asegura de nuevo estar frenado
    delay(5);                            // Pausa ligera
  }

  Serial.println(">> Norte fijado con éxito. Entrando en Modo de Espera de Pelota.");
  estadoActual = ESPERANDO_PELOTA;  // Configura el estado inicial antes de entrar al loop infinito

  // Crear red WiFi propia
  WiFi.softAP(ssid, password);

  Serial.print("IP del ESP32: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.begin();

  debugInfo += "Servidor iniciado\n";
}

/* ===================== LOOP DEL PARTIDO ================================== */
void loop() {  // Función de pensamiento que ocurre miles de veces por segundo
  server.handleClient();
  actualizarRumbo();  // Actualiza brújula obligatoriamente
  leerUART();         // Extrae los datos obligatoriamente

  // Condición viva: Si dice '1' y el mensaje no tiene más de 400ms de antigüedad
  bool haySenal = (estadoIR == 1) && (millis() - ultimoDato < 400);

  if (haySenal) {                   // Si hay visión...
    tUltimaVezPelota = millis();    // Actualiza la hora de posesión
    pelotaPerdidaReciente = false;  // Baja la bandera de pérdida
  }

#if MODO == 0
  /* --- MODO CALIBRACIÓN VISUAL (Puro Diagnóstico) --- */
  detener();                                                                // Motor cortado permanentemente
  static unsigned long t = 0;                                               // Cronómetro local
  if (millis() - t > 250) {                                                 // Si pasó un cuarto de segundo...
    t = millis();                                                           // Resetea cronómetro local
    if (haySenal) Serial.printf(">> Ángulo detectado = %.1f\n", anguloIR);  // Muestra datos en PC
    else Serial.println(">> Sin señal IR");
  }
#else
  /* --- MODO COMPETENCIA DESTRUCTORA --- */

  // Extrae la diferencia de grados (Si no hay pelota, fuerza a cero para no hacer cálculos locos)
  float errApunte = haySenal ? errorAngular(anguloIR) : 0;

  // Condición severa de posesión: Hay luz Y está dentro del cono frontal Y satura suficientes receptores
  bool pelotaPegada = haySenal && (abs(errApunte) <= TOL_APUNTADO) && (nIR >= N_CAPTURA);

  /* --- CEREBRO SUPERIOR: CAMBIO DE ESTADOS Y BANDERAS --- */
  if (estadoActual == ESPERANDO_PELOTA) {                                               // Si el robot está en la línea de salida bloqueado...
    if (haySenal) estadoActual = PERSIGUIENDO;                                          // Desbloquéalo y ponlo a atacar al ver la luz de inicio
  } else if (pelotaPegada && estadoActual != REGRESANDO && estadoActual != FRENANDO) {  // Si ya la atrapó...
    estadoActual = FRENANDO;                                                            // Dispara los frenos antes de ir a portería
    tFrenoIniciado = millis();                                                          // Anota la hora de inicio de frenado
  } else if (estadoActual == REGRESANDO) {                                              // Si iba a la portería...
    if (millis() - tUltimaVezPelota > 1000) {                                           // ...pero lleva más de 1 segundo sin sentirla...
      estadoActual = BUSCANDO;                                                          // Aborta la misión y vuelve a patrullar
      pelotaPerdidaReciente = true;                                                     // Sube la bandera de reinicio táctico
      pasoBusqueda = 0;                                                                 // Fuerza al patrón de búsqueda a iniciar desde el paso cero (Pausa)
      tBusqueda = millis();                                                             // Resetea el reloj del patrón
    }
  } else if (estadoActual != FRENANDO && estadoActual != REGRESANDO) {  // Para todas las demás circunstancias ordinarias...
    if (haySenal) {                                                     // Si la ve...
      estadoActual = PERSIGUIENDO;                                      // ...ataca.
    } else {                                                            // Si no la ve...
      if (!pelotaPerdidaReciente) {                                     // ...y acaba de suceder este instante...
        pelotaPerdidaReciente = true;                                   // Sube la bandera táctica
        pasoBusqueda = 0;                                               // Arranca el patrón de búsqueda desde el paso cero
        tBusqueda = millis();                                           // Resetea el reloj del patrón
      }
      estadoActual = BUSCANDO;  // Ordena ejecutar la fase de búsqueda
    }
  }

  const char* nombre = "?";  // Crea un texto vacío para almacenar el nombre de la acción en la terminal

  /* --- CEREBRO FÍSICO: CONTROL DE ACTUADORES SEGÚN EL ESTADO --- */
  switch (estadoActual) {

    case ESPERANDO_PELOTA:                        // Acciones de bloqueo
      detener();                                  // Se queda petrificado
      nombre = "BLOQUEADO (Esperando arranque)";  // Etiqueta diagnóstico
      break;

    case BUSCANDO:                                        // NUEVO PATRÓN TÁCTICO SECTORIAL
      if (pasoBusqueda == 0) {                            // PASO 0
        detener();                                        // Frena el chasis
        nombre = "BUSQUEDA: Pausa 1s (Escuchando ecos)";  // Diagnóstico
        if (millis() - tBusqueda >= 1000) {               // Si ya pasó 1 segundo (1000ms)...
          pasoBusqueda = 1;                               // Avanza a la siguiente instrucción
          tBusqueda = millis();                           // Resetea cronómetro de la sub-rutina
        }
      } else if (pasoBusqueda == 1) {             // PASO 1
        avanzarSuave(VEL_BUSCAR, 0);              // Empuja hacia adelante
        nombre = "BUSQUEDA: Avance de Patrulla";  // Diagnóstico
        if (millis() - tBusqueda >= 600) {        // Si ya pasaron 600ms avanzando...
          pasoBusqueda = 2;                       // Avanza de instrucción
          tBusqueda = millis();                   // Resetea cronómetro
        }
      } else if (pasoBusqueda == 2) {       // PASO 2
        detener();                          // Detiene inercia
        nombre = "BUSQUEDA: Pausa Visual";  // Diagnóstico
        if (millis() - tBusqueda >= 300) {  // Después de 300ms...
          pasoBusqueda = 3;                 // Avanza
          tBusqueda = millis();             // Resetea
        }
      } else if (pasoBusqueda == 3) {          // PASO 3
        girarEnSitio(VEL_GIRO);                // Fuerza rotación hacia la derecha
        nombre = "BUSQUEDA: Escaneo Derecha";  // Diagnóstico
        if (millis() - tBusqueda >= 400) {     // Después de girar 400ms...
          pasoBusqueda = 4;                    // Avanza
          tBusqueda = millis();                // Resetea
        }
      } else if (pasoBusqueda == 4) {              // PASO 4
        avanzarSuave(VEL_BUSCAR, 0);               // Avanza hacia esa nueva dirección
        nombre = "BUSQUEDA: Avance Diagonal Der";  // Diagnóstico
        if (millis() - tBusqueda >= 600) {         // Después de avanzar 600ms...
          pasoBusqueda = 5;                        // Avanza
          tBusqueda = millis();                    // Resetea
        }
      } else if (pasoBusqueda == 5) {       // PASO 5
        detener();                          // Detiene inercia
        nombre = "BUSQUEDA: Pausa Visual";  // Diagnóstico
        if (millis() - tBusqueda >= 300) {  // Después de 300ms...
          pasoBusqueda = 6;                 // Avanza
          tBusqueda = millis();             // Resetea
        }
      } else if (pasoBusqueda == 6) {            // PASO 6
        girarEnSitio(-VEL_GIRO);                 // Rota hacia la Izquierda
        nombre = "BUSQUEDA: Escaneo Izquierda";  // Diagnóstico
        if (millis() - tBusqueda >= 800) {       // El tiempo es doble (800) para cruzar todo el frente y mirar al otro lado
          pasoBusqueda = 7;                      // Avanza
          tBusqueda = millis();                  // Resetea
        }
      } else if (pasoBusqueda == 7) {              // PASO 7 FINAL
        avanzarSuave(VEL_BUSCAR, 0);               // Avanza hacia la izquierda
        nombre = "BUSQUEDA: Avance Diagonal Izq";  // Diagnóstico
        if (millis() - tBusqueda >= 600) {         // Después de avanzar 600ms...
          pasoBusqueda = 0;                        // DECRETA UN LOOP DE BÚSQUEDA. Vuelve al paso 0 y repite todo.
          tBusqueda = millis();                    // Resetea cronómetro.
        }
      }
      break;  // Rompe el bloque de evaluación

    case PERSIGUIENDO:                                            // Acciones de ataque
      if (abs(errApunte) > TOL_APUNTADO) {                        // Si está muy chueco respecto a la pelota
        int sentido = (errApunte > 0) ? VEL_GIRO : -VEL_GIRO;     // Si la pelota esta en TOL_APUNTADO a 180 grados, gira derecha, de 181 a  
        girarEnSitio(sentido);                                    // Ejecuta el giro físico
        tSenalEstable = 0;                                        // Rompe el tiempo de confirmación para que no ataque a lo tonto
        nombre = "PERSIGUE -> ENCUADRANDO";                       // Diagnóstico
      } else {                                                    // Si ya la tiene frente a los ojos...
        if (tSenalEstable == 0) tSenalEstable = millis();         // Inicia el reloj de validación de sombras
        if (millis() - tSenalEstable < T_CONFIRMA_MS) {           // Mientras no se cumpla el medio segundo de seguridad...
          detener();                                              // ...no hace nada físicamente
          nombre = "FILTRO VISUAL (Confirmando luz)";             // Diagnóstico
        } else {                                                  // Si la señal ya superó el medio segundo ininterrumpido...
          int corr = constrain((int)(errApunte * 2.0), -30, 30);  // Crea un ajuste cinemático de máximo 30 puntos de PWM
          avanzarSuave(VEL_AVANCE, corr);                         // Ordena a los motores sumarle ese ajuste a un lado y restarlo al otro
          nombre = "ATACANDO PELOTA A FONDO";                     // Diagnóstico
        }
      }
      break;  // Fin del bloque

    case FRENANDO:                            // Acción de protección electrónica
      detener();                              // Corta el pulso de las ruedas. El chasis derrapa físicamente un poco.
      nombre = "FRENADO ELECTROMAGNÉTICO";    // Diagnóstico
      if (millis() - tFrenoIniciado > 250) {  // Al pasar un cuarto de segundo (250ms)...
        estadoActual = REGRESANDO;            // Es seguro encender de nuevo los transistores y rotar a portería
      }
      break;

    case REGRESANDO:
      {                                                          // Fase de anotación
        float errYaw = errorAngular(yaw);                        // Extrae la diferencia contra la pared Norte
        if (abs(errYaw) > TOL_PORTERIA) {                        // Si la pared Norte no está de frente todavía...
          int sentidoYaw = (errYaw < 0) ? VEL_GIRO : -VEL_GIRO;  // Valor positivo: Gira derecha, negativo: Gira izquierda
          girarEnSitio(sentidoYaw);                              // Empieza a girar con la pelota atorada en el bumper
          nombre = "PORTERIA -> GIRANDO BRÚJULA";                // Diagnóstico
        } else {                                                 // Si la pared ya está de frente a nosotros...
          int corr = constrain((int)(errYaw * -2.0), -30, 30);   // Calcula desviación y crea ajuste
          avanzarSuave(VEL_AVANCE, corr);                        // Acelera en línea recta usando el ajuste para no curvarse
          nombre = "PORTERIA -> REMATE DE FRENTE";               // Diagnóstico
        }
        break;  // Fin del bloque
      }
  }

  // --- REPORTE CONSTANTE A LA COMPUTADORA ---
  static unsigned long t = 0;                                                                  // Reloj exclusivo de la pantalla serial
  if (millis() - t > 250) {                                                                    // Imprime la información solo 4 veces por segundo para no trabar el IDE
    t = millis();                                                                              // Resetea reloj de pantalla
    Serial.printf("%s | AnguloIR=%.1f Senal=%d N_Sens=%d ErrAp=%.1f GyroYaw=%.1f Veci:%s \n",  // Imprime toda la línea
                  nombre, anguloIR, haySenal, nIR, errApunte, yaw, recepVecinos);
  }



  // Ejemplo de actualización
  static unsigned long tiempo = 0;

  if (millis() - tiempo > 1000) {
    tiempo = millis();

    debugInfo =
      "Tiempo: " + String(millis()) + "\n" + "Memoria libre: " + String(ESP.getFreeHeap()) + "\n" + "Estado del robot: " + String(estadoActual) + "\n" + "Accion: " + nombre + "\n" + "Angulo: " + String(anguloIR) + "\n" + "Receptores Activos: " + String(nIR) + "\n" + "Yaw: " + String(yaw) + "\n" + "Error de apunte: " + String(errApunte) + "\n" + "Hay señal? " + String(haySenal ? "Si" : "No") + "\n" + "Receptores Vecinos: " + recepVecinos + "\n";
  }
#endif  // Cierra las directivas de los modos de competencia y calibración



  delay(25);  // Pequeña tregua al procesador Dual Core para ejecutar rutinas ocultas de la placa
}
