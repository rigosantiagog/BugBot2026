/*
 * ============================================================================
 *  motores.h  —  BugBot 2026  v4
 *  Control de bajo nivel de los 4 motores DC con ruedas mecanum.
 *  Hardware: 2 drivers TB6612FNG + periférico LEDC del ESP32 (core 3.x)
 * ============================================================================
 *  RESPONSABILIDADES:
 *   - Inicializar GPIO y PWM de los 4 motores
 *   - Controlar cada rueda: dirección (IN1/IN2) + velocidad (PWM)
 *   - Cinemática mecanum: traduce (adelante, lateral, giro) → velocidades/rueda
 *   - Rampa de aceleración: evita picos de corriente al arrancar motores
 *
 *  TABLA DE VERDAD TB6612FNG:
 *   IN1=H IN2=L PWM>0 → gira sentido A    IN1=L IN2=H PWM>0 → gira sentido B
 *   IN1=H IN2=H       → freno corto real (short brake, cortocircuita el motor)
 *   IN1=L IN2=L       → stop de alta impedancia (Hi-Z, el motor QUEDA LIBRE
 *                        y sigue girando por inercia — NO usar para frenar)
 *   STBY=L → driver deshabilitado (ambos motores del driver, sin importar IN1/IN2)
 *
 *  COMPATIBILIDAD: ESP32 Arduino core 3.x
 *   ledcAttach(pin, freq, bits)  → configura PWM en el pin (canal automático)
 *   ledcWrite(pin, duty)         → escribe duty cycle usando el número de pin
 * ============================================================================
 */

#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>        // Tipos y funciones base del entorno Arduino
#include "config_robot.h"   // Pines y constantes del robot (única fuente de verdad)

// ============================================================================
// motores_inicializar()
// Configura GPIO y PWM. Llámala UNA VEZ en setup() antes de mover el robot.
// ============================================================================
void motores_inicializar() {
  Serial.println("[MOTORES] Configurando 4 motores y 2 drivers TB6612FNG...");

  // STBY controla ambos drivers con un cable en Y — inicia en LOW (apagado)
  pinMode(PIN_STBY_DRIVERS, OUTPUT);         // Pin de habilitación global como salida
  digitalWrite(PIN_STBY_DRIVERS, LOW);       // LOW = drivers en standby (seguro al inicio)
  Serial.println("[MOTORES] STBY=LOW — drivers deshabilitados durante init");

  // Pines de dirección Driver #1 (FR y RR — llantas derechas)
  pinMode(PIN_FR_IN1, OUTPUT);  // AIN1 del driver #1: dirección llanta FR
  pinMode(PIN_FR_IN2, OUTPUT);  // AIN2 del driver #1: dirección llanta FR
  pinMode(PIN_RR_IN1, OUTPUT);  // BIN1 del driver #1: dirección llanta RR
  pinMode(PIN_RR_IN2, OUTPUT);  // BIN2 del driver #1: dirección llanta RR
  Serial.println("[MOTORES] Driver #1 (FR/RR): pines de direccion configurados");

  // Pines de dirección Driver #2 (FL y RL — llantas izquierdas)
  pinMode(PIN_RL_IN1, OUTPUT);  // AIN1 del driver #2: dirección llanta RL
  pinMode(PIN_RL_IN2, OUTPUT);  // AIN2 del driver #2: dirección llanta RL
  pinMode(PIN_FL_IN1, OUTPUT);  // BIN1 del driver #2: dirección llanta FL
  pinMode(PIN_FL_IN2, OUTPUT);  // BIN2 del driver #2: dirección llanta FL
  Serial.println("[MOTORES] Driver #2 (FL/RL): pines de direccion configurados");

  // Configura PWM en los 4 pines de velocidad (API nueva del core 3.x)
  // ledcAttach(pin, frecuencia_Hz, resolución_bits) — el core asigna canal internamente
  ledcAttach(PIN_FR_PWM, PWM_FREQ_HZ, PWM_RESOLUCION_BITS); // PWM para llanta FR (GPIO25)
  ledcAttach(PIN_RR_PWM, PWM_FREQ_HZ, PWM_RESOLUCION_BITS); // PWM para llanta RR (GPIO33)
  ledcAttach(PIN_RL_PWM, PWM_FREQ_HZ, PWM_RESOLUCION_BITS); // PWM para llanta RL (GPIO13)
  ledcAttach(PIN_FL_PWM, PWM_FREQ_HZ, PWM_RESOLUCION_BITS); // PWM para llanta FL (GPIO19)
  Serial.print("[MOTORES] PWM configurado: ");
  Serial.print(PWM_FREQ_HZ);
  Serial.print(" Hz, ");
  Serial.print(PWM_RESOLUCION_BITS);
  Serial.println(" bits (0-255)");

  // Habilita ambos drivers — a partir de aquí los motores pueden responder
  digitalWrite(PIN_STBY_DRIVERS, HIGH);      // HIGH = drivers activos
  Serial.println("[MOTORES] STBY=HIGH — ambos drivers habilitados");
  Serial.println("[MOTORES] Inicializacion completa. Robot listo para moverse.");
}

// ============================================================================
// mover_rueda()  [uso interno, llamada desde mover_mecanum()]
// Controla una rueda individual: dirección + velocidad con signo.
//
// Parámetros:
//   pinIN1, pinIN2 → pines GPIO de dirección del driver
//   pinPWM         → pin GPIO del canal PWM del driver
//   velocidad      → -255 a +255  (signo = sentido, magnitud = rapidez)
//                    0 = freno suave  |  >0 = adelante  |  <0 = atrás
// ============================================================================
void mover_rueda(uint8_t pinIN1, uint8_t pinIN2, uint8_t pinPWM, int velocidad) {
  // Limita la velocidad al rango válido para proteger el driver y el motor
  velocidad = constrain(velocidad, -PWM_VELOCIDAD_MAX, PWM_VELOCIDAD_MAX);

  if (velocidad > 0) {
    digitalWrite(pinIN1, HIGH);        // IN1=HIGH → define sentido de giro A (adelante)
    digitalWrite(pinIN2, LOW);         // IN2=LOW  → complementario de IN1
    ledcWrite(pinPWM, velocidad);      // Duty cycle positivo (0-255)

  } else if (velocidad < 0) {
    digitalWrite(pinIN1, LOW);         // IN1=LOW  → define sentido de giro B (atrás)
    digitalWrite(pinIN2, HIGH);        // IN2=HIGH → complementario de IN1
    ledcWrite(pinPWM, -velocidad);     // PWM siempre positivo: negamos el valor

  } else {
    // --- CORRECCIÓN CRÍTICA vs version anterior ---
    // IN1=LOW, IN2=LOW en el TB6612FNG NO es un freno: es "Stop" de alta
    // impedancia (Hi-Z). El motor queda ELECTRICAMENTE DESCONECTADO y sigue
    // girando libre por inercia (coasting) — el comentario anterior decia
    // "freno suave" pero eso era incorrecto segun la hoja de datos del chip.
    //
    // El freno REAL del TB6612FNG ("short brake") es IN1=HIGH, IN2=HIGH:
    // cortocircuita las dos terminales del motor a traves del puente H,
    // disipando la energia cinetica y deteniendo la rueda casi al instante.
    //
    // Esto importa mucho en BugBot 2026 porque los movimientos se alternan
    // de direccion (ej. Adelante -> Atras) tras solo unos cientos de ms de
    // pausa. Si la rueda sigue girando por inercia (coasting) cuando se
    // comanda la reversa, el back-EMF del motor se SUMA al voltaje de la
    // bateria durante el cambio de polaridad ("plugging"), generando un
    // pico de corriente mucho mayor que un arranque normal desde reposo —
    // suficiente para colapsar el voltaje y reiniciar la ESP32 aunque la
    // rampa de aceleracion este activa (la rampa limita el duty cycle
    // promedio, pero no evita el pico instantaneo del cambio de polaridad).
    digitalWrite(pinIN1, HIGH);        // IN1=HIGH, IN2=HIGH → freno corto real (short brake)
    digitalWrite(pinIN2, HIGH);        // Cortocircuita el motor: frena casi instantaneamente
    ledcWrite(pinPWM, 0);              // PWM=0 → sin voltaje adicional durante el frenado
  }
}

// ============================================================================
// motores_detener()
// Frena las 4 ruedas simultáneamente con FRENO CORTO REAL (IN1=IN2=HIGH),
// no coasting. Esto detiene las ruedas casi al instante en vez de dejarlas
// girar libres por inercia — crítico para evitar picos de corriente por
// "plugging" cuando el siguiente movimiento invierte la dirección.
// Úsala siempre al terminar un movimiento.
// ============================================================================
void motores_detener() {
  mover_rueda(PIN_FR_IN1, PIN_FR_IN2, PIN_FR_PWM, 0); // Frena llanta delantera derecha
  mover_rueda(PIN_RR_IN1, PIN_RR_IN2, PIN_RR_PWM, 0); // Frena llanta trasera derecha
  mover_rueda(PIN_RL_IN1, PIN_RL_IN2, PIN_RL_PWM, 0); // Frena llanta trasera izquierda
  mover_rueda(PIN_FL_IN1, PIN_FL_IN2, PIN_FL_PWM, 0); // Frena llanta delantera izquierda
}

// ============================================================================
// mover_mecanum()
// Cinemática inversa del chasis mecanum: traduce comandos de movimiento
// normalizados en velocidades individuales para cada rueda.
//
// Parámetros normalizados (-1.0 a +1.0):
//   adelante  → +1.0 = avanzar al frente    -1.0 = retroceder
//   lateral   → +1.0 = deslizar a la derecha  -1.0 = deslizar a la izquierda
//   giro      → +1.0 = rotar horario (in situ)  -1.0 = rotar antihorario
//   velocidadBase → escala máxima del PWM (0-255)
//
// FÓRMULAS DE CINEMÁTICA MECANUM (ruedas con rodillos a 45°):
//   FR = adelante - lateral - giro    FL = adelante + lateral + giro
//   RR = adelante + lateral - giro    RL = adelante - lateral + giro
// ============================================================================
void mover_mecanum(float adelante, float lateral, float giro, int velocidadBase) {
  // Calcula velocidad normalizada de cada rueda con las fórmulas mecanum
  float fr = adelante - lateral - giro;  // Llanta delantera derecha
  float fl = adelante + lateral + giro;  // Llanta delantera izquierda
  float rr = adelante + lateral - giro;  // Llanta trasera derecha
  float rl = adelante - lateral + giro;  // Llanta trasera izquierda

  // Normalización: si alguna rueda necesita más del 100%, escala todas
  // proporcionalmente para mantener la dirección de movimiento correcta
  float maximo = max(max(fabs(fr), fabs(fl)), max(fabs(rr), fabs(rl)));
  if (maximo > 1.0f) {   // Solo normaliza si algún valor supera el límite
    fr /= maximo;         // Divide todas por el máximo para que queden en [-1, 1]
    fl /= maximo;
    rr /= maximo;
    rl /= maximo;
  }

  // Convierte velocidades normalizadas [-1.0, 1.0] a PWM con signo [-255, 255]
  // y envía la señal física a cada driver
  mover_rueda(PIN_FR_IN1, PIN_FR_IN2, PIN_FR_PWM, (int)(fr * velocidadBase)); // FR
  mover_rueda(PIN_FL_IN1, PIN_FL_IN2, PIN_FL_PWM, (int)(fl * velocidadBase)); // FL
  mover_rueda(PIN_RR_IN1, PIN_RR_IN2, PIN_RR_PWM, (int)(rr * velocidadBase)); // RR
  mover_rueda(PIN_RL_IN1, PIN_RL_IN2, PIN_RL_PWM, (int)(rl * velocidadBase)); // RL
}

// ============================================================================
// mover_mecanum_con_rampa()
// Igual que mover_mecanum() pero con RAMPA DE ACELERACIÓN al inicio.
//
// PROPÓSITO CRÍTICO: evitar el pico de corriente que resetea la ESP32.
// Cuando 4 motores DC arrancan de 0 a plena velocidad simultáneamente,
// el pico de corriente puede superar 4-5 A durante 50-100 ms. Este pico:
//   1. Induce ruido en los cables paralelos → corrompe el bus I2C
//   2. Puede colapsar el voltaje USB → resetea la ESP32 (brownout)
//
// La rampa sube gradualmente de VELOCIDAD_ARRANQUE a velocidadObjetivo
// en RAMPA_DURACION_MS milisegundos, distribuyendo el pico en el tiempo.
// Durante la rampa también actualiza el giroscopio para no perder lecturas.
//
// Parámetros: igual que mover_mecanum() más un puntero a función para
// actualizar el giroscopio durante la rampa sin crear dependencia circular.
// ============================================================================
void mover_mecanum_con_rampa(float adelante, float lateral, float giro,
                              int velocidadObjetivo, void (*actualizarGiro)()) {
  Serial.print("[RAMPA] Acelerando de PWM=");
  Serial.print(VELOCIDAD_ARRANQUE);   // Velocidad inicial de la rampa
  Serial.print(" a PWM=");
  Serial.print(velocidadObjetivo);    // Velocidad objetivo al final
  Serial.print(" en ");
  Serial.print(RAMPA_DURACION_MS);    // Duración de la rampa
  Serial.println(" ms...");

  unsigned long tInicioRampa = millis(); // Tiempo de inicio de la rampa

  // Bucle de rampa: sube la velocidad gradualmente durante RAMPA_DURACION_MS
  while (millis() - tInicioRampa < RAMPA_DURACION_MS) {
    // Calcula qué fracción del tiempo de rampa ha pasado (0.0 a 1.0)
    float progreso = (float)(millis() - tInicioRampa) / (float)RAMPA_DURACION_MS;
    progreso = constrain(progreso, 0.0f, 1.0f); // Asegura que esté en [0, 1]

    // Interpola linealmente entre VELOCIDAD_ARRANQUE y velocidadObjetivo
    // Al inicio progreso≈0 → velocidad≈ARRANQUE
    // Al final progreso≈1 → velocidad≈OBJETIVO
    int velocidadActual = VELOCIDAD_ARRANQUE +
                          (int)((velocidadObjetivo - VELOCIDAD_ARRANQUE) * progreso);

    mover_mecanum(adelante, lateral, giro, velocidadActual); // Aplica velocidad actual

    if (actualizarGiro != nullptr) {
      actualizarGiro(); // Actualiza el giroscopio durante la rampa (puntero a función)
    }
  }

  // Al terminar la rampa, aplica la velocidad objetivo final exacta
  mover_mecanum(adelante, lateral, giro, velocidadObjetivo);
  Serial.println("[RAMPA] Velocidad objetivo alcanzada.");
}

#endif // MOTORES_H
