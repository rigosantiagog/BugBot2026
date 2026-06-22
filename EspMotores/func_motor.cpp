/**
 * @file func_motor.cpp
 * @brief Implementación de las funciones de control de motores.
 * 
 * Utiliza los pines y constantes definidos en config.h.
 * Los motores se controlan mediante el driver TB6612FNG.
 * 
 * El chasis es Mecanum: para avanzar, todas las ruedas giran hacia adelante.
 * Para girar en sitio, las ruedas izquierdas giran hacia adelante y las derechas hacia atrás (o viceversa).
 * La corrección (corr) se aplica sumando a un lado y restando al otro para hacer giros suaves.
 */

#include "func_motor.h"
#include "config.h"

/**
 * @brief Inicializa un pin como canal PWM.
 * @param pin GPIO a configurar.
 */
void pwmInit(int pin) {
  ledcAttach(pin, PWM_FREQ, PWM_RES);
}

/**
 * @brief Escribe un valor PWM en un pin.
 * @param pin GPIO.
 * @param d  Valor (0-255).
 */
void pwmWrite(int pin, int d) {
  ledcWrite(pin, d);
}

/**
 * @brief Controla una rueda con dirección y velocidad.
 * @param pwm  Pin PWM.
 * @param in1  Pin de dirección 1.
 * @param in2  Pin de dirección 2.
 * @param v    Velocidad (positiva = adelante, negativa = atrás). Se satura a PWM_MAX.
 * @param inv  Si true, invierte el sentido (para motores montados al revés).
 */
void rueda(int pwm, int in1, int in2, int v, bool inv) {
  if (inv) v = -v;                               // Invierte si es necesario
  v = constrain(v, -PWM_MAX, PWM_MAX);           // Limita al rango seguro

  if (v > 0) {                                   // Adelante
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmWrite(pwm, v);
  } else if (v < 0) {                            // Atrás
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwmWrite(pwm, -v);
  } else {                                       // Detenido (ambos pines en LOW, PWM=0)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(pwm, 0);
  }
}

// --- Funciones rápidas para cada rueda ---
void FL(int v) { rueda(FL_PWM, FL_IN1, FL_IN2, v, INVERTIR_FL); }
void FR(int v) { rueda(FR_PWM, FR_IN1, FR_IN2, v, INVERTIR_FR); }
void RL(int v) { rueda(RL_PWM, RL_IN1, RL_IN2, v, INVERTIR_RL); }
void RR(int v) { rueda(RR_PWM, RR_IN1, RR_IN2, v, INVERTIR_RR); }

/**
 * @brief Aplica velocidades a las cuatro ruedas.
 */
void ruedas(int fl, int fr, int rl, int rr) {
  FL(fl);
  FR(fr);
  RL(rl);
  RR(rr);
}

/**
 * @brief Frenado suave: decrementa la velocidad gradualmente.
 *        Reduce velAvanceActual en RAMPA_PASO hasta 0.
 */
void detener() {
  if (velAvanceActual > 0) {
    velAvanceActual -= RAMPA_PASO;
    if (velAvanceActual < 0) velAvanceActual = 0;
  } else if (velAvanceActual < 0) {
    velAvanceActual += RAMPA_PASO;
    if (velAvanceActual > 0) velAvanceActual = 0;
  }
  ruedas(velAvanceActual, velAvanceActual, velAvanceActual, velAvanceActual);
}

/**
 * @brief Frenado brusco: detiene todas las ruedas inmediatamente.
 *        Pone velAvanceActual a 0 y escribe 0 en todas las ruedas.
 *        Se usa para el remate a portería o paradas de emergencia.
 */
void frenar() {
  velAvanceActual = 0;
  ruedas(0, 0, 0, 0);
}

/**
 * @brief Gira sobre el eje central aplicando velocidades opuestas.
 * @param v  Velocidad (positiva = derecha, negativa = izquierda).
 */
void girarEnSitio(int v) {
  ruedas(v, -v, v, -v);
}

/**
 * @brief Avanza con aceleración controlada y corrección angular.
 * @param objetivo  Velocidad máxima deseada.
 * @param corr      Corrección (se suma a la izquierda y resta a la derecha).
 */
void avanzarSuave(int objetivo, int corr) {
  // Acelera o desacelera hacia el objetivo
  if (velAvanceActual < objetivo) velAvanceActual += RAMPA_PASO;
  if (velAvanceActual > objetivo) velAvanceActual = objetivo;
  // Aplica la corrección: lado izquierdo +corr, lado derecho -corr
  ruedas(velAvanceActual + corr, velAvanceActual - corr,
         velAvanceActual + corr, velAvanceActual - corr);
}

void inicializarMotores() {
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
}
