/**
 * @file func_motor.cpp
 * @brief Implementación del control de motores con driver TB6612FNG.
 * 
 * El chasis es Mecanum: para avanzar, todas las ruedas giran hacia adelante.
 * Para girar en sitio, las ruedas izquierdas giran hacia adelante y las derechas hacia atrás.
 * La corrección (corr) se aplica sumando a un lado y restando al otro.
 */
#include "func_motor.h"
#include "config.h"

// ----- Inicialización PWM -----
void pwmInit(int pin) {
  ledcAttach(pin, PWM_FREQ, PWM_RES);   // Adjunta el pin al canal LEDC con frecuencia y resolución
}

void pwmWrite(int pin, int d) {
  ledcWrite(pin, d);                    // Escribe el valor PWM en el pin
}

// ----- Control de una rueda -----
void rueda(int pwm, int in1, int in2, int v, bool inv) {
  if (inv) v = -v;                               // Invierte el sentido si está marcado
  v = constrain(v, -PWM_MAX, PWM_MAX);           // Limita al rango seguro

  if (v > 0) {                                   // Giro hacia adelante
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmWrite(pwm, v);
  } else if (v < 0) {                            // Giro hacia atrás
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwmWrite(pwm, -v);
  } else {                                       // Detenido
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(pwm, 0);
  }
}

// ----- Funciones rápidas para cada rueda (pasan los pines definidos en config.h) -----
void FL(int v) { rueda(FL_PWM, FL_IN1, FL_IN2, v, INVERTIR_FL); }
void FR(int v) { rueda(FR_PWM, FR_IN1, FR_IN2, v, INVERTIR_FR); }
void RL(int v) { rueda(RL_PWM, RL_IN1, RL_IN2, v, INVERTIR_RL); }
void RR(int v) { rueda(RR_PWM, RR_IN1, RR_IN2, v, INVERTIR_RR); }

// ----- Aplica velocidades a las cuatro ruedas -----
void ruedas(int fl, int fr, int rl, int rr) {
  FL(fl);
  FR(fr);
  RL(rl);
  RR(rr);
}

// ----- Frenado suave (con rampa) -----
void detener() {
  if (velAvanceActual > 0) {
    velAvanceActual -= RAMPA_PASO;                // Decrementa gradualmente
    if (velAvanceActual < 0) velAvanceActual = 0;
  } else if (velAvanceActual < 0) {
    velAvanceActual += RAMPA_PASO;                // Si es negativo, lo sube (no debería ocurrir)
    if (velAvanceActual > 0) velAvanceActual = 0;
  }
  ruedas(velAvanceActual, velAvanceActual, velAvanceActual, velAvanceActual);
}

// ----- Frenado brusco (inmediato) -----
void frenar() {
  velAvanceActual = 0;                            // Reinicia la rampa
  ruedas(0, 0, 0, 0);                             // Apaga todas las ruedas
}

// ----- Giro en sitio -----
void girarEnSitio(int v) {
  ruedas(v, -v, v, -v);                           // Lado izquierdo y derecho opuestos
}

// ----- Avance con aceleración controlada y corrección angular -----
void avanzarSuave(int objetivo, int corr) {
  // Acelera o desacelera gradualmente hacia el objetivo
  if (velAvanceActual < objetivo) velAvanceActual += RAMPA_PASO;
  if (velAvanceActual > objetivo) velAvanceActual = objetivo;

  // Aplica la corrección: lado izquierdo +corr, lado derecho -corr
  // Para chasis Mecanum, ambos lados reciben la misma corrección para curvar
  ruedas(velAvanceActual + corr, velAvanceActual - corr,
         velAvanceActual + corr, velAvanceActual - corr);
}

// ----- Inicialización de pines y PWM de los motores -----
void inicializarMotores() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);                         // Deshabilita los drivers temporalmente

  // Pines de dirección (IN1, IN2) de ambos drivers
  int pd[] = { FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2 };
  for (int p : pd) pinMode(p, OUTPUT);            // Configura todos como salidas

  // Inicializa los canales PWM para las cuatro ruedas
  pwmInit(FL_PWM);
  pwmInit(FR_PWM);
  pwmInit(RL_PWM);
  pwmInit(RR_PWM);

  frenar();                                        // Asegura que todas las ruedas estén detenidas
  digitalWrite(STBY, HIGH);                        // Habilita los drivers
}