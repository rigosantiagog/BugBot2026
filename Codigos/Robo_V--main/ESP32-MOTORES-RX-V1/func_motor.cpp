#include "func_motor.h"
#include "config.h"

void pwmInit(int pin) {
  ledcAttach(pin, PWM_FREQ, PWM_RES);
}  // Inicializa el pin
void pwmWrite(int pin, int d) {
  ledcWrite(pin, d);
}  // Escribe la velocidad

// Rutina que enciende una rueda específica
void rueda(int pwm, int in1, int in2, int v, bool inv) {  // Recibe los 3 pines, la velocidad y la inversión
  if (inv) v = -v;                                        // Si la inversión es verdadera, voltea el signo matemático
  v = constrain(v, -PWM_MAX, PWM_MAX);                    // Evita que un error matemático envíe más de 182 de PWM
  if (v > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmWrite(pwm, v);
  }  // Configuración de giro hacia adelante
  else if (v < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwmWrite(pwm, -v);
  }  // Configuración de giro hacia atrás
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(pwm, 0);
  }  // Apaga el giro por completo
}

// Sub-rutinas rápidas para no escribir tanto en el loop
void FL(int v) {
  rueda(FL_PWM, FL_IN1, FL_IN2, v, INVERTIR_FL);
}  // Llama a la rueda Delantera Izquierda
void FR(int v) {
  rueda(FR_PWM, FR_IN1, FR_IN2, v, INVERTIR_FR);
}  // Llama a la rueda Delantera Derecha
void RL(int v) {
  rueda(RL_PWM, RL_IN1, RL_IN2, v, INVERTIR_RL);
}  // Llama a la rueda Trasera Izquierda
void RR(int v) {
  rueda(RR_PWM, RR_IN1, RR_IN2, v, INVERTIR_RR);
}  // Llama a la rueda Trasera Derecha

void ruedas(int fl, int fr, int rl, int rr) {
  FL(fl);
  FR(fr);
  RL(rl);
  RR(rr);
}  // Aplica valores a las 4 a la vez
void detener() {
  ruedas(0, 0, 0, 0);
  velAvanceActual = 0;
}  // Frena las 4 ruedas y reinicia la aceleración
void girarEnSitio(int v) {
  ruedas(v, -v, v, -v);
}  // Si se le da un valor positivo: Gira a la derecha, negativo: Gira a la izquierda

// Sistema antideslizamiento y aceleración controlada
void avanzarSuave(int objetivo, int corr) {                       // Recibe la velocidad máxima deseada y una corrección angular
  if (velAvanceActual < objetivo) velAvanceActual += RAMPA_PASO;  // Si va más lento, acelera sumando 4
  if (velAvanceActual > objetivo) velAvanceActual = objetivo;     // Si se pasó, lo recorta al máximo permitido
  ruedas(velAvanceActual + corr, velAvanceActual - corr,          // Aplica la aceleración y tuerce la llanta según el error (corr)
         velAvanceActual + corr, velAvanceActual - corr);         // Valor positivo: Avanza mientras gira a la derecha, negativo: Avanza mientras gira a la izqueirda
}