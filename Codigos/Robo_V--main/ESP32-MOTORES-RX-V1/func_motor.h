#ifndef FUNC_MOTOR_H
#define FUNC_MOTOR_H

void pwmInit(int pin);          // Inicializa el pin
void pwmWrite(int pin, int d);  // Escribe la velocidad

// Rutina que enciende una rueda específica
void rueda(int pwm, int in1, int in2, int v, bool inv);
// Sub-rutinas rápidas para no escribir tanto en el loop
void FL(int v);                               // Llama a la rueda Delantera Izquierda
void FR(int v);                               // Llama a la rueda Delantera Derecha
void RL(int v);                               // Llama a la rueda Trasera Izquierda
void RR(int v);                               // Llama a la rueda Trasera Derecha
void ruedas(int fl, int fr, int rl, int rr);  // Aplica valores a las 4 a la vez
void detener();                               // Frena las 4 ruedas y reinicia la aceleración
void girarEnSitio(int v);                     // Aplica fuerza contraria entre lados para pivotear sobre su eje
// Sistema antideslizamiento y aceleración controlada
void avanzarSuave(int objetivo, int corr);  // Recibe la velocidad máxima deseada y una corrección angular




#endif