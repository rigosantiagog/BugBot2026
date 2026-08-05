#ifndef FUNC_MOTOR_H
#define FUNC_MOTOR_H

void pwmInit(int pin);          // Inicializa un pin como PWM
void pwmWrite(int pin, int d);  // Escribe un valor PWM (0-255)

// Controla una rueda individual (dirección + velocidad)
void rueda(int pwm, int in1, int in2, int v, bool inv);

// Funciones rápidas para cada rueda
void FL(int v);
void FR(int v);
void RL(int v);
void RR(int v);

// Aplica velocidades a las cuatro ruedas simultáneamente
void ruedas(int fl, int fr, int rl, int rr);

void detener();          // Frenado suave (con rampa)
void frenar();           // Frenado brusco (inmediato)
void girarEnSitio(int v); // Gira sobre su eje (v positivo = derecha)
void avanzarSuave(int objetivo, int corr); // Avanza con aceleración controlada y corrección
void inicializarMotores(); // Configura los pines y PWM de los motores

#endif