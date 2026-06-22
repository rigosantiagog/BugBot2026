/**
 * @file func_motor.h
 * @brief Declaraciones de las funciones de control de motores.
 * 
 * Proporciona:
 * - Inicialización PWM.
 * - Control individual de ruedas.
 * - Movimientos combinados (avance, giro, detención suave y brusca).
 * 
 * Hardware: 2x TB6612FNG, 4x Motores DC con ruedas Mecanum.
 */

#ifndef FUNC_MOTOR_H
#define FUNC_MOTOR_H

void pwmInit(int pin);          // Inicializa un pin como PWM con la frecuencia y resolución definidas
void pwmWrite(int pin, int d);  // Escribe un valor de PWM (0-255)

// Controla una rueda específica (dirección + velocidad)
void rueda(int pwm, int in1, int in2, int v, bool inv);

// Funciones rápidas para cada rueda
void FL(int v);   // Frontal Izquierda
void FR(int v);   // Frontal Derecha
void RL(int v);   // Trasera Izquierda
void RR(int v);   // Trasera Derecha

// Aplica velocidades a las cuatro ruedas a la vez
void ruedas(int fl, int fr, int rl, int rr);

/**
 * @brief Frenado suave: reduce gradualmente la velocidad hasta 0.
 *        Utiliza la rampa definida en RAMPA_PASO.
 */
void detener();

/**
 * @brief Frenado brusco: pone todas las ruedas a 0 inmediatamente.
 *        Reinicia la variable de rampa.
 */
void frenar();

/**
 * @brief Gira sobre el eje central.
 * @param v  Velocidad (positiva = derecha, negativa = izquierda).
 */
void girarEnSitio(int v);

/**
 * @brief Avanza con aceleración controlada y corrección angular.
 * @param objetivo  Velocidad máxima deseada (PWM).
 * @param corr      Corrección para curvar (positivo = derecha).
 */
void avanzarSuave(int objetivo, int corr);

void inicializarMotores();

#endif
