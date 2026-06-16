/**
 * @file func_ultrasonicos.cpp
 * @brief Implementación de la lectura de ultrasonidos en una tarea de FreeRTOS.
 */

#include "func_ultrasonicos.h"
#include "config.h"
#include <Arduino.h>

/**
 * @brief Mide la distancia con un HC-SR04 usando timeout.
 * @return Distancia en cm, o 999 si no hay eco.
 */
int medirDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  
  long duracion = pulseIn(pinEcho, HIGH, 15000); // Timeout 15 ms (~2.5 m)
  if (duracion == 0) return 999;
  return duracion * 0.034 / 2;
}

/**
 * @brief Tarea de FreeRTOS que se ejecuta en el Núcleo 0.
 *        Lee los cuatro ultrasonidos secuencialmente cada 30 ms.
 */
void TareaUltrasonicos(void * pvParameters) {
  // Configura pines
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  for(;;) {
    distFrente = medirDistancia(TRIG_F, ECHO_F);
    distAtras  = medirDistancia(TRIG_B, ECHO_B);
    distIzq    = medirDistancia(TRIG_L, ECHO_L);
    distDer    = medirDistancia(TRIG_R, ECHO_R);
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Crea y lanza la tarea de ultrasonidos en el Núcleo 0.
 */
void iniciarUltrasonicos() {
  xTaskCreatePinnedToCore(
    TareaUltrasonicos,
    "Lectura_HCSR04",
    2048,
    NULL,
    1,
    NULL,
    0     // Núcleo 0
  );
}