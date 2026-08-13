#include "func_ultrasonicos.h"
#include "config.h"

#include <Arduino.h>

int medirDistancia(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  
  // Timeout de 15000 microsegundos (~2.5 metros) que cubre la cancha de 243x182cm
  // Si el eco tarda más, aborta la espera para no colgarse
  long duracion = pulseIn(pinEcho, HIGH, 15000); 
  
  if (duracion == 0) return 999; // 999 significa "No hay pared cerca o campo libre"
  
  int distanciaCm = duracion * 0.034 / 2; // Convierte el tiempo en centímetros
  return distanciaCm;
}

// TAREA DE FREERTOS (Bucle infinito del Núcleo 0)
void TareaUltrasonicos(void * pvParameters) {
  // Configura pines al iniciar
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  for(;;) {
    // Escaneo en modo "Round-Robin" (Uno tras otro para evitar que los ecos choquen entre sí)
    distFrente = medirDistancia(TRIG_F, ECHO_F);
    distAtras  = medirDistancia(TRIG_B, ECHO_B);
    distIzq    = medirDistancia(TRIG_L, ECHO_L);
    distDer    = medirDistancia(TRIG_R, ECHO_R);

    // Pausa de 30ms obligatoria para que el eco se disipe en la cancha
    vTaskDelay(30 / portTICK_PERIOD_MS); 
  }
}

// Función para encender este núcleo desde el código principal del Anillo
void iniciarUltrasonicos() {
  xTaskCreatePinnedToCore(
    TareaUltrasonicos,   // Función a ejecutar
    "Lectura_HCSR04",    // Nombre interno
    2048,                // Memoria asignada
    NULL,                // Sin parámetros extra
    1,                   // Prioridad
    NULL,                // Identificador
    0                    // Asignado al Núcleo 0 (El Infrarrojo corre en el 1)
  );
}
