#include "func_ultrasonicos.h"
#include "config.h"
#include <Arduino.h>

static int histF[FILTRO_ULTRASONIDOS] = {0};
static int histB[FILTRO_ULTRASONIDOS] = {0};
static int histL[FILTRO_ULTRASONIDOS] = {0};
static int histR[FILTRO_ULTRASONIDOS] = {0};

int medirDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duracion = pulseIn(echo, HIGH, 20000);
  if (duracion == 0) return 999;
  return duracion * 0.034 / 2;
}

int filtrar(int hist[], int nueva) {
  static int cont = 0;
  hist[cont % FILTRO_ULTRASONIDOS] = nueva;
  cont++;
  long suma = 0;
  int count = 0;
  for (int i = 0; i < FILTRO_ULTRASONIDOS; i++) {
    if (hist[i] != 999) {
      suma += hist[i];
      count++;
    }
  }
  return (count == 0) ? 999 : suma / count;
}

void inicializarPinesUltrasonidos() {
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
}

void TareaUltrasonicos(void * pvParameters) {
  inicializarPinesUltrasonidos();
  for (;;) {
    int rawF = medirDistancia(TRIG_F, ECHO_F);
    int rawB = medirDistancia(TRIG_B, ECHO_B);
    int rawL = medirDistancia(TRIG_L, ECHO_L);
    int rawR = medirDistancia(TRIG_R, ECHO_R);
    distFrente = filtrar(histF, rawF);
    distAtras  = filtrar(histB, rawB);
    distIzq    = filtrar(histL, rawL);
    distDer    = filtrar(histR, rawR);
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

void iniciarUltrasonicos() {
  xTaskCreatePinnedToCore(TareaUltrasonicos, "HCSR04", 2048, NULL, 1, NULL, 0);
}