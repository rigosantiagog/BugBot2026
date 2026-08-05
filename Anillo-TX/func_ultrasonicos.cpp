#include "func_ultrasonicos.h"
#include "config.h"
#include <Arduino.h>
#include <cmath>
#include <vector>
#include <algorithm>

// Historiales de filtrado
static int histF[FILTRO_ULTRASONIDOS] = {0};
static int histB[FILTRO_ULTRASONIDOS] = {0};
static int histL[FILTRO_ULTRASONIDOS] = {0};
static int histR[FILTRO_ULTRASONIDOS] = {0};

// Índices independientes para el filtro de cada sensor
static int indiceF = 0;
static int indiceB = 0;
static int indiceL = 0;
static int indiceR = 0;

SensorEcho sensoresEcho[4] = {
  {ECHO_F, 0, 0, false},
  {ECHO_B, 0, 0, false},
  {ECHO_L, 0, 0, false},
  {ECHO_R, 0, 0, false}
};

SemaphoreHandle_t semaforoEcho = NULL;

// ISR Optimizado con cambio de contexto inmediato
void IRAM_ATTR isrEcho(void* arg) {
  SensorEcho* sensor = (SensorEcho*) arg;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (digitalRead(sensor->pin) == HIGH) {
    sensor->t_inicio = micros();
  } else {
    sensor->t_fin = micros();
    sensor->listo = true;
    xSemaphoreGiveFromISR(semaforoEcho, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }
}

void inicializarPinesUltrasonidos() {
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  
  if (semaforoEcho == NULL) {
    semaforoEcho = xSemaphoreCreateBinary();
  }
}

int medirDistanciaInterrupt(int trig, SensorEcho& sensor) {
  sensor.listo = false;
  sensor.t_inicio = 0;
  sensor.t_fin = 0;

  attachInterruptArg(digitalPinToInterrupt(sensor.pin), isrEcho, &sensor, CHANGE);
  
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  if (xSemaphoreTake(semaforoEcho, pdMS_TO_TICKS(30)) == pdTRUE) {
    detachInterrupt(digitalPinToInterrupt(sensor.pin));
    
    if (sensor.t_fin > sensor.t_inicio && sensor.t_inicio != 0) {
      unsigned long duracion = sensor.t_fin - sensor.t_inicio;
      return (int)(duracion * 0.034 / 2);
    }
    return 999; 
  } else {
    detachInterrupt(digitalPinToInterrupt(sensor.pin));
    return 999;
  }
}

int filtrar(int hist[], int &indice, int nueva) {
  hist[indice % FILTRO_ULTRASONIDOS] = nueva;
  indice++;
  
  long suma = 0;
  int count = 0;
  for (int i = 0; i < FILTRO_ULTRASONIDOS; i++) {
    if (hist[i] != 999) {
      suma += hist[i];
      count++;
    }
  }
  return (count == 0) ? 999 : (int)(suma / count);
}

// =========================================================================
// ALGORITMO DE LOCALIZACIÓN ABSOLUTA (Estructuras y Función)
// =========================================================================
const float ANCHO_MAPA  = 182.0f; // Cancha X (cm)
const float ALTO_MAPA   = 243.0f; // Cancha Y (cm)
const float MITAD_ROBOT = 11.0f;  // Radio del robot (cm)
const float EPSILON     = 1e-6f;  // CORREGIDO: Cambiado de EPS a EPSILON para evitar colisión de macros

struct SensorVector { float vx, vy, L; };
struct Candidato { float cx, cy, error; bool exacto; };

void calcularPosicionAbsoluta(float theta_imu, float df, float dd, float da, float di) {
  // Si algún sensor físico falló por completo en el filtro, no arriesgamos la precisión
  if (df >= 998.0f || dd >= 998.0f || da >= 998.0f || di >= 998.0f) {
    robotX = -999.0f;
    robotY = -999.0f;
    return;
  }

  // Normalizar ángulo náutico de la brújula/IMU
  float grados = theta_imu;
  while (grados < 0.0f)   grados += 360.0f;
  while (grados >= 360.0f) grados -= 360.0f;
  
  // CORREGIDO: Se cambió 'f_PI' por la macro estándar de C++ 'M_PI'
  float rad = grados * M_PI / 180.0f; 
  float s = std::sin(rad);
  float c = std::cos(rad);

  // Distancias al centro geométrico del robot
  float Lf = df + MITAD_ROBOT;
  float Ld = dd + MITAD_ROBOT;
  float La = da + MITAD_ROBOT;
  float Li = di + MITAD_ROBOT;

  SensorVector sensores[4] = {
    { s,  c, Lf},   // Frente
    { c, -s, Ld},   // Derecha
    {-s, -c, La},   // Atrás
    {-c,  s, Li}    // Izquierda
  };

  std::vector<float> xs, ys;
  xs.reserve(8); ys.reserve(8);

  // Proyectar intersecciones candidatas contra los bordes de la cancha
  // CORREGIDO: Uso de EPSILON en lugar de EPS
  for (int i = 0; i < 4; ++i) {
    float vx = sensores[i].vx;
    float vy = sensores[i].vy;
    float L  = sensores[i].L;

    if (vx < -EPSILON) { float cx = -L * vx; if (cx >= -EPSILON && cx <= ANCHO_MAPA + EPSILON) xs.push_back(cx); }
    else if (vx > EPSILON) { float cx = ANCHO_MAPA - L * vx; if (cx >= -EPSILON && cx <= ANCHO_MAPA + EPSILON) xs.push_back(cx); }

    if (vy < -EPSILON) { float cy = -L * vy; if (cy >= -EPSILON && cy <= ALTO_MAPA + EPSILON) ys.push_back(cy); }
    else if (vy > EPSILON) { float cy = ALTO_MAPA - L * vy; if (cy >= -EPSILON && cy <= ALTO_MAPA + EPSILON) ys.push_back(cy); }
  }

  // Limpiar duplicados con tolerancia flotante
  auto limpiarDuplicados = [](std::vector<float>& v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end(), [](float a, float b) {
      return std::fabs(a - b) < 1e-3f;
    }), v.end());
  };
  limpiarDuplicados(xs);
  limpiarDuplicados(ys);

  std::vector<Candidato> exactos;
  exactos.reserve(xs.size() * ys.size());

  for (float cx : xs) {
    for (float cy : ys) {
      float error_total = 0.0f;
      bool exacto = true;

      for (int i = 0; i < 4; ++i) {
        float vx = sensores[i].vx;
        float vy = sensores[i].vy;
        float L  = sensores[i].L;

        // CORREGIDO: Uso de EPSILON en lugar de EPS
        float t_izq = (vx < -EPSILON) ? (0.0f - cx) / vx : 3.402823e+38f;
        float t_der = (vx >  EPSILON) ? (ANCHO_MAPA - cx) / vx : 3.402823e+38f;
        float t_inf = (vy < -EPSILON) ? (0.0f - cy) / vy : 3.402823e+38f;
        float t_sup = (vy >  EPSILON) ? (ALTO_MAPA - cy) / vy : 3.402823e+38f;

        float t_min = std::min({t_izq, t_der, t_inf, t_sup});
        float dif = std::fabs(t_min - L);
        
        error_total += dif * dif;
        if (dif > 0.5f) { // Tolerancia máxima por rebote/ruido físico
          exacto = false;
        }
      }
      if (exacto) {
        exactos.push_back({cx, cy, error_total, true});
      }
    }
  }

  // Si no se encuentra un punto coherente, tronamos el cálculo
  if (exactos.empty()) {
    robotX = -999.0f;
    robotY = -999.0f;
    return;
  }

  // Evaluar unicidad
  float x0 = exactos[0].cx;
  float y0 = exactos[0].cy;
  bool misma_x = true;
  bool misma_y = true;

  for (const auto& e : exactos) {
    if (std::fabs(e.cx - x0) > 0.5f) misma_x = false;
    if (std::fabs(e.cy - y0) > 0.5f) misma_y = false;
  }

  // Truene estricto a -999.0f si el dato se vuelve "aleatorio"
  if (misma_x && misma_y) {
    robotX = x0;
    robotY = y0;
  } 
  else if (misma_x && !misma_y) {
    robotX = x0;        // X es exacta
    robotY = -999.0f;   // Y es imprecisa/aleatoria -> truena
  } 
  else if (misma_y && !misma_x) {
    robotX = -999.0f;   // X es imprecisa/aleatoria -> truena
    robotY = y0;        // Y es exacta
  } 
  else {
    robotX = -999.0f;
    robotY = -999.0f;
  }
}

// =========================================================================
// TAREA PRINCIPAL DE FREERTOS (CORE 0)
// =========================================================================
void TareaUltrasonicos(void * pvParameters) {
  inicializarPinesUltrasonidos();
  
  for (;;) {
    int rawF = medirDistanciaInterrupt(TRIG_F, sensoresEcho[0]);
    vTaskDelay(pdMS_TO_TICKS(5));
    int rawB = medirDistanciaInterrupt(TRIG_B, sensoresEcho[1]);
    vTaskDelay(pdMS_TO_TICKS(5));
    int rawL = medirDistanciaInterrupt(TRIG_L, sensoresEcho[2]);
    vTaskDelay(pdMS_TO_TICKS(5));
    int rawR = medirDistanciaInterrupt(TRIG_R, sensoresEcho[3]);
    
    // Guardar en las variables globales filtradas
    distFrente = filtrar(histF, indiceF, rawF);
    distAtras  = filtrar(histB, indiceB, rawB);
    distIzq    = filtrar(histL, indiceL, rawL);
    distDer    = filtrar(histR, indiceR, rawR);
    
    // Ejecutar el cálculo matemático usando el ángulo IMU global
    calcularPosicionAbsoluta(yaw, distFrente, distDer, distAtras, distIzq);
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void iniciarUltrasonicos() {
  xTaskCreatePinnedToCore(TareaUltrasonicos, "HCSR04", 3072, NULL, 1, NULL, 0); 
}
