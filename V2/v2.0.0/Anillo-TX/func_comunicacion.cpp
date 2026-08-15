#include "config.h"
#include "func_comunicacion.h"

/**
 * @brief Convierte activo[16] en un bitmap de 16 bits.
 */
uint16_t obtenerBitmapIR() {
  uint16_t bitmapIR = 0;
  for (int i = 0; i < 16; i++) {
    if (activo[i]) bitmapIR |= (1 << i);
  }
  return bitmapIR;
}

/**
 * @brief Procesa la respuesta de motores: el yaw actual (float, 4 bytes).
 *        Cabecera 0x55 0xAA, luego 4 bytes float y checksum.
 */
void recibirYaw() {
  static uint8_t state = 0;
  static uint8_t buffer[4];
  static uint8_t idx = 0;
  static uint8_t chkCalculado = 0;
  static unsigned long tTimeout = 0;

  if (state != 0 && (millis() - tTimeout > 15)) {
    state = 0;
    idx = 0;
  }

  while (Enlace.available()) {
    uint8_t b = Enlace.read();
    tTimeout = millis();

    switch (state) {
      case 0:
        if (b == 0x55) state = 1;
        break;
      case 1:
        if (b == 0xAA) {
          state = 2;
          idx = 0;
          chkCalculado = 0;
        } else {
          state = 0;
        }
        break;
      case 2:
        buffer[idx++] = b;
        chkCalculado ^= b;
        if (idx >= 4) state = 3;
        break;
      case 3:
        if (b == chkCalculado) {
          float yawRecibido;
          memcpy(&yawRecibido, &buffer[0], sizeof(float));
          yaw = yawRecibido;
        }
        state = 0;
        idx = 0;
        break;
    }
  }
}

/**
 * @brief Construye y envía la trama completa hacia la ESP32 de motores.
 *        Ajusta el ángulo con OFFSET_FRENTE para alinear el frente del robot.
 */
void enviarTramaMotores(float angulo, uint8_t estado, uint8_t totalActivos,
                        uint16_t distFrente, uint16_t distAtras,
                        uint16_t distIzq, uint16_t distDer,
                        float robotX, float robotY) {
  TramaData miTrama;

  // Ajustar ángulo según el offset definido en config.h
  float anguloAjustado = angulo + OFFSET_FRENTE;
  if (anguloAjustado >= 360.0f) anguloAjustado -= 360.0f;
  if (anguloAjustado < 0.0f) anguloAjustado += 360.0f;

  miTrama.angulo       = anguloAjustado;
  miTrama.estado       = estado;
  miTrama.totalActivos = totalActivos;
  miTrama.distFrente   = distFrente;
  miTrama.distAtras    = distAtras;
  miTrama.distIzq      = distIzq;
  miTrama.distDer      = distDer;
  miTrama.bitmapIR     = obtenerBitmapIR();
  miTrama.posX         = robotX;
  miTrama.posY         = robotY;

  uint8_t* ptrBytes = (uint8_t*)&miTrama;
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(TramaData); i++) {
    checksum ^= ptrBytes[i];
  }

  Enlace.write(0xAA);
  Enlace.write(0x55);
  Enlace.write(ptrBytes, sizeof(TramaData));
  Enlace.write(checksum);
}
