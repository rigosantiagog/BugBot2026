#ifndef FUNC_COMUNICACION_H
#define FUNC_COMUNICACION_H

#include <Arduino.h>

// Trama que se envia hacia la ESP32 de motores. "packed" para que el compilador NO
// agregue bytes de relleno entre campos: el tamano en el cable debe ser exactamente
// 4+1+1+2+2+2+2+2+4+4 = 24 bytes, sin importar el compilador o la plataforma.
struct __attribute__((packed)) TramaData {
  float    angulo;        // Angulo hacia la pelota, ya ajustado con OFFSET_FRENTE
  uint8_t  estado;        // 1 si se detecto pelota, 0 si no
  uint8_t  totalActivos;  // Cantidad de sensores IR activos en la cadena detectada
  uint16_t distFrente;    // Distancia ultrasonido frente (cm)
  uint16_t distAtras;     // Distancia ultrasonido atras (cm)
  uint16_t distIzq;       // Distancia ultrasonido izquierda (cm)
  uint16_t distDer;       // Distancia ultrasonido derecha (cm)
  uint16_t bitmapIR;      // Bit i = 1 si el sensor IR i detecta la pelota
  float    posX;          // Posicion estimada del robot en X dentro de la cancha
  float    posY;          // Posicion estimada del robot en Y dentro de la cancha
};

uint16_t obtenerBitmapIR();
void enviarTramaMotores(float angulo, uint8_t estado, uint8_t totalActivos,
                        uint16_t distFrente, uint16_t distAtras,
                        uint16_t distIzq, uint16_t distDer,
                        float robotX, float robotY);
void recibirYaw();

#endif
