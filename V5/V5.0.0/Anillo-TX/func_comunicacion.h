#ifndef FUNC_COMUNICACION_H
#define FUNC_COMUNICACION_H

#include "config.h"

uint16_t obtenerBitmapIR();
void enviarTramaMotores(float angulo, uint8_t estado, uint8_t totalActivos,
                        uint16_t distFrente, uint16_t distAtras,
                        uint16_t distIzq, uint16_t distDer,
                        float robotX, float robotY);
void recibirYaw();

#endif