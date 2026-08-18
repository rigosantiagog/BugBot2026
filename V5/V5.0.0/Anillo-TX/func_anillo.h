#ifndef FUNC_ANILLO_H
#define FUNC_ANILLO_H

#include <Arduino.h>

int ubicarPelota();
void fotorreceptoresActivos(int& totalActivos);
void imprimirArregloSensores(uint16_t bitmap);
void aplicarFiltroPersistenciaIR(int& totalActivos);
const char* obtenerOrientacionCardinal(float anguloGrados);

#endif