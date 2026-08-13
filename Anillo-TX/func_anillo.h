// func_anillo.h
#ifndef FUNC_ANILLO_H
#define FUNC_ANILLO_H

#include <Arduino.h>

int ubicarPelota();                              // Encuentra la cadena más larga y calcula el centroide
void fotorreceptoresActivos(int& totalActivos);  // Lee todos los sensores
void imprimirArregloSensores(uint16_t bitmap);   // Imprime el bitmap de 16 sensores como arreglo [0/1] en el Serial

#endif
