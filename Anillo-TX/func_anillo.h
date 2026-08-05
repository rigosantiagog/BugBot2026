// func_anillo.h
#ifndef FUNC_ANILLO_H
#define FUNC_ANILLO_H

int ubicarPelota();                              // Encuentra la cadena más larga y calcula el ángulo centroide (actualiza "angulo")
void fotorreceptoresActivos(int& totalActivos);  // Lee los 16 sensores vía multiplexor y llena activo[16]

#endif
