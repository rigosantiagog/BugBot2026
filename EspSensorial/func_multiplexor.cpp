// func_multiplexor.cpp
#include <Arduino.h>
#include "config.h"

/**
 * @brief Selecciona un canal del CD74HC4067 escribiendo los bits en S0-S3.
 * @param canal Número de canal (0-15).
 */
void seleccionarCanal(int canal) {
  digitalWrite(pinS0, bitRead(canal, 0));
  digitalWrite(pinS1, bitRead(canal, 1));
  digitalWrite(pinS2, bitRead(canal, 2));
  digitalWrite(pinS3, bitRead(canal, 3));
}
