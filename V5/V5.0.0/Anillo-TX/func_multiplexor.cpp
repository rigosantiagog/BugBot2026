#include <Arduino.h>
#include "config.h"

void seleccionarCanal(int canal) {
  digitalWrite(pinS0, bitRead(canal, 0));
  digitalWrite(pinS1, bitRead(canal, 1));
  digitalWrite(pinS2, bitRead(canal, 2));
  digitalWrite(pinS3, bitRead(canal, 3));
}