#include <Arduino.h>
#include "config.h"

//Permite elegir mediante los 4 pines de comunicacion uno de los 16 fotorreceptores
void seleccionarCanal(int canal) {
  digitalWrite(pinS0, bitRead(canal, 0)); 
  digitalWrite(pinS1, bitRead(canal, 1)); 
  digitalWrite(pinS2, bitRead(canal, 2)); 
  digitalWrite(pinS3, bitRead(canal, 3)); 
}
