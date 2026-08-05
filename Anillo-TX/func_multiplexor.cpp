// func_multiplexor.cpp
#include <Arduino.h>
#include "config.h"

/**
 * @brief Selecciona un canal del CD74HC4067 escribiendo los bits en S0-S3.
 *        El multiplexor decodifica esos 4 bits para conectar internamente
 *        el sensor TSSP58038 correspondiente hacia la línea SIG.
 * @param canal Número de canal (0-15).
 *
 * Nota de mantenimiento: bitRead(canal, n) extrae el bit "n" del número de canal.
 * Por ejemplo, canal=5 (binario 0101) -> S0=1, S1=0, S2=1, S3=0.
 * Si se cambia el orden físico de S0-S3 en el cableado, basta con reordenar
 * estas cuatro líneas; no es necesario tocar nada más en el proyecto.
 */
void seleccionarCanal(int canal) {
  digitalWrite(pinS0, bitRead(canal, 0));
  digitalWrite(pinS1, bitRead(canal, 1));
  digitalWrite(pinS2, bitRead(canal, 2));
  digitalWrite(pinS3, bitRead(canal, 3));
}
