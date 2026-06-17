/*
 * Emisor de Señal Infrarroja Modulada (Continua 38 kHz)
 * Hardware: Arduino UNO, LED IR, Resistencia 330 ohms.
 */

const int pinLedIR = 3; 

void setup() {
  pinMode(pinLedIR, OUTPUT);
  // Se genera una onda constante a 38000 Hz.
  tone(pinLedIR, 40000); 
}

void loop() {
  // El emisor funciona en segundo plano sin instrucciones adicionales en el bucle.
}