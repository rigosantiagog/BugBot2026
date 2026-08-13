/* ==================================================================
   ESP32 ANILLO (TX) v10 - Centroide IR + Radar Ultrasónico
   ================================================================== */
#include <Arduino.h> 
#include <string.h>
#include "config.h"
#include "func_ultrasonicos.h"
#include "func_multiplexor.h"
#include "func_anillo.h"

void setup() {
  Serial.begin(115200); 
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN); 
  
  pinMode(pinS0, OUTPUT); pinMode(pinS1, OUTPUT);
  pinMode(pinS2, OUTPUT); pinMode(pinS3, OUTPUT);
  pinMode(pinSIG, INPUT_PULLUP);   
  
  // Enciende el radar en el Núcleo 0 de forma paralela
  iniciarUltrasonicos(); 
  
  Serial.println("\n=== ESP32 ANILLO (TX) v10 Listo ==="); 
}

void loop() {
  // LÓGICA DE DETECCIÓN INFRARROJA (Corriendo en el Núcleo 1)
  int totalActivos = 0; 

  fotorreceptoresActivos(totalActivos);

  int mejorLargo = ubicarPelota();

  int estado = (mejorLargo > 0) ? 1 : 0; 

  // ENSAMBLE DEL PAQUETE UART (Añadiendo F, B, L, R para las paredes)
  Enlace.print("A:"); Enlace.print(angulo, 1); 
  Enlace.print(" C:"); Enlace.print(estado); 
  Enlace.print(" N:"); Enlace.print(totalActivos);
  Enlace.print(" F:"); Enlace.print(distFrente);
  Enlace.print(" B:"); Enlace.print(distAtras);
  Enlace.print(" L:"); Enlace.print(distIzq);
  Enlace.print(" R:"); Enlace.print(distDer);
  for(int i = 0; i < 16; i++) {
    Enlace.print(activo[i]);
    Enlace.print(",");
  }
  Enlace.println();

  // Monitor serie local para verificar que los ultrasónicos envían datos reales
  Serial.printf("TX -> IR[A:%.1f C:%d] RADAR[F:%d B:%d L:%d R:%d]\n ", 
                 angulo, estado, distFrente, distAtras, distIzq, distDer); 

  
  delay(50); 
}
