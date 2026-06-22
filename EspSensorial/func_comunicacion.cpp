  #include "config.h"
  #include "func_comunicacion.h"



  uint16_t obtenerBitmapIR() {
    uint16_t bitmapIR = 0;
    for (int i = 0; i < 16; i++) {
      if (activo[i]) bitmapIR |= (1 << i);
    }
    return bitmapIR;
  }
  void recibirYaw() {
    static uint8_t state = 0;
    static uint8_t buffer[4];
    static uint8_t idx = 0;
    static uint8_t chkCalculado = 0;
    static unsigned long tTimeout = 0;

    // Timeout de resguardo rápido por si se corta la ráfaga
    if (state != 0 && (millis() - tTimeout > 15)) {
      state = 0;
      idx = 0;
    }

    while (Enlace.available()) {
      uint8_t b = Enlace.read();
      tTimeout = millis();

      switch (state) {
        case 0:
          if (b == 0x55) state = 1; // Cabecera invertida enviada por motores
          break;
        case 1:
          if (b == 0xAA) {
            state = 2;
            idx = 0;
            chkCalculado = 0;
          } else {
            state = 0;
          }
          break;
        case 2:
          buffer[idx++] = b;
          chkCalculado ^= b;
          if (idx >= 4) {
            state = 3;
          }
          break;
        case 3:
          if (b == chkCalculado) {
            float yawRecibido;
            memcpy(&yawRecibido, &buffer[0], sizeof(float));
            
            // Actualiza la variable global 'angulo' (usada por tu localización absoluta)
            yaw = yawRecibido; 
          }
          state = 0;
          idx = 0;
          break;
      }
    }
  }
  void enviarTramaMotores(uint8_t estado, int totalActivos) {
    TramaData miTrama;
    miTrama.angulo       = angulo; // Manda el último rumbo que conoce el anillo
    miTrama.estado       = estado;
    miTrama.totalActivos = (uint8_t)totalActivos;
    miTrama.distFrente   = (uint16_t)distFrente;
    miTrama.distAtras    = (uint16_t)distAtras;
    miTrama.distIzq      = (uint16_t)distIzq;
    miTrama.distDer      = (uint16_t)distDer;
    miTrama.bitmapIR     = obtenerBitmapIR();
    miTrama.posX         = robotX; 
    miTrama.posY         = robotY;

    uint8_t* ptrBytes = (uint8_t*)&miTrama;
    uint8_t checksum = 0;
    for (int i = 0; i < sizeof(TramaData); i++) {
      checksum ^= ptrBytes[i];
    }

    Enlace.write(0xAA);
    Enlace.write(0x55);
    Enlace.write(ptrBytes, sizeof(TramaData)); 
    Enlace.write(checksum);
  }