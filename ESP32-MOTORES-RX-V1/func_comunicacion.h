#ifndef FUNC_COMUNICACION_H
#define FUNC_COMUNICACION_H

void leerUART();          // Procesa la trama recibida desde el anillo y responde con el yaw
void arduino_OTA();       // Inicializa el servicio OTA
void debugLog(const String& texto); // Envía un mensaje al servidor de logs (opcional)

#endif