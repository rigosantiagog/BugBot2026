#ifndef FUNC_UART_H
#define FUNC_UART_H

uint16_t obtenerBitmapIR();
void enviarTramaMotores(uint8_t estado, int totalActivos);
void recibirYaw();

#endif