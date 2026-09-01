/*
 * ============================================================================
 *  comunicacion_anillo.h  —  BugBot 2026
 *  Comunicacion UART2 con la ESP32 del anillo IR (ESP32 #2).
 * ============================================================================
 *  RESPONSABILIDAD DE ESTE MODULO:
 *   - Recibir la TramaData que el anillo manda cada ~50ms (angulo, estado,
 *     distancias de los 4 ultrasonidos, bitmap de sensores IR, posicion).
 *   - Mandar de vuelta el yaw actual de este ESP32 (de motores) hacia el
 *     anillo, en el formato que su funcion recibirYaw() ya espera.
 *
 *  PROTOCOLO (debe coincidir EXACTO con el proyecto del anillo):
 *   Anillo -> Motores:  0xAA 0x55 + 24 bytes de TramaData + checksum XOR
 *   Motores -> Anillo:  0x55 0xAA + 4 bytes de float (yaw) + checksum XOR
 *   Baudrate: 38400, 8N1 (ver BAUD_UART_ANILLO en config_robot.h)
 *
 *  NOTA DE ESTILO: este modulo es "header-only" (todo el codigo vive aqui,
 *  sin .cpp separado), igual que motores.h/giroscopio.h/movimientos.h en
 *  este mismo proyecto. Es INTENCIONAL: si este archivo se incluyera desde
 *  un .cpp separado ADEMAS del .ino, cada funcion quedaria definida dos
 *  veces y el enlazador fallaria con "multiple definition". Al ser header-
 *  only e incluirse SOLO desde el .ino, todo vive en una sola unidad de
 *  compilacion, igual que el resto del proyecto.
 * ============================================================================
 */

#ifndef COMUNICACION_ANILLO_H   // Guarda de inclusion: evita compilar este archivo dos veces
#define COMUNICACION_ANILLO_H

#include <Arduino.h>            // Tipos base (uint8_t, uint16_t, etc.) y HardwareSerial
#include "config_robot.h"       // Pines UART2 (PIN_UART_RX2/TX2), BAUD_UART_ANILLO, MAX_ANTIGUEDAD_TRAMA_MS
#include "giroscopio.h"         // giro_obtenerYaw(): necesario para armar el mensaje de vuelta
#include "debug_wifi.h"         // Debug.print/println: para reportar estado por Serial+WiFi

// ============================================================================
// TramaData — DEBE SER BYTE POR BYTE IDENTICA a la definida en el proyecto
// del anillo (func_comunicacion.h). Si un campo cambia de un lado y no del
// otro, la trama se desalinea y todo se lee corrupto (aunque el checksum de
// cada trama individual siga dando bien, porque el checksum no detecta que
// los CAMPOS esten mal alineados entre si, solo que los BYTES no cambiaron
// en el camino). "packed" evita que el compilador meta bytes de relleno.
// ============================================================================
struct __attribute__((packed)) TramaData {
  float    angulo;        // Angulo hacia la pelota (0-360), ya ajustado con OFFSET_FRENTE del anillo
  uint8_t  estado;         // 1 si el anillo detecto pelota, 0 si no
  uint8_t  totalActivos;  // Cantidad de sensores IR activos en la cadena detectada
  uint16_t distFrente;    // Distancia ultrasonido frente (cm); 999 = sin lectura/fuera de rango
  uint16_t distAtras;     // Distancia ultrasonido atras (cm)
  uint16_t distIzq;       // Distancia ultrasonido izquierda (cm)
  uint16_t distDer;       // Distancia ultrasonido derecha (cm)
  uint16_t bitmapIR;      // Bit i = 1 si el sensor IR i del anillo detecta la pelota
  float    posX;          // Posicion estimada del robot en X dentro de la cancha (aun no implementada del lado del anillo)
  float    posY;          // Posicion estimada del robot en Y dentro de la cancha (aun no implementada del lado del anillo)
};

// Objeto UART2 dedicado a hablar con el anillo. El "2" indica que usa el
// periferico UART numero 2 del ESP32 (UART0 ya lo usa el Serial normal por USB).
static HardwareSerial EnlaceAnillo(2);

// Ultima TramaData valida recibida (se sobreescribe cada vez que llega una nueva
// y pasa el checksum). "static" para que solo este archivo pueda modificarla
// directamente; el resto del proyecto la lee via obtenerUltimaTramaAnillo().
static TramaData _ultimaTrama = {0};

// Timestamp (millis()) de la ultima vez que _ultimaTrama se actualizo con datos
// validos. Sirve para detectar si el enlace UART con el anillo se cayo.
static unsigned long _tUltimaTramaValida = 0;

// ----------------------------------------------------------------------------
// comunicacionAnillo_inicializar()
// Abre el puerto UART2 hacia el anillo. Llamar UNA VEZ en setup(), despues
// de que config_robot.h ya este incluido (usa PIN_UART_RX2/TX2/BAUD).
// ----------------------------------------------------------------------------
void comunicacionAnillo_inicializar() {
  // Abre el UART2 con el baudrate y pines definidos en config_robot.h.
  // El orden de parametros de HardwareSerial::begin() en el core de ESP32 es:
  // begin(baudrate, config_de_bits, pin_RX, pin_TX)
  EnlaceAnillo.begin(BAUD_UART_ANILLO, SERIAL_8N1, PIN_UART_RX2, PIN_UART_TX2);

  Debug.print("[COMUNICACION-ANILLO] UART2 iniciado a ");   // Confirma por debug que arranco
  Debug.print(BAUD_UART_ANILLO);                             // Baudrate configurado
  Debug.print(" baudios (RX=GPIO");
  Debug.print(PIN_UART_RX2);                                 // Pin de recepcion (trama del anillo)
  Debug.print(", TX=GPIO");
  Debug.print(PIN_UART_TX2);                                 // Pin de transmision (yaw hacia el anillo)
  Debug.println(")");
}

// ----------------------------------------------------------------------------
// recibirTramaAnillo()
// Maquina de estados igual en espiritu a recibirYaw() del lado del anillo,
// pero para la trama de 24 bytes en vez de 4. Se recorre byte por byte SOLO
// con lo que ya llego al buffer del UART (available()), asi que nunca
// bloquea el loop() esperando datos que todavia no llegaron.
// Devuelve true SOLO en la vuelta de loop() en la que se completo una trama
// nueva y su checksum fue correcto.
// ----------------------------------------------------------------------------
bool recibirTramaAnillo() {
  // "static" en variables locales: conservan su valor entre llamadas sucesivas
  // a esta funcion (es literalmente el "estado" de la maquina de estados).
  static uint8_t state         = 0;                  // Fase actual del parser (0=esperando cabecera...)
  static uint8_t buffer[sizeof(TramaData)];           // Bytes de la trama acumulados hasta ahora
  static uint8_t idx           = 0;                   // Cuantos bytes de la trama ya se guardaron
  static uint8_t chkCalculado  = 0;                   // XOR acumulado de los bytes recibidos (para validar)
  static unsigned long tTimeout = 0;                  // Marca de tiempo del ultimo byte recibido

  bool tramaNueva = false;   // Se pone en true SOLO si esta llamada completo una trama valida

  // Si llevamos "a medias" una trama (state!=0) y pasaron mas de 15ms sin
  // recibir el siguiente byte esperado, se asume que la trama se corrompio
  // o se perdio un byte en el camino, y se reinicia el parser desde cero.
  // Evita quedarse "atorado" esperando para siempre un byte que ya no va a llegar.
  if (state != 0 && (millis() - tTimeout > 15)) {
    state = 0;    // Vuelve a esperar la cabecera desde el principio
    idx = 0;      // Descarta lo que se habia acumulado de la trama incompleta
  }

  // Procesa TODOS los bytes que ya esten disponibles en el buffer del UART
  // (puede ser 0, 1, o varios de una vez, dependiendo de cuanto haya llegado
  // desde la ultima vez que se llamo a esta funcion).
  while (EnlaceAnillo.available()) {
    uint8_t b = EnlaceAnillo.read();   // Lee y consume el siguiente byte del buffer UART
    tTimeout = millis();                // Refresca la marca de tiempo: llego un byte valido

    switch (state) {
      case 0:
        // Esperando el PRIMER byte de la cabecera (0xAA, segun enviarTramaMotores()
        // del lado del anillo, que manda Enlace.write(0xAA) primero)
        if (b == 0xAA) state = 1;       // Coincide: avanza a esperar el segundo byte
        // Si no coincide, se queda en state=0 (sigue esperando, descarta el byte)
        break;

      case 1:
        // Esperando el SEGUNDO byte de la cabecera (0x55)
        if (b == 0x55) {
          state = 2;                    // Cabecera completa: ahora vienen los datos
          idx = 0;                      // Reinicia el contador de bytes de datos
          chkCalculado = 0;             // Reinicia el checksum acumulado
        } else {
          state = 0;                    // No era la cabecera esperada: vuelve a empezar
        }
        break;

      case 2:
        // Acumulando los bytes de la TramaData (24 bytes en total)
        buffer[idx++] = b;              // Guarda este byte en su posicion dentro del buffer
        chkCalculado ^= b;              // Lo incorpora al checksum XOR que se va calculando
        if (idx >= sizeof(TramaData)) { // Ya se juntaron los 24 bytes esperados
          state = 3;                    // Pasa a esperar el byte de checksum
        }
        break;

      case 3:
        // Ultimo byte: el checksum que mando el anillo (XOR de sus 24 bytes)
        if (b == chkCalculado) {
          // El checksum coincide: la trama llego integra, sin corrupcion.
          memcpy(&_ultimaTrama, buffer, sizeof(TramaData)); // Copia los bytes crudos a la struct
          _tUltimaTramaValida = millis();                    // Marca el momento de esta trama valida
          tramaNueva = true;                                 // Avisa al llamador que hay dato fresco
        }
        // Si el checksum NO coincide, se descarta esta trama silenciosamente
        // (la siguiente trama, 50ms despues, va a traer datos nuevos de todos modos)
        state = 0;    // Vuelve a esperar la proxima cabecera
        idx = 0;      // Reinicia el contador de bytes para la proxima trama
        break;
    }
  }

  return tramaNueva;   // true solo si ESTA llamada completo una trama nueva y valida
}

// ----------------------------------------------------------------------------
// obtenerUltimaTramaAnillo()
// Devuelve una referencia de solo lectura a la ultima TramaData valida que
// se recibio (sin importar si fue en esta vuelta de loop() o antes). Usar
// junto con tramaAnilloEsReciente() para saber si todavia se puede confiar
// en estos datos o si el enlace con el anillo se cayo.
// ----------------------------------------------------------------------------
const TramaData& obtenerUltimaTramaAnillo() {
  return _ultimaTrama;   // Devuelve una referencia de solo lectura (el llamador no puede modificarla)
}

// ----------------------------------------------------------------------------
// tramaAnilloEsReciente()
// true si la ultima trama valida llego hace menos de MAX_ANTIGUEDAD_TRAMA_MS
// (definido en config_robot.h). Si nunca ha llegado ninguna trama,
// _tUltimaTramaValida sigue en 0 y millis() ya es mucho mayor -> correctamente
// da false ("no reciente").
// ----------------------------------------------------------------------------
bool tramaAnilloEsReciente() {
  return (millis() - _tUltimaTramaValida) <= MAX_ANTIGUEDAD_TRAMA_MS;
}

// ----------------------------------------------------------------------------
// enviarYawAAnillo()
// Arma y manda el mensaje que recibirYaw() (del lado del anillo) espera:
// cabecera 0x55 0xAA, luego los 4 bytes crudos del float yaw, luego 1 byte
// de checksum XOR de esos 4 bytes. Se puede llamar en cada vuelta del loop():
// es una escritura corta, no representa carga significativa al UART ni al CPU.
// ----------------------------------------------------------------------------
void enviarYawAAnillo() {
  float yawActual = giro_obtenerYaw();     // Lee el yaw actual calculado por giroscopio.h

  uint8_t bytesYaw[sizeof(float)];          // Arreglo para los 4 bytes crudos del float
  memcpy(bytesYaw, &yawActual, sizeof(float)); // Copia la representacion en memoria del float a bytes

  uint8_t checksum = 0;                     // Acumulador del checksum XOR
  for (size_t i = 0; i < sizeof(float); i++) {
    checksum ^= bytesYaw[i];                // Combina cada byte del yaw en el checksum
  }

  EnlaceAnillo.write(0x55);                 // Primer byte de cabecera (coincide con recibirYaw())
  EnlaceAnillo.write(0xAA);                 // Segundo byte de cabecera
  EnlaceAnillo.write(bytesYaw, sizeof(float)); // Los 4 bytes del yaw
  EnlaceAnillo.write(checksum);              // Byte final de verificacion
}

#endif // COMUNICACION_ANILLO_H
