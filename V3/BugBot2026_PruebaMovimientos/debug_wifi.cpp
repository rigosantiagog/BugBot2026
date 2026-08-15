// debug_wifi.cpp
#include "debug_wifi.h"
#include "config_robot.h"

// Cliente TCP que se conecta HACIA el servidor de logs (puerto_anillo.py) que corre
// en la laptop/PC. El ESP32 es quien inicia la conexion, no al reves.
static WiFiClient clienteLogs;

static unsigned long ultimoIntento = 0;                  // millis() del ultimo intento de conexion
constexpr unsigned long INTERVALO_RECONEXION = 3000;      // Reintenta cada 3 s si no esta conectado
constexpr int32_t TIMEOUT_CONEXION_MS = 500;               // Maximo que puede "congelarse" el loop al intentar conectar

// --- Buffer de linea para las escrituras hacia WiFi ---
// Muchas partes del proyecto (motores.h, movimientos.h, giroscopio.h) arman UNA
// linea de debug con VARIOS Debug.print() seguidos. Si cada print() se manda de
// inmediato por WiFi, cada uno viaja como un paquete TCP separado y el log queda
// partido en varias lineas con timestamps distintos. Se acumulan los bytes aqui
// y se mandan de un jalon al completarse una linea ('\n') o si el buffer se llena.
// El Serial (USB) sigue recibiendo cada byte de inmediato, sin cambios.
constexpr size_t TAMANO_BUFFER_LINEA = 512;
static char   bufferLinea[TAMANO_BUFFER_LINEA];
static size_t posBuffer = 0;

DebugStream Debug;   // Instancia global usada en todo el proyecto en lugar de Serial directamente

/**
 * @brief Manda lo acumulado en bufferLinea hacia el servidor de logs.
 *
 *        CORREGIDO: la version anterior solo escribia si clienteLogs.availableForWrite()
 *        reportaba espacio disponible. En la practica esto causaba que TODO el debug
 *        dejara de llegar por WiFi (solo se veia en el Serial local) -- availableForWrite()
 *        no siempre reporta el espacio real de forma confiable en todas las versiones
 *        del core de ESP32, y si devuelve 0 de mas, se descartaba la linea completa
 *        cada vez. Ahora se escribe directo con clienteLogs.write(); la proteccion
 *        contra bloqueos largos se hace con setTimeout() al momento de conectar
 *        (ver atenderDebugWiFi/esperarConexionDebugWiFi mas abajo), que limita
 *        cuanto puede tardar CUALQUIER operacion sobre este socket, en vez de
 *        adivinar de antemano si "cabe" o no.
 */
static void _volcarBufferAWiFi() {
  if (posBuffer == 0) return;                     // Nada acumulado, no hay nada que mandar
  if (clienteLogs.connected()) {
    clienteLogs.write((const uint8_t*)bufferLinea, posBuffer);
  }
  posBuffer = 0;                                    // Buffer libre para la siguiente linea
}

/**
 * @brief Escribe un solo byte: al Serial de inmediato (como siempre), y lo acumula
 *        en el buffer de linea para el WiFi (se manda completo al llegar un '\n').
 */
size_t DebugStream::write(uint8_t caracter) {
  Serial.write(caracter);                         // El USB recibe cada byte de inmediato

  if (posBuffer < TAMANO_BUFFER_LINEA) {
    bufferLinea[posBuffer++] = (char)caracter;
  }
  if (caracter == '\n' || posBuffer >= TAMANO_BUFFER_LINEA) {
    _volcarBufferAWiFi();                          // Linea completa (o buffer lleno): se manda de un jalon
  }
  return 1;
}

/**
 * @brief Escribe un bloque de bytes: al Serial de una vez (eficiente), y byte a byte
 *        hacia el buffer de linea del WiFi para que el corte por '\n' funcione igual
 *        sin importar si el llamador uso print()/println()/printf() o un bloque.
 */
size_t DebugStream::write(const uint8_t *buffer, size_t tamano) {
  Serial.write(buffer, tamano);
  for (size_t i = 0; i < tamano; i++) {
    if (posBuffer < TAMANO_BUFFER_LINEA) {
      bufferLinea[posBuffer++] = (char)buffer[i];
    }
    if (buffer[i] == '\n' || posBuffer >= TAMANO_BUFFER_LINEA) {
      _volcarBufferAWiFi();
    }
  }
  return tamano;
}

/**
 * @brief Solo deja listo el mensaje inicial; el primer intento de conexion real ocurre
 *        en esperarConexionDebugWiFi() (setup) o atenderDebugWiFi() (loop).
 */
void iniciarDebugWiFi() {
  Serial.printf("Debug WiFi: se intentara conectar a %s:%d (puerto_anillo.py)\n",
                IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI);
}

// Aplica un tope de tiempo a CUALQUIER operacion futura sobre el socket (lectura o
// escritura). Se llama justo despues de un connect() exitoso. Con esto, aunque la
// senal WiFi se degrade despues (ej. el robot se aleja del AP), ninguna escritura
// de debug puede congelar el loop por mas de este tiempo -- mucho mas confiable
// que adivinar con availableForWrite() si "cabe" el mensaje.
static void _aplicarTimeoutSocket() {
  clienteLogs.setTimeout(200);   // 200 ms maximo por operacion sobre este socket
}

/**
 * @brief Espera de forma ACOTADA a que se conecte el servidor de logs. Pensada para
 *        llamarse UNA VEZ en setup(), antes de mover el robot, para asegurar que el
 *        log ya este grabando. A diferencia de un while() sin limite, esta funcion
 *        SIEMPRE regresa tras timeoutMs como maximo, conectado o no.
 */
bool esperarConexionDebugWiFi(unsigned long timeoutMs) {
  Serial.printf("Esperando conexion al servidor de logs (maximo %lu ms)...\n", timeoutMs);
  unsigned long tInicio = millis();

  while (!clienteLogs.connected()) {
    if (millis() - tInicio >= timeoutMs) {
      Serial.println("*** No se conecto al servidor de logs a tiempo. Se continua SIN el, reintentando en segundo plano durante la operacion. ***");
      return false;   // No bloquea para siempre: el robot puede arrancar de todos modos
    }

    Serial.println("Reintentando servidor logs...");
    if (clienteLogs.connect(IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI, TIMEOUT_CONEXION_MS)) {
      _aplicarTimeoutSocket();
      Serial.println("Conectado a servidor logs.");
      return true;
    }

    yield();          // Cede CPU: evita disparar el Task Watchdog durante la espera
    delay(200);       // Pausa entre reintentos para no saturar la red/CPU
  }
  return true;   // Ya estaba conectado
}

/**
 * @brief Mantiene la conexion hacia el servidor de logs durante la operacion normal:
 *        UN SOLO intento no bloqueante cada INTERVALO_RECONEXION ms. Debe llamarse
 *        en cada vuelta del loop().
 *
 *        IMPORTANTE: a proposito NO tiene un while() de espera aqui. Ese while()
 *        (que asegura conexion antes de MOVER el robot) ya vive en
 *        esperarConexionDebugWiFi(), pensada para llamarse UNA VEZ en setup().
 *        Si esta funcion tambien esperara indefinidamente, cualquier caida de WiFi
 *        a mitad de una prueba (ej. el robot se aleja del AP) congelaria el robot
 *        completo hasta reconectar -- exactamente lo que se corrigio antes.
 */
void atenderDebugWiFi() {
  if (clienteLogs.connected()) return;   // Ya conectado, nada que hacer

  if (millis() - ultimoIntento < INTERVALO_RECONEXION) return;   // Aun no toca reintentar
  ultimoIntento = millis();

  Serial.println("Reintentando servidor logs...");
  if (clienteLogs.connect(IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI, TIMEOUT_CONEXION_MS)) {
    _aplicarTimeoutSocket();
    Serial.println("Conectado a servidor logs.");
  }
  // Si connect() devuelve false, se reintenta en la siguiente llamada (proxima
  // vuelta del loop), nunca en un bucle aqui mismo.
}
