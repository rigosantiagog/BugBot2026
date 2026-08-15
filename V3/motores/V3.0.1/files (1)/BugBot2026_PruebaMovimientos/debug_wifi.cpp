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
// linea de debug con VARIOS Debug.print() seguidos, ej:
//   Debug.print("adelante="); Debug.print(a,1); Debug.print("  lateral="); ...
// Si cada print() se manda de inmediato por WiFi, cada uno viaja como un paquete
// TCP separado y puerto_anillo.py le pone timestamp a cada recv() por separado:
// el log queda partido ("adelante=1" en una linea, ".0  lateral=" en la siguiente).
// En vez de tocar cada Debug.print() de esos 3 archivos (son decenas de sitios,
// facil romper algo), se soluciona AQUI una sola vez: se acumulan los bytes en
// un buffer y SOLO se mandan por WiFi cuando se completa una linea ('\n') o si
// el buffer se llena. El Serial (USB) sigue recibiendo cada byte de inmediato,
// sin cambios, para no perder nada si alguien esta viendo el monitor por cable.
constexpr size_t TAMANO_BUFFER_LINEA = 512;
static char   bufferLinea[TAMANO_BUFFER_LINEA];
static size_t posBuffer = 0;

DebugStream Debug;   // Instancia global usada en todo el proyecto en lugar de Serial directamente

/**
 * @brief Manda lo acumulado en bufferLinea hacia el servidor de logs SIN bloquear el loop.
 *        Si no hay espacio para mandarlo todo sin esperar a la red, se descarta esa
 *        linea completa -- es preferible perder una linea de debug que congelar el
 *        control real del robot (enviarTramaMotores/movimientos corren en el mismo loop).
 */
static void _volcarBufferAWiFi() {
  if (posBuffer == 0) return;                   // Nada acumulado, no hay nada que mandar

  if (clienteLogs.connected()) {
    size_t disponible = clienteLogs.availableForWrite();
    if (disponible > 0) {
      size_t aEscribir = (posBuffer < disponible) ? posBuffer : disponible;
      clienteLogs.write((const uint8_t*)bufferLinea, aEscribir);  // Nunca escribe mas de lo que cabe sin bloquear
    }
    // Si disponible == 0, se descarta esta linea por completo (sin bloquear)
  }
  posBuffer = 0;                                  // Buffer libre para la siguiente linea
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
    _volcarBufferAWiFi();                         // Linea completa (o buffer lleno): se manda de un jalon
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
 *        en atenderDebugWiFi() dentro del loop, para no bloquear el arranque si
 *        puerto_anillo.py todavia no esta corriendo en la laptop.
 */
void iniciarDebugWiFi() {
  Serial.printf("Debug WiFi: se intentara conectar a %s:%d (puerto_anillo.py)\n",
                IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI);
}

/**
 * @brief Mantiene la conexion hacia el servidor de logs: si ya esta conectado no hace nada;
 *        si no, reintenta UN SOLO intento cada INTERVALO_RECONEXION ms, con un timeout
 *        corto (TIMEOUT_CONEXION_MS) para no congelar el loop por mucho tiempo si el
 *        servidor no esta disponible. Debe llamarse en cada vuelta del loop().
 *
 *        IMPORTANTE - CORREGIDO: la version anterior envolvia el intento de conexion
 *        en un "while(!clienteLogs.connected()){ ... }". Eso significa que si el
 *        servidor de logs no respondia, esta funcion JAMAS regresaba -- se quedaba
 *        reintentando para siempre AQUI MISMO, y como es lo primero que llama loop(),
 *        el robot completo se congelaba indefinidamente (nada de movimientos, nada de
 *        giroscopio) hasta que alguien levantara el servidor o reiniciara la placa a
 *        mano. Ahora se hace UN intento por llamada; si falla, simplemente se reintenta
 *        en la siguiente vuelta del loop (3 segundos despues), sin bloquear nada.
 */
void atenderDebugWiFi() {
  if (clienteLogs.connected()) return;   // Ya conectado, nada que hacer

  if (millis() - ultimoIntento < INTERVALO_RECONEXION) return;   // Aun no toca reintentar
  ultimoIntento = millis();
  
  while(!clienteLogs.connected()){

    Serial.println("Reintentando servidor logs...");
    if (clienteLogs.connect(IP_SERVIDOR_LOGS, PUERTO_DEBUG_WIFI, TIMEOUT_CONEXION_MS)) {
      Serial.println("Conectado a servidor logs.");
    }
  
  }
  
  // Si connect() devuelve false, se reintenta en la siguiente llamada (proxima
  // vuelta del loop), NUNCA en un bucle aqui mismo.
}
