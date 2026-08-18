/*
 * ============================================================================
 *  BugBot2026_PruebaMovimientos.ino  —  BugBot 2026
 *  Archivo principal del sketch: contiene setup() y loop().
 * ============================================================================
 *  CAMBIOS v3 (antiruido y antibrownout):
 *   - Se desactiva el detector de brownout por software para evitar reinicios
 *     involuntarios cuando los motores generan picos de corriente al arrancar.
 *     NOTA: esto no reemplaza los capacitores de desacople en hardware;
 *     ambas soluciones deben usarse juntas para máxima estabilidad.
 *   - El I2C ya tiene timeout (en giroscopio.h), pero ahora el main loop
 *     detecta fallos consecutivos y reinicia el bus I2C si es necesario.
 *   - Se aumenta el delay antes de la calibración para dar tiempo a que la
 *     fuente de alimentación se estabilice tras conectar las baterías.
 * ============================================================================
 */

// Incluye el driver del detector de brownout del ESP32 (parte del IDF)
// Esto es necesario para poder desactivarlo por software
#include "soc/soc.h"           // Registros del SoC del ESP32
#include "soc/rtc_cntl_reg.h"  // Registro de control del brownout detector
#include <WiFi.h>

// Credenciales WiFi (punto de acceso)
const char* ssid = "ESP32_DEBUG_MOTORES";
const char* password = "12345678";

String DebugInfo = "Iniciando Debug ....... \n";


// Módulos del proyecto (deben estar en la misma carpeta que este .ino)
#include "config_robot.h"   // Pines y parámetros: incluir PRIMERO
#include "motores.h"        // Control de los 4 motores mecanum
#include "giroscopio.h"     // Control del MPU6500 por I2C
#include "movimientos.h"    // Maniobras y corrección de orientación (queda disponible para diagnostico)
#include "comunicacion_anillo.h"  // Recibe TramaData del anillo, manda el yaw de vuelta
#include "perseguir_pelota.h"     // Logica de decision: TramaData -> movimiento mecanum
#include "esp_system.h"  // Necesario para esp_reset_reason() y las constantes ESP_RST_*
#include "debug_wifi.h"

// Contador de fallos I2C consecutivos para detectar si el bus necesita reset
static uint8_t _fallosI2CConsecutivos = 0;
#define MAX_FALLOS_I2C_ANTES_DE_RESET  20  // Si falla 20 veces seguidas, reinicia el bus

// ============================================================================
// setup()
// Se ejecuta UNA SOLA VEZ al encender o reiniciar la ESP32.
// ============================================================================

void imprimirRazonReset() {
  esp_reset_reason_t razon = esp_reset_reason();  // Consulta el motivo del ultimo reinicio guardado por el propio chip

  Debug.print("Motivo del ultimo reinicio: ");  // Etiqueta para ubicar el dato en el monitor serial
  switch (razon) {
    case ESP_RST_POWERON:  Debug.println("Encendido normal (power-on)"); break;                          // Arranque limpio, no hubo falla previa
    case ESP_RST_BROWNOUT: Debug.println("BROWNOUT: caida de voltaje detectada por hardware"); break;     // Confirma que SI fue voltaje (aunque el detector por software este deshabilitado)
    case ESP_RST_TASK_WDT: Debug.println("WATCHDOG DE TAREA: el loop() se bloqueo demasiado tiempo"); break; // Apunta a codigo colgado, ej. I2C sin timeout efectivo
    case ESP_RST_INT_WDT:  Debug.println("WATCHDOG DE INTERRUPCION: una interrupcion tardo demasiado"); break; // Puede indicar una ISR bloqueada
    case ESP_RST_PANIC:    Debug.println("PANICO: crash de software (ej. acceso a memoria invalido)"); break; // Bug de codigo, no relacionado a voltaje
    case ESP_RST_SW:       Debug.println("Reinicio por software (ESP.restart())"); break;                 // Reinicio intencional del propio programa
    default:                Debug.printf("Otro motivo, codigo: %d\n", razon); break;                       // Cualquier caso no listado arriba
  }
}


void setup() {

   // --- WiFi y OTA ---
  WiFi.softAP(ssid, password);

   // --- Debug remoto por WiFi: se arranca justo despues del softAP, ya con la red activa ---
  iniciarDebugWiFi();

  // Espera (con tope de tiempo) a que el servidor de logs conecte ANTES de mover
  // el robot, para no perder las primeras lineas del log. Si no conecta a tiempo,
  // arranca de todos modos -- nunca se queda esperando para siempre.
  esperarConexionDebugWiFi(TIMEOUT_ESPERA_LOGS_MS);


  // --- PASO CRÍTICO: Desactiva el detector de brownout por software ---
  // El brownout detector reinicia la ESP32 cuando el voltaje de alimentación
  // cae por debajo de un umbral (~2.4V). Cuando los motores arrancan, generan
  // un pico de demanda de corriente que puede hacer caer momentáneamente el
  // voltaje de la ESP32 por debajo de ese umbral, causando un reset indeseado.
  // Al desactivarlo, la ESP32 tolera caídas momentáneas sin reiniciarse.
  // IMPORTANTE: esto no es peligroso para el hardware, es un ajuste de firmware.
  // La solución COMPLETA requiere capacitores de desacople en hardware también.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // 0 = desactiva el brownout detector

  // Inicia el monitor serial a 115200 baudios
  Serial.begin(115200);
  delay(500);  // Espera a que el monitor serial se conecte

  // --- CORRECCIÓN: imprimirRazonReset() estaba ANTES de Serial.begin() ---
  // Serial.print()/println() no hacen nada util si el puerto aun no esta
  // inicializado: el mensaje con la causa real del reinicio se perdia por
  // completo y nunca aparecia en el monitor, aunque la funcion se ejecutaba
  // correctamente. Debe ir DESPUES de Serial.begin()+delay() para ser visible.
  imprimirRazonReset();
  Debug.println("[SISTEMA] Brownout detector DESACTIVADO por software");

  Debug.println();
  Debug.println("############################################################");
  Debug.println("#      BugBot 2026 - Prueba de Movimientos Mecanum         #");
  Debug.println("#      Giroscopio MPU6500 - ESP32 Arduino core 3.x         #");
  Debug.println("#      v4: Freno activo real + diagnostico de reinicio     #");
  Debug.println("############################################################");
  Debug.println();

  // --- Paso 1: Inicializa los motores ---
  Debug.println("[SETUP] Paso 1/4: Inicializando subsistema de motores...");
  motores_inicializar();
  Debug.println("[SETUP] Paso 1/4: Motores OK");
  Debug.println();

  // --- Paso 2: Inicializa el giroscopio ---
  Debug.println("[SETUP] Paso 2/4: Inicializando giroscopio MPU6500...");
  giro_inicializar();
  Debug.println("[SETUP] Paso 2/4: Giroscopio OK");
  Debug.println();

  // --- Paso 3: Calibra el giroscopio ---
  // Espera más tiempo que antes para que la fuente de alimentación se
  // estabilice completamente después de conectar las baterías de motores
  Debug.println("[SETUP] Paso 3/4: Calibracion del giroscopio...");
  Debug.println("[SETUP] *** COLOCA EL ROBOT EN SU POSICION INICIAL Y NO LO MUEVAS ***");
  Debug.println("[SETUP] Esperando 5 segundos para estabilizacion de la fuente...");
  delay(5000);  // 5 segundos (antes eran 3) para mayor estabilidad

  giro_calibrarYFijarNorte();
  Debug.println("[SETUP] Paso 3/4: Calibracion OK");
  Debug.println();

  // --- Paso 4: Abre la comunicacion UART2 con la ESP32 del anillo ---
  Debug.println("[SETUP] Paso 4/4: Iniciando comunicacion con el anillo IR...");
  comunicacionAnillo_inicializar();
  Debug.println("[SETUP] Paso 4/4: Comunicacion con el anillo OK");
  Debug.println();

  Debug.println("[SETUP] *** INICIALIZACION COMPLETA ***");
  Debug.println("[SETUP] El robot comenzara a perseguir la pelota en 2 segundos...");
  delay(2000);
}

// ============================================================================
// reiniciarBusI2C()
// Reinicia el bus I2C si se detectan demasiados fallos consecutivos.
// Los motores DC generan EMI que puede corromper el bus I2C; esta función
// lo recupera sin necesidad de reiniciar toda la ESP32.
// ============================================================================
void reiniciarBusI2C() {
  Debug.println("[I2C] Demasiados fallos consecutivos — reiniciando bus I2C...");
  Wire.end();          // Termina el bus I2C actual
  delay(10);           // Pausa breve para que los pines se descarguen
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);  // Reinicia el bus con los mismos pines
  Wire.setClock(100000);                  // Vuelve a configurar a 100 kHz
  delay(50);           // Espera a que el sensor se estabilice
  _fallosI2CConsecutivos = 0;            // Resetea el contador de fallos
  Debug.println("[I2C] Bus I2C reiniciado correctamente.");
}

// ============================================================================
// loop()
// Ciclo principal que se repite indefinidamente. A diferencia de la version
// de pruebas (que ejecutaba una secuencia larga y bloqueante de movimientos),
// este loop() es CORTO y NO BLOQUEANTE en cada vuelta: lee lo mas reciente
// del anillo, decide un movimiento, lo aplica, y vuelve a empezar. Esto es
// necesario para poder reaccionar en tiempo real a una pelota que se mueve.
// ============================================================================
void loop() {
  atenderDebugWiFi();      // Mantiene/reintenta la conexion hacia el servidor de logs (puerto_anillo.py)

  recibirTramaAnillo();    // Revisa el UART2: si llego una TramaData nueva y valida, la guarda
  giro_actualizarYaw();    // Mantiene el yaw actualizado con la lectura mas reciente del MPU6500
  enviarYawAAnillo();      // Manda el yaw actual de vuelta al anillo (lo usa para su propio debug)

  perseguirPelota();       // Decide y ejecuta UN paso de movimiento segun la ultima trama recibida

  // Verifica el estado del bus I2C en cada vuelta (los motores generan ruido
  // EMI que puede degradar el bus I2C con el tiempo, igual que en la version
  // de pruebas de movimientos)
  if (!giro_busOK()) {
    _fallosI2CConsecutivos++;                                  // Cuenta fallos seguidos del I2C
    if (_fallosI2CConsecutivos >= MAX_FALLOS_I2C_ANTES_DE_RESET) {
      reiniciarBusI2C();                                       // Recupera el bus si esta muy degradado
    }
  } else {
    _fallosI2CConsecutivos = 0;                                // Lectura OK: reinicia el contador de fallos
  }

  yield();      // Cede CPU al scheduler de FreeRTOS: evita disparar el Task Watchdog del ESP32
  delay(20);    // Pequena pausa de ritmo: el anillo manda datos nuevos cada ~50ms, revisar mucho
                // mas rapido que eso no gana respuesta real y solo satura CPU/UART/I2C sin necesidad
}
