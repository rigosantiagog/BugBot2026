/*
 * ============================================================================
 *  giroscopio.h  —  BugBot 2026
 *  Módulo de lectura, calibración y seguimiento de orientación (yaw/eje Z)
 *  usando el sensor inercial MPU6500 por comunicación I2C directa.
 *  No requiere librerías externas: solo Wire.h del core del ESP32.
 * ============================================================================
 *  RESPONSABILIDADES DE ESTE MÓDULO:
 *   1. Inicializar el bus I2C y configurar el chip MPU6500.
 *   2. Calibrar el offset de reposo del eje Z del giroscopio.
 *   3. Integrar la velocidad angular en el tiempo para estimar el yaw.
 *   4. Reportar si el robot está alineado con su orientación inicial (0°).
 *   5. Recuperar el bus I2C automáticamente si hay fallos consecutivos.
 *
 *  SOLUCIONES AL RUIDO EMI DE LOS MOTORES DC:
 *   ✓ I2C a 100 kHz (modo estándar, más inmune que 400 kHz)
 *   ✓ Timeout configurable en Wire.requestFrom() (I2C_TIMEOUT_MS)
 *   ✓ _tAnterior se actualiza SOLO cuando la lectura I2C es exitosa,
 *     evitando que dt sea enorme en la siguiente lectura válida
 *   ✓ Reinicio automático del bus Wire tras I2C_FALLOS_PARA_REINICIO fallos
 *   ✓ DAC1/DAC2 desactivados en GPIO25/26 para liberar esos pines para LEDC
 *
 *  REGISTROS DEL MPU6500 UTILIZADOS:
 *   0x6B (PWR_MGMT_1)  : despertar el sensor del modo sleep
 *   0x1B (GYRO_CONFIG) : configurar el rango de medición (±250°/s)
 *   0x47 (GYRO_ZOUT_H) : byte alto de la velocidad angular en Z
 *   0x48 (GYRO_ZOUT_L) : byte bajo  de la velocidad angular en Z
 *
 *  SENSIBILIDAD DEL GIROSCOPIO EN RANGO ±250°/s:
 *   32768 LSB = 250 °/s  →  1 LSB = 250/32768 = 0.007633 °/s
 * ============================================================================
 */

#ifndef GIROSCOPIO_H   // Guarda de inclusión: evita compilar este archivo dos veces
#define GIROSCOPIO_H

#include <Arduino.h>        // Tipos y funciones base del entorno Arduino
#include <Wire.h>           // Librería I2C incluida en el core del ESP32
#include "driver/dac.h"     // API de bajo nivel del DAC del ESP32 (para desactivarlo)
#include "config_robot.h"   // Pines, direcciones I2C y constantes de configuración

// ============================================================================
// CONSTANTES INTERNAS — Registros del MPU6500
// ============================================================================
#define MPU_REG_PWR_MGMT_1   0x6B  // Registro de gestión de energía (sleep/wake)
#define MPU_REG_GYRO_CONFIG  0x1B  // Registro de configuración del giroscopio
#define MPU_REG_GYRO_ZOUT_H  0x47  // Byte alto (MSB) de la velocidad angular en Z

// Factor de conversión: LSB crudo → grados/segundo
// Con rango ±250°/s: sensibilidad = 250 / 32768 = 0.007633 (°/s) / LSB
#define GYRO_SENS   0.007633f

// ============================================================================
// VARIABLES PRIVADAS DEL MÓDULO
// Declaradas static para que solo sean visibles dentro de este archivo.
// NO acceder directamente desde otros módulos; usar las funciones públicas.
// ============================================================================

static float         _offsetZ         = 0.0f;  // Error de cero del eje Z (°/s)
                                                // Calculado durante la calibración en reposo
static float         _yaw             = 0.0f;  // Ángulo acumulado desde el norte inicial (°)
                                                // Positivo = giro horario, negativo = antihorario
static unsigned long _tAnteriorMicros = 0;     // Timestamp de la última integración exitosa
                                                // Se actualiza SOLO con lecturas I2C válidas
static bool          _i2cOK           = true;  // Estado de la última operación I2C
                                                // true = éxito, false = fallo/timeout
static uint8_t       _fallosConsec    = 0;     // Contador de fallos I2C consecutivos
                                                // Al superar I2C_FALLOS_PARA_REINICIO → reset bus

// ============================================================================
// FUNCIÓN PRIVADA: _reiniciarBusI2C()
// Reinicia el bus I2C en caliente sin reiniciar la ESP32.
// Se llama automáticamente cuando hay demasiados fallos consecutivos.
// El reinicio en caliente recupera el bus si un transiente de EMI lo trabó.
// ============================================================================
static void _reiniciarBusI2C() {
  Debug.println("[I2C] *** Reiniciando bus I2C por fallos consecutivos ***");
  Wire.end();                                  // Deshabilita el periférico I2C
  delay(10);                                   // Pausa para descargar los pines SDA/SCL
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);        // Reinicia I2C con los mismos pines
  Wire.setClock(100000);                        // Restaura velocidad a 100 kHz
  Wire.setTimeOut(150);                        // IMPORTANTE: Wire.begin() olvida el timeout anterior,
                                                // hay que volver a fijarlo tras cada reinicio del bus
  delay(50);                                   // Espera a que el sensor reconozca el bus
  _fallosConsec = 0;                           // Resetea el contador de fallos
  Debug.println("[I2C] Bus I2C reiniciado correctamente. Continuando...");
}

// ============================================================================
// FUNCIÓN PRIVADA: _escribirRegistro()
// Escribe un byte en un registro interno del MPU6500 por I2C.
// Retorna true si la escritura fue exitosa, false si hubo error de bus.
// [Uso exclusivamente interno — no llamar desde otros módulos]
// ============================================================================
static bool _escribirRegistro(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU6500_ADDR);        // Inicia transmisión I2C al sensor
  Wire.write(reg);                              // Envía la dirección del registro destino
  Wire.write(valor);                            // Envía el valor a escribir en ese registro
  uint8_t codigoError = Wire.endTransmission(true); // Confirma la transmisión; true = libera el bus

  if (codigoError != 0) {
    // Código de error Wire: 0=OK, 1=buffer lleno, 2=NACK addr, 3=NACK data, 4=otro
    Debug.print("[I2C] ERROR al escribir registro 0x");
    Debug.print(reg, HEX);                    // Imprime número de registro en hexadecimal
    Debug.print(" -> codigo: ");
    Debug.println(codigoError);               // Imprime el código de error Wire
    return false;                              // Indica fallo al llamador
  }
  return true;                                 // Escritura completada sin errores
}

// ============================================================================
// FUNCIÓN PRIVADA: _leerCrudoZ()
// Lee los 2 bytes (16 bits con signo) de velocidad angular del eje Z.
// Implementa timeout configurable para no bloquearse si hay ruido EMI.
// Gestiona automáticamente el contador de fallos y reinicia el bus si es necesario.
//
// CORRECCIÓN CRÍTICA vs versión anterior:
//   _tAnteriorMicros se actualiza SOLO aquí cuando la lectura es exitosa.
//   En la versión anterior se actualizaba siempre en giro_actualizarYaw(),
//   incluso en fallos, haciendo que dt fuera enorme en la siguiente lectura
//   válida y amplificando artificialmente la deriva del yaw.
//
// Retorna: valor crudo int16 (–32768 a +32767), o 0 si la lectura falló.
// ============================================================================
static int16_t _leerCrudoZ() {
  // --- FASE 1: Apunta el registro interno del sensor a GYRO_ZOUT_H ---
  Wire.beginTransmission(MPU6500_ADDR);          // Comienza diálogo I2C con el sensor
  Wire.write(MPU_REG_GYRO_ZOUT_H);               // Envía la dirección del registro a leer
  uint8_t err = Wire.endTransmission(false);      // false = repeated start (no libera el bus)
                                                   // Permite la lectura inmediata a continuación

  if (err != 0) {
    // El sensor no reconoció su dirección: bus trabado o sensor desconectado
    _i2cOK = false;                              // Marca el fallo
    _fallosConsec++;                             // Incrementa contador de fallos consecutivos
    if (_fallosConsec >= I2C_FALLOS_PARA_REINICIO) {
      _reiniciarBusI2C();                        // Reinicia el bus si hay demasiados fallos
    }
    return 0;                                    // Retorna 0 para no corromper el yaw
  }

  // --- FASE 2: Solicita los 2 bytes de datos (GYRO_ZOUT_H y GYRO_ZOUT_L) ---
  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)2);  // Pide 2 bytes al sensor

  // Espera los datos con timeout para no bloquearse por ruido EMI
  unsigned long tEspera = millis();              // Marca el inicio de la espera
  while (Wire.available() < 2) {                // Mientras no haya 2 bytes en el buffer...
    yield();                                     // Cede el CPU al scheduler de FreeRTOS: evita que
                                                  // esta espera activa bloquee la tarea IDLE y dispare
                                                  // el Task Watchdog del ESP32 mientras el bus responde
    if (millis() - tEspera > I2C_TIMEOUT_MS) {  // ¿Superó el tiempo límite?
      // Timeout: el sensor no respondió en I2C_TIMEOUT_MS milisegundos
      // Esto ocurre cuando el ruido EMI de los motores corrompe la transmisión
      _i2cOK = false;                            // Marca el fallo
      _fallosConsec++;                           // Incrementa contador de fallos
      if (_fallosConsec >= I2C_FALLOS_PARA_REINICIO) {
        _reiniciarBusI2C();                      // Reinicia el bus si hay muchos fallos seguidos
      }
      return 0;  // Retorna 0 → giro_actualizarYaw() no modifica _yaw ni _tAnteriorMicros
    }
  }

  // --- FASE 3: Lee los 2 bytes y combínalos en un valor de 16 bits con signo ---
  uint8_t byteAlto = Wire.read();               // Byte más significativo (bits 15-8)
  uint8_t byteBajo = Wire.read();               // Byte menos significativo (bits 7-0)

  // Lectura exitosa: resetea los contadores de error
  _i2cOK        = true;                         // Marca la operación como exitosa
  _fallosConsec = 0;                            // Resetea el contador de fallos consecutivos

  // Combina los dos bytes en un entero de 16 bits con signo (complemento a 2)
  return (int16_t)((byteAlto << 8) | byteBajo); // Desplaza el byte alto 8 bits y OR con el bajo
}

// ============================================================================
// FUNCIÓN PÚBLICA: giro_busOK()
// Retorna el estado de la última operación I2C con el sensor.
// true  = la última lectura fue exitosa
// false = la última lectura falló (timeout o error de bus)
// Útil para que otros módulos detecten degradación del bus sin leer el sensor.
// ============================================================================
bool giro_busOK() {
  return _i2cOK;  // Devuelve la bandera de estado de la última lectura
}

// ============================================================================
// FUNCIÓN PÚBLICA: giro_inicializar()
// Configura el hardware necesario para usar el MPU6500:
//   1. Desactiva el DAC del ESP32 en GPIO25 y GPIO26 (conflicto con LEDC/PWM)
//   2. Inicia el bus I2C a 100 kHz con los pines definidos en config_robot.h
//   3. Despierta el MPU6500 (sale del modo sleep de fábrica)
//   4. Configura el rango del giroscopio en ±250°/s (máxima sensibilidad)
// Debe llamarse UNA SOLA VEZ en setup(), antes de giro_calibrarYFijarNorte().
// ============================================================================
void giro_inicializar() {
  Debug.println("[GIROSCOPIO] Iniciando modulo de giroscopio...");

  // --- PASO 1: Desactiva el DAC del ESP32 en GPIO25 y GPIO26 ---
  // El ESP32 tiene dos DAC (Digital-Analógico) que comparten los pines físicos:
  //   DAC1 = GPIO25  (mismo pin que PIN_FR_PWM, el PWM del motor FR)
  //   DAC2 = GPIO26  (mismo pin que PIN_FR_IN1, la dirección del motor FR)
  // Si el DAC permanece activo, esos pines no responden como GPIO/PWM digital.
  // dac_output_disable() libera el pin para otros usos (LEDC/GPIO).
  dac_output_disable(DAC_CHANNEL_1);            // Libera GPIO25 del periférico DAC1
  dac_output_disable(DAC_CHANNEL_2);            // Libera GPIO26 del periférico DAC2
  Debug.println("[GIROSCOPIO] DAC desactivado en GPIO25 (DAC1) y GPIO26 (DAC2)");
  Debug.println("[GIROSCOPIO] GPIO25 y GPIO26 ahora disponibles para PWM/GPIO");

  // --- PASO 2: Inicia el bus I2C ---
  // 100 kHz (modo estándar) es más lento que 400 kHz (fast mode) pero tiene
  // mayor inmunidad al ruido EMI generado por los motores DC porque:
  //   - Los flancos de señal son más lentos → menos susceptibles a interferencia
  //   - El timing tiene más margen para recuperarse de transientes de ruido
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);         // Inicia I2C: SDA=GPIO21, SCL=GPIO22
  Wire.setClock(100000);                          // Velocidad = 100 kHz (1 bit cada 10 µs)

  // --- CORRECCIÓN CRÍTICA: timeout a nivel de driver ---
  // Wire.setTimeOut() limita cuánto puede bloquearse INTERNAMENTE el driver
  // I2C del ESP32 (dentro de endTransmission/requestFrom) si el bus se traba
  // por ruido EMI de los motores (ej. SCL retenido en LOW a medio byte).
  // Sin esto, el timeout manual de _leerCrudoZ() NO protege ese tramo, porque
  // el bloqueo ocurre ANTES de llegar al bucle que sí tiene timeout propio.
  // Este es el hueco mas probable detras de los reinicios: si el driver se
  // congela aqui, el bucle de movimiento nunca cede CPU y el Task Watchdog
  // del ESP32 (independiente del brownout detector) termina reiniciando todo.
  Wire.setTimeOut(150);                           // Maximo 150ms de bloqueo interno del driver I2C
  Debug.print("[GIROSCOPIO] I2C iniciado -> SDA=GPIO");
  Debug.print(PIN_MPU_SDA);                    // Imprime el número de pin SDA
  Debug.print(", SCL=GPIO");
  Debug.print(PIN_MPU_SCL);                    // Imprime el número de pin SCL
  Debug.println(", frecuencia=100 kHz (modo estandar, alta inmunidad a ruido)");

  // --- PASO 3: Despierta el MPU6500 ---
  // El MPU6500 arranca en modo sleep por defecto para ahorrar energía.
  // Escribir 0x00 en PWR_MGMT_1 desactiva el sleep y usa el reloj interno.
  Debug.println("[GIROSCOPIO] Despertando MPU6500 del modo sleep...");
  if (_escribirRegistro(MPU_REG_PWR_MGMT_1, 0x00)) {  // 0x00 = wake up + internal clock
    Debug.println("[GIROSCOPIO] MPU6500 despertado correctamente");
  } else {
    Debug.println("[GIROSCOPIO] *** ADVERTENCIA: No se pudo despertar el MPU6500 ***");
    Debug.println("[GIROSCOPIO] Verifica la conexion VCC, GND, SDA, SCL del sensor");
  }
  delay(100);  // Espera 100ms para que el oscilador interno del sensor estabilice

  // --- PASO 4: Configura el rango del giroscopio ---
  // El registro GYRO_CONFIG controla el rango de medición:
  //   0x00 = ±250°/s  (máxima sensibilidad, mínimo ruido — ELEGIDO)
  //   0x08 = ±500°/s
  //   0x10 = ±1000°/s
  //   0x18 = ±2000°/s (mínima sensibilidad, máximo rango)
  // Para el robot de fútbol, ±250°/s es más que suficiente: un giro típico
  // no supera 150-180°/s, y este rango da la mayor precisión disponible.
  Debug.println("[GIROSCOPIO] Configurando rango del giroscopio...");
  if (_escribirRegistro(MPU_REG_GYRO_CONFIG, 0x00)) {  // 0x00 = rango ±250°/s
    Debug.println("[GIROSCOPIO] Rango configurado: +/- 250 grados/segundo (maxima sensibilidad)");
  }
  delay(50);   // Espera 50ms para que la configuración sea aplicada por el sensor

  // Resumen del estado del módulo
  Debug.println("[GIROSCOPIO] *** MPU6500 inicializado y listo ***");
  Debug.print("[GIROSCOPIO] Timeout I2C configurado: ");
  Debug.print(I2C_TIMEOUT_MS);
  Debug.println(" ms");
  Debug.print("[GIROSCOPIO] Reinicio automatico de bus tras ");
  Debug.print(I2C_FALLOS_PARA_REINICIO);
  Debug.println(" fallos consecutivos");
}

// ============================================================================
// FUNCIÓN PÚBLICA: giro_calibrarYFijarNorte()
// Calcula el offset de reposo del eje Z del giroscopio tomando múltiples
// muestras mientras el robot está QUIETO, y fija el yaw actual como 0° (norte).
//
// El offset es el valor que reporta el sensor cuando NO hay rotación real.
// Todo giroscopio tiene un error de cero (bias); calcularlo y restarlo en cada
// lectura es esencial para que el yaw integrado no derive en reposo.
//
// ⚠ REQUISITO: El robot debe estar COMPLETAMENTE QUIETO durante esta función.
//   Cualquier vibración o movimiento durante la calibración introduce error.
// ============================================================================
void giro_calibrarYFijarNorte() {
  Debug.println("[GIROSCOPIO] *** CALIBRACION INICIADA ***");
  Debug.println("[GIROSCOPIO] *** NO MOVER EL ROBOT DURANTE LA CALIBRACION ***");

  double   sumaLecturas  = 0.0;  // Acumulador de velocidades angulares (para calcular promedio)
  int      muestrasOK    = 0;    // Contador de lecturas I2C exitosas (descarta lecturas corruptas)

  // Toma MUESTRAS_CALIBRACION_GIRO lecturas en reposo
  for (int i = 0; i < MUESTRAS_CALIBRACION_GIRO; i++) {
    int16_t valorCrudo = _leerCrudoZ();          // Lee la velocidad angular cruda del eje Z

    if (_i2cOK) {
      // Solo acumula si la lectura fue exitosa (filtra lecturas corruptas por ruido)
      sumaLecturas += (double)valorCrudo * GYRO_SENS; // Convierte LSB a °/s y acumula
      muestrasOK++;                                // Incrementa el contador de válidas
    }

    delay(2);   // Pausa de 2ms entre muestras: no satura el bus I2C (500 × 2ms = 1s total)

    // Imprime el progreso en el monitor serial cada 100 muestras
    if (i % 100 == 0) {
      Debug.print("[GIROSCOPIO] Calibrando: ");
      Debug.print(i);                           // Muestra el progreso actual
      Debug.print("/");
      Debug.print(MUESTRAS_CALIBRACION_GIRO);   // Muestra el total de muestras
      Debug.print("  validas: ");
      Debug.print(muestrasOK);                  // Muestra cuántas fueron exitosas
      Debug.print("  offset parcial: ");
      Debug.print(muestrasOK > 0 ? (float)(sumaLecturas / muestrasOK) : 0.0f, 4);
      Debug.println(" grados/s");               // Muestra el offset parcial hasta ahora
    }
  }

  // Calcula el offset final como el promedio de todas las lecturas válidas
  if (muestrasOK > 0) {
    _offsetZ = (float)(sumaLecturas / muestrasOK);  // Promedio del error de cero en °/s
    Debug.print("[GIROSCOPIO] Offset de cero calculado: ");
    Debug.print(_offsetZ, 5);                   // Imprime con 5 decimales para máxima info
    Debug.println(" grados/segundo");
  } else {
    // Ninguna lectura fue exitosa durante toda la calibración
    _offsetZ = 0.0f;                             // Sin offset calculado
    Debug.println("[GIROSCOPIO] *** ERROR CRITICO: 0 muestras validas en calibracion ***");
    Debug.println("[GIROSCOPIO] Posibles causas:");
    Debug.println("[GIROSCOPIO]   - Sensor MPU6500 no conectado o dañado");
    Debug.println("[GIROSCOPIO]   - Dirección I2C incorrecta (revisar pin AD0)");
    Debug.println("[GIROSCOPIO]   - Problema en bus SDA/SCL (cortocircuito, cables sueltos)");
  }

  // Fija el yaw actual como el "norte inicial" (referencia = 0°)
  _yaw             = 0.0f;      // Orientación actual declarada como 0° (norte)
  _tAnteriorMicros = micros();  // Inicia el reloj de integración desde este instante
  _fallosConsec    = 0;         // Resetea el contador de fallos

  // Resumen final de la calibración
  Debug.println("[GIROSCOPIO] *** CALIBRACION COMPLETA ***");
  Debug.print("[GIROSCOPIO] Muestras totales: ");
  Debug.print(MUESTRAS_CALIBRACION_GIRO);
  Debug.print("  exitosas: ");
  Debug.print(muestrasOK);
  Debug.print("  (");
  Debug.print((muestrasOK * 100) / MUESTRAS_CALIBRACION_GIRO); // Porcentaje de éxito
  Debug.println("%)");
  Debug.println("[GIROSCOPIO] Orientacion actual fijada como NORTE INICIAL = 0 grados");
}

// ============================================================================
// FUNCIÓN PÚBLICA: giro_actualizarYaw()
// Integra la velocidad angular del eje Z en el tiempo transcurrido para
// estimar el ángulo de orientación (yaw) acumulado desde el norte inicial.
//
// MÉTODO: Integración de Euler — Ángulo += VelocidadAngular × ΔTiempo
//
// CORRECCIÓN CRÍTICA: _tAnteriorMicros se actualiza DENTRO de _leerCrudoZ()
// solo cuando la lectura es exitosa. Si hay un fallo I2C (timeout por EMI),
// esta función retorna sin modificar ni el yaw ni el timestamp.
// Así se evita que dt sea artificialmente grande en la siguiente lectura válida,
// lo que causaba deriva falsa en la versión anterior.
//
// Debe llamarse lo más frecuentemente posible para mayor precisión de integración.
// Con motores encendidos, llamarla en cada iteración del bucle de movimiento.
// ============================================================================
void giro_actualizarYaw() {
  unsigned long ahoraMicros = micros();    // Captura el tiempo actual en microsegundos

  int16_t valorCrudo = _leerCrudoZ();     // Lee la velocidad angular cruda del eje Z
                                           // (si hay fallo I2C, retorna 0 y _i2cOK=false)

  if (!_i2cOK) {
    // Lectura fallida por ruido I2C: no integra para no corromper el yaw
    // NO actualiza _tAnteriorMicros: el dt se acumulará hasta la próxima lectura válida
    // pero solo contará el tiempo de ESTA lectura fallida, no el dt total acumulado
    // porque _tAnteriorMicros ya fue actualizado en la última lectura exitosa.
    return;  // Sale sin modificar _yaw ni _tAnteriorMicros
  }

  // --- Lectura exitosa: calcula dt y actualiza el timestamp ---
  float dt = (ahoraMicros - _tAnteriorMicros) / 1000000.0f;  // ΔTiempo en segundos
  _tAnteriorMicros = ahoraMicros;           // Actualiza el timestamp para la próxima llamada

  // Descarta intervalos de tiempo anómalos (>500ms indica que el sistema estuvo ocupado
  // mucho tiempo sin leer el giroscopio, como durante un delay largo o un reinicio de Wire)
  if (dt > 0.5f) {
    // dt demasiado grande: probablemente hubo una pausa larga en el código
    // Actualiza el timestamp pero no integra para no introducir un salto de ángulo enorme
    Debug.print("[GIROSCOPIO] dt anomalo ignorado: ");
    Debug.print(dt * 1000.0f, 1);          // Imprime dt en milisegundos
    Debug.println(" ms (reinicio de Wire o delay largo previo)");
    return;  // No integra; el próximo ciclo tendrá un dt normal
  }

  // --- Convierte lectura cruda a velocidad angular real y resta el offset ---
  float velAngular = (valorCrudo * GYRO_SENS) - _offsetZ;  // Velocidad real en °/s

  // Aplica zona muerta: si la velocidad es menor a 0.05°/s, se considera ruido
  // y no se integra. Reduce la deriva en reposo causada por el ruido del sensor.
  if (fabs(velAngular) < 0.05f) {
    return;  // Velocidad menor al umbral de ruido: no integra
  }

  // --- Integra el ángulo (método de Euler) ---
  _yaw += velAngular * dt;                  // Ángulo_nuevo = Ángulo_anterior + ω × dt

  // Normaliza el yaw al rango [-180°, +180°] para evitar que crezca sin límite
  if (_yaw >  180.0f) _yaw -= 360.0f;      // Si supera +180°: resta una vuelta completa
  if (_yaw < -180.0f) _yaw += 360.0f;      // Si baja de -180°: suma una vuelta completa
}

// ============================================================================
// FUNCIÓN PÚBLICA: giro_obtenerYaw()
// Retorna el ángulo de orientación actual respecto al norte inicial.
// Rango: -180° a +180°
// Positivo (+) = el robot giró en sentido horario desde el norte inicial.
// Negativo (-) = el robot giró en sentido antihorario desde el norte inicial.
// ============================================================================
float giro_obtenerYaw() {
  return _yaw;  // Devuelve el ángulo acumulado desde la calibración
}

// ============================================================================
// FUNCIÓN PÚBLICA: giro_estaAlineadoConNorte()
// Retorna true si el robot está dentro del margen de tolerancia respecto
// al norte inicial (yaw ≈ 0°).
// La tolerancia está definida en config_robot.h como TOLERANCIA_ORIENTACION_GRADOS.
// Si el yaw está dentro del margen, se considera que el robot "apunta al norte".
// ============================================================================
bool giro_estaAlineadoConNorte() {
  return fabs(_yaw) <= TOLERANCIA_ORIENTACION_GRADOS;  // |yaw| ≤ tolerancia → alineado
}

// ============================================================================
// FUNCIÓN PÚBLICA: giro_reiniciarBusManual()
// Permite que código externo (por ejemplo el .ino principal) fuerce un
// reinicio del bus I2C si detecta problemas que este módulo no vio.
// Útil para recuperación desde el loop principal o desde movimientos.h.
// ============================================================================
void giro_reiniciarBusManual() {
  Debug.println("[GIROSCOPIO] Reinicio manual del bus I2C solicitado desde codigo externo");
  _reiniciarBusI2C();  // Llama a la función privada de reinicio
}

#endif // GIROSCOPIO_H
