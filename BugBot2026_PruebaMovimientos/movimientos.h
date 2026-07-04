/*
 * ============================================================================
 *  movimientos.h  —  BugBot 2026
 *  Catálogo de movimientos mecanum y corrección de orientación hacia el norte.
 * ============================================================================
 *  RESPONSABILIDAD DE ESTE MÓDULO:
 *   - Definir cada maniobra del robot como una función con nombre descriptivo.
 *   - Ejecutar la secuencia completa de prueba de todos los movimientos.
 *   - Corregir la orientación del robot si se desvió del norte inicial.
 *
 *  PROTECCIONES IMPLEMENTADAS:
 *   - Watchdog en cada movimiento: si el bucle interno se bloquea (por
 *     un fallo I2C u otro problema), fuerza la salida tras WATCHDOG_FACTOR
 *     veces la duración normal del movimiento.
 *   - Prints periódicos dentro del bucle de movimiento para poder ver en
 *     el monitor serial si el sistema está corriendo o se congeló.
 *   - Watchdog de tiempo en la corrección de orientación para no girar
 *     indefinidamente si el giroscopio reporta datos incorrectos.
 * ============================================================================
 */

#ifndef MOVIMIENTOS_H   // Guarda de inclusión
#define MOVIMIENTOS_H

#include <Arduino.h>        // Tipos y funciones base del entorno Arduino
#include "config_robot.h"   // Pines y constantes del robot
#include "motores.h"        // Control de bajo nivel de los motores mecanum
#include "giroscopio.h"     // Lectura, yaw y recuperación del bus I2C

// ============================================================================
// CONSTANTES DE ESTE MÓDULO
// Se definen AQUÍ ARRIBA para que estén disponibles antes de ser usadas
// ============================================================================

// Tiempo máximo que puede durar UN movimiento antes de que el watchdog
// fuerce la salida del bucle (duración normal × WATCHDOG_FACTOR)
// Ejemplo: 1500ms × 3 = 4500ms máximo antes de abortar por bloqueo
#define WATCHDOG_MOV_MS   ((unsigned long)DURACION_MOVIMIENTO_MS * WATCHDOG_FACTOR)

// Tiempo máximo permitido para la corrección de orientación completa.
// Si tras 10 segundos el robot no se alineó, aborta para no girar infinito.
#define WATCHDOG_CORRECCION_MS  10000   // 10 segundos máximo para realinearse

// ============================================================================
// FUNCIÓN: ejecutarMovimiento()
// Ejecuta UNA maniobra durante DURACION_MOVIMIENTO_MS milisegundos.
// Mantiene el giroscopio actualizado dentro del bucle de movimiento.
// Incluye watchdog para evitar bloqueos por fallo I2C u otros problemas.
//
// Parámetros:
//   nombre   -> Nombre descriptivo del movimiento (solo para el serial)
//   adelante -> Componente adelante/atrás (-1.0 a +1.0)
//   lateral  -> Componente lateral izq/der (-1.0 a +1.0)
//   giro     -> Componente de rotación antihorario/horario (-1.0 a +1.0)
// ============================================================================
void ejecutarMovimiento(const char* nombre, float adelante, float lateral, float giro) {
  Serial.println("--------------------------------------------------");
  Serial.print("[MOV] INICIANDO: ");
  Serial.println(nombre);                         // Nombre descriptivo del movimiento
  Serial.print("[MOV] Params -> adelante=");
  Serial.print(adelante, 1);                      // Componente adelante/atrás (-1 a +1)
  Serial.print("  lateral=");
  Serial.print(lateral, 1);                       // Componente lateral (-1 a +1)
  Serial.print("  giro=");
  Serial.print(giro, 1);                          // Componente de rotación (-1 a +1)
  Serial.print("  PWM objetivo=");
  Serial.print(VELOCIDAD_PRUEBA);                 // Velocidad PWM máxima del movimiento
  Serial.print("  rampa=");
  Serial.print(RAMPA_DURACION_MS);
  Serial.println("ms");

  unsigned long tInicio    = millis();             // Timestamp de inicio del movimiento
  unsigned long tWatchdog  = millis();             // Timestamp para control del watchdog
  uint32_t      iteraciones = 0;                   // Contador de ciclos del bucle principal

  // --- FASE 1: RAMPA DE ACELERACIÓN ---
  // Sube la velocidad gradualmente para evitar el pico de corriente que
  // causaba el reset de la ESP32. Llama a giro_actualizarYaw durante la rampa.
  Serial.println("[MOV] Fase 1: Rampa de aceleracion...");
  mover_mecanum_con_rampa(adelante, lateral, giro, VELOCIDAD_PRUEBA, giro_actualizarYaw);
  Serial.println("[MOV] Fase 2: Velocidad de crucero...");

  // --- FASE 2: VELOCIDAD DE CRUCERO ---
  // Mantiene la velocidad objetivo hasta completar DURACION_MOVIMIENTO_MS total
  while (millis() - tInicio < DURACION_MOVIMIENTO_MS) {

    // WATCHDOG: si el bucle tarda más de lo esperado, sale forzadamente
    // Tiempo máximo = DURACION_MOVIMIENTO_MS × WATCHDOG_FACTOR
    if (millis() - tWatchdog > WATCHDOG_MOV_MS) {
      Serial.print("[MOV] *** WATCHDOG en '");
      Serial.print(nombre);
      Serial.print("' t=");
      Serial.print(millis() - tWatchdog);
      Serial.println("ms — salida forzada ***");
      break;  // Rompe el bucle para no quedarse atascado indefinidamente
    }

    mover_mecanum(adelante, lateral, giro, VELOCIDAD_PRUEBA); // Mantiene velocidad crucero

    // Actualiza el giroscopio — maneja internamente timeout y reinicio de bus I2C
    giro_actualizarYaw();

    // CORRECCIÓN: cede CPU al scheduler de FreeRTOS en cada iteración.
    // Este bucle puede correr varios segundos seguidos sin ningún delay();
    // si el I2C se degrada por EMI de los motores, sin este yield() la tarea
    // IDLE del ESP32 puede quedarse sin CPU el tiempo suficiente para que el
    // Task Watchdog reinicie la placa (mecanismo distinto al brownout detector).
    yield();

    iteraciones++;  // Cuenta cuántas iteraciones del bucle se completaron

    // Imprime estado del movimiento cada ~200 iteraciones para no saturar el serial
    if (iteraciones % 200 == 0) {
      Serial.print("[MOV] '");
      Serial.print(nombre);
      Serial.print("'  t=");
      Serial.print(millis() - tInicio);           // Tiempo total transcurrido (ms)
      Serial.print("ms  yaw=");
      Serial.print(giro_obtenerYaw(), 2);          // Orientación actual (grados)
      Serial.print((char)176);                     // Símbolo de grado '°'
      Serial.print("  I2C=");
      Serial.println(giro_busOK() ? "OK" : "FALLO"); // Estado del bus I2C
    }
  }

  motores_detener();  // Frena las 4 ruedas al terminar el movimiento

  // Imprime resumen del movimiento completado
  Serial.print("[MOV] TERMINADO: '");
  Serial.print(nombre);
  Serial.print("'  yaw=");
  Serial.print(giro_obtenerYaw(), 2);
  Serial.print((char)176);
  Serial.print("  duracion=");
  Serial.print(millis() - tInicio);
  Serial.println("ms");

  // Pausa entre movimientos: da tiempo al bus I2C de estabilizarse
  Serial.print("[MOV] Pausa: ");
  Serial.print(PAUSA_ENTRE_MOVS_MS);
  Serial.println("ms");
  delay(PAUSA_ENTRE_MOVS_MS);  // Pausa configurable en config_robot.h
}

// ============================================================================
// FUNCIÓN: ejecutarSecuenciaCompletaDeMovimientos()
// Ejecuta EN ORDEN todos los movimientos posibles del chasis mecanum.
// Se llama desde loop() en el archivo principal (.ino).
// ============================================================================
void ejecutarSecuenciaCompletaDeMovimientos() {
  // Encabezado de la secuencia en el monitor serial
  Serial.println("==================================================");
  Serial.println("[SEQ] INICIANDO SECUENCIA COMPLETA DE MOVIMIENTOS");
  Serial.println("[SEQ] Total de movimientos: 12");
  Serial.println("==================================================");

  // --- GRUPO 1: Movimientos lineales puros ---
  // Solo componente adelante/atrás, sin lateral ni giro
  Serial.println("[SEQ] === GRUPO 1: Movimientos lineales ===");
  ejecutarMovimiento("Adelante",           1.0f,  0.0f,  0.0f); // Avanza recto
  ejecutarMovimiento("Atras",             -1.0f,  0.0f,  0.0f); // Retrocede recto

  // --- GRUPO 2: Movimientos laterales puros ---
  // Exclusivo de ruedas mecanum: se desliza sin girar
  Serial.println("[SEQ] === GRUPO 2: Movimientos laterales (exclusivo mecanum) ===");
  ejecutarMovimiento("Lateral derecha",    0.0f,  1.0f,  0.0f); // Desliza a la derecha
  ejecutarMovimiento("Lateral izquierda",  0.0f, -1.0f,  0.0f); // Desliza a la izquierda

  // --- GRUPO 3: Movimientos diagonales ---
  // Combinación de adelante/atrás + lateral
  Serial.println("[SEQ] === GRUPO 3: Movimientos diagonales ===");
  ejecutarMovimiento("Diagonal adelante-derecha",   1.0f,  1.0f,  0.0f); // 45° adelante-derecha
  ejecutarMovimiento("Diagonal adelante-izquierda", 1.0f, -1.0f,  0.0f); // 45° adelante-izquierda
  ejecutarMovimiento("Diagonal atras-derecha",     -1.0f,  1.0f,  0.0f); // 45° atrás-derecha
  ejecutarMovimiento("Diagonal atras-izquierda",   -1.0f, -1.0f,  0.0f); // 45° atrás-izquierda

  // --- GRUPO 4: Rotaciones sobre el propio eje ---
  // Solo componente de giro, sin desplazamiento
  Serial.println("[SEQ] === GRUPO 4: Rotaciones in situ ===");
  ejecutarMovimiento("Giro horario",       0.0f,  0.0f,  1.0f); // Gira a la derecha sobre el eje
  ejecutarMovimiento("Giro antihorario",   0.0f,  0.0f, -1.0f); // Gira a la izquierda sobre el eje

  // --- GRUPO 5: Movimientos curvos ---
  // Combinación de avance + giro para trayectorias curvas
  Serial.println("[SEQ] === GRUPO 5: Movimientos curvos ===");
  ejecutarMovimiento("Curva adelante-horario",      1.0f,  0.0f,  0.5f); // Avanza girando a la derecha
  ejecutarMovimiento("Curva adelante-antihorario",  1.0f,  0.0f, -0.5f); // Avanza girando a la izquierda

  // Resumen final de la secuencia
  Serial.println("==================================================");
  Serial.println("[SEQ] SECUENCIA COMPLETA FINALIZADA (12/12 movimientos)");
  Serial.print("[SEQ] Yaw acumulado al finalizar: ");
  Serial.print(giro_obtenerYaw(), 2);
  Serial.println(" grados");
  Serial.println("==================================================");
}

// ============================================================================
// FUNCIÓN: corregirOrientacionHaciaNorte()
// Gira lentamente el robot en el sentido más corto hasta que el yaw vuelva
// a estar dentro de la tolerancia respecto al norte inicial (0°).
// Incluye watchdog para evitar giro infinito si el giroscopio falla.
// ============================================================================
void corregirOrientacionHaciaNorte() {
  Serial.println("--------------------------------------------------");
  Serial.println("[COR] Robot FUERA de orientacion inicial. Iniciando correccion...");
  Serial.print("[COR] Yaw actual: ");
  Serial.print(giro_obtenerYaw(), 2);
  Serial.print((char)176);                       // Símbolo de grados
  Serial.print("  Tolerancia: +/-");
  Serial.print(TOLERANCIA_ORIENTACION_GRADOS, 1);
  Serial.println((char)176);

  unsigned long tInicio         = millis();      // Tiempo de inicio de la corrección
  unsigned long tUltimoPrint    = 0;             // Control de impresión periódica

  // Gira hasta estar alineado dentro de la tolerancia o hasta que el watchdog lo detenga
  while (!giro_estaAlineadoConNorte()) {

    // Watchdog de tiempo: aborta si la corrección tarda demasiado
    if (millis() - tInicio > WATCHDOG_CORRECCION_MS) {
      Serial.print("[COR] *** WATCHDOG: correccion abortada tras ");
      Serial.print(WATCHDOG_CORRECCION_MS / 1000);
      Serial.println(" segundos — posible fallo del giroscopio ***");
      Serial.print("[COR] Yaw al abortar: ");
      Serial.print(giro_obtenerYaw(), 2);
      Serial.println((char)176);
      break;   // Sale del bucle aunque no haya llegado al norte
    }

    // Actualiza el yaw antes de decidir el sentido de corrección
    giro_actualizarYaw();
    float yawActual = giro_obtenerYaw();    // Lee el ángulo actual

    // Decide el sentido de giro para acortar la distancia angular al norte:
    //   Si yaw > 0: el robot giró en sentido horario → corrige girando antihorario (-1.0)
    //   Si yaw < 0: el robot giró en sentido antihorario → corrige girando horario (+1.0)
    float sentido = (yawActual > 0.0f) ? -1.0f : 1.0f;

    // Gira lentamente con velocidad reducida para mayor precisión
    mover_mecanum(0.0f, 0.0f, sentido, VELOCIDAD_CORRECCION_GIRO);

    // CORRECCIÓN: cede CPU al scheduler de FreeRTOS en cada iteración,
    // por la misma razón que en ejecutarMovimiento() — este bucle también
    // puede correr varios segundos sin ceder CPU si el I2C falla seguido.
    yield();

    // Imprime el progreso de la corrección cada 300ms (no en cada ciclo)
    if (millis() - tUltimoPrint > 300) {
      Serial.print("[COR] Yaw: ");
      Serial.print(yawActual, 2);
      Serial.print((char)176);
      Serial.print("  Girando: ");
      Serial.print(sentido > 0 ? "HORARIO" : "ANTIHORARIO");
      Serial.print("  Velocidad: ");
      Serial.print(VELOCIDAD_CORRECCION_GIRO);
      Serial.print("  Tiempo: ");
      Serial.print(millis() - tInicio);
      Serial.println("ms");
      tUltimoPrint = millis();   // Actualiza el tiempo del último print
    }
  }

  // Detiene los motores al terminar la corrección
  motores_detener();

  // Imprime el resultado final de la corrección
  Serial.print("[COR] Correccion FINALIZADA. Yaw final: ");
  Serial.print(giro_obtenerYaw(), 2);
  Serial.print((char)176);
  Serial.print("  Tiempo total: ");
  Serial.print(millis() - tInicio);
  Serial.println("ms");
  Serial.println("--------------------------------------------------");
}

#endif // MOVIMIENTOS_H
