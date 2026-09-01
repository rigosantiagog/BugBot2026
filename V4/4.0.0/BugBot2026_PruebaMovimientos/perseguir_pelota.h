/*
 * ============================================================================
 *  perseguir_pelota.h  —  BugBot 2026
 *  Logica de decision: convierte la TramaData del anillo en un comando de
 *  movimiento mecanum concreto (avanzar hacia la pelota, girar buscandola,
 *  o frenar cerca de una pared).
 * ============================================================================
 *  RESPONSABILIDAD DE ESTE MODULO:
 *   - NO habla directo con el hardware: usa mover_mecanum() de motores.h,
 *     que ya sabe traducir (adelante,lateral,giro,velocidad) a las 4 ruedas.
 *   - NO lee el UART directamente: usa obtenerUltimaTramaAnillo() y
 *     tramaAnilloEsReciente() de comunicacion_anillo.h.
 *   - Se llama UNA VEZ por cada vuelta de loop() (no bloqueante, no tiene
 *     bucles internos ni delay()): siempre reacciona al dato mas reciente.
 *
 *  NOTA DE ESTILO: header-only (sin .cpp separado), igual que el resto de
 *  los modulos de este proyecto (motores.h, giroscopio.h, movimientos.h,
 *  comunicacion_anillo.h). Ver la nota completa en comunicacion_anillo.h
 *  sobre por que NO debe crearse un .cpp separado para este archivo.
 * ============================================================================
 */

#ifndef PERSEGUIR_PELOTA_H   // Guarda de inclusion: evita compilar este archivo dos veces
#define PERSEGUIR_PELOTA_H

#include <Arduino.h>              // constrain(), fabs(), tipos base
#include "config_robot.h"         // Constantes ajustables (VELOCIDAD_BUSQUEDA, ZONA_MUERTA...)
#include "motores.h"               // mover_mecanum(): ejecuta el movimiento en las 4 ruedas
#include "comunicacion_anillo.h"  // TramaData, obtenerUltimaTramaAnillo(), tramaAnilloEsReciente()
#include "debug_wifi.h"            // Debug.print/println: reporte de que esta haciendo el robot

// ----------------------------------------------------------------------------
// _girarBuscando()
// [uso interno] Rotacion pura in-situ, sin avanzar ni deslizar, a velocidad
// constante y reducida. Se usa cuando no hay pelota detectada o el enlace
// con el anillo no es confiable.
// ----------------------------------------------------------------------------
static void _girarBuscando() {
  // adelante=0, lateral=0, giro=+1.0 (horario) -> rotacion pura sobre su propio eje.
  // Nota: siempre busca hacia el mismo lado (horario) por simplicidad; si se
  // prefiere alternar el sentido cada cierto tiempo, es el lugar para agregarlo.
  mover_mecanum(0.0f, 0.0f, 1.0f, VELOCIDAD_BUSQUEDA);
}

// ----------------------------------------------------------------------------
// _normalizarError()
// [uso interno] Convierte un angulo en rango 0-360 (como lo manda el anillo)
// al error angular mas corto respecto al frente del robot, en rango -180..+180.
// Ejemplo: 350 grados (casi al frente, un poquito hacia un lado) se convierte
// en -10 grados en vez de quedarse como +350, que seria un giro absurdamente
// largo si se usara tal cual.
// ----------------------------------------------------------------------------
static float _normalizarError(float anguloCrudo) {
  float error = anguloCrudo;               // Punto de partida: el angulo tal como llego
  if (error > 180.0f) error -= 360.0f;     // Si esta en la mitad "lejana", se resta una vuelta completa
  return error;                             // Queda en rango -180..+180 (el camino mas corto)
}

// ----------------------------------------------------------------------------
// _perseguirConAngulo()
// [uso interno] Calcula y aplica el movimiento hacia la pelota dado su error
// angular (ya normalizado) y la distancia frontal reportada por el anillo.
// ----------------------------------------------------------------------------
static void _perseguirConAngulo(float anguloError, uint16_t distFrenteCm) {
  // --- Componente de giro: proporcional al error, saturado en +-1.0 ---
  // ANGULO_GIRO_MAXIMO es el error a partir del cual ya se gira "a fondo".
  // Si el error de angulo es NEGATIVO (pelota mas cerca por el otro lado que
  // el que asume esta formula), el robot va a girar hacia el lado contrario
  // al esperado -- si eso pasa en la prueba real, invertir el signo aqui
  // (cambiar "anguloError" por "-anguloError" en la linea de abajo).
  float direccionGiro = constrain(anguloError / ANGULO_GIRO_MAXIMO, -1.0f, 1.0f);

  // --- Zona muerta: si el error es chico, no se corrige el rumbo ---
  // Evita que el robot "tiemble" corrigiendo constantemente por ruido de
  // +-1-2 grados en la lectura del anillo mientras deberia ir derecho.
  if (fabs(anguloError) <= ZONA_MUERTA_ANGULO_GRADOS) {
    direccionGiro = 0.0f;                  // Dentro de la zona muerta: va derecho, sin girar
  }

  // --- Factor angular: entre mas grande el giro necesario, menos avance ---
  // Con error=0 -> factor=1.0 (avanza a toda velocidad, sin restar nada).
  // Con error grande -> el factor baja, pero nunca por debajo de 0.3 (30%),
  // para que el robot siempre siga avanzando algo mientras gira, en vez de
  // quedarse "pivoteando" en el mismo lugar sin acercarse a la pelota.
  float factorAngular = 1.0f - constrain(fabs(anguloError) / 90.0f, 0.0f, 0.7f);

  // --- Factor de distancia: frena progresivamente cerca de una pared ---
  // distFrenteCm entre 0 y DISTANCIA_FRENADO_CM -> factor entre 0.0 y 1.0.
  // Mas alla de DISTANCIA_FRENADO_CM (incluyendo el 999 de "sin lectura" que
  // manda el anillo cuando no hay nada en rango) -> factor=1.0, sin frenar.
  float factorDistancia = constrain((float)distFrenteCm / (float)DISTANCIA_FRENADO_CM, 0.0f, 1.0f);

  // --- Velocidad final: combina ambos factores sobre la velocidad maxima ---
  int velocidadFinal = (int)(VELOCIDAD_PERSEGUIR_MAX * factorAngular * factorDistancia);

  // Si la velocidad calculada es demasiado baja para vencer la friccion
  // estatica del motor, se redondea a 0 (frena en vez de "zumbar" sin moverse)
  if (velocidadFinal < VELOCIDAD_MINIMA_UTIL) {
    velocidadFinal = 0;
  }

  // adelante=1.0 (siempre "hacia adelante" en direccion), lateral=0.0 (no se
  // usa desplazamiento lateral en este modo), giro=direccionGiro (la correccion
  // proporcional calculada arriba), y la velocidad real ya viene escalada
  // por ambos factores en velocidadFinal.
  mover_mecanum(1.0f, 0.0f, direccionGiro, velocidadFinal);
}

// ----------------------------------------------------------------------------
// perseguirPelota()
// Decide y ejecuta UN paso de movimiento segun la ultima TramaData del anillo:
//   - Si no hay pelota (estado=0) o el enlace con el anillo esta caido
//     (tramaAnilloEsReciente()==false): gira despacio buscando.
//   - Si hay pelota: avanza hacia ella con correccion de giro proporcional
//     al error de angulo, reduciendo la velocidad si el giro es grande y/o
//     si se acerca a una pared (usando distFrente del anillo).
// Llamar en CADA vuelta de loop(), sin delay() antes ni despues que no sea
// el de ritmo general del loop.
// ----------------------------------------------------------------------------
void perseguirPelota() {
  // Primero se verifica que el enlace UART con el anillo este vivo. Si no,
  // NUNCA se debe confiar en datos viejos para decidir hacia donde avanzar:
  // se trata igual que "sin pelota" y el robot gira buscando por seguridad.
  if (!tramaAnilloEsReciente()) {
    Debug.println("[PERSEGUIR] Enlace con el anillo caido o sin datos aun -> girando buscando");
    _girarBuscando();
    return;   // No hay mas nada que hacer esta vuelta de loop()
  }

  // Se obtiene la ultima trama valida recibida (referencia de solo lectura)
  const TramaData& trama = obtenerUltimaTramaAnillo();

  if (trama.estado == 0) {
    // El anillo no detecto la pelota en su ultima lectura: buscar girando
    Debug.println("[PERSEGUIR] Sin pelota detectada -> girando buscando");
    _girarBuscando();
    return;
  }

  // Hay pelota detectada: calcula el error angular respecto al frente del
  // robot (el angulo que manda el anillo ya viene ajustado con su OFFSET_FRENTE,
  // asi que 0 grados = pelota justo al frente del robot)
  float anguloError = _normalizarError(trama.angulo);

  // Aplica el control proporcional + frenado por distancia y mueve el robot
  _perseguirConAngulo(anguloError, trama.distFrente);

  // Reporte periodico de que esta haciendo el robot (util para depurar por WiFi)
  Debug.print("[PERSEGUIR] Pelota detectada. anguloError=");
  Debug.print(anguloError, 1);                 // Error angular con 1 decimal
  Debug.print("  distFrente=");
  Debug.print(trama.distFrente);                // Distancia frontal en cm (o 999 si no hay lectura)
  Debug.println("cm");
}

#endif // PERSEGUIR_PELOTA_H
