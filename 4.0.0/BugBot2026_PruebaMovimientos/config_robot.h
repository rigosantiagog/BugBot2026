/*
 * ============================================================================
 *  config_robot.h  —  BugBot 2026  v4
 *  Archivo CENTRAL de configuración: todos los pines y parámetros ajustables.
 * ============================================================================
 *  REGLA DE ORO: Si necesitas cambiar un pin o valor numérico,
 *  este es el ÚNICO archivo que debes tocar.
 *
 *  ESTRUCTURA DEL HARDWARE:
 *   Driver TB6612FNG #1  ->  Llantas DERECHAS  (FR, RR)  lado izq. ESP32
 *   Driver TB6612FNG #2  ->  Llantas IZQUIERDAS (FL, RL)  lado der. ESP32
 *   MPU6500 Giroscopio   ->  I2C  SDA=GPIO21  SCL=GPIO22
 *   STBY compartido (cable en Y)  ->  GPIO4
 *
 *  COMPATIBILIDAD: ESP32 Arduino core 3.x
 *   PWM: ledcAttach(pin,freq,bits) + ledcWrite(pin,duty)  — sin canales manuales
 * ============================================================================
 */

#ifndef CONFIG_ROBOT_H
#define CONFIG_ROBOT_H

// ============================================================================
// SECCIÓN 1 — STBY GLOBAL
// HIGH = ambos drivers activos  |  LOW = ambos en standby
// ============================================================================
#define PIN_STBY_DRIVERS   4    // GPIO4  -> STBY de los dos TB6612FNG (cable en Y)

// ============================================================================
// SECCIÓN 2 — DRIVER #1: LLANTAS DERECHAS  (FR delantera, RR trasera)
// Físicamente en el lado IZQUIERDO de la ESP32 (vista superior)
// Canal A -> Motor FR   |   Canal B -> Motor RR
// ============================================================================
#define PIN_FR_PWM   25   // GPIO25 -> PWMA  velocidad FR  (comparte hw con DAC1 — se desactiva)
#define PIN_FR_IN1   26   // GPIO26 -> AIN1  dirección FR
#define PIN_FR_IN2   27   // GPIO27 -> AIN2  dirección FR

#define PIN_RR_PWM   33   // GPIO33 -> PWMB  velocidad RR
#define PIN_RR_IN1   32   // GPIO32 -> BIN1  dirección RR
#define PIN_RR_IN2   14   // GPIO14 -> BIN2  dirección RR

// ============================================================================
// SECCIÓN 3 — DRIVER #2: LLANTAS IZQUIERDAS  (FL delantera, RL trasera)
// Físicamente en el lado DERECHO de la ESP32 (vista superior)
// Canal A -> Motor RL   |   Canal B -> Motor FL   (invertido por topología del chasis)
// ============================================================================
#define PIN_RL_PWM   13   // GPIO13 -> PWMA  velocidad RL
#define PIN_RL_IN1   23   // GPIO23 -> AIN1  dirección RL
#define PIN_RL_IN2    2   // GPIO2  -> AIN2  dirección RL  (pin de boot — ok una vez arrancado)

#define PIN_FL_PWM   19   // GPIO19 -> PWMB  velocidad FL
#define PIN_FL_IN1   18   // GPIO18 -> BIN1  dirección FL
#define PIN_FL_IN2    5   // GPIO5  -> BIN2  dirección FL  (pin de boot — ok una vez arrancado)

// ============================================================================
// SECCIÓN 4 — GIROSCOPIO MPU6500  (I2C)
// ============================================================================
#define PIN_MPU_SDA   21    // GPIO21 -> SDA datos I2C
#define PIN_MPU_SCL   22    // GPIO22 -> SCL reloj I2C
#define MPU6500_ADDR  0x68  // Dirección I2C cuando AD0=GND  (AD0=VCC → 0x69)

// ============================================================================
// SECCIÓN 5 — UART2: COMUNICACIÓN CON LA ESP32 DEL ANILLO IR
// ============================================================================
#define PIN_UART_RX2   16   // GPIO16 -> RX2  recibe la TramaData del anillo (conectar a GPIO25 del anillo)
#define PIN_UART_TX2   17   // GPIO17 -> TX2  manda el yaw de vuelta al anillo (conectar a GPIO26 del anillo)
                             // GPIO17 es el TX2 por defecto del ESP32 y estaba libre en este mapa de pines.
                             // IMPORTANTE: falta cablear fisicamente Motores-GPIO17 -> Anillo-GPIO26
                             // (Motores-GPIO16 <- Anillo-GPIO25 deberia ya estar conectado de antes).
#define BAUD_UART_ANILLO  38400   // Debe coincidir EXACTO con Enlace.begin() del lado del anillo

// ============================================================================
// SECCIÓN 6 — PARÁMETROS PWM  (periférico LEDC del ESP32 core 3.x)
// ============================================================================
#define PWM_FREQ_HZ         2000  // Frecuencia PWM = 2 kHz  (válido TB6612FNG: 1-20 kHz)
#define PWM_RESOLUCION_BITS    8  // Resolución 8 bits → duty cycle de 0 a 255
#define PWM_VELOCIDAD_MAX    255  // Duty cycle al 100%

// ============================================================================
// SECCIÓN 7 — VELOCIDADES Y RAMPA DE ACELERACIÓN
//
// PROBLEMA RESUELTO AQUÍ:
//   Cuando los 4 motores arrancan simultáneamente a plena velocidad, el pico
//   de corriente puede superar 4-5 A durante 50-100 ms. Este pico induce ruido
//   en los cables paralelos y puede colapsar el voltaje USB de la ESP32,
//   causando un reset (brownout) aunque el detector de brownout esté desactivado.
//
// SOLUCIÓN — RAMPA DE ACELERACIÓN:
//   En lugar de saltar de 0 a VELOCIDAD_PRUEBA de golpe, el sistema sube el
//   duty cycle gradualmente desde VELOCIDAD_ARRANQUE hasta VELOCIDAD_PRUEBA
//   en RAMPA_DURACION_MS milisegundos. Esto distribuye el pico de corriente
//   en el tiempo, reduciendo el pico instantáneo de ~5 A a ~1-2 A.
// ============================================================================

#define VELOCIDAD_ARRANQUE        40  // PWM inicial de la rampa (0-255) — muy bajo para arranque suave

// --- PRUEBA DE DIAGNOSTICO TEMPORAL (ver conversacion con Claude) ---
// Bajado de 150 a 90 SOLO para aislar la causa del reinicio en linea recta.
// Logica de la prueba:
//   - Si con PWM=90 la secuencia completa corre SIN reiniciarse -> confirma
//     que el problema es corriente/voltaje insuficiente bajo carga sostenida
//     (fuente/bateria/cableado no aguantan la demanda de los 4 motores a 150).
//   - Si el reinicio persiste IGUAL incluso a 90 -> descarta corriente pura
//     como causa unica, apunta a algo mas grave (corto intermitente, BMS de
//     la bateria cortando por sobrecorriente incluso a baja demanda, o una
//     conexion de alta resistencia que cae significativo con cualquier carga).
// Una vez identificada y corregida la causa raiz en hardware, subir este
// valor de nuevo gradualmente (90 -> 120 -> 150 -> 200) validando en cada
// paso que la secuencia completa corra sin reiniciar antes de subir mas.
#define VELOCIDAD_PRUEBA           90  // PWM objetivo temporal para diagnostico (era 150)
                                      // Si los motores van muy despacio, sube de a 10 hasta max 200
#define RAMPA_DURACION_MS        400  // Milisegundos para subir de ARRANQUE a OBJETIVO
                                      // 400ms distribuye el pico de corriente con seguridad

#define DURACION_MOVIMIENTO_MS  1800  // Duración total de cada movimiento (ms), incluye la rampa
#define PAUSA_ENTRE_MOVS_MS      600  // Pausa entre movimientos — da tiempo al I2C de recuperarse
#define PAUSA_FINAL_MS          2000  // Pausa de 2 s al terminar la secuencia completa

// ============================================================================
// SECCIÓN 8 — GIROSCOPIO Y CORRECCIÓN DE ORIENTACIÓN
// ============================================================================
#define MUESTRAS_CALIBRACION_GIRO      500  // Lecturas en reposo para calcular el offset de cero
#define TOLERANCIA_ORIENTACION_GRADOS  3.0f // Si |yaw| ≤ 3° → robot alineado con el norte
#define VELOCIDAD_CORRECCION_GIRO      100  // PWM de corrección de rumbo (lento para precisión)

// ============================================================================
// SECCIÓN 9 — ROBUSTEZ DEL BUS I2C
// ============================================================================
#define I2C_TIMEOUT_MS            50   // ms máximos esperando respuesta del MPU6500
                                       // Aumentado a 50ms para tolerar ruido EMI de motores
#define I2C_FALLOS_PARA_REINICIO  10   // Fallos consecutivos antes de reiniciar Wire en caliente

// ============================================================================
// SECCIÓN 10 — WATCHDOG DE MOVIMIENTO
// ============================================================================
#define WATCHDOG_FACTOR            3   // Tiempo max = DURACION_MOVIMIENTO_MS × 3
#define WATCHDOG_CORRECCION_MS  10000  // 10 s máximos para corrección de orientación

constexpr char IP_SERVIDOR_LOGS[] = "192.168.4.2";
constexpr uint16_t PUERTO_DEBUG_WIFI = 5000;   // Debe coincidir con PORT en puerto_anillo.py

// Tiempo maximo que setup() espera, UNA SOLA VEZ al arrancar, a que se conecte
// el servidor de logs antes de seguir con motores/giroscopio/movimientos. Si se
// agota este tiempo, el robot arranca de todos modos (sin bloquear para siempre)
// y sigue reintentando conectar en segundo plano durante la operacion normal.
constexpr unsigned long TIMEOUT_ESPERA_LOGS_MS = 15000;   // 15 segundos

// ============================================================================
// SECCIÓN 11 — PERSECUCIÓN DE LA PELOTA (integración con el anillo IR)
//
// Decisiones de diseño confirmadas con el usuario:
//   - Sin pelota (estado=0, o enlace con el anillo caido) -> girar despacio buscando
//   - Zona muerta angular para ir derecho sin corregir      -> 8 grados
//   - Velocidad: proporcional al error de angulo, Y ADEMAS frena cerca de paredes
//     usando distFrente que ya viene en la TramaData del anillo
// ============================================================================

// Velocidad PWM (0-255) mientras el robot gira buscando la pelota (estado=0)
#define VELOCIDAD_BUSQUEDA          70

// Zona muerta: si |anguloError| esta dentro de este rango, se considera que la
// pelota ya esta "al frente" y el robot avanza derecho sin corregir el rumbo.
// Evita que el robot tiemble corrigiendo constantemente por ruido de +-1-2 grados.
#define ZONA_MUERTA_ANGULO_GRADOS   8.0f

// Angulo de error en el que la correccion de giro llega a su maximo (+-1.0,
// es decir, el 100% de VELOCIDAD_PERSEGUIR_MAX destinado a girar). Con 45
// grados: un error de 45 grados o mas ya gira a la maxima tasa disponible.
#define ANGULO_GIRO_MAXIMO          45.0f

// Velocidad PWM (0-255) maxima al perseguir la pelota, ANTES de aplicarle
// los factores de reduccion por angulo y por cercania a una pared.
#define VELOCIDAD_PERSEGUIR_MAX     130

// Distancia (cm) a partir de la cual empieza a frenar el avance hacia el
// frente. A esta distancia o menos el factor de frenado es < 1.0; en 0 cm
// el factor es 0 (frenado total). Los ultrasonidos reportan 999 cuando no
// detectan nada dentro de su rango, lo cual naturalmente da factor=1.0
// (sin frenado) porque 999 es mucho mayor a esta distancia de frenado.
#define DISTANCIA_FRENADO_CM        25

// Si la velocidad final calculada (ya con los 2 factores aplicados) queda
// por debajo de este PWM, se redondea a 0 en vez de dejar el motor "zumbando"
// sin moverse (los motores DC necesitan un PWM minimo para vencer la friccion
// estatica; un PWM muy bajo solo calienta el motor sin producir movimiento util).
#define VELOCIDAD_MINIMA_UTIL       25

// Si no llega una TramaData valida del anillo en mas de este tiempo, se
// considera el enlace UART caido (cable suelto, anillo reiniciandose, etc.)
// y el robot entra en modo busqueda por seguridad, sin importar cual haya
// sido el ultimo estado recibido -- nunca debe seguir avanzando "a ciegas"
// con datos viejos del anillo.
#define MAX_ANTIGUEDAD_TRAMA_MS     300


#endif // CONFIG_ROBOT_H
