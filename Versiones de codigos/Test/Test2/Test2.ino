/* ==================================================================
   ESP32 MOTORES (RX) v8 - Búsqueda Táctica Sectorial Integrada
   ================================================================== */
#include <Arduino.h> // Incluye las rutinas principales de la ESP32
#include <Wire.h>    // Incluye la librería para leer el giroscopio por I2C

#define MODO 1 // 0 = Modo de Calibración quieto | 1 = Modo Competencia Activo

// ==================================================================
// PARÁMETROS AJUSTABLES
// ==================================================================
const float FRENTE_ANILLO = 0.0;   // Ángulo exacto donde el frente del chasis ataca la pelota
const float TOL_APUNTADO  = 40.0;  // *** AUMENTADO A 40 PARA PRUEBAS CON LED ***
const unsigned long T_QUIETO_MS = 4000; // Milisegundos de congelamiento inicial para afinar MPU
const int N_CAPTURA = 3;           // Cantidad de sensores iluminados que confirman posesión

const float YAW_PORTERIA = 0.0;  // Grados hacia la red del oponente
const float TOL_PORTERIA = 15.0;   // Margen de desviación permitido al avanzar a la portería

unsigned long long tiempoparacambiodeestado = 0;

/* ===================== PINES DE CONTROL DE MOTORES ========================== */
#define STBY  4 // Pin de habilitación (Standby) de los chips TB6612FNG

#define FL_PWM 25 // Control de velocidad Motor Frontal Izquierdo
#define FL_IN1 26 // Pin lógico de dirección 1 Frontal Izquierdo
#define FL_IN2 27 // Pin lógico de dirección 2 Frontal Izquierdo

#define RL_PWM 33 // Control de velocidad Motor Trasero Izquierdo
#define RL_IN1 32 // Pin lógico de dirección 1 Trasero Izquierdo
#define RL_IN2 14 // Pin lógico de dirección 2 Trasero Izquierdo

#define FR_PWM 13 // Control de velocidad Motor Frontal Derecho
#define FR_IN1 23 // Pin lógico de dirección 1 Frontal Derecho
#define FR_IN2 2  // Pin lógico de dirección 2 Frontal Derecho

#define RR_PWM 19 // Control de velocidad Motor Trasero Derecho
#define RR_IN1 18 // Pin lógico de dirección 1 Trasero Derecho
#define RR_IN2 5  // Pin lógico de dirección 2 Trasero Derecho

const bool INVERTIR_FL = false; // Parámetro para invertir giro rueda frontal izquierda
const bool INVERTIR_FR = false; // Parámetro para invertir giro rueda frontal derecha
const bool INVERTIR_RL = false; // Parámetro para invertir giro rueda trasera izquierda
const bool INVERTIR_RR = false; // Parámetro para invertir giro rueda trasera derecha

#define PWM_FREQ 20000 // Frecuencia de los motores en Hercios (20kHz)
#define PWM_RES  8     // Definición a 8 bits (Valores disponibles de 0 a 255)

const int PWM_MAX = 182;     // Tope absoluto de PWM para no quemar motores Faulhaber
const int VEL_AVANCE = 120;  // Potencia constante de ataque a la pelota
const int VEL_GIRO = 85;     // Potencia para rotar en el propio eje
const int VEL_BUSCAR = 80;   // Potencia para movimientos exploratorios

int velAvanceActual = 0;     // Variable que almacena en qué velocidad va la aceleración
const int RAMPA_PASO = 4;    // Puntos de velocidad que suma por cada ciclo (aceleración suave)
unsigned long tSenalEstable = 0; // Cronómetro para descartar brillos falsos rápidos
const unsigned long T_CONFIRMA_MS = 500; // Requisito de medio segundo de visión antes de atacar

/* ===================== COMUNICACIÓN UART =========================== */
#define RX_PIN 34 // Pin GPIO de entrada conectado al cable que viene del anillo
#define TX_PIN 17 // Pin de salida serial (Desactivado físicamente)

HardwareSerial Enlace(2); // Inicia el puerto serial 2 por hardware nativo
char uartBuffer[64];      // Memoria de 64 caracteres para guardar la frase que llega
int bufIndex = 0;         // Indicador de la posición actual dentro de la memoria

float anguloIR = -1.0;    // Variable que recibe los grados de la pelota
int estadoIR = 0;         // Variable que recibe el estado (Pelota sí o Pelota no)
int nIR = 0;              // Variable que recibe el conteo de sensores iluminados
unsigned long ultimoDato = 0; // Temporizador para apagar el robot si se corta el cable

/* ===================== NAVEGACIÓN MPU ====================== */
const uint8_t MPU_ADDR = 0x68; // Dirección de red I2C del sensor de movimiento
float yaw = 0.0;               // Orientación actual del robot en la cancha
float gyroZoffset = 0.0;       // Basura electrónica que genera el MPU estando quieto (se calibra)
unsigned long tPrev = 0;       // Diferencial de tiempo para integrar ángulos matemáticos

/* ===================== MÁQUINA DE ESTADOS COMPLEJA ===================== */
enum EstadoRobot { ESPERANDO_PELOTA, BUSCANDO, PERSIGUIENDO, FRENANDO, REGRESANDO }; // Fases del partido
EstadoRobot estadoActual = ESPERANDO_PELOTA; // Arranca en fase de bloqueo inicial
unsigned long tFrenoIniciado = 0;            // Marca de inicio del frenado de protección
unsigned long tUltimaVezPelota = 0;          // Marca del instante donde se vio la bola por última vez

// --- NUEVAS VARIABLES PARA LA SECUENCIA DE BÚSQUEDA ---
int pasoBusqueda = 0;            // Guarda el paso actual (0 al 7) del patrón exploratorio
unsigned long tBusqueda = 0;     // Temporizador exclusivo para medir las pausas y avances del patrón
bool pelotaPerdidaReciente = false; // Bandera para saber si acabamos de perder la pelota

/* ====== RUTINAS DE COMPATIBILIDAD PWM ====== */
#if ESP_ARDUINO_VERSION_MAJOR >= 3 // Si el entorno Arduino está en versión moderna (v3+)
  void pwmInit(int pin){ ledcAttach(pin, PWM_FREQ, PWM_RES); } // Inicializa el pin
  void pwmWrite(int pin, int d){ ledcWrite(pin, d); } // Escribe la velocidad
#else // Si el entorno Arduino es versión antigua (v2.x)
  int canalDe(int pin){ // Asigna canales fijos para evitar colisiones
    if(pin==FL_PWM)return 0; if(pin==RL_PWM)return 1; if(pin==FR_PWM)return 2; if(pin==RR_PWM)return 3; return 0; 
  }
  void pwmInit(int pin){ int ch=canalDe(pin); ledcSetup(ch, PWM_FREQ, PWM_RES); ledcAttachPin(pin, ch); } // Vincula pines y canales
  void pwmWrite(int pin, int d){ ledcWrite(canalDe(pin), d); } // Escribe velocidad en canal
#endif

// Rutina que enciende una rueda específica
void rueda(int pwm, int in1, int in2, int v, bool inv){ // Recibe los 3 pines, la velocidad y la inversión
  if(inv) v = -v; // Si la inversión es verdadera, voltea el signo matemático
  v = constrain(v, -PWM_MAX, PWM_MAX); // Evita que un error matemático envíe más de 182 de PWM
  if(v > 0){ digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  pwmWrite(pwm, v); } // Configuración de giro hacia adelante
  else if(v < 0){ digitalWrite(in1, LOW); digitalWrite(in2, HIGH); pwmWrite(pwm, -v); } // Configuración de giro hacia atrás
  else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); pwmWrite(pwm, 0); } // Apaga el giro por completo
}

// Sub-rutinas rápidas para no escribir tanto en el loop
void FL(int v){ rueda(FL_PWM, FL_IN1, FL_IN2, v, INVERTIR_FL); } // Llama a la rueda Delantera Izquierda
void FR(int v){ rueda(FR_PWM, FR_IN1, FR_IN2, v, INVERTIR_FR); } // Llama a la rueda Delantera Derecha
void RL(int v){ rueda(RL_PWM, RL_IN1, RL_IN2, v, INVERTIR_RL); } // Llama a la rueda Trasera Izquierda
void RR(int v){ rueda(RR_PWM, RR_IN1, RR_IN2, v, INVERTIR_RR); } // Llama a la rueda Trasera Derecha

void ruedas(int fl, int fr, int rl, int rr){ FL(fl); FR(fr); RL(rl); RR(rr); } // Aplica valores a las 4 a la vez
void detener(){ ruedas(0, 0, 0, 0); velAvanceActual = 0; } // Frena las 4 ruedas y reinicia la aceleración
void girarEnSitio(int v){ ruedas(v, -v, v, -v); } // Aplica fuerza contraria entre lados para pivotear sobre su eje

// Sistema antideslizamiento y aceleración controlada
void avanzarSuave(int objetivo, int corr){ // Recibe la velocidad máxima deseada y una corrección angular
  if(velAvanceActual < objetivo) velAvanceActual += RAMPA_PASO; // Si va más lento, acelera sumando 4
  if(velAvanceActual > objetivo) velAvanceActual = objetivo; // Si se pasó, lo recorta al máximo permitido
  ruedas(velAvanceActual + corr, velAvanceActual - corr, // Aplica la aceleración y tuerce la llanta según el error (corr)
         velAvanceActual + corr, velAvanceActual - corr); // Aplica a las llantas traseras también
}

/* ===================== FUNCIONES DEL SENSOR MPU I2C =========================== */
void mpuW(uint8_t r, uint8_t v){ // Función para escribir en el MPU
  Wire.beginTransmission(MPU_ADDR); // Llama al dispositivo MPU por la red I2C
  Wire.write(r); // Señala la posición de memoria interna a cambiar
  Wire.write(v); // Escribe el nuevo valor
  Wire.endTransmission(); // Cierra el paquete de datos
}

int16_t mpuGz(){ // Función para extraer datos físicos del MPU
  Wire.beginTransmission(MPU_ADDR); // Llama al MPU
  Wire.write(0x47); // Pide los datos de la rotación en el Eje Z
  Wire.endTransmission(false); // Pausa la línea sin cerrarla
  Wire.requestFrom(MPU_ADDR, (uint8_t)2); // Exige que el MPU le devuelva 2 bytes de datos
  return (Wire.read() << 8) | Wire.read(); // Une los dos bytes y los devuelve a la matemática
}

bool mpuInit(){ // Función de arranque del giroscopio
  Wire.begin(21, 22); // Enciende los pines SDA y SCL de la ESP32
  Wire.setClock(400000); // Establece el protocolo a modo ultra rápido (Fast Mode)
  Wire.setTimeout(25); // Ordena a la ESP32 no bloquearse si los cables fallan o sufren ruido de motores
  Wire.beginTransmission(MPU_ADDR); // Pregunta si hay alguien en la dirección del MPU
  if(Wire.endTransmission() != 0) return false; // Si nadie responde, reporta falla
  mpuW(0x6B, 0x00); delay(100); // Apaga el modo dormido del MPU
  mpuW(0x1B, 0x00); delay(50);  // Configura la precisión máxima del giroscopio
  return true; // Reporta éxito
}

void calibrarGyro(){ // Rutina de autocalibración térmica
  long s = 0; // Acumulador muy grande
  for(int i=0; i<500; i++){ // Toma 500 fotografías estáticas
    s += mpuGz(); // Suma el valor erróneo del sensor
    delay(3); // Pequeña pausa entre fotos
  } 
  gyroZoffset = (float)s / 500.0; // Saca el promedio del error y lo guarda como compensación
}

void actualizarRumbo(){ // Rutina que se llama en cada milisegundo del loop
  unsigned long n = micros(); // Lee el reloj atómico interno en microsegundos
  float dt = (n - tPrev) / 1000000.0; // Calcula cuántas fracciones de segundo pasaron desde la última vez
  tPrev = n; // Actualiza la marca temporal
  float gz = (mpuGz() - gyroZoffset) / 131.0; // Extrae el grado/segundo puro aplicando la compensación calibrada
  yaw += gz * dt; // Integra el cálculo sumándolo al rumbo absoluto general
}

float errorAngular(float obj, float act){ // Rutina de la ruta más corta
  float e = obj - act; // Resta los ángulos a lo bruto
  while(e > 180) e -= 360; // Si le pide dar casi una vuelta entera a la derecha, mejor gira a la izquierda
  while(e < -180) e += 360; // Aplica lo mismo pero para la izquierda
  return e; // Devuelve los grados exactos que se deben mover
}

/* ===================== CEREBRO UART =========================== */
void leerUART(){ // Rutina que escucha a la placa del Anillo
  while(Enlace.available()){ // Entra a un ciclo cerrado siempre que haya letras esperando en el cable
    char c = Enlace.read(); // Jala la primera letra y la saca de la fila
    if(c == '\n'){ // Detecta si la letra es un salto de línea invisible (fin del mensaje)
      uartBuffer[bufIndex] = '\0'; // Agrega un carácter especial para cerrar el texto
      float a; int c_st, n; // Prepara variables vacías
      if(sscanf(uartBuffer, "A:%f C:%d N:%d", &a, &c_st, &n) >= 2){ // Destripa el texto y mete los números en las variables
        anguloIR = a; // Actualiza globalmente el ángulo
        estadoIR = c_st; // Actualiza globalmente el estado (0/1)
        nIR = n; // Actualiza globalmente el conteo de luz
        ultimoDato = millis(); // Marca la hora exacta de la recepción
      }
      bufIndex = 0; // Vuelve el índice a cero para pisar el mensaje viejo con el nuevo
    } else if(c != '\r' && bufIndex < 63){ // Si aún quedan letras y no se ha llenado la memoria
      uartBuffer[bufIndex++] = c; // Guarda la letra y avanza un cuadro de la memoria
    }
  }
}

/* ===================== SETUP DEL PARTIDO ================================== */
void setup(){ // Función que corre una sola vez al encender el switch
  Serial.begin(115200); // Enciende el reporte hacia la laptop
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN); // Enciende el oído del robot a 38400 baudios
  delay(300); // Pausa existencial para que la circuitería despierte
  Serial.println("\n=== ESP32 MOTORES v8 (Búsqueda Táctica) ==="); // Reporte

  pinMode(STBY, OUTPUT); // Declara el pin de seguridad de motores como salida de voltaje
  digitalWrite(STBY, LOW); // Baja el voltaje a cero para que los motores no puedan girar (Seguro activado)
  
  int pd[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2}; // Lista de pines de dirección
  for(int p : pd) pinMode(p, OUTPUT); // Bucle rápido que declara los 8 pines como salidas
  
  pwmInit(FL_PWM); pwmInit(FR_PWM); pwmInit(RL_PWM); pwmInit(RR_PWM); // Inicia los 4 canales de pulsos eléctricos
  detener(); // Garantiza por software que el pulso sea 0
  digitalWrite(STBY, HIGH); // Quita el seguro, el robot ya tiene fuerza en las llantas

  if(mpuInit()) Serial.println("MPU OK."); // Reporta si la brújula existe
  else Serial.println("AVISO: MPU no responde."); // Alarma si hay cables desconectados
  
  Serial.println(">> BLOQUEO: Calibrando norte estático. NO mover el carro.");
  calibrarGyro(); // Toma las 500 muestras estáticas de calibración
  yaw = 0.0; // Fija ese punto como el Norte Oficial (0 grados)
  tPrev = micros(); // Inicia el reloj de integración
  unsigned long t0 = millis(); // Inicia un cronómetro para los 4 segundos
  
  while(millis() - t0 < T_QUIETO_MS){ // Queda atrapado aquí durante 4000 milisegundos
    actualizarRumbo(); // Mantiene la matemática corriendo para limpiar ruidos iniciales
    detener(); // Asegura de nuevo estar frenado
    delay(5); // Pausa ligera
  }
  
  Serial.println(">> Norte fijado con éxito. Entrando en Modo de Espera de Pelota.");
  estadoActual = ESPERANDO_PELOTA; // Configura el estado inicial antes de entrar al loop infinito
}

/* ===================== LOOP DEL PARTIDO ================================== */
void loop(){ // Función de pensamiento que ocurre miles de veces por segundo
  actualizarRumbo(); // Actualiza brújula obligatoriamente
  leerUART();        // Extrae los datos obligatoriamente

  // Condición viva: Si dice '1' y el mensaje no tiene más de 400ms de antigüedad
  bool haySenal = (estadoIR == 1) && (millis() - ultimoDato < 400);

  if(haySenal) { // Si hay visión...
    tUltimaVezPelota = millis(); // Actualiza la hora de posesión
    pelotaPerdidaReciente = false; // Baja la bandera de pérdida
  }

#if MODO == 0
  /* --- MODO CALIBRACIÓN VISUAL (Puro Diagnóstico) --- */
  detener(); // Motor cortado permanentemente
  static unsigned long t = 0; // Cronómetro local
  if(millis() - t > 250){ // Si pasó un cuarto de segundo...
    t = millis(); // Resetea cronómetro local
    if(haySenal) Serial.printf(">> Ángulo detectado = %.1f\n", anguloIR); // Muestra datos en PC
    else         Serial.println(">> Sin señal IR");
  }
#else
  /* --- MODO COMPETENCIA DESTRUCTORA --- */
  
  // Extrae la diferencia de grados (Si no hay pelota, fuerza a cero para no hacer cálculos locos)
  float errApunte = haySenal ? errorAngular(FRENTE_ANILLO, anguloIR) : 0;
  
  // Condición severa de posesión: Hay luz Y está dentro del cono frontal Y satura suficientes receptores
  bool pelotaPegada = haySenal && (abs(errApunte) <= TOL_APUNTADO) && (nIR >= N_CAPTURA);

  /* --- CEREBRO SUPERIOR: CAMBIO DE ESTADOS Y BANDERAS --- */
  haySenal = 1;
  if (estadoActual == ESPERANDO_PELOTA) { // Si el robot está en la línea de salida bloqueado...
    if (haySenal) estadoActual = PERSIGUIENDO; // Desbloquéalo y ponlo a atacar al ver la luz de inicio
  }
  else if(estadoActual == PERSIGUIENDO && millis() - tiempoparacambiodeestado >= 7000) {
    
    
  }

  const char* nombre = "?"; // Crea un texto vacío para almacenar el nombre de la acción en la terminal

  /* --- CEREBRO FÍSICO: CONTROL DE ACTUADORES SEGÚN EL ESTADO --- */
  switch(estadoActual){
    
    case ESPERANDO_PELOTA: // Acciones de bloqueo
      detener(); // Se queda petrificado
      nombre = "BLOQUEADO (Esperando arranque)"; // Etiqueta diagnóstico
      break;

    case BUSCANDO: // NUEVO PATRÓN TÁCTICO SECTORIALtSenalEstable
      if (pasoBusqueda == 0) { // PASO 0
        detener(); // Frena el chasis
        nombre = "BUSQUEDA: Pausa 1s (Escuchando ecos)"; // Diagnóstico
        if (millis() - tBusqueda >= 1000) { // Si ya pasó 1 segundo (1000ms)...
          pasoBusqueda = 1; // Avanza a la siguiente instrucción
          tBusqueda = millis(); // Resetea cronómetro de la sub-rutina
        }
      }
      else if (pasoBusqueda == 1) { // PASO 1
        avanzarSuave(VEL_BUSCAR, 0); // Empuja hacia adelante
        nombre = "BUSQUEDA: Avance de Patrulla"; // Diagnóstico
        if (millis() - tBusqueda >= 600) { // Si ya pasaron 600ms avanzando...
          pasoBusqueda = 2; // Avanza de instrucción
          tBusqueda = millis(); // Resetea cronómetro
        }
      }
      else if (pasoBusqueda == 2) { // PASO 2
        detener(); // Detiene inercia
        nombre = "BUSQUEDA: Pausa Visual"; // Diagnóstico
        if (millis() - tBusqueda >= 300) { // Después de 300ms...
          pasoBusqueda = 3; // Avanza
          tBusqueda = millis(); // Resetea
        }
      }
      else if (pasoBusqueda == 3) { // PASO 3
        girarEnSitio(VEL_GIRO); // Fuerza rotación hacia la derecha
        nombre = "BUSQUEDA: Escaneo Derecha"; // Diagnóstico
        if (millis() - tBusqueda >= 400) { // Después de girar 400ms...
          pasoBusqueda = 4; // Avanza
          tBusqueda = millis(); // Resetea
        }
      }
      else if (pasoBusqueda == 4) { // PASO 4
        avanzarSuave(VEL_BUSCAR, 0); // Avanza hacia esa nueva dirección
        nombre = "BUSQUEDA: Avance Diagonal Der"; // Diagnóstico
        if (millis() - tBusqueda >= 600) { // Después de avanzar 600ms...
          pasoBusqueda = 5; // Avanza
          tBusqueda = millis(); // Resetea
        }
      }
      else if (pasoBusqueda == 5) { // PASO 5
        detener(); // Detiene inercia
        nombre = "BUSQUEDA: Pausa Visual"; // Diagnóstico
        if (millis() - tBusqueda >= 300) { // Después de 300ms...
          pasoBusqueda = 6; // Avanza
          tBusqueda = millis(); // Resetea
        }
      }
      else if (pasoBusqueda == 6) { // PASO 6
        girarEnSitio(-VEL_GIRO); // Rota hacia la Izquierda
        nombre = "BUSQUEDA: Escaneo Izquierda"; // Diagnóstico
        if (millis() - tBusqueda >= 800) { // El tiempo es doble (800) para cruzar todo el frente y mirar al otro lado
          pasoBusqueda = 7; // Avanza
          tBusqueda = millis(); // Resetea
        }
      }
      else if (pasoBusqueda == 7) { // PASO 7 FINAL
        avanzarSuave(VEL_BUSCAR, 0); // Avanza hacia la izquierda
        nombre = "BUSQUEDA: Avance Diagonal Izq"; // Diagnóstico
        if (millis() - tBusqueda >= 600) { // Después de avanzar 600ms...
          pasoBusqueda = 0; // DECRETA UN LOOP DE BÚSQUEDA. Vuelve al paso 0 y repite todo.
          tBusqueda = millis(); // Resetea cronómetro.
        } 
      }
      break; // Rompe el bloque de evaluación

    case PERSIGUIENDO: // Acciones de ataque
      if(abs(errApunte) > TOL_APUNTADO){ // Si está muy chueco respecto a la pelota
        int sentido = (errApunte > 0) ? VEL_GIRO : -VEL_GIRO;  // Elige un giro negativo o positivo
        girarEnSitio(sentido); // Ejecuta el giro físico
        tSenalEstable = 0; // Rompe el tiempo de confirmación para que no ataque a lo tonto
        nombre = "PERSIGUE -> ENCUADRANDO"; // Diagnóstico
      } else { // Si ya la tiene frente a los ojos...
        if(tSenalEstable == 0) tSenalEstable = millis(); // Inicia el reloj de validación de sombras
        if(millis() - tSenalEstable < T_CONFIRMA_MS){ // Mientras no se cumpla el medio segundo de seguridad...
          detener(); // ...no hace nada físicamente
          nombre = "FILTRO VISUAL (Confirmando luz)"; // Diagnóstico
        } else { // Si la señal ya superó el medio segundo ininterrumpido...
          int corr = constrain((int)(errApunte * 2.0), -30, 30); // Crea un ajuste cinemático de máximo 30 puntos de PWM
          avanzarSuave(VEL_AVANCE, corr);  // Ordena a los motores sumarle ese ajuste a un lado y restarlo al otro
          nombre = "ATACANDO PELOTA A FONDO"; // Diagnóstico
        }
      }
      break; // Fin del bloque

    case FRENANDO: // Acción de protección electrónica
      detener(); // Corta el pulso de las ruedas. El chasis derrapa físicamente un poco.
      nombre = "FRENADO ELECTROMAGNÉTICO"; // Diagnóstico
      if (millis() - tFrenoIniciado > 250) { // Al pasar un cuarto de segundo (250ms)...
        estadoActual = REGRESANDO; // Es seguro encender de nuevo los transistores y rotar a portería
      }
      break;

    case REGRESANDO: { // Fase de anotación
      float errYaw = errorAngular(YAW_PORTERIA, yaw); // Extrae la diferencia contra la pared Norte
      if(abs(errYaw) > TOL_PORTERIA){ // Si la pared Norte no está de frente todavía...
        int sentidoYaw = (errYaw > 0) ? VEL_GIRO : -VEL_GIRO; // Asigna giro derecho o izquierdo
        girarEnSitio(sentidoYaw); // Empieza a girar con la pelota atorada en el bumper
        nombre = "PORTERIA -> GIRANDO BRÚJULA"; // Diagnóstico
      } else { // Si la pared ya está de frente a nosotros...
        int corr = constrain((int)(errYaw * 2.0), -30, 30); // Calcula desviación y crea ajuste
        avanzarSuave(VEL_AVANCE, corr); // Acelera en línea recta usando el ajuste para no curvarse
        nombre = "PORTERIA -> REMATE DE FRENTE"; // Diagnóstico
      }
      break; // Fin del bloque
    }
  }

  // --- REPORTE CONSTANTE A LA COMPUTADORA ---
  static unsigned long t = 0; // Reloj exclusivo de la pantalla serial
  if(millis() - t > 250){ // Imprime la información solo 4 veces por segundo para no trabar el IDE
    t = millis(); // Resetea reloj de pantalla
    Serial.printf("%s | AnguloIR=%.1f Senal=%d N_Sens=%d ErrAp=%.1f GyroYaw=%.1f\n", // Imprime toda la línea
                  nombre, anguloIR, haySenal, nIR, errApunte, yaw); 
  }
#endif // Cierra las directivas de los modos de competencia y calibración

  delay(25); // Pequeña tregua al procesador Dual Core para ejecutar rutinas ocultas de la placa
}