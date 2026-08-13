/* ==================================================================
   ESP32 ANILLO (TX) v8 - Analizador de Centroide 40kHz
   ================================================================== */
#include <Arduino.h> // Incluye las funciones básicas del núcleo ESP32
#include <math.h>    // Incluye herramientas matemáticas como seno, coseno y atan2

// --- PINES DEL MULTIPLEXOR ---
const int pinS0 = 19; // Pin digital conectado al selector S0 del CD74HC4067
const int pinS1 = 18; // Pin digital conectado al selector S1 del CD74HC4067
const int pinS2 = 17; // Pin digital conectado al selector S2 del CD74HC4067
const int pinS3 = 16; // Pin digital conectado al selector S3 del CD74HC4067
const int pinSIG = 4; // Pin digital que recibe la lectura del sensor multiplexado

const int totalSensores = 16; // Declaramos la cantidad total de fotoreceptores del anillo
const float GRADOS_POR_SENSOR = 22.5; // Calculamos el ángulo que abarca cada sensor (360/16)

// --- COMUNICACIÓN UART ---
#define RX_PIN 26 // Definimos el pin de recepción UART1 (No usado, pero requerido)
#define TX_PIN 25 // Definimos el pin de transmisión UART1 que envía datos a los motores

HardwareSerial Enlace(1); // Inicializamos el puerto Serial por Hardware número 1

// Función para cambiar de canal en el multiplexor mediante números binarios
void seleccionarCanal(int canal) {
  digitalWrite(pinS0, bitRead(canal, 0)); // Extrae el primer bit del número y lo aplica a S0
  digitalWrite(pinS1, bitRead(canal, 1)); // Extrae el segundo bit del número y lo aplica a S1
  digitalWrite(pinS2, bitRead(canal, 2)); // Extrae el tercer bit del número y lo aplica a S2
  digitalWrite(pinS3, bitRead(canal, 3)); // Extrae el cuarto bit del número y lo aplica a S3
}

void setup() {
  Serial.begin(115200); // Abre el puerto serie USB para ver datos en la computadora
  
  Enlace.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN); // Abre el puerto serial hacia la placa de motores a 38400 baudios
  
  pinMode(pinS0, OUTPUT); // Configura el pin S0 como salida de voltaje
  pinMode(pinS1, OUTPUT); // Configura el pin S1 como salida de voltaje
  pinMode(pinS2, OUTPUT); // Configura el pin S2 como salida de voltaje
  pinMode(pinS3, OUTPUT); // Configura el pin S3 como salida de voltaje
  
  pinMode(pinSIG, INPUT_PULLUP); // Configura el pin de lectura con una resistencia interna hacia 3.3V
  
  Serial.println("\n=== ESP32 ANILLO (TX) v8 Listo ==="); // Imprime un mensaje de éxito en pantalla
}

void loop() {
  bool activo[16]; // Crea una lista de 16 espacios booleanos (verdadero/falso)
  int totalActivos = 0; // Crea un contador en cero para saber cuántos sensores ven luz

  // 1. CICLO DE LECTURA DE SENSORES
  for (int i = 0; i < totalSensores; i++) { // Repite este bloque 16 veces (del 0 al 15)
    seleccionarCanal(i); // Le dice al multiplexor que escuche al sensor número 'i'
    delayMicroseconds(100); // Hace una micro-pausa para estabilizar la corriente
    activo[i] = (digitalRead(pinSIG) == LOW); // Guarda 'verdadero' si el pin detecta luz (LOW)
    if (activo[i]) totalActivos++; // Si guardó 'verdadero', suma 1 al contador
  }

  // 2. CICLO FILTRO DE REFLEJOS
  int mejorInicio = -1; // Guarda dónde empieza el mejor grupo (por ahora no hay ninguno)
  int mejorLargo = 0; // Guarda el tamaño del mejor grupo (por ahora cero)
  int iniActual = -1; // Variables temporales para el grupo que se está evaluando
  int largoActual = 0; // Tamaño temporal del grupo actual
  
  for (int k = 0; k < totalSensores * 2; k++) { // Recorre el arreglo el doble de veces para enlazar el final con el principio
    int i = k % totalSensores; // Asegura que el número vuelva a 0 al llegar a 16
    if (activo[i]) { // Si este sensor está viendo luz...
      if (largoActual == 0) iniActual = i; // ...y es el primero de una cadena, marca su posición
      largoActual++; // Aumenta el largo de esta cadena
      if (largoActual > mejorLargo) { // Si esta cadena superó a la anterior...
        mejorLargo = largoActual; // ...se convierte en el nuevo récord de tamaño
        mejorInicio = iniActual; // ...y se guarda su inicio
      }
    } else { // Si este sensor NO está viendo luz...
      largoActual = 0; // Rompe la cadena y reinicia el conteo temporal
    }
  }
  if (mejorLargo > totalSensores) mejorLargo = totalSensores; // Evita que la matemática se rompa si cuenta más de 16

  // 3. MATEMÁTICA DE CENTROIDE
  float angulo = -1.0; // Ángulo nulo por defecto
  if (mejorLargo > 0) { // Solo calcula si al menos un sensor detectó luz
    float sx = 0; // Componente horizontal del vector en cero
    float sy = 0; // Componente vertical del vector en cero
    for (int j = 0; j < mejorLargo; j++) { // Itera solamente sobre el grupo de sensores iluminados
      int idx = (mejorInicio + j) % totalSensores; // Obtiene el índice real del sensor
      float a = radians(idx * GRADOS_POR_SENSOR); // Pasa la posición del sensor de grados a radianes
      sx += cos(a); // Calcula el coseno y lo suma al eje X
      sy += sin(a); // Calcula el seno y lo suma al eje Y
    }
    angulo = degrees(atan2(sy, sx)); // Arcotangente de Y y X, lo vuelve a convertir a grados
    if (angulo < 0) angulo += 360.0; // Si el ángulo quedó negativo, lo ajusta a una brújula de 360
  }

  // 4. TRANSMISIÓN DE DATOS A LA OTRA PLACA
  int estado = (mejorLargo > 0) ? 1 : 0; // Define estado 1 si la pelota está en cancha, o 0 si desapareció
  
  Enlace.print("A:"); // Envía la letra 'A:' por el cable amarillo (TX)
  Enlace.print(angulo, 1); // Envía el valor del ángulo con 1 decimal
  Enlace.print(" C:"); // Envía el separador ' C:'
  Enlace.print(estado); // Envía el estado (1 o 0)
  Enlace.print(" N:"); // Envía el separador ' N:'
  Enlace.println(totalActivos); // Envía la cantidad de sensores activos y da un salto de línea (\n)

  Serial.printf("TX -> A:%.1f C:%d N:%d\n", angulo, estado, totalActivos); // Imprime un eco en la PC para diagnóstico
  
  delay(50); // Descansa 50 milisegundos antes de repetir el ciclo completo
}