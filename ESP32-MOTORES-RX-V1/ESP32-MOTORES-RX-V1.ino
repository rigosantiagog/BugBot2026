// ... (todo el código anterior permanece igual hasta el case REGRESANDO)

    case REGRESANDO:                                // Avance hacia la portería rival
      {
        static unsigned long tInicioGiro = 0;       // Tiempo en que se inició el giro actual
        static float errYawAnterior = 999.0f;       // Para detectar si el error disminuye

        float errYaw = errorAngular(yaw);           // Calcula el error de orientación respecto al norte (portería)
        if (abs(errYaw) > TOL_PORTERIA) {           // Si no está alineado, gira
          // Control proporcional: velocidad = K * |error|, limitada entre GIRO_MIN_PWM y VEL_GIRO
          int pwmGiro = constrain((int)(abs(errYaw) * K_GIRO), GIRO_MIN_PWM, VEL_GIRO);
          int sentidoYaw = (errYaw < 0) ? pwmGiro : -pwmGiro; // Positivo = derecha, negativo = izquierda
          girarEnSitio(sentidoYaw);
          nombre = "PORTERIA -> GIRANDO BRÚJULA";

          // Control de timeout: si el error no disminuye en T_GIRO_TIMEOUT, abortar
          if (abs(errYaw) < abs(errYawAnterior) - 2.0f) { // El error está mejorando
            tInicioGiro = millis();                // Reinicia el cronómetro
          }
          errYawAnterior = errYaw;

          if (millis() - tInicioGiro > T_GIRO_TIMEOUT) {
            // Lleva mucho tiempo girando sin progreso → pasar a búsqueda
            estadoActual = BUSCANDO;
            pelotaPerdidaReciente = true;
            pasoBusqueda = 0;
            tBusqueda = millis();
            nombre = "TIMEOUT GIRO -> BUSCANDO";
            break;  // Sale del case para evitar ejecutar más código
          }
        } else {
          // Está alineado: comprueba distancia frontal para remate
          if (distFrente < DIST_FRENADO_CM) {       // Si la pared (portería) está a menos de DIST_FRENADO_CM
            frenar();                               // Frenado brusco para impulsar la pelota
            nombre = "PORTERIA -> REMATE (FRENADO SECO)";
          } else {
            int corr = constrain((int)(errYaw * -2.0), -30, 30); // Corrección para mantener línea recta
            avanzarSuave(VEL_AVANCE, corr);         // Avanza suavemente hacia la portería
            nombre = "PORTERIA -> AVANZANDO";
          }
          // Reinicia variables de control de giro al estar alineado
          tInicioGiro = millis();
          errYawAnterior = 999.0f;
        }
        break;
      }

// ... (el resto del código permanece igual)
