/*
 * PulsoAlert v1.5.3 - ESP32-C3 OLED 72x40 + MAX30102
 *
 * GPIO5 SDA / GPIO6 SCL: OLED y MAX30102 (I2C compartido)
 * GPIO4: driver motor (activo HIGH; NO conectar motor directo)
 * GPIO8: LED integrado (activo LOW)
 * GPIO9: boton BOOT (activo LOW; evitar pulsarlo al encender)
 * GPIO0: lectura ADC de bateria mediante divisor 100k/100k
 *
 * DESCRIPCION DEL FIRMWARE:
 *   - Detecta latidos y calcula BPM promedio de cuatro muestras.
 *   - Confirma frecuencia baja antes de iniciar la alerta.
 *   - Activa el motor con tres pulsos por ciclo y 1-5 ciclos configurables.
 *   - Hace parpadear el LED independientemente del motor durante 1-10 min.
 *   - Confirma recuperacion durante 2 s y cancela la alerta.
 *   - Muestra BPM, estado y bateria en OLED 72x40.
 *   - Permite alternar pantallas con boton BOOT.
 *   - Lee voltaje de bateria por ADC mediante divisor resistivo.
 *
 * BASE VERIFICADA v1.4.2:
 *   - Corrige LED permanentemente encendido: ahora parpadea.
 *   - Conserva la logica de motor, sensor, bateria, boton y OLED.
 *   - Agrega documentacion de variables, estados y funciones.
 *
 * v1.5.0: BLE GATT, lecturas y notificaciones (validado con la web).
 * v1.5.1: escritura BLE de configuraciones, persistencia NVS y voltaje
 *         real de LiPo en bytes 6-7 de MEASUREMENT (GATT v1.1).
 * v1.5.3: version validada en placa y web; identificadores actualizados.
 *         Sin cambios en la logica de medicion, alertas o BLE.
 *
 * CONTRATO BLE: UUID conservados de GATT v1.0. MEASUREMENT pasa a 8 bytes:
 *  0-1 BPM (uint16 LE; 0=no valido); 2 contacto; 3 bateria % (255=desconocida);
 *  4 alerta; 5 estado; 6-7 voltaje bateria en mV (uint16 LE; 0=no disponible).
 *  Web v0.5.0 debe actualizarse para leer los dos bytes adicionales.
 *
 * CONFIG_STATUS (4 bytes): codigo 0=inicial, 1=guardado, 2=invalido,
 *  3=error NVS; parametro 0=ninguno, 1=umbral, 2=ciclos, 3=LED;
 *  bytes 2-3: valor vigente del parametro (uint16 LE).
 *
 * NOTA: La bateria % sigue desconocida hasta calibrar una estimacion.
 * Persistencia de datos confirmada
 * NO ES UN DISPOSITIVO MEDICO. No hay autenticacion BLE en esta version.
 * Prototipo educativo, no dispositivo medico.
 * No interpretar perdida de senal como frecuencia cardiaca baja.
 * La bateria LiPo requiere alimentacion regulada/protegida apropiada.
 */
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <MAX30105.h>
#include "heartRate.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>   // Memoria NVS no volatil del ESP32-C3.
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>  // Desacopla callbacks BLE del bucle de medicion.

// ============================================================
// 1. HARDWARE Y CONFIGURACION
// ============================================================
// Bus I2C compartido por el MAX30102 y la pantalla OLED.
constexpr uint8_t PIN_SDA = 5;
constexpr uint8_t PIN_SCL = 6;
// Salida hacia el driver del motor (no alimentar el motor desde GPIO).
constexpr uint8_t PIN_MOTOR = 4;
// LED integrado activo con nivel bajo (LOW).
constexpr uint8_t PIN_LED = 8;
// Boton integrado BOOT: activo en LOW; pin de arranque de la placa.
constexpr uint8_t PIN_BOOT = 9;
// Entrada ADC para medir la bateria a traves de divisor 100k/100k.
constexpr uint8_t PIN_BATERIA = 0;
constexpr uint8_t MOTOR_ON = HIGH;
constexpr uint8_t MOTOR_OFF = LOW;
constexpr uint8_t LED_ON = LOW;
constexpr uint8_t LED_OFF = HIGH;

MAX30105 sensor;
U8G2_SSD1306_72X40_ER_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ============================================================
// 2. MEDICION DE FRECUENCIA CARDIACA
// ============================================================
constexpr uint32_t IR_MINIMO = 50000;
constexpr unsigned long SIN_LATIDO_MS = 4000;
constexpr uint8_t NUM_MUESTRAS_BPM = 4;
uint8_t muestrasBPM[NUM_MUESTRAS_BPM] = {};
uint8_t indiceBPM = 0;
uint8_t muestrasValidas = 0;
float bpmInstantaneo = 0.0f;
float bpmPromedio = 0.0f;
unsigned long ultimoLatido = 0;
bool dedoPresente = false;
bool bpmValido = false;

// ============================================================
// 3. ALERTAS: UMBRAL, RECUPERACION Y ACTUADORES
// ============================================================
// Parametros configurables por BLE; iniciales o restaurados desde NVS.
// Frecuencia promedio por debajo de la cual se confirma una alerta.
// Valores efectivos: se recuperan de NVS al iniciar y pueden cambiar por BLE.
// Se usan enteros porque el contrato GATT define valores enteros.
uint16_t umbralBPM = 50;
constexpr unsigned long CONFIRMAR_BAJA_MS = 5000;
// Tiempo de BPM recuperado antes de cancelar la alerta activa.
constexpr unsigned long CONFIRMAR_RECUPERACION_MS = 2000;
// Duracion maxima de la indicacion luminosa desde el inicio de la alerta.
// Duracion configurable expresada en minutos; el temporizador usa ms.
uint16_t duracionLEDMinutos = 2;
// Patron temporal compartido: 300 ms ON, 200 ms OFF, 3 pulsos,
// seguidos por una pausa de 1000 ms entre ciclos.
constexpr unsigned long MOTOR_PULSO_MS = 300;
constexpr unsigned long MOTOR_ENTREPULSOS_MS = 200;
constexpr unsigned long MOTOR_ENTRECICLOS_MS = 1000;
constexpr uint8_t PULSOS_POR_CICLO = 3;
// Numero maximo de ciclos por episodio, configurable de 1 a 5.
uint8_t maxCiclos = 3;

// Estados del episodio: normal, confirmando baja, activo y
// confirmando recuperacion. La alerta no se rearma mientras siga activa.
enum class EstadoAlerta : uint8_t { NORMAL, CONFIRMANDO_BAJA, ACTIVA, CONFIRMANDO_RECUPERACION };
// Fases independientes de la maquina de estados del motor.
enum class EstadoMotor : uint8_t { INACTIVO, PULSO, PAUSA_PULSO, PAUSA_CICLO, TERMINADO };
EstadoAlerta estadoAlerta = EstadoAlerta::NORMAL;
EstadoMotor estadoMotor = EstadoMotor::INACTIVO;
// Marcas de tiempo para confirmar baja, recuperacion y controlar salidas.
unsigned long inicioBaja = 0;
unsigned long inicioRecuperacion = 0;
unsigned long inicioLED = 0;
unsigned long inicioFaseMotor = 0;
uint8_t pulsosTerminados = 0;
uint8_t ciclosTerminados = 0;
// Indica si el LED sigue dentro de su ventana de aviso (no su nivel fisico).
bool ledActivo = false;

// ============================================================
// 4. BATERIA: DIVISOR RESISTIVO Y UMBRALES
// ============================================================
// Resistencias del divisor: la entrada ADC recibe aproximadamente
// la mitad del voltaje de la bateria.
constexpr float R1_BATERIA = 100000.0f;
constexpr float R2_BATERIA = 100000.0f;
constexpr float FACTOR_DIVISOR = (R1_BATERIA + R2_BATERIA) / R2_BATERIA;
constexpr float CALIBRACION_BATERIA = 1.0f; // Ajustar comparando con multimetro
// Umbrales orientativos de voltaje; requieren calibracion en hardware.
constexpr float BATERIA_BAJA_V = 3.50f;
constexpr float BATERIA_CRITICA_V = 3.30f;
constexpr unsigned long LECTURA_BATERIA_MS = 2000;
constexpr unsigned long CONFIRMAR_BATERIA_MS = 10000;
constexpr uint8_t MUESTRAS_ADC = 16;

enum class EstadoBateria : uint8_t { NORMAL, BAJA, CRITICA };
// Prototipo explicito: evita fallo de autogeneracion de Arduino con enums.
EstadoBateria clasificarBateria(float voltaje);
EstadoBateria estadoBateria = EstadoBateria::NORMAL;
EstadoBateria estadoBateriaPendiente = EstadoBateria::NORMAL;
float voltajeBateria = 0.0f;
unsigned long ultimaBateria = 0;
unsigned long inicioEstadoBateria = 0;
bool bateriaInicializada = false;

// ============================================================
// 5. PANTALLAS Y BOTON BOOT
// ============================================================
// PRINCIPAL muestra frecuencia y alertas; CONFIGURACION muestra
// voltaje de bateria y umbral configurado.
enum class PantallaOLED : uint8_t { PRINCIPAL, CONFIGURACION };
PantallaOLED pantallaActual = PantallaOLED::PRINCIPAL;
constexpr unsigned long REBOTE_BOOT_MS = 50;
constexpr unsigned long OLED_INTERVALO_MS = 200;
unsigned long ultimoCambioBoot = 0;
unsigned long ultimaOLED = 0;
bool bootLecturaAnterior = HIGH;
bool bootEstadoEstable = HIGH;

// ============================================================
// 6. FUNCIONES DEL MOTOR Y LED
// ============================================================
// Fuerza la salida del motor al estado apagado.
void apagarMotor() { digitalWrite(PIN_MOTOR, MOTOR_OFF); }
// Apaga el LED y deshabilita futuros pulsos en este episodio.
void apagarLED() { digitalWrite(PIN_LED, LED_OFF); ledActivo = false; }
// Cancela y reinicia por completo la secuencia del motor.
void detenerMotor() {
  apagarMotor();
  estadoMotor = EstadoMotor::INACTIVO;
  pulsosTerminados = 0;
  ciclosTerminados = 0;
}
// Inicia el primer pulso del motor sin bloquear el bucle principal.
void iniciarMotor(unsigned long ahora) {
  pulsosTerminados = 0;
  ciclosTerminados = 0;
  inicioFaseMotor = ahora;
  estadoMotor = EstadoMotor::PULSO;
  digitalWrite(PIN_MOTOR, MOTOR_ON);
}
/*
 * Inicia la ventana de aviso del LED. El primer pulso comienza encendido.
 * El LED NO depende de la maquina de estados del motor: puede continuar
 * parpadeando una vez terminados los tres ciclos de vibracion.
 */
void iniciarLED(unsigned long ahora) {
  inicioLED = ahora;
  ledActivo = ((static_cast<unsigned long>(duracionLEDMinutos) * 60000UL) > 0);
  digitalWrite(PIN_LED, ledActivo ? LED_ON : LED_OFF);
}

/*
 * Calcula el patron de parpadeo del LED mediante el tiempo transcurrido.
 * Reutiliza las duraciones del motor, pero no su estado ni sus contadores.
 *
 * Un ciclo consiste en tres pulsos ON de 300 ms, con pausas OFF de
 * 200 ms entre pulsos y una pausa OFF de 1000 ms entre ciclos.
 * El LED se apaga al alcanzar la duracion configurada y no vuelve a encenderse hasta
 * que se inicie un nuevo episodio de alerta.
 */
void actualizarLED(unsigned long ahora) {
  if (!ledActivo) return;

  const unsigned long transcurrido = ahora - inicioLED;
  if (transcurrido >= (static_cast<unsigned long>(duracionLEDMinutos) * 60000UL)) {
    apagarLED();
    return;
  }

  const unsigned long periodoPulso = MOTOR_PULSO_MS + MOTOR_ENTREPULSOS_MS;
  const unsigned long tiempoPulsos =
      PULSOS_POR_CICLO * MOTOR_PULSO_MS +
      (PULSOS_POR_CICLO - 1) * MOTOR_ENTREPULSOS_MS;
  const unsigned long duracionCiclo = tiempoPulsos + MOTOR_ENTRECICLOS_MS;

  // Evitar divisiones por cero si se modifican parametros en el futuro.
  if (PULSOS_POR_CICLO == 0 || MOTOR_PULSO_MS == 0 ||
      periodoPulso == 0 || duracionCiclo == 0) {
    apagarLED();
    return;
  }

  const unsigned long posicionCiclo = transcurrido % duracionCiclo;
  const bool enZonaPulsos = posicionCiclo < tiempoPulsos;
  const bool pulsoEncendido = enZonaPulsos &&
      ((posicionCiclo % periodoPulso) < MOTOR_PULSO_MS);

  // El LED integrado utiliza logica invertida: LOW significa encendido.
  digitalWrite(PIN_LED, pulsoEncendido ? LED_ON : LED_OFF);
}

/*
 * Avanza el motor por sus fases sin delay(): pulso, pausa corta,
 * pausa entre ciclos y finalizacion definitiva del episodio.
 */
void actualizarMotor(unsigned long ahora) {
  switch (estadoMotor) {
    case EstadoMotor::PULSO:
      if (ahora - inicioFaseMotor >= MOTOR_PULSO_MS) {
        apagarMotor();
        ++pulsosTerminados;
        inicioFaseMotor = ahora;
        estadoMotor = EstadoMotor::PAUSA_PULSO;
      }
      break;
    case EstadoMotor::PAUSA_PULSO:
      if (ahora - inicioFaseMotor >= MOTOR_ENTREPULSOS_MS) {
        inicioFaseMotor = ahora;
        if (pulsosTerminados < PULSOS_POR_CICLO) {
          digitalWrite(PIN_MOTOR, MOTOR_ON);
          estadoMotor = EstadoMotor::PULSO;
        } else {
          ++ciclosTerminados;
          estadoMotor = (ciclosTerminados >= maxCiclos)
            ? EstadoMotor::TERMINADO : EstadoMotor::PAUSA_CICLO;
        }
      }
      break;
    case EstadoMotor::PAUSA_CICLO:
      if (ahora - inicioFaseMotor >= MOTOR_ENTRECICLOS_MS) {
        pulsosTerminados = 0;
        inicioFaseMotor = ahora;
        estadoMotor = EstadoMotor::PULSO;
        digitalWrite(PIN_MOTOR, MOTOR_ON);
      }
      break;
    default: break;
  }
}

// ============================================================
// 7. SENSOR MAX30102
// ============================================================
// Descarta BPM antiguos al retirar el dedo o perder latidos recientes.
void reiniciarMediciones() {
  bpmInstantaneo = 0;
  bpmPromedio = 0;
  bpmValido = false;
  indiceBPM = 0;
  muestrasValidas = 0;
  ultimoLatido = 0;
  for (uint8_t i = 0; i < NUM_MUESTRAS_BPM; ++i) muestrasBPM[i] = 0;
}
/*
 * Lee el infrarrojo, determina contacto y detecta latidos.
 * Conserva el algoritmo de la v1.4.1 sin cambios funcionales.
 * Una perdida de senal NO equivale a frecuencia cardiaca baja.
 */
void actualizarSensor(unsigned long ahora) {
  const uint32_t ir = sensor.getIR();
  if (ir < IR_MINIMO) {
    if (dedoPresente) { dedoPresente = false; reiniciarMediciones(); }
    return;
  }
  if (!dedoPresente) { dedoPresente = true; reiniciarMediciones(); }
  if (checkForBeat(ir)) {
    if (ultimoLatido != 0) {
      const unsigned long intervalo = ahora - ultimoLatido;
      if (intervalo > 0) {
        const float bpm = 60000.0f / intervalo;
        if (bpm >= 20.0f && bpm <= 220.0f) {
          bpmInstantaneo = bpm;
          muestrasBPM[indiceBPM] = (uint8_t)(bpm + 0.5f);
          indiceBPM = (indiceBPM + 1) % NUM_MUESTRAS_BPM;
          if (muestrasValidas < NUM_MUESTRAS_BPM) ++muestrasValidas;
          uint16_t suma = 0;
          for (uint8_t i = 0; i < muestrasValidas; ++i) suma += muestrasBPM[i];
          bpmPromedio = suma / (float)muestrasValidas;
          bpmValido = true;
        }
      }
    }
    ultimoLatido = ahora;
  }
  if (ultimoLatido != 0 && ahora - ultimoLatido > SIN_LATIDO_MS) {
    reiniciarMediciones();
  }
}

// ============================================================
// 8. MAQUINA DE ESTADOS DE ALERTA
// ============================================================
// Apaga ambas salidas y devuelve el sistema al estado normal.
void cancelarAlerta() {
  detenerMotor();
  apagarLED();
  estadoAlerta = EstadoAlerta::NORMAL;
  inicioBaja = 0;
  inicioRecuperacion = 0;
}
// Inicia motor y LED juntos y da prioridad a la pantalla principal.
void activarAlerta(unsigned long ahora) {
  estadoAlerta = EstadoAlerta::ACTIVA;
  iniciarMotor(ahora);
  iniciarLED(ahora);
  pantallaActual = PantallaOLED::PRINCIPAL; // Prioridad visual de alerta
}
/*
 * Maquina de estados de alerta:
 *  NORMAL -> confirma BPM bajo por 5 s -> ACTIVA.
 *  ACTIVA -> confirma recuperacion por 2 s -> NORMAL.
 *  Si la senal deja de ser valida, cancela inmediatamente.
 *  El LED puede agotarse antes de que termine el episodio;
 *  no se rearma por ello.
 */
void actualizarAlerta(unsigned long ahora) {
  if (!dedoPresente || !bpmValido) { cancelarAlerta(); return; }
  const bool bpmBajas = bpmPromedio < umbralBPM;
  switch (estadoAlerta) {
    case EstadoAlerta::NORMAL:
      if (bpmBajas) { inicioBaja = ahora; estadoAlerta = EstadoAlerta::CONFIRMANDO_BAJA; }
      break;
    case EstadoAlerta::CONFIRMANDO_BAJA:
      if (!bpmBajas) estadoAlerta = EstadoAlerta::NORMAL;
      else if (ahora - inicioBaja >= CONFIRMAR_BAJA_MS) activarAlerta(ahora);
      break;
    case EstadoAlerta::ACTIVA:
      if (!bpmBajas) {
        inicioRecuperacion = ahora;
        estadoAlerta = EstadoAlerta::CONFIRMANDO_RECUPERACION;
      }
      break;
    case EstadoAlerta::CONFIRMANDO_RECUPERACION:
      if (bpmBajas) estadoAlerta = EstadoAlerta::ACTIVA;
      else if (ahora - inicioRecuperacion >= CONFIRMAR_RECUPERACION_MS) cancelarAlerta();
      break;
  }
}

// ============================================================
// 9. LECTURA Y CLASIFICACION DE BATERIA
// ============================================================
// Promedia lecturas ADC en milivoltios y reconstruye voltaje real.
float leerVoltajeBateria() {
  uint32_t suma = 0;
  for (uint8_t i = 0; i < MUESTRAS_ADC; ++i) suma += analogReadMilliVolts(PIN_BATERIA);
  const float voltajeADC = (suma / (float)MUESTRAS_ADC) / 1000.0f;
  return voltajeADC * FACTOR_DIVISOR * CALIBRACION_BATERIA;
}
// Clasifica voltaje medido segun los umbrales establecidos.
EstadoBateria clasificarBateria(float voltaje) {
  if (voltaje < BATERIA_CRITICA_V) return EstadoBateria::CRITICA;
  if (voltaje < BATERIA_BAJA_V) return EstadoBateria::BAJA;
  return EstadoBateria::NORMAL;
}
// Actualiza cada 2 s y exige 10 s de estabilidad para cambiar estado.
void actualizarBateria(unsigned long ahora) {
  if (bateriaInicializada && ahora - ultimaBateria < LECTURA_BATERIA_MS) return;
  ultimaBateria = ahora;
  voltajeBateria = leerVoltajeBateria();
  const EstadoBateria nuevo = clasificarBateria(voltajeBateria);
  if (!bateriaInicializada) {
    bateriaInicializada = true;
    estadoBateria = nuevo;
    estadoBateriaPendiente = nuevo;
    inicioEstadoBateria = ahora;
    return;
  }
  if (nuevo != estadoBateriaPendiente) {
    estadoBateriaPendiente = nuevo;
    inicioEstadoBateria = ahora;
  }
  if (nuevo != estadoBateria && ahora - inicioEstadoBateria >= CONFIRMAR_BATERIA_MS)
    estadoBateria = nuevo;
}

// ============================================================
// 10. BOTON BOOT: ALTERNAR PANTALLAS SIN BLOQUEOS
// ============================================================
// Detecta pulsaciones BOOT con antirrebote sin bloquear el sistema.
void actualizarBotonBoot(unsigned long ahora) {
  const bool lectura = digitalRead(PIN_BOOT);
  if (lectura != bootLecturaAnterior) ultimoCambioBoot = ahora;
  if (ahora - ultimoCambioBoot >= REBOTE_BOOT_MS && lectura != bootEstadoEstable) {
    bootEstadoEstable = lectura;
    if (bootEstadoEstable == LOW) {
      pantallaActual = (pantallaActual == PantallaOLED::PRINCIPAL)
        ? PantallaOLED::CONFIGURACION : PantallaOLED::PRINCIPAL;
      ultimaOLED = ahora - OLED_INTERVALO_MS;
    }
  }
  bootLecturaAnterior = lectura;
}

// ============================================================
// 11. OLED 72x40: ESTADO + BPM GRANDES + BATERIA
// ============================================================
void mostrarPantallaPrincipal() {
  // Primera linea: prioridad a alerta cardiaca.
  const bool alerta = (estadoAlerta == EstadoAlerta::ACTIVA ||
                       estadoAlerta == EstadoAlerta::CONFIRMANDO_RECUPERACION);
  oled.setFont(u8g2_font_5x7_tr);
  if (alerta) oled.drawStr(0, 7, "ALERTA FC!");
  else if (!dedoPresente) oled.drawStr(0, 7, "SIN CONTACTO");
  else if (!bpmValido) oled.drawStr(0, 7, "LEYENDO...");
  else if (estadoAlerta == EstadoAlerta::CONFIRMANDO_BAJA)
    oled.drawStr(0, 7, "FC BAJA?");
  else oled.drawStr(0, 7, "MONITOREO");

 
  // ========================================================
  // BPM: NUMEROS GRANDES EN NEGRITA Y ETIQUETA DINAMICA
  // ========================================================
  
  // Centro: numeros BPM grandes; etiqueta compacta a la derecha.

  if (bpmValido) {

    char texto[8];
    snprintf(texto, sizeof(texto), "%.0f", bpmPromedio);

    // Fuente de numeros grandes con trazo grueso.
    oled.setFont(u8g2_font_helvB14_tr);
    oled.setFontMode(1);

    // Dibujar BPM.
    oled.drawStr(0, 29, texto);

    // Calcular el ancho real de los numeros.
    uint8_t anchoNumeros = oled.getStrWidth(texto);

    // Colocar la etiqueta inmediatamente despues.
    uint8_t posicionBPM = anchoNumeros + 3;

    oled.setFont(u8g2_font_5x7_tf);

    // Evitar que la etiqueta salga de la pantalla.
    if (posicionBPM + oled.getStrWidth("BPM") <= 72) {
      oled.drawStr(posicionBPM, 27, "BPM");
    }
  }
  else {
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 26, "-- BPM");
  }


  // Ultima linea: estado abreviado de bateria.
  oled.setFont(u8g2_font_5x7_tr);
  if (estadoBateria == EstadoBateria::CRITICA) oled.drawStr(0, 39, "BAT: CARGAR");
  else if (estadoBateria == EstadoBateria::BAJA) oled.drawStr(0, 39, "BAT: BAJA");
  else oled.drawStr(0, 39, "BAT: OK");
}
// Pantalla secundaria: voltaje medido y umbral de BPM.
void mostrarPantallaConfiguracion() {
  char texto[22];
  oled.setFont(u8g2_font_5x7_tr);
  oled.drawStr(0, 7, "CONFIG.");
  snprintf(texto, sizeof(texto), "BAT: %.2fV", voltajeBateria);
  oled.drawStr(0, 18, texto);
  snprintf(texto, sizeof(texto), "UMB: %u BPM", static_cast<unsigned>(umbralBPM));
  oled.drawStr(0, 28, texto);
  oled.drawStr(0, 39, "BOOT: VOLVER");
}
// Refresca OLED cada 200 ms; las alertas tienen prioridad visual.
void actualizarOLED(unsigned long ahora) {
  if (ahora - ultimaOLED < OLED_INTERVALO_MS) return;
  ultimaOLED = ahora;
  oled.clearBuffer();
  const bool alerta = (estadoAlerta == EstadoAlerta::ACTIVA ||
                       estadoAlerta == EstadoAlerta::CONFIRMANDO_RECUPERACION);
  if (alerta || pantallaActual == PantallaOLED::PRINCIPAL)
    mostrarPantallaPrincipal();
  else mostrarPantallaConfiguracion();
  oled.sendBuffer();
}

// ============================================================
// 12. MONITOR SERIE
// ============================================================
constexpr unsigned long SERIAL_INTERVALO_MS = 2000;
unsigned long ultimoSerial = 0;
// Publica BPM, contacto y estado de bateria cada 2 segundos.
void actualizarSerial(unsigned long ahora) {

    // Evita enviar datos cuando no hay un monitor USB CDC conectado.
  // No afecta al sensor, OLED, BLE ni a las alertas.
  if (!Serial) return;

  if (ahora - ultimoSerial < SERIAL_INTERVALO_MS) return;
  ultimoSerial = ahora;
  Serial.print("BPM: "); Serial.print(bpmPromedio, 1);
  Serial.print(" | Contacto: "); Serial.print(dedoPresente ? "SI" : "NO");
  Serial.print(" | Bateria: "); Serial.print(voltajeBateria, 3);
  Serial.print(" V | Estado: ");
  switch (estadoBateria) {
    case EstadoBateria::NORMAL: Serial.println("NORMAL"); break;
    case EstadoBateria::BAJA: Serial.println("BAJA"); break;
    case EstadoBateria::CRITICA: Serial.println("CRITICA"); break;
  }
}

// ============================================================
// 12. BLE GATT + CONFIGURACION PERSISTENTE (NVS)
// ============================================================
// UUID acordados con PulsoAlert Web: NO cambiarlos unilateralmente.
constexpr char BLE_NOMBRE[] = "PulsoAlert";
constexpr char UUID_SERVICIO[] = "f0a10000-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_MEDICION[] = "f0a10001-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_UMBRAL[] = "f0a10002-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_CICLOS[] = "f0a10003-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_LED[] = "f0a10004-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_ESTADO_CONFIG[] = "f0a10005-7b4d-4c6a-9e20-3b8c8f100001";

// Namespace y claves NVS. Mantenerlos para preservar ajustes entre versiones.
constexpr char NVS_ESPACIO[] = "pulsoalert";
constexpr char NVS_UMBRAL[] = "umbral";
constexpr char NVS_CICLOS[] = "ciclos";
constexpr char NVS_LED[] = "ledmin";

// Publicacion de mediciones cada segundo sin detener la adquisicion del sensor.
constexpr unsigned long BLE_PUBLICACION_MS = 1000;
unsigned long ultimaPublicacionBLE = 0;

// Objetos creados una sola vez; son propiedad de la biblioteca BLE.
BLEServer* servidorBLE = nullptr;
BLECharacteristic* caracteristicaMedicion = nullptr;
BLECharacteristic* caracteristicaUmbral = nullptr;
BLECharacteristic* caracteristicaCiclos = nullptr;
BLECharacteristic* caracteristicaLED = nullptr;
BLECharacteristic* caracteristicaEstadoConfig = nullptr;

// Variables modificadas desde callbacks BLE; solo son banderas de comunicacion.
volatile bool clienteBLEConectado = false;
volatile bool reiniciarPublicidadBLE = false;

// Identificadores definidos en el contrato GATT v1.0.
enum class ParametroBLE : uint8_t { NINGUNO = 0, UMBRAL = 1, CICLOS = 2, LED = 3 };
enum class ResultadoBLE : uint8_t { INICIAL = 0, GUARDADO = 1, INVALIDO = 2, ERROR_NVS = 3 };

/**
 * @brief Solicitud de escritura capturada por el callback BLE.
 * @details Los callbacks NO escriben NVS: dejan el trabajo al loop principal.
 * @param parametro Identificador del ajuste solicitado (1, 2 o 3).
 * @param longitud Numero de bytes recibidos (se conserva para validar).
 * @param datos Hasta dos bytes, en orden little-endian.
 */
struct SolicitudBLE {
  uint8_t parametro;
  uint8_t longitud;
  uint8_t datos[2];
};

/**
 * @brief Prototipos explicitos de funciones que reciben tipos propios BLE.
 * @details El preprocesador de Arduino genera prototipos automaticamente y,
 *          dependiendo de la version del IDE, puede colocarlos antes de
 *          las declaraciones de enum y struct. Estas declaraciones manuales
 *          garantizan que los tipos ya se conocen al declarar las funciones.
 *          No modifican la logica ni los datos del contrato GATT.
 */
void publicarParametroBLE(ParametroBLE parametro);
uint16_t valorVigenteBLE(ParametroBLE parametro);
void publicarEstadoConfiguracion(ResultadoBLE resultado, ParametroBLE parametro);
void procesarSolicitudBLE(const SolicitudBLE& solicitud);

// Cola FreeRTOS: protege el intercambio entre la tarea BLE y el loop de Arduino.
QueueHandle_t colaEscriturasBLE = nullptr;

/**
 * @brief Convierte un entero sin signo a dos bytes little-endian.
 * @param destino Arreglo con espacio para al menos dos bytes.
 * @param valor Entero a serializar.
 */
void escribirUint16LE(uint8_t* destino, uint16_t valor) {
  destino[0] = static_cast<uint8_t>(valor & 0xFF);
  destino[1] = static_cast<uint8_t>((valor >> 8) & 0xFF);
}

/**
 * @brief Interpreta dos bytes little-endian como un entero sin signo.
 * @param origen Arreglo de al menos dos bytes, previamente validado.
 * @return Valor uint16 recibido.
 */
uint16_t leerUint16LE(const uint8_t* origen) {
  return static_cast<uint16_t>(origen[0]) |
         (static_cast<uint16_t>(origen[1]) << 8);
}

/**
 * @brief Recupera ajustes persistidos, validando sus rangos.
 * @details Si NVS no existe, falla o contiene valores fuera de rango,
 *          se mantienen los valores iniciales seguros del prototipo.
 */
void cargarConfiguracion() {
  Preferences preferencias;
  if (!preferencias.begin(NVS_ESPACIO, true)) {
    Serial.println("NVS: lectura no disponible; usando valores iniciales");
    return;
  }
  const uint16_t umbral = preferencias.getUShort(NVS_UMBRAL, 50);
  const uint8_t ciclos = preferencias.getUChar(NVS_CICLOS, 3);
  const uint16_t led = preferencias.getUShort(NVS_LED, 2);
  preferencias.end();
  umbralBPM = (umbral >= 30 && umbral <= 120) ? umbral : 50;
  maxCiclos = (ciclos >= 1 && ciclos <= 5) ? ciclos : 3;
  duracionLEDMinutos = (led >= 1 && led <= 10) ? led : 2;
  Serial.printf("NVS: umbral=%u BPM | ciclos=%u | LED=%u min\n",
                umbralBPM, maxCiclos, duracionLEDMinutos);
}

/**
 * @brief Escribe el valor vigente de un parametro en su caracteristica GATT.
 * @param parametro Ajuste que debe actualizarse.
 * @details Permite a la web verificar el valor mediante lectura posterior.
 */
void publicarParametroBLE(ParametroBLE parametro) {
  uint8_t bytes[2] = {};
  switch (parametro) {
    case ParametroBLE::UMBRAL:
      if (caracteristicaUmbral) {
        escribirUint16LE(bytes, umbralBPM);
        caracteristicaUmbral->setValue(bytes, 2);
      }
      break;
    case ParametroBLE::CICLOS:
      if (caracteristicaCiclos) {
        bytes[0] = maxCiclos;
        caracteristicaCiclos->setValue(bytes, 1);
      }
      break;
    case ParametroBLE::LED:
      if (caracteristicaLED) {
        escribirUint16LE(bytes, duracionLEDMinutos);
        caracteristicaLED->setValue(bytes, 2);
      }
      break;
    default: break;
  }
}

/**
 * @brief Obtiene el valor vigente para CONFIG_STATUS.
 * @param parametro Identificador del ajuste.
 * @return Valor actual, o cero si no corresponde a un parametro conocido.
 */
uint16_t valorVigenteBLE(ParametroBLE parametro) {
  switch (parametro) {
    case ParametroBLE::UMBRAL: return umbralBPM;
    case ParametroBLE::CICLOS: return maxCiclos;
    case ParametroBLE::LED: return duracionLEDMinutos;
    default: return 0;
  }
}

/**
 * @brief Informa el resultado de una escritura con CONFIG_STATUS.
 * @param resultado Codigo 0..3 definido en el contrato.
 * @param parametro Identificador 0..3 del ajuste procesado.
 * @details Los bytes 2 y 3 reflejan el valor REAL vigente, no el solicitado.
 */
void publicarEstadoConfiguracion(ResultadoBLE resultado, ParametroBLE parametro) {
  if (!caracteristicaEstadoConfig) return;
  uint8_t datos[4] = {
    static_cast<uint8_t>(resultado), static_cast<uint8_t>(parametro), 0, 0
  };
  escribirUint16LE(&datos[2], valorVigenteBLE(parametro));
  caracteristicaEstadoConfig->setValue(datos, sizeof(datos));
  if (clienteBLEConectado) caracteristicaEstadoConfig->notify();
}

/**
 * @brief Recibe una escritura GATT y la entrega a la cola de procesamiento.
 * @details No modifica configuracion ni escribe memoria desde el callback.
 */
class EventosEscrituraBLE : public BLECharacteristicCallbacks {
 public:
  /**
   * @brief Captura el valor binario enviado por el navegador.
   * @param caracteristica Caracteristica cuyo valor acaba de escribirse.
   */
  void onWrite(BLECharacteristic* caracteristica) override {
    ParametroBLE parametro = ParametroBLE::NINGUNO;
    if (caracteristica == caracteristicaUmbral) parametro = ParametroBLE::UMBRAL;
    else if (caracteristica == caracteristicaCiclos) parametro = ParametroBLE::CICLOS;
    else if (caracteristica == caracteristicaLED) parametro = ParametroBLE::LED;
    if (parametro == ParametroBLE::NINGUNO || colaEscriturasBLE == nullptr) return;

    // 'auto' permite compilar con versiones de la biblioteca que devuelven
    // String (core ESP32 nuevo) o std::string (versiones anteriores).
    const auto recibido = caracteristica->getValue();
    SolicitudBLE solicitud = {};
    solicitud.parametro = static_cast<uint8_t>(parametro);
    solicitud.longitud = static_cast<uint8_t>(recibido.length() > 255 ? 255 : recibido.length());
    if (recibido.length() > 0) solicitud.datos[0] = static_cast<uint8_t>(recibido[0]);
    if (recibido.length() > 1) solicitud.datos[1] = static_cast<uint8_t>(recibido[1]);
    if (xQueueSend(colaEscriturasBLE, &solicitud, 0) != pdTRUE) {
      // Una sola escritura a la vez es requisito del contrato de la web.
      // Si la cola se llena, no afirmamos que la operacion fue guardada.
      Serial.println("BLE: cola de configuracion llena; escritura no procesada");
    }
  }
};

/**
 * @brief Marca los eventos de conexion del cliente y reanuncio BLE.
 */
class EventosServidorBLE : public BLEServerCallbacks {
 public:
  /** @brief Notifica que un navegador se conecto. */
  void onConnect(BLEServer* servidor) override {
    (void)servidor;
    clienteBLEConectado = true;
    Serial.println("BLE: cliente conectado");
  }
  /** @brief Solicita reiniciar publicidad tras la desconexion. */
  void onDisconnect(BLEServer* servidor) override {
    (void)servidor;
    clienteBLEConectado = false;
    reiniciarPublicidadBLE = true;
    Serial.println("BLE: cliente desconectado");
  }
};

/**
 * @brief Valida, persiste y aplica una escritura BLE en el loop principal.
 * @param solicitud Solicitud con identificador, longitud y bytes recibidos.
 * @details No cambia el valor efectivo hasta comprobar que NVS lo guardo.
 *          En caso de error, se restaura la caracteristica al valor vigente.
 */
void procesarSolicitudBLE(const SolicitudBLE& solicitud) {
  const ParametroBLE parametro = static_cast<ParametroBLE>(solicitud.parametro);
  const uint8_t longitudEsperada = (parametro == ParametroBLE::CICLOS) ? 1 : 2;
  const uint16_t valor = longitudEsperada == 1
      ? solicitud.datos[0] : leerUint16LE(solicitud.datos);
  bool valido = solicitud.longitud == longitudEsperada;
  switch (parametro) {
    case ParametroBLE::UMBRAL: valido = valido && valor >= 30 && valor <= 120; break;
    case ParametroBLE::CICLOS: valido = valido && valor >= 1 && valor <= 5; break;
    case ParametroBLE::LED: valido = valido && valor >= 1 && valor <= 10; break;
    default: valido = false; break;
  }
  if (!valido) {
    publicarParametroBLE(parametro);
    publicarEstadoConfiguracion(ResultadoBLE::INVALIDO, parametro);
    Serial.printf("BLE: parametro %u invalido (longitud=%u, valor=%u)\n",
                  solicitud.parametro, solicitud.longitud, valor);
    return;
  }

  Preferences preferencias;
  bool guardado = preferencias.begin(NVS_ESPACIO, false);
  if (guardado) {
    size_t escritos = 0;
    switch (parametro) {
      case ParametroBLE::UMBRAL: escritos = preferencias.putUShort(NVS_UMBRAL, valor); guardado = escritos == 2; break;
      case ParametroBLE::CICLOS: escritos = preferencias.putUChar(NVS_CICLOS, static_cast<uint8_t>(valor)); guardado = escritos == 1; break;
      case ParametroBLE::LED: escritos = preferencias.putUShort(NVS_LED, valor); guardado = escritos == 2; break;
      default: guardado = false; break;
    }
    preferencias.end();
  }
  if (!guardado) {
    publicarParametroBLE(parametro);
    publicarEstadoConfiguracion(ResultadoBLE::ERROR_NVS, parametro);
    Serial.printf("BLE: error al guardar parametro %u en NVS\n", solicitud.parametro);
    return;
  }

  // Aplicar solamente despues de guardar: valores seguros tras reiniciar.
  switch (parametro) {
    case ParametroBLE::UMBRAL: umbralBPM = valor; break;
    case ParametroBLE::CICLOS: maxCiclos = static_cast<uint8_t>(valor); break;
    case ParametroBLE::LED: duracionLEDMinutos = valor; break;
    default: break;
  }
  publicarParametroBLE(parametro);
  publicarEstadoConfiguracion(ResultadoBLE::GUARDADO, parametro);
  Serial.printf("BLE: parametro %u guardado=%u\n", solicitud.parametro, valor);
}

/**
 * @brief Procesa solicitudes BLE pendientes sin bloquear esperando mensajes.
 * @details Se atiende como maximo una solicitud por iteracion del loop.
 */
void actualizarConfiguracionBLE() {
  if (colaEscriturasBLE == nullptr) return;
  SolicitudBLE solicitud = {};
  if (xQueueReceive(colaEscriturasBLE, &solicitud, 0) == pdTRUE)
    procesarSolicitudBLE(solicitud);
}

/**
 * @brief Construye MEASUREMENT v1.1 de 8 bytes y notifica al navegador.
 * @details Los primeros seis bytes son compatibles con GATT v1.0.
 *          Bateria en porcentaje = 255 (no calibrada); voltaje en mV.
 */
void publicarMedicionBLE() {
  if (!caracteristicaMedicion) return;
  const uint16_t bpm = (dedoPresente && bpmValido)
      ? static_cast<uint16_t>(bpmPromedio + 0.5f) : 0;
  const bool alerta = (estadoAlerta == EstadoAlerta::ACTIVA ||
                       estadoAlerta == EstadoAlerta::CONFIRMANDO_RECUPERACION);
  const uint8_t estado = !dedoPresente ? 0 : (bpmValido ? 1 : 2);
  // La lectura ADC y divisor ya estan validados en el hardware del proyecto.
  // Se limita al rango uint16 y se envia 0 si la lectura no esta disponible.
  const uint16_t milivoltios = bateriaInicializada && voltajeBateria > 0.0f
      ? static_cast<uint16_t>(constrain(voltajeBateria * 1000.0f, 0.0f, 65535.0f) + 0.5f)
      : 0;
  uint8_t paquete[8] = {};
  escribirUint16LE(&paquete[0], bpm); // 0-1: BPM uint16 LE.
  paquete[2] = dedoPresente ? 1 : 0; // 2: contacto.
  paquete[3] = 255;                 // 3: bateria % desconocida.
  paquete[4] = alerta ? 1 : 0;      // 4: alerta activa.
  paquete[5] = estado;              // 5: estado de medicion.
  escribirUint16LE(&paquete[6], milivoltios); // 6-7: voltaje LiPo mV.
  caracteristicaMedicion->setValue(paquete, sizeof(paquete));
  if (clienteBLEConectado) caracteristicaMedicion->notify();
}

/**
 * @brief Inicializa BLE, cinco caracteristicas, callbacks y publicidad.
 * @details Los UUID siguen siendo los de GATT v1.0; la medicion usa 8 bytes.
 */
void iniciarBLE() {
  colaEscriturasBLE = xQueueCreate(8, sizeof(SolicitudBLE));
  if (colaEscriturasBLE == nullptr) {
    Serial.println("ERROR BLE: no se pudo crear la cola de escrituras");
    return;
  }
  BLEDevice::init(BLE_NOMBRE);
  servidorBLE = BLEDevice::createServer();
  servidorBLE->setCallbacks(new EventosServidorBLE());
  BLEService* servicio = servidorBLE->createService(UUID_SERVICIO);
  caracteristicaMedicion = servicio->createCharacteristic(
      UUID_MEDICION, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  caracteristicaMedicion->addDescriptor(new BLE2902());
  caracteristicaUmbral = servicio->createCharacteristic(
      UUID_UMBRAL, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  caracteristicaCiclos = servicio->createCharacteristic(
      UUID_CICLOS, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  caracteristicaLED = servicio->createCharacteristic(
      UUID_LED, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  caracteristicaUmbral->setCallbacks(new EventosEscrituraBLE());
  caracteristicaCiclos->setCallbacks(new EventosEscrituraBLE());
  caracteristicaLED->setCallbacks(new EventosEscrituraBLE());
  caracteristicaEstadoConfig = servicio->createCharacteristic(
      UUID_ESTADO_CONFIG, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  caracteristicaEstadoConfig->addDescriptor(new BLE2902());
  publicarParametroBLE(ParametroBLE::UMBRAL);
  publicarParametroBLE(ParametroBLE::CICLOS);
  publicarParametroBLE(ParametroBLE::LED);
  publicarEstadoConfiguracion(ResultadoBLE::INICIAL, ParametroBLE::NINGUNO);
  publicarMedicionBLE();
  servicio->start();
  BLEAdvertising* publicidad = BLEDevice::getAdvertising();
  publicidad->addServiceUUID(UUID_SERVICIO);
  publicidad->setScanResponse(true);
  publicidad->start();
  Serial.println("BLE: PulsoAlert GATT v1.1 anunciado (8 bytes + escritura NVS)");
}

/**
 * @brief Mantiene las notificaciones y reinicia publicidad tras desconectar.
 * @param ahora Tiempo actual en milisegundos (millis).
 */
void actualizarBLE(unsigned long ahora) {
  if (reiniciarPublicidadBLE) {
    reiniciarPublicidadBLE = false;
    BLEDevice::startAdvertising();
    Serial.println("BLE: publicidad reiniciada");
  }
  if (ahora - ultimaPublicacionBLE >= BLE_PUBLICACION_MS) {
    ultimaPublicacionBLE = ahora;
    publicarMedicionBLE();
  }
}

// ============================================================
// 13. INICIALIZACION Y BUCLE PRINCIPAL
// ============================================================
// Inicializa GPIO, I2C, pantalla, sensor y lectura de bateria.
void setup() {
  Serial.begin(115200);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  apagarMotor();
  apagarLED();
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATERIA, ADC_11db);
  Wire.begin(PIN_SDA, PIN_SCL);
  oled.setI2CAddress(0x3C * 2);
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x7_tr);
  oled.drawStr(0, 12, "PULSOALERT");
  oled.drawStr(0, 25, "V1.5.3");
  oled.sendBuffer();
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30102");
    oled.clearBuffer();
    oled.drawStr(0, 15, "ERROR SENSOR");
    oled.sendBuffer();
    while (true) { apagarMotor(); apagarLED(); delay(1000); }
  }
  sensor.setup();
  actualizarBateria(millis());
  cargarConfiguracion(); // Restaurar umbral, ciclos y LED ANTES de anunciar BLE.
  iniciarBLE();
  Serial.println("PulsoAlert v1.5.3 iniciado (BLE escritura + NVS + bateria mV)");
}
/*
 * Bucle principal cooperativo: sensor, alertas, motor, LED, bateria,
 * boton, pantalla y monitor serie. Ninguna fase de actuadores usa delay().
 */
void loop() {
  const unsigned long ahora = millis();
  actualizarSensor(ahora);
  actualizarAlerta(ahora);
  actualizarMotor(ahora);
  actualizarLED(ahora);
  actualizarBateria(ahora);
  actualizarBotonBoot(ahora);
  actualizarOLED(ahora);
//  actualizarSerial(ahora);
  actualizarConfiguracionBLE(); // Ejecutar NVS fuera de los callbacks BLE.
  actualizarBLE(ahora);
}
