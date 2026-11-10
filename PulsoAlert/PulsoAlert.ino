/*
 * PulsoAlert v1.5.0 - ESP32-C3 OLED 72x40 + MAX30102
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
 *   - Activa el motor con tres pulsos por ciclo y un maximo de tres ciclos.
 *   - Hace parpadear el LED independientemente del motor hasta 120 s.
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
 * NUEVO v1.5.0: servicio BLE GATT v1.0, lecturas y notificaciones.
 * Escritura de configuracion y persistencia: NO implementadas aun.
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
// Valores de configuracion actuales; se podran exponer por BLE en otra etapa.
// Frecuencia promedio por debajo de la cual se confirma una alerta.
constexpr float UMBRAL_BPM = 50.0f;
constexpr unsigned long CONFIRMAR_BAJA_MS = 5000;
// Tiempo de BPM recuperado antes de cancelar la alerta activa.
constexpr unsigned long CONFIRMAR_RECUPERACION_MS = 2000;
// Duracion maxima de la indicacion luminosa desde el inicio de la alerta.
constexpr unsigned long LED_MAX_MS = 120000;
// Patron temporal compartido: 300 ms ON, 200 ms OFF, 3 pulsos,
// seguidos por una pausa de 1000 ms entre ciclos.
constexpr unsigned long MOTOR_PULSO_MS = 300;
constexpr unsigned long MOTOR_ENTREPULSOS_MS = 200;
constexpr unsigned long MOTOR_ENTRECICLOS_MS = 1000;
constexpr uint8_t PULSOS_POR_CICLO = 3;
constexpr uint8_t MAX_CICLOS = 3;

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
  ledActivo = (LED_MAX_MS > 0);
  digitalWrite(PIN_LED, ledActivo ? LED_ON : LED_OFF);
}

/*
 * Calcula el patron de parpadeo del LED mediante el tiempo transcurrido.
 * Reutiliza las duraciones del motor, pero no su estado ni sus contadores.
 *
 * Un ciclo consiste en tres pulsos ON de 300 ms, con pausas OFF de
 * 200 ms entre pulsos y una pausa OFF de 1000 ms entre ciclos.
 * El LED se apaga al alcanzar LED_MAX_MS y no vuelve a encenderse hasta
 * que se inicie un nuevo episodio de alerta.
 */
void actualizarLED(unsigned long ahora) {
  if (!ledActivo) return;

  const unsigned long transcurrido = ahora - inicioLED;
  if (transcurrido >= LED_MAX_MS) {
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
          estadoMotor = (ciclosTerminados >= MAX_CICLOS)
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
  const bool bpmBajas = bpmPromedio < UMBRAL_BPM;
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
  snprintf(texto, sizeof(texto), "UMB: %.0f BPM", UMBRAL_BPM);
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
  if (ahora - ultimoSerial < SERIAL_INTERVALO_MS) return;
  ultimoSerial = ahora;
  Serial.print("BPM: "); Serial.print(bpmPromedio, 1);
  Serial.print(" | Dedo: "); Serial.print(dedoPresente ? "SI" : "NO");
  Serial.print(" | Bateria: "); Serial.print(voltajeBateria, 3);
  Serial.print(" V | Estado: ");
  switch (estadoBateria) {
    case EstadoBateria::NORMAL: Serial.println("NORMAL"); break;
    case EstadoBateria::BAJA: Serial.println("BAJA"); break;
    case EstadoBateria::CRITICA: Serial.println("CRITICA"); break;
  }
}

// ============================================================
// 14. BLUETOOTH LOW ENERGY: GATT v1.0 (ETAPA DE CONEXION)
// ============================================================
// Los UUID son los definidos en el contrato compartido con PulsoAlert Web.
// Las caracteristicas de configuracion son SOLO LECTURA en esta etapa.
// No se anuncian permisos de escritura que el firmware aun no implementa.
constexpr char BLE_NOMBRE[] = "PulsoAlert";
constexpr char UUID_SERVICIO[] = "f0a10000-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_MEDICION[] = "f0a10001-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_UMBRAL[] = "f0a10002-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_CICLOS[] = "f0a10003-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_LED[] = "f0a10004-7b4d-4c6a-9e20-3b8c8f100001";
constexpr char UUID_ESTADO_CONFIG[] = "f0a10005-7b4d-4c6a-9e20-3b8c8f100001";

// Tiempo entre publicaciones BLE: no introduce pausas en el loop principal.
constexpr unsigned long BLE_PUBLICACION_MS = 1000;
unsigned long ultimaPublicacionBLE = 0;

// Punteros administrados por la biblioteca BLE, creados en iniciarBLE().
BLEServer* servidorBLE = nullptr;
BLECharacteristic* caracteristicaMedicion = nullptr;
BLECharacteristic* caracteristicaUmbral = nullptr;
BLECharacteristic* caracteristicaCiclos = nullptr;
BLECharacteristic* caracteristicaLED = nullptr;
BLECharacteristic* caracteristicaEstadoConfig = nullptr;

// Solo se modifica desde los callbacks BLE; volatile indica acceso asincrono.
volatile bool clienteBLEConectado = false;
volatile bool reiniciarPublicidadBLE = false;

/**
 * @brief Atiende los eventos de conexion y desconexion del servidor BLE.
 * Los callbacks no realizan operaciones de sensor ni actualizaciones OLED.
 */
class EventosServidorBLE : public BLEServerCallbacks {
 public:
  /** @brief Marca la conexion de un cliente BLE. */
  void onConnect(BLEServer* servidor) override {
    (void)servidor; // Parametro requerido por la biblioteca, no utilizado.
    clienteBLEConectado = true;
    Serial.println("BLE: cliente conectado");
  }

  /** @brief Solicita reanunciar el dispositivo despues de desconectar. */
  void onDisconnect(BLEServer* servidor) override {
    (void)servidor;
    clienteBLEConectado = false;
    reiniciarPublicidadBLE = true;
    Serial.println("BLE: cliente desconectado");
  }
};

/**
 * @brief Codifica un entero de 16 bits en orden little-endian.
 * @param destino Puntero a dos bytes de salida.
 * @param valor Numero que se desea codificar.
 */
void escribirUint16LE(uint8_t* destino, uint16_t valor) {
  destino[0] = static_cast<uint8_t>(valor & 0xFF);
  destino[1] = static_cast<uint8_t>((valor >> 8) & 0xFF);
}

/**
 * @brief Actualiza el paquete de medicion de seis bytes del contrato GATT.
 * No inventa porcentaje de bateria: 255 indica desconocido mientras no
 * exista una estimacion porcentual validada para la bateria real.
 */
void publicarMedicionBLE() {
  if (caracteristicaMedicion == nullptr) return;

  // BPM 0 significa lectura no valida; el promedio se redondea al entero.
  const uint16_t bpm = (dedoPresente && bpmValido)
    ? static_cast<uint16_t>(bpmPromedio + 0.5f) : 0;

  // La alerta se considera activa tambien durante la confirmacion de recuperacion.
  const bool alerta = (estadoAlerta == EstadoAlerta::ACTIVA ||
                       estadoAlerta == EstadoAlerta::CONFIRMANDO_RECUPERACION);

  // Estado 0: esperando contacto; 1: midiendo; 2: contacto sin BPM valido.
  const uint8_t estado = !dedoPresente ? 0 : (bpmValido ? 1 : 2);
  uint8_t paquete[6] = {};
  escribirUint16LE(paquete, bpm);  // Bytes 0-1: BPM uint16 LE.
  paquete[2] = dedoPresente ? 1 : 0; // Byte 2: contacto.
  paquete[3] = 255;                // Byte 3: bateria desconocida.
  paquete[4] = alerta ? 1 : 0;     // Byte 4: alerta activa.
  paquete[5] = estado;             // Byte 5: estado de medicion.

  caracteristicaMedicion->setValue(paquete, sizeof(paquete));
  if (clienteBLEConectado) caracteristicaMedicion->notify();
}

/**
 * @brief Crea el servidor, las cinco caracteristicas y la publicidad BLE.
 * Las tres configuraciones se exponen con valores reales fijos de v1.4.2.
 * CONFIG_STATUS permanece en cero porque no hay operaciones de escritura.
 */
void iniciarBLE() {
  BLEDevice::init(BLE_NOMBRE);
  servidorBLE = BLEDevice::createServer();
  servidorBLE->setCallbacks(new EventosServidorBLE());
  BLEService* servicio = servidorBLE->createService(UUID_SERVICIO);

  caracteristicaMedicion = servicio->createCharacteristic(
    UUID_MEDICION, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  caracteristicaMedicion->addDescriptor(new BLE2902());

  caracteristicaUmbral = servicio->createCharacteristic(UUID_UMBRAL, BLECharacteristic::PROPERTY_READ);
  caracteristicaCiclos = servicio->createCharacteristic(UUID_CICLOS, BLECharacteristic::PROPERTY_READ);
  caracteristicaLED = servicio->createCharacteristic(UUID_LED, BLECharacteristic::PROPERTY_READ);
  caracteristicaEstadoConfig = servicio->createCharacteristic(
    UUID_ESTADO_CONFIG, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  caracteristicaEstadoConfig->addDescriptor(new BLE2902());

  // En esta etapa los valores son constantes; no se modifica la logica de alerta.
  uint8_t umbral[2];
  escribirUint16LE(umbral, static_cast<uint16_t>(UMBRAL_BPM));
  caracteristicaUmbral->setValue(umbral, sizeof(umbral));
  uint8_t ciclos[1] = { MAX_CICLOS };
  caracteristicaCiclos->setValue(ciclos, sizeof(ciclos));
  uint8_t duracion[2];
  escribirUint16LE(duracion, static_cast<uint16_t>(LED_MAX_MS / 60000UL));
  caracteristicaLED->setValue(duracion, sizeof(duracion));
  uint8_t estadoConfig[4] = {0, 0, 0, 0};
  caracteristicaEstadoConfig->setValue(estadoConfig, sizeof(estadoConfig));

  publicarMedicionBLE(); // Valor inicial disponible incluso antes de conectar.
  servicio->start();
  BLEAdvertising* publicidad = BLEDevice::getAdvertising();
  publicidad->addServiceUUID(UUID_SERVICIO);
  publicidad->setScanResponse(true);
  publicidad->start();
  Serial.println("BLE: PulsoAlert anunciando servicio GATT v1.0");
}

/**
 * @brief Mantiene las notificaciones y recupera la publicidad tras desconectar.
 * @param ahora Tiempo actual de millis() en milisegundos.
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
  oled.drawStr(0, 25, "V1.5.0");
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
  iniciarBLE();
  Serial.println("PulsoAlert v1.5.0 iniciado (BLE solo lectura)");
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
  actualizarSerial(ahora);
  actualizarBLE(ahora);
}
