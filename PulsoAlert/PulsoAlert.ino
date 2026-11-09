/*
 * PulsoAlert v1.4.1 - ESP32-C3 OLED 72x40 + MAX30102
 *
 * GPIO5 SDA / GPIO6 SCL: OLED y MAX30102 (I2C compartido)
 * GPIO4: driver motor (activo HIGH; NO conectar motor directo)
 * GPIO8: LED integrado (activo LOW)
 * GPIO9: boton BOOT (activo LOW; evitar pulsarlo al encender)
 * GPIO0: lectura ADC de bateria mediante divisor 100k/100k
 *
 * Prototipo educativo, no dispositivo medico.
 * No interpretar perdida de senal como frecuencia cardiaca baja.
 * La bateria LiPo requiere alimentacion regulada/protegida apropiada.
 */
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <MAX30105.h>
#include "heartRate.h"

// ============================================================
// 1. HARDWARE Y CONFIGURACION
// ============================================================
constexpr uint8_t PIN_SDA = 5;
constexpr uint8_t PIN_SCL = 6;
constexpr uint8_t PIN_MOTOR = 4;
constexpr uint8_t PIN_LED = 8;
constexpr uint8_t PIN_BOOT = 9;
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
constexpr float UMBRAL_BPM = 50.0f;
constexpr unsigned long CONFIRMAR_BAJA_MS = 5000;
constexpr unsigned long CONFIRMAR_RECUPERACION_MS = 2000;
constexpr unsigned long LED_MAX_MS = 120000;
constexpr unsigned long MOTOR_PULSO_MS = 300;
constexpr unsigned long MOTOR_ENTREPULSOS_MS = 200;
constexpr unsigned long MOTOR_ENTRECICLOS_MS = 1000;
constexpr uint8_t PULSOS_POR_CICLO = 3;
constexpr uint8_t MAX_CICLOS = 3;

enum class EstadoAlerta : uint8_t { NORMAL, CONFIRMANDO_BAJA, ACTIVA, CONFIRMANDO_RECUPERACION };
enum class EstadoMotor : uint8_t { INACTIVO, PULSO, PAUSA_PULSO, PAUSA_CICLO, TERMINADO };
EstadoAlerta estadoAlerta = EstadoAlerta::NORMAL;
EstadoMotor estadoMotor = EstadoMotor::INACTIVO;
unsigned long inicioBaja = 0;
unsigned long inicioRecuperacion = 0;
unsigned long inicioLED = 0;
unsigned long inicioFaseMotor = 0;
uint8_t pulsosTerminados = 0;
uint8_t ciclosTerminados = 0;
bool ledActivo = false;

// ============================================================
// 4. BATERIA: DIVISOR RESISTIVO Y UMBRALES
// ============================================================
constexpr float R1_BATERIA = 100000.0f;
constexpr float R2_BATERIA = 100000.0f;
constexpr float FACTOR_DIVISOR = (R1_BATERIA + R2_BATERIA) / R2_BATERIA;
constexpr float CALIBRACION_BATERIA = 1.0f; // Ajustar comparando con multimetro
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
void apagarMotor() { digitalWrite(PIN_MOTOR, MOTOR_OFF); }
void apagarLED() { digitalWrite(PIN_LED, LED_OFF); ledActivo = false; }
void detenerMotor() {
  apagarMotor();
  estadoMotor = EstadoMotor::INACTIVO;
  pulsosTerminados = 0;
  ciclosTerminados = 0;
}
void iniciarMotor(unsigned long ahora) {
  pulsosTerminados = 0;
  ciclosTerminados = 0;
  inicioFaseMotor = ahora;
  estadoMotor = EstadoMotor::PULSO;
  digitalWrite(PIN_MOTOR, MOTOR_ON);
}
void iniciarLED(unsigned long ahora) {
  digitalWrite(PIN_LED, LED_ON);
  ledActivo = true;
  inicioLED = ahora;
}
void actualizarLED(unsigned long ahora) {
  if (ledActivo && ahora - inicioLED >= LED_MAX_MS) apagarLED();
}
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
void reiniciarMediciones() {
  bpmInstantaneo = 0;
  bpmPromedio = 0;
  bpmValido = false;
  indiceBPM = 0;
  muestrasValidas = 0;
  ultimoLatido = 0;
  for (uint8_t i = 0; i < NUM_MUESTRAS_BPM; ++i) muestrasBPM[i] = 0;
}
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
void cancelarAlerta() {
  detenerMotor();
  apagarLED();
  estadoAlerta = EstadoAlerta::NORMAL;
  inicioBaja = 0;
  inicioRecuperacion = 0;
}
void activarAlerta(unsigned long ahora) {
  estadoAlerta = EstadoAlerta::ACTIVA;
  iniciarMotor(ahora);
  iniciarLED(ahora);
  pantallaActual = PantallaOLED::PRINCIPAL; // Prioridad visual de alerta
}
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
float leerVoltajeBateria() {
  uint32_t suma = 0;
  for (uint8_t i = 0; i < MUESTRAS_ADC; ++i) suma += analogReadMilliVolts(PIN_BATERIA);
  const float voltajeADC = (suma / (float)MUESTRAS_ADC) / 1000.0f;
  return voltajeADC * FACTOR_DIVISOR * CALIBRACION_BATERIA;
}
EstadoBateria clasificarBateria(float voltaje) {
  if (voltaje < BATERIA_CRITICA_V) return EstadoBateria::CRITICA;
  if (voltaje < BATERIA_BAJA_V) return EstadoBateria::BAJA;
  return EstadoBateria::NORMAL;
}
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
// 13. INICIALIZACION Y BUCLE PRINCIPAL
// ============================================================
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
  oled.drawStr(0, 25, "V1.4.1");
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
  Serial.println("PulsoAlert v1.4.1 iniciado");
}
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
}
