
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <MAX30105.h>
#include <heartRate.h>

#define SDA_PIN 5
#define SCL_PIN 6

// GPIO propuesto para el controlador del motor.
// PENDIENTE: confirmar la conexion electrica.

#define MOTOR_PIN 4
#define LED_PIN 8

// LED integrado activo en LOW
#define LED_ENCENDIDO LOW
#define LED_APAGADO HIGH

// =====================================
// CONFIGURACION PULSOALERT
// =====================================

// Frecuencia cardiaca minima permitida
int UMBRAL_BPM = 45;

// Tiempo bajo el umbral para activar alerta
const unsigned long TIEMPO_ALERTA = 2000;

// Patron de vibracion
const unsigned long DURACION_PULSO = 300;
const unsigned long PAUSA_PULSO = 200;
const unsigned long PAUSA_CICLO = 1000;

const byte PULSOS_POR_CICLO = 3;
const byte MAX_CICLOS = 4;

// Tiempo maximo sin detectar latidos
// Evita actuar sobre una medicion antigua.
const unsigned long TIEMPO_SIN_LATIDO = 4000;

// =====================================
// COMPONENTES
// =====================================

U8G2_SSD1306_72X40_ER_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE,
  SCL_PIN,
  SDA_PIN);

MAX30105 sensor;

// =====================================
// MEDICION BPM - VERSION ORIGINAL
// =====================================

const long UMBRAL_DEDO = 50000;

const byte CANTIDAD_MUESTRAS = 4;
byte historialBPM[CANTIDAD_MUESTRAS];

byte indice = 0;
byte muestrasValidas = 0;

unsigned long ultimoLatido = 0;
unsigned long ultimaPantalla = 0;

float bpm = 0;
int bpmPromedio = 0;

bool dedoDetectado = false;

// =====================================
// CONTROL DE ALERTA
// =====================================

enum EstadoAlerta {
  NORMAL,
  ESPERANDO_ALERTA,
  VIBRANDO,
  ALERTA_FINALIZADA
};

EstadoAlerta estadoAlerta = NORMAL;

unsigned long inicioBPMBajo = 0;
unsigned long ultimoCambioMotor = 0;

byte cicloActual = 0;
byte pulsoActual = 0;

bool motorEncendido = false;

// =====================================
// FUNCIONES DEL MOTOR
// =====================================

void controlarMotor(bool encender) {
    motorEncendido = encender;

    // Motor: controlador activo en HIGH (provisional)
    digitalWrite(
        MOTOR_PIN,
        encender ? HIGH : LOW
    );

    // LED integrado: activo en LOW
    digitalWrite(
        LED_PIN,
        encender ? LED_ENCENDIDO : LED_APAGADO
    );
}

void detenerVibracion() {
  controlarMotor(false);

  cicloActual = 0;
  pulsoActual = 0;
  ultimoCambioMotor = 0;
}

void reiniciarAlerta() {
  detenerVibracion();

  inicioBPMBajo = 0;
  estadoAlerta = NORMAL;
}

void iniciarVibracion(unsigned long ahora) {
  estadoAlerta = VIBRANDO;

  cicloActual = 1;
  pulsoActual = 1;

  ultimoCambioMotor = ahora;
  controlarMotor(true);

  Serial.println("ALERTA: Vibracion iniciada");
}

// =====================================
// MAQUINA DE ESTADOS DE VIBRACION
// =====================================

void actualizarVibracion(unsigned long ahora) {

  if (estadoAlerta != VIBRANDO) {
    return;
  }

  unsigned long transcurrido =
    ahora - ultimoCambioMotor;

  if (motorEncendido) {

    if (transcurrido >= DURACION_PULSO) {
      controlarMotor(false);
      ultimoCambioMotor = ahora;
    }

    return;
  }

  // Motor apagado: esperar entre pulsos
  if (pulsoActual < PULSOS_POR_CICLO) {

    if (transcurrido >= PAUSA_PULSO) {
      pulsoActual++;

      controlarMotor(true);
      ultimoCambioMotor = ahora;
    }

    return;
  }

  // Finalizar despues del ultimo ciclo
  if (cicloActual >= MAX_CICLOS) {
    estadoAlerta = ALERTA_FINALIZADA;

    Serial.println("ALERTA: Ciclos completados");
    return;
  }

  // Esperar antes del siguiente ciclo
  if (transcurrido >= PAUSA_CICLO) {
    cicloActual++;
    pulsoActual = 1;

    controlarMotor(true);
    ultimoCambioMotor = ahora;
  }
}

// =====================================
// EVALUACION DEL UMBRAL
// =====================================

void evaluarAlerta(unsigned long ahora) {

  // Sin medicion valida no se activa alerta
  if (!dedoDetectado || muestrasValidas == 0 || ultimoLatido == 0 || ahora - ultimoLatido > TIEMPO_SIN_LATIDO) {

    reiniciarAlerta();
    return;
  }

  // Frecuencia recuperada
  if (bpmPromedio >= UMBRAL_BPM) {

    if (estadoAlerta != NORMAL) {
      Serial.println("Frecuencia recuperada");
    }

    reiniciarAlerta();
    return;
  }

  // Primera lectura por debajo del umbral
  if (estadoAlerta == NORMAL) {
    estadoAlerta = ESPERANDO_ALERTA;
    inicioBPMBajo = ahora;

    Serial.println("BPM bajo: iniciando temporizador");
  }

  // Confirmar frecuencia baja durante 5 segundos
  if (estadoAlerta == ESPERANDO_ALERTA && ahora - inicioBPMBajo >= TIEMPO_ALERTA) {

    iniciarVibracion(ahora);
  }

  // No reiniciar la alerta si ya completo
  // los tres ciclos y el BPM sigue bajo.
  actualizarVibracion(ahora);
}

// =====================================
// CONFIGURACION
// =====================================

void setup() {
  Serial.begin(115200);

  // Preparar niveles inactivos antes de habilitar salidas
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(LED_PIN, LED_APAGADO);

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  oled.begin();
  oled.setContrast(255);

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 12, "INICIANDO");
  oled.sendBuffer();

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR MAX30102");

    oled.clearBuffer();
    oled.drawStr(0, 15, "ERROR");
    oled.drawStr(0, 30, "SENSOR");
    oled.sendBuffer();

    while (true) {
      delay(1000);
    }
  }

  // Configuracion ORIGINAL del MAX30102
  sensor.setup();

  sensor.setPulseAmplitudeRed(0x0A);
  sensor.setPulseAmplitudeGreen(0);

  Serial.println("PulsoAlert Firmware 1.2");
  Serial.print("Umbral BPM: ");
  Serial.println(UMBRAL_BPM);
}

// =====================================
// BUCLE PRINCIPAL
// =====================================

void loop() {
  long ir = sensor.getIR();

  dedoDetectado = ir > UMBRAL_DEDO;

  if (!dedoDetectado) {
    bpm = 0;
    bpmPromedio = 0;
    muestrasValidas = 0;
    indice = 0;
    ultimoLatido = 0;
  } else if (checkForBeat(ir)) {
    unsigned long ahora = millis();

    if (ultimoLatido != 0) {
      float intervalo =
        (ahora - ultimoLatido) / 1000.0f;

      bpm = 60.0f / intervalo;

      if (bpm >= 30 && bpm <= 220) {
        historialBPM[indice] = (byte)bpm;

        indice =
          (indice + 1) % CANTIDAD_MUESTRAS;

        if (muestrasValidas < CANTIDAD_MUESTRAS)
          muestrasValidas++;

        int suma = 0;

        for (byte i = 0; i < muestrasValidas; i++) {
          suma += historialBPM[i];
        }

        bpmPromedio = suma / muestrasValidas;

        Serial.print("BPM: ");
        Serial.print(bpm);
        Serial.print(" | Promedio: ");
        Serial.println(bpmPromedio);
      }
    }

    ultimoLatido = ahora;
  }

  unsigned long ahora = millis();

  // Evaluar alerta y controlar vibracion
  evaluarAlerta(ahora);

  // Actualizar pantalla cada 200 ms
  if (ahora - ultimaPantalla >= 200) {
    ultimaPantalla = ahora;

    oled.clearBuffer();

    oled.setFont(u8g2_font_6x10_tf);

    if (estadoAlerta == VIBRANDO || estadoAlerta == ALERTA_FINALIZADA) {

      oled.drawStr(0, 9, "ALERTA");
    } else {
      oled.drawStr(0, 9, "PULSO");
    }

    if (!dedoDetectado) {
      oled.drawStr(0, 27, "SIN DEDO");
    } else if (muestrasValidas == 0 || ultimoLatido == 0 || ahora - ultimoLatido > TIEMPO_SIN_LATIDO) {

      oled.drawStr(0, 27, "LEYENDO");
    } else {
      char texto[8];

      snprintf(
        texto,
        sizeof(texto),
        "%d",
        bpmPromedio);

      oled.setFont(u8g2_font_helvB14_tr);
      oled.drawStr(0, 30, texto);

      oled.setFont(u8g2_font_5x7_tf);
      oled.drawStr(43, 30, "BPM");
    }

    oled.sendBuffer();
  }
}
