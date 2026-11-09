
/*
 * ============================================================
 * PROYECTO: PULSOALERT
 * VERSION : 1.3
 * PLACA   : ESP32-C3 OLED 0.42"
 * SENSOR  : MAX30102
 * ============================================================
 *
 * DESCRIPCION:
 *
 * Sistema experimental de monitoreo de frecuencia cardiaca.
 *
 * Funcionalidades:
 *  - Medicion de BPM mediante MAX30102.
 *  - Visualizacion en pantalla OLED.
 *  - Deteccion de frecuencia inferior al umbral.
 *  - Confirmacion temporal antes de generar una alerta.
 *  - Control de motor de vibracion.
 *  - Control independiente del LED integrado.
 *  - Cancelacion al recuperar el pulso o retirar el dedo.
 *
 * IMPORTANTE:
 * Prototipo educativo. No es un dispositivo medico.
 *
 * ============================================================
 */


// ============================================================
// 1. BIBLIOTECAS
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <MAX30105.h>
#include <heartRate.h>


// ============================================================
// 2. CONFIGURACION DE PINES
// ============================================================

// Bus I2C compartido por OLED y MAX30102.
// Pines verificados en el montaje actual.
#define SDA_PIN 5
#define SCL_PIN 6

// Salida de control del motor mediante transistor.
// Controlador activo en HIGH.
#define MOTOR_PIN 4

// LED integrado de la placa ESP32-C3.
// El LED funciona con logica invertida.
#define LED_PIN 8

#define LED_ENCENDIDO LOW
#define LED_APAGADO HIGH


// ============================================================
// 3. PARAMETROS CONFIGURABLES
// ============================================================
//
// Estos parametros podran modificarse posteriormente
// desde una aplicacion Android mediante Bluetooth LE.
//
// Los tiempos se expresan en milisegundos.
// ============================================================

// Frecuencia minima aceptada antes de generar una alerta.
int UMBRAL_BPM = 50;

// Tiempo que debe mantenerse el BPM bajo para activar alerta.
unsigned long TIEMPO_ALERTA = 5000;

// Numero maximo de ciclos de vibracion del motor.
byte MAX_CICLOS_MOTOR = 3;

// Tiempo maximo de funcionamiento del LED.
// 120000 ms equivalen a 2 minutos.
unsigned long TIEMPO_MAX_LED = 15000;

// Duracion de cada pulso activo.
unsigned long DURACION_PULSO = 300;

// Pausa entre pulsos del mismo ciclo.
unsigned long PAUSA_PULSO = 200;

// Pausa entre ciclos completos.
unsigned long PAUSA_CICLO = 1000;

// Cantidad de pulsos que componen cada ciclo.
const byte PULSOS_POR_CICLO = 3;


// ============================================================
// 4. PARAMETROS DE MEDICION
// ============================================================

// Intensidad infrarroja minima para considerar dedo presente.
const long UMBRAL_DEDO = 50000;

// Cantidad de lecturas utilizadas para calcular promedio BPM.
const byte CANTIDAD_MUESTRAS = 4;

// Rango aceptado para las mediciones individuales.
const float BPM_MIN_VALIDO = 30.0f;
const float BPM_MAX_VALIDO = 220.0f;

// Tiempo maximo sin detectar un latido.
// Despues de este tiempo, el BPM deja de considerarse vigente.
//
// NOTA:
// Este valor es provisional y requiere validacion para
// frecuencias cardiacas muy bajas.
const unsigned long TIEMPO_SIN_LATIDO = 4000;


// ============================================================
// 5. INTERVALOS DE ACTUALIZACION
// ============================================================

// Tiempo entre actualizaciones de pantalla.
const unsigned long INTERVALO_PANTALLA = 200;


// ============================================================
// 6. COMPONENTES
// ============================================================

// Pantalla OLED integrada de 72 x 40 pixeles.
U8G2_SSD1306_72X40_ER_F_HW_I2C oled(
    U8G2_R0,
    U8X8_PIN_NONE,
    SCL_PIN,
    SDA_PIN
);

// Sensor MAX30102.
// La biblioteca utiliza la clase MAX30105.
MAX30105 sensor;


// ============================================================
// 7. VARIABLES DE FRECUENCIA CARDIACA
// ============================================================

// Historial circular de mediciones BPM.
byte historialBPM[CANTIDAD_MUESTRAS] = {0};

// Posicion donde se guardara la siguiente medicion.
byte indice = 0;

// Numero de muestras validas disponibles.
byte muestrasValidas = 0;

// Instante del ultimo latido detectado.
unsigned long ultimoLatido = 0;

// Instante de la ultima actualizacion de pantalla.
unsigned long ultimaPantalla = 0;

// Frecuencia calculada a partir del ultimo intervalo.
float bpm = 0;

// Promedio de las ultimas mediciones validas.
int bpmPromedio = 0;

// Indica si el sensor detecta un dedo.
bool dedoDetectado = false;


// ============================================================
// 8. ESTADOS DEL SISTEMA DE ALERTA
// ============================================================
//
// NORMAL:
// No existe condicion de alerta.
//
// ESPERANDO_ALERTA:
// El BPM esta bajo el umbral y se esta contando el tiempo.
//
// ALERTA_ACTIVA:
// Motor y LED ejecutan sus respectivos patrones.
//
// ALERTA_FINALIZADA:
// Se completaron los tiempos maximos.
// No se repite hasta que desaparezca la condicion de alerta.
// ============================================================

enum EstadoAlerta {
    NORMAL,
    ESPERANDO_ALERTA,
    ALERTA_ACTIVA,
    ALERTA_FINALIZADA
};

// Estado actual de la alerta.
EstadoAlerta estadoAlerta = NORMAL;

// Instante en que comenzo la condicion de BPM bajo.
unsigned long inicioBPMBajo = 0;

// Instante en que se activo la alerta.
unsigned long inicioAlerta = 0;

// Estado fisico de las salidas.
bool motorEncendido = false;
bool ledEncendido = false;

// Permiten registrar una sola vez la finalizacion.
bool motorFinalizado = false;
bool ledFinalizado = false;


// ============================================================
// 9. CONTROL DE SALIDAS
// ============================================================

/*
 * Activa o desactiva el motor de vibracion.
 *
 * El GPIO controla un transistor o circuito de potencia.
 * El motor no se conecta directamente al ESP32-C3.
 */
void controlarMotor(bool encender) {

    // Evitar escrituras innecesarias.
    if (motorEncendido == encender) {
        return;
    }

    motorEncendido = encender;

    digitalWrite(
        MOTOR_PIN,
        encender ? HIGH : LOW
    );
}


/*
 * Controla el LED integrado de la placa.
 *
 * El LED utiliza logica invertida:
 * LOW  = encendido
 * HIGH = apagado
 */
void controlarLED(bool encender) {

    if (ledEncendido == encender) {
        return;
    }

    ledEncendido = encender;

    digitalWrite(
        LED_PIN,
        encender ? LED_ENCENDIDO : LED_APAGADO
    );
}


// ============================================================
// 10. REINICIO DEL SISTEMA DE ALERTA
// ============================================================

/*
 * Cancela cualquier alerta y restablece sus variables.
 *
 * Se utiliza cuando:
 *  - Se retira el dedo.
 *  - La frecuencia se recupera.
 *  - La medicion deja de ser valida.
 */
void reiniciarAlerta() {

    controlarMotor(false);
    controlarLED(false);

    inicioBPMBajo = 0;
    inicioAlerta = 0;

    motorFinalizado = false;
    ledFinalizado = false;

    estadoAlerta = NORMAL;
}


// ============================================================
// 11. INICIO DE UNA ALERTA
// ============================================================

/*
 * Registra el instante de inicio de la alerta.
 *
 * A partir de este momento:
 *  - El motor dispone de MAX_CICLOS_MOTOR ciclos.
 *  - El LED dispone de TIEMPO_MAX_LED milisegundos.
 */
void iniciarAlerta(unsigned long ahora) {

    inicioAlerta = ahora;

    motorFinalizado = false;
    ledFinalizado = false;

    estadoAlerta = ALERTA_ACTIVA;

    Serial.println();
    Serial.println("================================");
    Serial.println("ALERTA DE FRECUENCIA BAJA");
    Serial.print("BPM: ");
    Serial.println(bpmPromedio);
    Serial.println("Motor y LED iniciados");
    Serial.println("================================");
}


// ============================================================
// 12. GENERADOR DEL PATRON DE PULSOS
// ============================================================

/*
 * Calcula si una salida debe estar encendida en un instante.
 *
 * Patron:
 *
 *  ON  300 ms
 *  OFF 200 ms
 *  ON  300 ms
 *  OFF 200 ms
 *  ON  300 ms
 *  OFF 1000 ms
 *
 * El patron se repite en ciclos.
 *
 * No utiliza delay(), permitiendo continuar con las
 * mediciones mientras se ejecuta la alerta.
 *
 * Parametros:
 *  transcurrido: tiempo desde el inicio de la alerta.
 *  numeroCiclo : recibe el indice del ciclo actual,
 *                comenzando en cero.
 *
 * Retorno:
 *  true  = salida encendida.
 *  false = salida apagada.
 */
bool obtenerEstadoPulso(
    unsigned long transcurrido,
    unsigned long &numeroCiclo
) {

    // Tiempo total de los pulsos y pausas internas.
    unsigned long tiempoPulsos =
        PULSOS_POR_CICLO * DURACION_PULSO +
        (PULSOS_POR_CICLO - 1) * PAUSA_PULSO;

    // Duracion completa de un ciclo.
    unsigned long duracionCiclo =
        tiempoPulsos + PAUSA_CICLO;

    // Proteccion frente a configuraciones invalidas.
    if (duracionCiclo == 0 ||
        DURACION_PULSO == 0 ||
        PULSOS_POR_CICLO == 0) {

        numeroCiclo = 0;
        return false;
    }

    // Determinar el ciclo actual.
    numeroCiclo = transcurrido / duracionCiclo;

    // Posicion temporal dentro del ciclo.
    unsigned long posicion =
        transcurrido % duracionCiclo;

    // Estamos en la pausa entre ciclos.
    if (posicion >= tiempoPulsos) {
        return false;
    }

    // Periodo de un pulso y su pausa.
    unsigned long periodoPulso =
        DURACION_PULSO + PAUSA_PULSO;

    // Encender solo durante la parte activa del pulso.
    return (posicion % periodoPulso) < DURACION_PULSO;
}


// ============================================================
// 13. DURACION MAXIMA DEL MOTOR
// ============================================================

/*
 * Calcula el instante en que termina el ultimo pulso
 * del ultimo ciclo permitido para el motor.
 *
 * No incluye la pausa posterior al ultimo ciclo,
 * porque el motor ya se encuentra apagado.
 */
unsigned long obtenerTiempoFinalMotor() {

    if (MAX_CICLOS_MOTOR == 0) {
        return 0;
    }

    unsigned long tiempoPulsos =
        PULSOS_POR_CICLO * DURACION_PULSO +
        (PULSOS_POR_CICLO - 1) * PAUSA_PULSO;

    unsigned long duracionCiclo =
        tiempoPulsos + PAUSA_CICLO;

    return
        (MAX_CICLOS_MOTOR - 1) * duracionCiclo +
        tiempoPulsos;
}


// ============================================================
// 14. ACTUALIZACION INDEPENDIENTE MOTOR / LED
// ============================================================

/*
 * Actualiza las salidas durante una alerta activa.
 *
 * MOTOR:
 * Se apaga definitivamente al completar sus ciclos.
 *
 * LED:
 * Continua ejecutando el patron hasta TIEMPO_MAX_LED.
 *
 * Ambos se detienen anticipadamente cuando
 * evaluarAlerta() detecta una condicion de cancelacion.
 */
void actualizarAlerta(unsigned long ahora) {

    if (estadoAlerta != ALERTA_ACTIVA) {
        return;
    }

    unsigned long transcurrido =
        ahora - inicioAlerta;

    unsigned long numeroCiclo = 0;

    // Estado del patron en el instante actual.
    bool pulsoActivo =
        obtenerEstadoPulso(
            transcurrido,
            numeroCiclo
        );

    // --------------------------------------------------------
    // CONTROL DEL MOTOR
    // --------------------------------------------------------

    unsigned long tiempoFinalMotor =
        obtenerTiempoFinalMotor();

    bool motorPermitido =
        transcurrido < tiempoFinalMotor &&
        numeroCiclo < MAX_CICLOS_MOTOR;

    controlarMotor(
        motorPermitido && pulsoActivo
    );

    if (!motorPermitido && !motorFinalizado) {

        motorFinalizado = true;

        Serial.println(
            "Motor: ciclos completados"
        );
    }

    // --------------------------------------------------------
    // CONTROL DEL LED
    // --------------------------------------------------------

    bool ledPermitido =
        transcurrido < TIEMPO_MAX_LED;

    controlarLED(
        ledPermitido && pulsoActivo
    );

    if (!ledPermitido && !ledFinalizado) {

        ledFinalizado = true;

        Serial.println(
            "LED: tiempo maximo completado"
        );
    }

    // --------------------------------------------------------
    // FINALIZACION GENERAL
    // --------------------------------------------------------

    if (!motorPermitido && !ledPermitido) {

        controlarMotor(false);
        controlarLED(false);

        estadoAlerta = ALERTA_FINALIZADA;

        Serial.println(
            "Alerta finalizada. Esperando recuperacion."
        );
    }
}


// ============================================================
// 15. EVALUACION DE FRECUENCIA CARDIACA
// ============================================================

/*
 * Evalua las condiciones necesarias para generar una alerta.
 *
 * Reglas:
 *
 * 1. Sin dedo o sin BPM valido: cancelar.
 * 2. Sin latidos recientes: cancelar.
 * 3. BPM recuperado: cancelar y habilitar nuevas alertas.
 * 4. BPM bajo: iniciar temporizador de confirmacion.
 * 5. Transcurrido TIEMPO_ALERTA: activar alerta.
 * 6. Al finalizar: no repetir hasta recuperacion.
 */
void evaluarAlerta(unsigned long ahora) {

    // --------------------------------------------------------
    // VALIDACION DE MEDICION
    // --------------------------------------------------------

    bool medicionValida =
        dedoDetectado &&
        muestrasValidas > 0 &&
        ultimoLatido != 0 &&
        ahora - ultimoLatido <= TIEMPO_SIN_LATIDO;

    if (!medicionValida) {

        if (estadoAlerta != NORMAL) {
            Serial.println(
                "Alerta cancelada: medicion no valida"
            );
        }

        reiniciarAlerta();
        return;
    }

    // --------------------------------------------------------
    // RECUPERACION DE FRECUENCIA
    // --------------------------------------------------------

    if (bpmPromedio >= UMBRAL_BPM) {

        if (estadoAlerta != NORMAL) {
            Serial.println(
                "Frecuencia recuperada"
            );
        }

        reiniciarAlerta();
        return;
    }

    // --------------------------------------------------------
    // DETECCION INICIAL DE BPM BAJO
    // --------------------------------------------------------

    if (estadoAlerta == NORMAL) {

        estadoAlerta = ESPERANDO_ALERTA;
        inicioBPMBajo = ahora;

        Serial.print("BPM bajo detectado: ");
        Serial.println(bpmPromedio);
    }

    // --------------------------------------------------------
    // CONFIRMACION TEMPORAL
    // --------------------------------------------------------

    if (estadoAlerta == ESPERANDO_ALERTA) {

        if (ahora - inicioBPMBajo >= TIEMPO_ALERTA) {

            iniciarAlerta(ahora);
        }
    }

    // --------------------------------------------------------
    // EJECUCION DEL PATRON
    // --------------------------------------------------------

    actualizarAlerta(ahora);
}


// ============================================================
// 16. REINICIO DE MEDICIONES BPM
// ============================================================

/*
 * Limpia las mediciones cuando se retira el dedo.
 *
 * No modifica la configuracion del sensor.
 */
void reiniciarMedicion() {

    bpm = 0;
    bpmPromedio = 0;

    muestrasValidas = 0;
    indice = 0;

    ultimoLatido = 0;
}


// ============================================================
// 17. ACTUALIZACION DE PANTALLA OLED
// ============================================================

/*
 * Presenta:
 *
 *  PULSO  : funcionamiento normal.
 *  BAJO   : esperando confirmacion.
 *  ALERTA : alerta activa o finalizada.
 *
 * Muestra SIN DEDO, LEYENDO o el promedio BPM.
 */
void actualizarPantalla(unsigned long ahora) {

    oled.clearBuffer();

    oled.setFont(u8g2_font_6x10_tf);

    // Encabezado segun estado del sistema.
    switch (estadoAlerta) {

        case ESPERANDO_ALERTA:
            oled.drawStr(0, 9, "BAJO");
            break;

        case ALERTA_ACTIVA:
        case ALERTA_FINALIZADA:
            oled.drawStr(0, 9, "ALERTA");
            break;

        default:
            oled.drawStr(0, 9, "PULSO");
            break;
    }

    // --------------------------------------------------------
    // CONTENIDO PRINCIPAL
    // --------------------------------------------------------

    if (!dedoDetectado) {

        oled.drawStr(0, 27, "SIN DEDO");
    }
    else if (
        muestrasValidas == 0 ||
        ultimoLatido == 0 ||
        ahora - ultimoLatido > TIEMPO_SIN_LATIDO
    ) {

        oled.drawStr(0, 27, "LEYENDO");
    }
    else {

        char texto[8];

        snprintf(
            texto,
            sizeof(texto),
            "%d",
            bpmPromedio
        );

        oled.setFont(u8g2_font_helvB14_tr);
        oled.drawStr(0, 30, texto);

        oled.setFont(u8g2_font_5x7_tf);
        oled.drawStr(43, 30, "BPM");
    }

    oled.sendBuffer();
}


// ============================================================
// 18. CONFIGURACION INICIAL
// ============================================================

void setup() {

    // --------------------------------------------------------
    // COMUNICACION SERIE
    // --------------------------------------------------------

    Serial.begin(115200);

    Serial.println();
    Serial.println("================================");
    Serial.println("PULSOALERT - FIRMWARE 1.3");
    Serial.println("================================");

    // --------------------------------------------------------
    // CONFIGURACION DE SALIDAS
    // --------------------------------------------------------

    // Establecer niveles seguros antes de activar GPIO.
    digitalWrite(MOTOR_PIN, LOW);
    digitalWrite(LED_PIN, LED_APAGADO);

    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    motorEncendido = false;
    ledEncendido = false;

    // --------------------------------------------------------
    // BUS I2C
    // --------------------------------------------------------

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // --------------------------------------------------------
    // PANTALLA OLED
    // --------------------------------------------------------

    oled.begin();
    oled.setContrast(255);

    oled.clearBuffer();

    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 12, "INICIANDO");

    oled.sendBuffer();

    // --------------------------------------------------------
    // SENSOR MAX30102
    // --------------------------------------------------------

    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {

        Serial.println("ERROR MAX30102");

        oled.clearBuffer();

        oled.drawStr(0, 15, "ERROR");
        oled.drawStr(0, 30, "SENSOR");

        oled.sendBuffer();

        // Detener ejecucion ante fallo de inicializacion.
        // Las salidas permanecen apagadas.
        while (true) {
            delay(1000);
        }
    }

    // Configuracion original del sensor.
    // Se conserva porque fue la mas estable en pruebas.
    sensor.setup();

    sensor.setPulseAmplitudeRed(0x0A);
    sensor.setPulseAmplitudeGreen(0);

    // --------------------------------------------------------
    // INFORMACION DE CONFIGURACION
    // --------------------------------------------------------

    Serial.println("MAX30102 iniciado");

    Serial.print("Umbral BPM: ");
    Serial.println(UMBRAL_BPM);

    Serial.print("Tiempo confirmacion (ms): ");
    Serial.println(TIEMPO_ALERTA);

    Serial.print("Ciclos motor: ");
    Serial.println(MAX_CICLOS_MOTOR);

    Serial.print("Tiempo LED (ms): ");
    Serial.println(TIEMPO_MAX_LED);

    Serial.println("Sistema listo");
}


// ============================================================
// 19. BUCLE PRINCIPAL
// ============================================================

void loop() {

    // --------------------------------------------------------
    // LECTURA DEL SENSOR
    // --------------------------------------------------------

    // Mantener el metodo original de lectura.
    long ir = sensor.getIR();

    dedoDetectado = ir > UMBRAL_DEDO;

    // --------------------------------------------------------
    // AUSENCIA DE DEDO
    // --------------------------------------------------------

    if (!dedoDetectado) {

        reiniciarMedicion();
    }

    // --------------------------------------------------------
    // DETECCION DE LATIDOS
    // --------------------------------------------------------

    else if (checkForBeat(ir)) {

        unsigned long ahora = millis();

        if (ultimoLatido != 0) {

            float intervalo =
                (ahora - ultimoLatido) / 1000.0f;

            if (intervalo > 0) {

                bpm = 60.0f / intervalo;

                // Aceptar solamente valores del rango definido.
                if (
                    bpm >= BPM_MIN_VALIDO &&
                    bpm <= BPM_MAX_VALIDO
                ) {

                    // Guardar medicion en historial circular.
                    historialBPM[indice] = (byte)bpm;

                    indice =
                        (indice + 1) % CANTIDAD_MUESTRAS;

                    if (muestrasValidas < CANTIDAD_MUESTRAS) {
                        muestrasValidas++;
                    }

                    // Calcular promedio de las muestras.
                    int suma = 0;

                    for (
                        byte i = 0;
                        i < muestrasValidas;
                        i++
                    ) {
                        suma += historialBPM[i];
                    }

                    bpmPromedio =
                        suma / muestrasValidas;

                    // Diagnostico por monitor serie.
                    Serial.print("BPM: ");
                    Serial.print(bpm);

                    Serial.print(" | Promedio: ");
                    Serial.println(bpmPromedio);
                }
            }
        }

        // Registrar el ultimo latido detectado.
        ultimoLatido = ahora;
    }

    // --------------------------------------------------------
    // CONTROL DE ALERTAS
    // --------------------------------------------------------

    unsigned long ahora = millis();

    evaluarAlerta(ahora);

    // --------------------------------------------------------
    // ACTUALIZACION DE PANTALLA
    // --------------------------------------------------------

    if (
        ahora - ultimaPantalla >= INTERVALO_PANTALLA
    ) {

        ultimaPantalla = ahora;

        actualizarPantalla(ahora);
    }
}
