/*
 *  MASTER - 3ra Estacion (V5 - Comunicacion GPIO)
 *
 *  PROTOCOLO DE COMUNICACION:
 *
 *  [Slave → Master]  PIN_AGV_SIGNAL (entrada):
 *    Pulso LOW→HIGH  = brazo en reposo, pieza depositada (confirmacion)
 *
 *  [Master → Slave]  Signal_to_robot (salida), diferenciado por duracion:
 *    Pulso HIGH ~250 ms        → CMD_START_BRAZO
 *    Pulso HIGH ~550 ms        → CMD_RELEASE_AGV
 *    HIGH sostenido > 700 ms   → EMERGENCIA (brazo se congela)
 *    Flanco HIGH→LOW           → REARME
 *
 *  CONEXIONES:
 *    IR sensores pieza → Pines 13, 4, 12
 *    Sensor AGV        → Pin 9
 
 *    Sensor seguridad  → Pin 3  (INPUT) prioridad absoluta
 *    Servos            → Pines 11, 10
 *    Conveyor          → Pin 8  (relay logica inversa: LOW=ON, HIGH=OFF)
 *    STOP              → Pin 2  (INPUT_PULLUP)
 *    RESET             → Pin 7  (INPUT_PULLUP)

 
 *    Signal_to_robot   → Pin 5  (OUTPUT)
 *    PIN_AGV_SIGNAL    → Pin 6  (INPUT)
 *    OLED              → SDA=A4, SCL=A5
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── Duraciones del protocolo GPIO ────────────────────────────
const unsigned long PULSE_START_MS      =  250;
const unsigned long PULSE_RELEASE_MS    =  550;
const unsigned long EMERGENCY_THRESHOLD =  700;

// ── OLED ─────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Pines ─────────────────────────────────────────────────────
const int low_ir          = 13;
const int med_ir          =  4;
const int high_ir         = 12;

const int sensor_agv      =  9;
const int sensor_seg      =  3;
const int conveyor        =  8;
const int stopPin         =  2;
const int resetPin        =  7;
const int Signal_to_robot =  5;
const int PIN_AGV_SIGNAL  =  6;

// ── Macros conveyor ──────────────────────────────────────────
#define CONVEYOR_ON   LOW
#define CONVEYOR_OFF  HIGH

// ── Tamanio de lote ──────────────────────────────────────────
const int BATCH_SIZE = 5;

// ── Servos clasificadores ────────────────────────────────────
Servo servo_med, servo_high;
const int def_pos  = 180;
const int med_pos  =  90;
const int high_pos =  90;

// ── Estados ──────────────────────────────────────────────────
enum state {
  IDLE,
  BRAZO_ACTIVO,
  CLASIFICANDO,
  BANDA_CORRIENDO,
  CICLO_COMPLETO,
  BATCH_WAIT,
  Stop
};
state current = IDLE;

// ── Contadores ───────────────────────────────────────────────
int count_low  = 0;
int count_med  = 0;
int count_high = 0;

// ── Timers ───────────────────────────────────────────────────
const unsigned long SERVO_TO_CONVEYOR  =  3500;
const unsigned long CONVEYOR_TIMEOUT   = 10000;
const unsigned long CYCLE_MSG_DURATION =  3000;
const unsigned long CLASSIFY_DELAY     =  3500;
const unsigned long displayInterval    =   100;

unsigned long tServoActivado       = 0;
unsigned long tBandaEncendida      = 0;
unsigned long tCicloCompleto       = 0;
unsigned long tSignal              = 0;
unsigned long displayTime          = 0;
unsigned long tClasificacionInicio = 0;

// ── Flags de entradas ────────────────────────────────────────
bool read_stop        = false;
bool read_seg         = false;
bool read_low         = false;
bool read_med         = false;
bool read_high        = false;
bool read_agv         = false;
bool read_agv_signal  = false;
bool prev_agv_signal  = false;
bool emergencyStopped = false;

// ── Flags de control ─────────────────────────────────────────
bool lowCounted         = false;
bool batchPending       = false;
bool clasificacionHecha = false;  // evita encender banda antes de clasificar
bool lastStopState      = HIGH;
bool lastResetState     = HIGH;
bool lastSegState       = LOW;

// ── Estado de la senal al Slave ──────────────────────────────
enum PulsoTipo { PULSO_NINGUNO, PULSO_START, PULSO_RELEASE, PULSO_EMERGENCIA };
PulsoTipo pulsoActivo = PULSO_NINGUNO;

// ─────────────────────────────────────────────────────────────
//  Senales al Slave
// ─────────────────────────────────────────────────────────────
void sendStart() {
  if (pulsoActivo != PULSO_NINGUNO) return;
  digitalWrite(Signal_to_robot, HIGH);
  tSignal     = millis();
  pulsoActivo = PULSO_START;
  Serial.println(F("Senal: START"));
}

void sendRelease() {
  if (pulsoActivo != PULSO_NINGUNO) return;
  digitalWrite(Signal_to_robot, HIGH);
  tSignal     = millis();
  pulsoActivo = PULSO_RELEASE;
  Serial.println(F("Senal: RELEASE"));
}

void sendEmergency() {
  digitalWrite(Signal_to_robot, HIGH);
  pulsoActivo = PULSO_EMERGENCIA;
  Serial.println(F("Senal: EMERGENCIA"));
}

void sendRearme() {
  digitalWrite(Signal_to_robot, LOW);
  pulsoActivo = PULSO_NINGUNO;
  Serial.println(F("Senal: REARME"));
}

void manageSignalPin() {
  if (pulsoActivo == PULSO_START &&
      millis() - tSignal >= PULSE_START_MS) {
    digitalWrite(Signal_to_robot, LOW);
    pulsoActivo = PULSO_NINGUNO;
  }
  if (pulsoActivo == PULSO_RELEASE &&
      millis() - tSignal >= PULSE_RELEASE_MS) {
    digitalWrite(Signal_to_robot, LOW);
    pulsoActivo = PULSO_NINGUNO;
  }
}

// ─────────────────────────────────────────────────────────────
//  Lectura de entradas
// ─────────────────────────────────────────────────────────────
void readInputs() {
  read_low  = digitalRead(low_ir)     == LOW;
  read_med  = digitalRead(med_ir)     == LOW;
  read_high = digitalRead(high_ir)    == LOW;
  read_agv  = digitalRead(sensor_agv) == LOW;
  read_seg  = digitalRead(sensor_seg) == LOW;

  // Sensor LOW: solo cuenta mientras la banda corre
  if (current == BANDA_CORRIENDO && read_low) {
    if (!lowCounted) {
      count_low++;
      lowCounted = true;
      Serial.println(F("Pieza chica contada."));
    }
  } else {
    lowCounted = false;
  }

  // Confirmacion del Slave: flanco LOW→HIGH
  prev_agv_signal = read_agv_signal;
  read_agv_signal = digitalRead(PIN_AGV_SIGNAL) == HIGH;

  // Boton STOP: latch
  bool currentStopState = digitalRead(stopPin);
  if (currentStopState == LOW && lastStopState == HIGH) {
    emergencyStopped = true;
  }
  lastStopState = currentStopState;

  // Sensor seguridad: latch al detectar presencia
  if (read_seg && !lastSegState) {
    emergencyStopped = true;
    Serial.println(F("Sensor seguridad: zona ocupada!"));
  }
  lastSegState = read_seg;

  // Boton RESET: libera latch solo si zona despejada
  bool currentResetState = digitalRead(resetPin);
  if (currentResetState == LOW && lastResetState == HIGH && !read_seg) {
    emergencyStopped = false;
  }
  lastResetState = currentResetState;

  read_stop = emergencyStopped;
}

// ─────────────────────────────────────────────────────────────
//  Verificar lote completo
// ─────────────────────────────────────────────────────────────
bool batchComplete() {
  return (count_low  >= BATCH_SIZE ||
          count_med  >= BATCH_SIZE ||
          count_high >= BATCH_SIZE);
}

void resetBatchCounters() {
  if (count_low  >= BATCH_SIZE) count_low  = 0;
  if (count_med  >= BATCH_SIZE) count_med  = 0;
  if (count_high >= BATCH_SIZE) count_high = 0;
  Serial.println(F("Contadores de lote reseteados."));
}

// ─────────────────────────────────────────────────────────────
//  Maquina de estados
// ─────────────────────────────────────────────────────────────
void updateStates() {
  unsigned long ahora = millis();

  // ── STOP/SEGURIDAD: prioridad absoluta ───────────────────
  if (current != Stop && read_stop) {
    servo_med.write(def_pos);
    servo_high.write(def_pos);
    digitalWrite(conveyor, CONVEYOR_OFF);
    sendEmergency();
    current = Stop;
    return;
  }

  switch (current) {

    case IDLE:
      if (read_agv) {
        Serial.println(F("AGV detectado. Activando brazo..."));
        sendStart();
        current = BRAZO_ACTIVO;
      }
      break;

    case BRAZO_ACTIVO:
      if (!prev_agv_signal && read_agv_signal) {
        Serial.println(F("Brazo en reposo. Esperando estabilizacion sensores..."));
        tClasificacionInicio = ahora;
        clasificacionHecha   = false;
        current = CLASIFICANDO;
      }
      break;

    case CLASIFICANDO:
      // Paso 1: esperar delay de estabilizacion y clasificar una sola vez
      if (!clasificacionHecha &&
          ahora - tClasificacionInicio >= CLASSIFY_DELAY) {
        if (read_med && read_high) {
          servo_high.write(high_pos);
          count_high++;
          Serial.println(F("Pieza grande confirmada."));
        } else if (read_med && !read_high) {
          servo_med.write(med_pos);
          count_med++;
          Serial.println(F("Pieza mediana confirmada."));
        } else {
          Serial.println(F("Pieza chica confirmada."));
        }
        clasificacionHecha = true;
        tServoActivado     = ahora;  // timer para banda empieza AQUI
      }

      // Paso 2: solo encender banda si ya se clasifico
      if (clasificacionHecha &&
          ahora - tServoActivado >= SERVO_TO_CONVEYOR) {
        Serial.println(F("Encendiendo banda."));
        digitalWrite(conveyor, CONVEYOR_ON);
        sendRelease();
        tBandaEncendida = ahora;
        current = BANDA_CORRIENDO;
      }
      break;

    case BANDA_CORRIENDO:
      if (ahora - tBandaEncendida >= CONVEYOR_TIMEOUT) {
        Serial.println(F("Apagando banda. Regresando servos."));
        digitalWrite(conveyor, CONVEYOR_OFF);
        servo_med.write(def_pos);
        servo_high.write(def_pos);
        tCicloCompleto = ahora;
        current = CICLO_COMPLETO;
      }
      break;

    case CICLO_COMPLETO:
      if (ahora - tCicloCompleto >= CYCLE_MSG_DURATION) {
        if (batchComplete()) {
          Serial.println(F("Lote completo. Esperando operario."));
          batchPending = true;
          current = BATCH_WAIT;
        } else {
          current = IDLE;
        }
      }
      break;

    case BATCH_WAIT:
      if (!read_stop && !read_seg) {
        resetBatchCounters();
        batchPending = false;
        current = IDLE;
        Serial.println(F("Lote retirado. Reanudando."));
      }
      break;

    case Stop:
      if (!read_stop) {
        sendRearme();
        current = batchPending ? BATCH_WAIT : IDLE;
      }
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  Display
// ─────────────────────────────────────────────────────────────
void updateDisplay() {
  if (millis() - displayTime < displayInterval) return;
  displayTime = millis();

  display.clearDisplay();

  if (current == Stop) {
    display.setTextSize(2);
    display.setCursor(10, 4);
    display.println("EMERGENCY");
    display.setCursor(35, 26);
    display.println("STOP!");
    display.setTextSize(1);
    display.setCursor(6, 45);
    display.print(F("Pulse RESET to go to IDLE"));
    display.display();
    return;
  }

  if (current == CICLO_COMPLETO) {
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("CYCLE");
    display.setCursor(10, 42);
    display.println("COMPLETE");
    display.display();
    return;
  }

  if (current == BATCH_WAIT) {
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 15);
    display.println("BATCH");
    display.setCursor(10, 37);
    display.println("COMPLETE");
    display.setTextSize(1);
    display.setCursor(10, 57);
    display.println("Remove pieces");
    display.display();
    return;
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("PIECE COUNTING");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 15);
  display.print("L: ");
  display.print(count_low);
  display.setCursor(0, 35);
  display.print("M: ");
  display.print(count_med);
  display.setCursor(0, 50);
  display.print("H: ");
  display.print(count_high);

  display.setTextSize(1);
  display.setCursor(70, 15);
  switch (current) {
    case IDLE:            display.print("IDLE");  break;
    case BRAZO_ACTIVO:    display.print("ROBOT ARM"); break;
    case CLASIFICANDO:    display.print("CLASSIFY");  break;
    case BANDA_CORRIENDO: display.print("CONVEYOR"); break;
    default: break;
  }

  display.display();
}

// ─────────────────────────────────────────────────────────────
//  Setup & Loop
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(low_ir,          INPUT);
  pinMode(med_ir,          INPUT);
  pinMode(high_ir,         INPUT);
  pinMode(sensor_agv,      INPUT);
  pinMode(sensor_seg,      INPUT);
  pinMode(conveyor,        OUTPUT);
  pinMode(stopPin,         INPUT_PULLUP);
  pinMode(resetPin,        INPUT_PULLUP);
  pinMode(Signal_to_robot, OUTPUT);
  pinMode(PIN_AGV_SIGNAL,  INPUT);

  digitalWrite(Signal_to_robot, LOW);
  digitalWrite(conveyor, CONVEYOR_OFF);

  servo_med.attach(11);
  servo_high.attach(10);
  servo_med.write(def_pos);
  servo_high.write(def_pos);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Wire.begin();
  display.clearDisplay();
  display.display();

  // Inicializar prev_agv_signal con estado real del pin
  prev_agv_signal = (digitalRead(PIN_AGV_SIGNAL) == HIGH);

  Serial.println(F("Maestro iniciado - V5"));
}

void loop() {
  readInputs();
  manageSignalPin();
  updateStates();
  updateDisplay();
}
