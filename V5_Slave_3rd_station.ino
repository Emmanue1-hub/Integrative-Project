/*
 *  SLAVE - 3ra Estación (V4 - Comunicación GPIO)
 *
 *  PROTOCOLO DE COMUNICACIÓN:
 *
 *  [Slave → Master]  PIN_AGV_SIGNAL (salida):
 *    Pulso LOW→HIGH  = brazo en reposo, pieza depositada (confirmación)
 *
 *  [Master → Slave]  MASTER_SIGNAL_PIN (entrada), diferenciado por duración:
 *    Pulso HIGH ~500 ms        → CMD_START_BRAZO
 *    Pulso HIGH ~1500 ms       → CMD_RELEASE_AGV
 *    HIGH sostenido > 2500 ms  → EMERGENCIA (congelar brazo)
 *    Flanco HIGH→LOW (frozen)  → REARME (volver a reposo)
 *
 *  CONEXIONES:
 *    MASTER_SIGNAL_PIN (Master→Slave) → Pin 3  (INPUT)
 *    PIN_AGV_SIGNAL    (Slave→Master) → Pin 4  (OUTPUT)

 *    Servo AGV         → Pin 5
 *    Servo Base        → Pin 9
 *    Servo Hombro      → Pin 10
 *    Servo Codo        → Pin 11
 *    Servo Pinza       → Pin 6
 */

#include <VarSpeedServo.h>

// ── Umbrales del protocolo ────────────────────────────────────
const unsigned long PULSE_START_MIN     =  200;  // min para ignorar ruido
const unsigned long PULSE_RELEASE_MIN   =  400;  // separa START de RELEASE
const unsigned long EMERGENCY_THRESHOLD =  700;  // bajo para detener brazo rapidamente

// ── Pines ─────────────────────────────────────────────────────
const int MASTER_SIGNAL_PIN = 3;
const int PIN_AGV_SIGNAL    = 4;
const int SERVO_AGV_PIN     = 5;
const int SERVO_BASE_PIN    =  9;
const int SERVO_HOMBRO_PIN  = 10;
const int SERVO_CODO_PIN    = 11;
const int SERVO_PINZA_PIN   =  6;

// ── Servos ────────────────────────────────────────────────────
VarSpeedServo servoBase, servoHombro, servoCodo, servoPinza;
VarSpeedServo servoAGV;

const int AGV_BLOQUEADO = 90;
const int AGV_LIBERADO  =  0;

// ── Posiciones del brazo ──────────────────────────────────────
const int posReposo[4]      = {180,  50, 180, 180};
const int posRecoger[4]     = {170,  90, 160, 180};
const int posCerrarPinza[4] = {170,  90, 145,  55};
const int posGetOut[4]      = { 80,  51, 180,  50};
const int posColocar[4]     = { 80,  65, 135,  55};

// ── Estados ───────────────────────────────────────────────────
enum EsclavoEstado {
  ESPERANDO_CMD,
  BRAZO_EN_SECUENCIA,
  ESPERANDO_LIBERACION,
  LIBERANDO_AGV
};

enum BrazoEstado {
  REPOSO,
  YENDO_A_RECOGER,
  CERRANDO_PINZA,
  YENDO_A_COLOCAR,
  ABRIENDO_PINZA,
  VOLVIENDO_A_REPOSO,
  EMERGENCIA_FREEZE
};

EsclavoEstado estadoEsclavo = ESPERANDO_CMD;
BrazoEstado   estadoBrazo   = REPOSO;

// ── Timers ────────────────────────────────────────────────────
const unsigned long tiempoEsperaPinza  = 500;
const unsigned long RELEASE_TIMEOUT    = 8000;
const unsigned long CONFIRMACION_PULSO = 100;

unsigned long tiempoAnterior   = 0;
unsigned long releaseStartTime = 0;
unsigned long tSignalHigh      = 0;
unsigned long tConfirmacion    = 0;

// ── Flags de entradas ─────────────────────────────────────────
bool masterSignal         = false;
bool prevMasterSignal     = false;
bool servosDetenidos      = false;
bool pinzaDetenida        = false;
bool tiempoEsperaCumplido = false;

// ── Flags de comandos decodificados ──────────────────────────
bool cmdStart   = false;
bool cmdRelease = false;
bool cmdRearme  = false;

// ── Flags de control ─────────────────────────────────────────
bool estado_cambiado      = false;
bool emergencyRecovery    = false;
bool enviandoConfirmacion = false;

// ─────────────────────────────────────────────────────────────
//  Lectura de entradas y decodificación del protocolo
// ─────────────────────────────────────────────────────────────
void readInputs() {
  masterSignal          = digitalRead(MASTER_SIGNAL_PIN) == HIGH;
  servosDetenidos       = !servoBase.isMoving() && !servoHombro.isMoving() && !servoCodo.isMoving();
  pinzaDetenida         = !servoPinza.isMoving();
  tiempoEsperaCumplido  = millis() - tiempoAnterior >= tiempoEsperaPinza;

  // Flanco de subida → registrar timestamp
  if (masterSignal && !prevMasterSignal) {
    tSignalHigh = millis();
  }

  // Flanco de bajada → medir duración y decodificar comando
  if (!masterSignal && prevMasterSignal) {
    unsigned long duracion = millis() - tSignalHigh;

    if (duracion >= PULSE_START_MIN && duracion < PULSE_RELEASE_MIN) {
      cmdStart = true;
      Serial.print(F("CMD: START (")); Serial.print(duracion); Serial.println(F(" ms)"));
    } else if (duracion >= PULSE_RELEASE_MIN && duracion < EMERGENCY_THRESHOLD) {
      cmdRelease = true;
      Serial.print(F("CMD: RELEASE (")); Serial.print(duracion); Serial.println(F(" ms)"));
    }
    if (duracion >= EMERGENCY_THRESHOLD && estadoBrazo == EMERGENCIA_FREEZE) {
      cmdRearme = true;
      Serial.println(F("CMD: REARME"));
    }
  }

  // HIGH sostenido → emergencia — se ejecuta aqui mismo, sin esperar updateStates()
  bool brazoIdle = (estadoBrazo == REPOSO && estadoEsclavo == ESPERANDO_CMD);
  if (masterSignal &&
      millis() - tSignalHigh >= EMERGENCY_THRESHOLD &&
      estadoBrazo != EMERGENCIA_FREEZE &&
      !brazoIdle) {
    servoBase.stop();
    servoHombro.stop();
    servoCodo.stop();
    servoPinza.stop();
    enviandoConfirmacion = false;
    digitalWrite(PIN_AGV_SIGNAL, LOW);
    estadoBrazo       = EMERGENCIA_FREEZE;
    estadoEsclavo     = BRAZO_EN_SECUENCIA;
    emergencyRecovery = true;
    estado_cambiado   = true;
    Serial.println(F("FREEZE: brazo detenido."));
  }

  prevMasterSignal = masterSignal;
}

// ─────────────────────────────────────────────────────────────
//  Pulso de confirmación no bloqueante (LOW → espera → HIGH)
// ─────────────────────────────────────────────────────────────
void manageConfirmacion() {
  if (!enviandoConfirmacion) return;
  if (millis() - tConfirmacion >= CONFIRMACION_PULSO) {
    digitalWrite(PIN_AGV_SIGNAL, HIGH);
    enviandoConfirmacion = false;
    Serial.println(F("Confirmacion enviada al Master."));
  }
}

void sendConfirmacion() {
  digitalWrite(PIN_AGV_SIGNAL, LOW);
  tConfirmacion        = millis();
  enviandoConfirmacion = true;
}

// ─────────────────────────────────────────────────────────────
//  Máquina de estados
// ─────────────────────────────────────────────────────────────
void updateStates() {

  // ── PRIORIDAD 2: Rearme ───────────────────────────────────
  if (cmdRearme) {
    cmdRearme       = false;
    estadoBrazo     = VOLVIENDO_A_REPOSO;
    estado_cambiado = true;
    tiempoAnterior  = millis();
    Serial.println(F("REARME: volviendo a reposo."));
    return;
  }

  // ── Máquina del Esclavo ───────────────────────────────────
  switch (estadoEsclavo) {

    case ESPERANDO_CMD:
      if (cmdStart) {
        cmdStart        = false;
        estadoEsclavo   = BRAZO_EN_SECUENCIA;
        estadoBrazo     = YENDO_A_RECOGER;
        estado_cambiado = true;
        tiempoAnterior  = millis();
        Serial.println(F("Iniciando secuencia del brazo."));
      }
      break;

    case BRAZO_EN_SECUENCIA:
      if (estadoBrazo == REPOSO) {
        if (emergencyRecovery) {
          emergencyRecovery = false;
          servoAGV.write(AGV_BLOQUEADO);
          estadoEsclavo = ESPERANDO_CMD;
          Serial.println(F("Recuperacion completada. AGV bloqueado."));
        } else {
          sendConfirmacion();
          estadoEsclavo = ESPERANDO_LIBERACION;
        }
      }
      break;

    case ESPERANDO_LIBERACION:
      if (cmdRelease) {
        cmdRelease       = false;
        estadoEsclavo    = LIBERANDO_AGV;
        releaseStartTime = millis();
        servoAGV.write(AGV_LIBERADO);
        Serial.println(F("Liberando AGV."));
      }
      break;

    case LIBERANDO_AGV:
      if (millis() - releaseStartTime > RELEASE_TIMEOUT) {
        servoAGV.write(AGV_BLOQUEADO);
        estadoEsclavo = ESPERANDO_CMD;
        Serial.println(F("AGV bloqueado. Listo para proximo ciclo."));
      }
      break;
  }

  // ── Máquina del Brazo ─────────────────────────────────────
  switch (estadoBrazo) {

    case YENDO_A_RECOGER:
      if (servosDetenidos && millis() - tiempoAnterior >= 500) {
        estadoBrazo     = CERRANDO_PINZA;
        estado_cambiado = true;
        tiempoAnterior  = millis();
        Serial.println(F("En posicion de recoger. Cerrando pinza."));
      }
      break;

    case CERRANDO_PINZA:
      if (tiempoEsperaCumplido && pinzaDetenida) {
        estadoBrazo     = YENDO_A_COLOCAR;
        estado_cambiado = true;
        tiempoAnterior  = millis();
        Serial.println(F("Pinza cerrada. Yendo a colocar."));
      }
      break;

    case YENDO_A_COLOCAR:
      if (servosDetenidos && millis() - tiempoAnterior >= 500) {
        estadoBrazo     = ABRIENDO_PINZA;
        estado_cambiado = true;
        tiempoAnterior  = millis();
        Serial.println(F("En posicion de colocar. Abriendo pinza."));
      }
      break;

    case ABRIENDO_PINZA:
      if (tiempoEsperaCumplido && pinzaDetenida) {
        estadoBrazo     = VOLVIENDO_A_REPOSO;
        estado_cambiado = true;
        tiempoAnterior  = millis();
        Serial.println(F("Pinza abierta. Volviendo a reposo."));
      }
      break;

    case VOLVIENDO_A_REPOSO:
      if (servosDetenidos && millis() - tiempoAnterior >= 500) {
        estadoBrazo     = REPOSO;
        estado_cambiado = true;
        Serial.println(F("Brazo en reposo."));
      }
      break;

    case EMERGENCIA_FREEZE:
    case REPOSO:
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  Salidas (solo cuando cambia el estado)
// ─────────────────────────────────────────────────────────────
void updateOutputs() {
  if (!estado_cambiado) return;
  estado_cambiado = false;

  switch (estadoBrazo) {

    case YENDO_A_RECOGER:
      servoBase.write(posRecoger[0],   15, true);
      servoCodo.write(posRecoger[2],   15, true);
      servoHombro.write(posRecoger[1], 15, true);
      servoPinza.write(posRecoger[3],  15, true);
      break;

    case CERRANDO_PINZA:
      servoPinza.write(posCerrarPinza[3], 15, true);
      servoCodo.write(posGetOut[2], 15, true);
      servoHombro.write(posGetOut[1], 15, true);
      break;

    case YENDO_A_COLOCAR:
      servoBase.write(posColocar[0],   15, true);
      servoHombro.write(posColocar[1], 15, true);
      servoCodo.write(posColocar[2],   15, true);
      break;

    case ABRIENDO_PINZA:
      servoHombro.write(80, 15, true);
      servoPinza.write(posReposo[3], 15, true);
      break;

    case VOLVIENDO_A_REPOSO:
      servoHombro.write(posReposo[1], 15, true);
      servoCodo.write(posGetOut[2],   15, true);

      servoBase.write(posReposo[0],   15, true);
      servoHombro.write(posReposo[1], 15, true);
      servoPinza.write(posReposo[3],  15, true);
      break;

    case EMERGENCIA_FREEZE:
    case REPOSO:
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  Setup & Loop
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println(F("Slave 3ra estacion - V5"));

  pinMode(MASTER_SIGNAL_PIN, INPUT);
  pinMode(PIN_AGV_SIGNAL,    OUTPUT);

  digitalWrite(PIN_AGV_SIGNAL, LOW);

  servoBase.attach(SERVO_BASE_PIN);
  servoHombro.attach(SERVO_HOMBRO_PIN);
  servoCodo.attach(SERVO_CODO_PIN);
  servoPinza.attach(SERVO_PINZA_PIN);
  servoAGV.attach(SERVO_AGV_PIN);

  servoBase.write(posReposo[0],   15, true);
  servoCodo.write(posReposo[2],   15, true);
  servoHombro.write(posReposo[1], 30, true);
  servoPinza.write(posReposo[3],  30, true);
  servoAGV.write(AGV_BLOQUEADO);
}

void loop() {
  readInputs();
  manageConfirmacion();
  updateStates();
  updateOutputs();
  delay(10);
}
