#include <VarSpeedServo.h>

VarSpeedServo servoBase, servoHombro, servoCodo, servoPinza, servoAGV;

const int Master_signal = 2;
const int release_agv   = 5;

const int servoBasePin   = 9;
const int servoHombroPin = 3;
const int servoCodoPin   = 11;
const int servoPinzaPin  = 6;

// ── NUEVO estado añadido ─────────────────────────────────────
// EMERGENCIA_FREEZE: el brazo se detiene en la posición actual y
// espera a que el Master baje la señal (REARME) antes de volver
// a posición de reposo. No se añade ningún GPIO nuevo; se reutiliza
// Master_signal con el siguiente protocolo:
//   - Pulso HIGH ≤ 500 ms → orden de inicio (igual que antes)
//   - HIGH sostenido > EMERGENCY_THRESHOLD ms → emergencia: freeze
//   - Flanco de bajada (HIGH→LOW) estando frozen → ir a reposo
enum BrazoEstado {
  REPOSO,
  YENDO_A_RECOGER,
  CERRANDO_PINZA,
  YENDO_A_COLOCAR,
  ABRIENDO_PINZA,
  VOLVIENDO_A_REPOSO,
  ESPERANDO,
  EMERGENCIA_FREEZE    // ← NUEVO
};

BrazoEstado estadoActual    = REPOSO;
BrazoEstado estadoSiguiente = REPOSO;

const int posReposo[4]      = {84,  55, 180,  180};
const int posRecoger[4]     = {84, 75, 110,  180};
const int posCerrarPinza[4] = {84, 105, 110,   50};
const int posGetOut[4]      = {0,   65, 180,   50};
const int posColocar[4]     = {0,   80, 150,   50};

unsigned long tiempoAnterior  = 0;
unsigned long t_release_agv   = 0;
unsigned long t_return_agv    = 0;
const unsigned long delay_release        = 2000;
const unsigned long delay_return         = 6000;
const unsigned long tiempoEsperaPinza   = 800;
const unsigned long tiempoEntreEstados  = 2500;

// ── Umbral para distinguir pulso de inicio vs. emergencia ────
// El pulso de inicio dura 500 ms; cualquier HIGH que supere este
// umbral se interpreta como emergencia sostenida del Master.
const unsigned long EMERGENCY_THRESHOLD = 700;

bool senalRecibida        = false;
bool prevSenalRecibida    = false;   // ← NUEVO: detección de flancos
unsigned long tSignalHigh = 0;       // ← NUEVO: marca el flanco de subida
bool servosDetenidos      = false;
bool pinzaDetenida        = false;
bool tiempoEsperaCumplido = false;
bool tiempoEstadoCumplido = false;
bool estado_cambiado      = false;
bool pending_release_agv  = false;
bool pending_return_agv   = false;
bool emergencyRecovery    = false;   // ← NUEVO: omite release AGV al recuperar

void readInputs() {
  senalRecibida = digitalRead(Master_signal) == HIGH;

  // ── Detección de flanco de subida para timestamp ─────────
  if (senalRecibida && !prevSenalRecibida) {
    tSignalHigh = millis();          // registrar cuándo subió la señal
  }
  prevSenalRecibida = senalRecibida;
  // ─────────────────────────────────────────────────────────

  servosDetenidos      = !servoBase.isMoving() && !servoHombro.isMoving() && !servoCodo.isMoving();
  pinzaDetenida        = !servoPinza.isMoving();
  tiempoEsperaCumplido = millis() - tiempoAnterior >= tiempoEsperaPinza;
  tiempoEstadoCumplido = millis() - tiempoAnterior >= tiempoEntreEstados;
}

void irAlSiguienteEstado(BrazoEstado siguiente) {
  estadoSiguiente = siguiente;
  estadoActual    = ESPERANDO;
  tiempoAnterior  = millis();
}

void updateStates() {

  // ── PRIORIDAD 1: Detectar emergencia (HIGH sostenido) ────────
  // Se activa en cualquier estado activo (no REPOSO, no ya frozen).
  // El estado REPOSO queda excluido porque ahí HIGH = inicio normal,
  // y si persiste, la máquina pasa a ESPERANDO donde sí se detecta.
  if (senalRecibida &&
      millis() - tSignalHigh >= EMERGENCY_THRESHOLD &&
      estadoActual != EMERGENCIA_FREEZE &&
      estadoActual != REPOSO) {

    servoBase.stop();
    servoHombro.stop();
    servoCodo.stop();
    servoPinza.stop();
    pending_release_agv = false;    // cancelar AGV pendiente
    pending_return_agv  = false;
    emergencyRecovery   = true;     // no relanzar AGV al recuperar
    estadoActual        = EMERGENCIA_FREEZE;
    estado_cambiado     = true;
    Serial.println("FREEZE: detenido. Esperando REARME del Master...");
    return;
  }

  // ── PRIORIDAD 2: Detectar REARME (flanco de bajada) ─────────
  // Solo mientras está frozen. El Master baja la señal al rearmar.
  if (!senalRecibida && estadoActual == EMERGENCIA_FREEZE) {
    Serial.println("REARME recibido. Volviendo a reposo...");
    estadoActual    = VOLVIENDO_A_REPOSO;
    estado_cambiado = true;
    return;
  }

  // ── Máquina de estados normal ────────────────────────────────
  switch (estadoActual) {

    case REPOSO:
      if (senalRecibida) {
        Serial.println("Señal recibida. Iniciando...");
        irAlSiguienteEstado(YENDO_A_RECOGER);
      }
      break;

    case YENDO_A_RECOGER:
      if (servosDetenidos) {
        Serial.println("En posición de recoger. Cerrando pinza...");
        irAlSiguienteEstado(CERRANDO_PINZA);
      }
      break;

    case CERRANDO_PINZA:
      if (tiempoEsperaCumplido && pinzaDetenida) {
        Serial.println("Pinza cerrada. Yendo a colocar...");
        irAlSiguienteEstado(YENDO_A_COLOCAR);
      }
      break;

    case YENDO_A_COLOCAR:
      if (servosDetenidos) {
        Serial.println("En posición de colocar. Abriendo pinza...");
        irAlSiguienteEstado(ABRIENDO_PINZA);
      }
      break;

    case ABRIENDO_PINZA:
      if (tiempoEsperaCumplido && pinzaDetenida) {
        Serial.println("Pinza abierta. Volviendo a reposo...");
        irAlSiguienteEstado(VOLVIENDO_A_REPOSO);
      }
      break;

    case VOLVIENDO_A_REPOSO:
      if (servosDetenidos) {
        Serial.println("En reposo.");
        if (!emergencyRecovery) {
          // Ciclo normal: liberar AGV
          pending_release_agv = true;
          t_release_agv       = millis();
        }
        emergencyRecovery = false;   // limpiar flag en cualquier caso
        irAlSiguienteEstado(REPOSO);
      }
      break;

    case ESPERANDO:
      if (tiempoEstadoCumplido) {
        estadoActual    = estadoSiguiente;
        tiempoAnterior  = millis();
        estado_cambiado = true;
        servosDetenidos = false;
      }
      break;

    case EMERGENCIA_FREEZE:
      // Espera pasiva: nada que hacer aquí.
      // La salida de este estado la maneja PRIORIDAD 2 arriba.
      break;
  }

  if (pending_release_agv && millis() - t_release_agv >= delay_release) {
    pending_release_agv = false;
    Serial.println("AGV moving out");
    servoAGV.write(90);
    pending_return_agv = true;
    t_return_agv       = millis();
  }
  if (pending_return_agv && millis() - t_return_agv >= delay_return) {
    pending_return_agv = false;
    servoAGV.write(0);
    Serial.println("AGV has moved out");
  }
}

void updateOutputs() {
  if (!estado_cambiado) return;
  estado_cambiado = false;

  switch (estadoActual) {

    case REPOSO:
      break;

    case YENDO_A_RECOGER:
      servoCodo.write(posRecoger[2],   20, true);
      servoHombro.write(posRecoger[1], 20, true);
      servoPinza.write(posRecoger[3],  40, true);
      break;

    case CERRANDO_PINZA:
      servoHombro.write(posCerrarPinza[1], 30, true);
      servoPinza.write(posCerrarPinza[3], 30, true);
      break;

    case YENDO_A_COLOCAR:
      servoHombro.write(posGetOut[1], 15, true);
      servoCodo.write(posGetOut[2],   15, true);
      servoBase.write(posColocar[0],   15, true);
      break;

    case ABRIENDO_PINZA:
      servoHombro.write(posColocar[1], 40, true);
      servoCodo.write(posColocar[2], 40, true);
      servoPinza.write(posReposo[3], 40, true);
      servoCodo.write(posReposo[2],   40, true);

      break;

    case VOLVIENDO_A_REPOSO:
      servoBase.write(posReposo[0],   50, true);
      servoHombro.write(posReposo[1], 50, true);
      servoCodo.write(posReposo[2],   50, true);
      break;

    case EMERGENCIA_FREEZE:
      // Los servos ya fueron detenidos con .stop() en updateStates().
      // No se envía ningún movimiento nuevo: el brazo queda congelado
      // en la posición que tenía al momento del STOP.
      break;

    case ESPERANDO:
      break;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(Master_signal, INPUT);

  servoBase.attach(servoBasePin);
  servoHombro.attach(servoHombroPin);
  servoCodo.attach(servoCodoPin);
  servoPinza.attach(servoPinzaPin);
  servoAGV.attach(release_agv);

  servoBase.write(posReposo[0],   30, true);
  servoCodo.write(posReposo[2],   30, true);
  servoHombro.write(posReposo[1], 30, true);
  servoPinza.write(posReposo[3],  30, true);
  servoAGV.write(0, 80, true);
}

void loop() {
  readInputs();
  updateStates();
  updateOutputs();
  delay(10);
}
