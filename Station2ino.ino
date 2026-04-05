#include <Servo.h>

// Pines
const int sensorPin = 2;       // Sensor infrarrojo digital
const int relay = 3;          // LED
const int servo1Pin = 9;       // Servo 1
const int servo2Pin = 10;      // Servo 2
const int stopButtonPin = 4;   // Botón de paro

// Objetos servo
Servo servo1;
Servo servo2;

// Estados
enum Estado {
INICIAL, 
Relay_ON, 
SERVOS_ON};
Estado estado = INICIAL;

// Variables de tiempo
unsigned long tiempoInicio = 0;
const unsigned long tiempoLed = 5000;     // 3 segundos
const unsigned long tiempoServos = 8000;  // 1 segundos

void setup() {
  pinMode(sensorPin, INPUT);
  pinMode(relay, OUTPUT);
  pinMode(stopButtonPin, INPUT_PULLUP); // Botón con resistencia interna
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Estado seguro inicial
  digitalWrite(relay, HIGH);
  servo1.write(90);
  servo2.write(90);
}

void loop() {
  // Verificar botón de paro
  if (digitalRead(stopButtonPin) == LOW) { 
    // Botón presionado → sistema detenido
    digitalWrite(relay, HIGH);
    servo1.write(90);
    servo2.write(90);
    estado = INICIAL; // Reinicia lógica
    return; // No hace nada más
  }

  switch (estado) {
    case INICIAL:
      if (digitalRead(sensorPin) == LOW) { // Objeto detectado
        digitalWrite(relay, LOW);
        tiempoInicio = millis();
        estado = Relay_ON;
      }
      break;

    case Relay_ON:
      if (millis() - tiempoInicio >= tiempoLed) {
        digitalWrite(ledPin, HIGH);
        servo1.write(0);
        servo2.write(0);
        tiempoInicio = millis();
        estado = SERVOS_ON;
      }
      break;

    case SERVOS_ON:
      if (millis() - tiempoInicio >= tiempoServos) {
        servo1.write(90);
        servo2.write(90);
        estado = INICIAL;
      }
      break;
  }
}
