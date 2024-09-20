#include <Arduino.h>
#include <ESP32Servo.h>

Servo servoMotor;

const int pinoServo = 33;  // Pino do transistor
const int pinoControle = 32; // Pino de controle do ESP32

void setup() {
    pinMode(pinoControle, OUTPUT);
    servoMotor.attach(pinoServo);
    Serial.begin(115200);
    
    
}

void loop() {
    digitalWrite(pinoControle, HIGH); // Ativa o TIP122
    delay(1000); // Espera 1 segundo
    servoMotor.write(0); // Fechar
    delay(1000); // Espera 1 segundo
    servoMotor.write(85); // Abrir
    delay(1000); // Espera 1 segundo
    digitalWrite(pinoControle, LOW); // Desativa o TIP122
}
