#include <ESP32Servo.h>

Servo servoMotor;

void setup() {
    servoMotor.attach(33);  // Pino do servo
    Serial.begin(115200);
}

void loop() {
    servoMotor.write(85);  // Tente mover o servo para 90 graus
    delay(3000);           // Aguarda 1 segundo
    servoMotor.write(180);   // Tente mover o servo para 0 graus
    delay(3000);           // Aguarda 1 segundo
}
