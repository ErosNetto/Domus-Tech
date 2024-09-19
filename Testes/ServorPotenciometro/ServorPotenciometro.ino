#include <ESP32Servo.h>

const int potPin = 34;  // Pino onde o potenciômetro está conectado
const int servoPin = 25;  // Pino PWM do servo

Servo myServo;  // Cria um objeto servo

void setup() {
  Serial.begin(115200);  // Inicializa a comunicação serial
  myServo.attach(servoPin);  // Conecta o servo ao pino definido
}

void loop() {
  int potValue = analogRead(potPin);  // Lê o valor do potenciômetro (0-4095 no ESP32)

  // Mapeia o valor do potenciômetro para a faixa de 0 a 180
  // 0 a 90 -> Servo gira em uma direção
  // 90 -> Servo parado
  // 90 a 180 -> Servo gira na outra direção
  int speed = map(potValue, 0, 4095, 0, 180);  

  myServo.write(speed);  // Controla a direção e a velocidade do servo contínuo

  // Imprime os valores para depuração
  Serial.print("Potenciômetro: ");
  Serial.print(potValue);
  Serial.print(" | Velocidade: ");
  Serial.println(speed);

  delay(15);  // Pequeno atraso para estabilidade
}
