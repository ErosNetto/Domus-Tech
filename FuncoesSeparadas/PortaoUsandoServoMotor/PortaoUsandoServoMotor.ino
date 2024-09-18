#include <Servo.h>

Servo servoMotor;

// Definições dos pinos
const int botao_unico = 4; // Pino do botão
const int anguloAberto = 180; // Ângulo para portão aberto
const int anguloFechado = 0; // Ângulo para portão fechado

enum EstadoPortao { PARADO, ABRINDO, FECHANDO };
EstadoPortao estadoAtual = PARADO;
EstadoPortao ultimoEstado = PARADO;

void setup() {
    pinMode(botao_unico, INPUT_PULLUP);
    
    servoMotor.attach(13); // Pino de controle do servo motor (ajuste conforme necessário)
    
    pararMotor();
    Serial.begin(115200);
}

void loop() {
    if (digitalRead(botao_unico) == LOW) {
        delay(200);  // Debounce
        
        if (estadoAtual == PARADO) {
            // Alterna entre abrir e fechar o portão
            if (ultimoEstado == FECHANDO || ultimoEstado == PARADO) {
                abrirPortao();
                estadoAtual = ABRINDO;
            } else if (ultimoEstado == ABRINDO) {
                fecharPortao();
                estadoAtual = FECHANDO;
            }
        } else {
            pararMotor();
            ultimoEstado = estadoAtual;
            estadoAtual = PARADO;
        }
    }
}

void abrirPortao() {
    servoMotor.write(anguloAberto);
    Serial.println("Portão abrindo...");
}

void fecharPortao() {
    servoMotor.write(anguloFechado);
    Serial.println("Portão fechando...");
}

void pararMotor() {
    // O servo motor mantém a posição quando o sinal PWM é constante.
    Serial.println("Motor parado.");
}
