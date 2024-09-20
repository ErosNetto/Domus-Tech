#include <Arduino.h>
#include <ESP32Servo.h>
#include "portao.h"

// Definições dos pinos e variáveis globais
int botao_unico;
int pinoServo;
Servo servoMotor;
const int anguloAberto = 85;  // Ângulo para abrir o portão
const int anguloFechado = 0;   // Ângulo para fechar o portão
bool portaoAberto = false;      // Estado inicial do portão (fechado)

void setupPortao(int pinoBotao, int pinoServo) {
    botao_unico = pinoBotao;
    ::pinoServo = pinoServo;
    
    pinMode(botao_unico, INPUT_PULLUP); 
    servoMotor.attach(pinoServo);
    
    // Inicializa o portão como fechado
    fecharPortao();
}

void loopPortao() {
    if (digitalRead(botao_unico) == LOW) {
        delay(200);  // Debounce para evitar leituras múltiplas rápidas
        
        // Alterna entre abrir e fechar o portão
        if (portaoAberto) {
            fecharPortao();
        } else {
            abrirPortao();
        }

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(botao_unico) == LOW) {
            delay(10);  // Espera o botão ser solto
        }
    }
}

void abrirPortao() {
    servoMotor.write(anguloAberto);  // Move o servo para o ângulo de abertura
    portaoAberto = true;              // Atualiza o estado para aberto
    Serial.println("Portão abrindo...");
}

void fecharPortao() {
    servoMotor.write(anguloFechado);  // Move o servo para o ângulo de fechamento
    portaoAberto = false;             // Atualiza o estado para fechado
    Serial.println("Portão fechando...");
}
