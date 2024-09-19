#include <Arduino.h>
#include <ESP32Servo.h>
#include "portao.h"

// Definições dos pinos e variáveis globais
int botao_unico;
int pinoServo;
Servo servoMotor;
const int anguloAberto = 180;
const int anguloFechado = 0;
enum EstadoPortao { PARADO, ABRINDO, FECHANDO } estadoAtual = PARADO;
enum EstadoPortao ultimoEstado = PARADO;

// Função para configurar o portão
void setupPortao(int pinoBotao, int pinoServo) {
    botao_unico = pinoBotao;
    ::pinoServo = pinoServo;
    
    pinMode(botao_unico, INPUT_PULLUP);
    servoMotor.attach(pinoServo);
    pararMotor();
}

// Função para atualizar o estado do portão
void loopPortao() {
    if (digitalRead(botao_unico) == LOW) {
        delay(200);  // Debounce para evitar leituras múltiplas rápidas
        
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
    Serial.println("Motor parado.");
}

// TESTE
// Função para acionar o portão externamente (exemplo)
void acionaPortao() {
    if (estadoAtual == PARADO) {
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