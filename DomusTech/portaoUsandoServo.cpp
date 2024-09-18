#include <Arduino.h>
#include <Servo.h>
#include "portaoUsandoServo.h"

// Definições dos pinos e variáveis globais
static int botao_unico;
static int pinoServo;
static Servo servoMotor;
static const int anguloAberto = 180;
static const int anguloFechado = 0;
static enum EstadoPortao { PARADO, ABRINDO, FECHANDO } estadoAtual = PARADO;
static enum EstadoPortao ultimoEstado = PARADO;

// Função para configurar o portão
void setupPortao(int pinoBotao, int pinoServo) {
    botao_unico = pinoBotao;
    ::pinoServo = pinoServo;
    
    pinMode(botao_unico, INPUT_PULLUP);
    servoMotor.attach(pinoServo);
    pararMotor();
    Serial.begin(115200);
}

// Função para atualizar o estado do portão
void atualizarPortao() {
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
    delay(1000);  // Simulação de tempo para o portão abrir (ajustar conforme necessário)
    pararMotor();
}

void fecharPortao() {
    servoMotor.write(anguloFechado);
    Serial.println("Portão fechando...");
    delay(1000);  // Simulação de tempo para o portão fechar (ajustar conforme necessário)
    pararMotor();
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