#include "portao.h"
#include <Arduino.h>

// Pinos do Portão
// const int in1 = 32;                     // Pino IN1 da Ponte H
// const int in2 = 33;                     // Pino IN2 da Ponte H
// const int ena = 25;                     // Pino ENA da Ponte H (PWM para controle de velocidade)
// const int limite_aberto = 34;           // Fim de curso do portão aberto
// const int limite_fechado = 35;          // Fim de curso do portão fechado
// const int botao_unico = 23;             // Botão único para abrir/parar/fechar portão

// setupPortao(in1, in2, ena, limite_aberto, limite_fechado, botao_unico);  // Parâmetros: in1, in2, ena, limite_aberto, limite_fechado, botao_unico

int in1, in2, ena, limite_aberto, limite_fechado, botao_unico;
enum EstadoPortao { PARADO, ABRINDO, FECHANDO };
EstadoPortao estadoAtual = PARADO;
EstadoPortao ultimoEstado = PARADO;

void setupPortao(int in1Pin, int in2Pin, int enaPin, int limiteAbertoPin, int limiteFechadoPin, int botaoUnicoPin) {
    in1 = in1Pin;
    in2 = in2Pin;
    ena = enaPin;
    limite_aberto = limiteAbertoPin;
    limite_fechado = limiteFechadoPin;
    botao_unico = botaoUnicoPin;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(limite_aberto, INPUT_PULLUP);
    pinMode(limite_fechado, INPUT_PULLUP);
    pinMode(botao_unico, INPUT_PULLUP);
    
    pararMotor();
    Serial.begin(115200);
}

void loopPortao() {
    if (digitalRead(botao_unico) == LOW) {
        delay(200);  // Debounce
        if (estadoAtual == PARADO) {
            if (digitalRead(limite_fechado) == LOW) {
                abrirPortao();
                estadoAtual = ABRINDO;
            } else if (digitalRead(limite_aberto) == LOW) {
                fecharPortao();
                estadoAtual = FECHANDO;
            } else {
                if (ultimoEstado == ABRINDO) {
                    fecharPortao();
                    estadoAtual = FECHANDO;
                } else {
                    abrirPortao();
                    estadoAtual = ABRINDO;
                }
            }
        } else {
            pararMotor();
            ultimoEstado = estadoAtual;
            estadoAtual = PARADO;
        }
    }
    
    verificarLimites();
}

void verificarLimites() {
    if (estadoAtual == ABRINDO && digitalRead(limite_aberto) == LOW) {
        pararMotor();
        estadoAtual = PARADO;
    } else if (estadoAtual == FECHANDO && digitalRead(limite_fechado) == LOW) {
        pararMotor();
        estadoAtual = PARADO;
    }
}

void abrirPortao() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, 80);
    Serial.println("Portão abrindo...\n");
}

void fecharPortao() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(ena, 80);
    Serial.println("Portão fechando...\n");
}

void pararMotor() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(ena, 0);
    Serial.println("Motor parado.\n");
}
