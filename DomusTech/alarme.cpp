#include <Arduino.h>
#include "alarme.h"

int btnAcionarAlarme;
int reedSwitchPin;
int buzzerPin;
bool alarmOn = false;

void setupAlarme(int bPin, int rPin, int buzzPin) {
    btnAcionarAlarme = bPin;
    reedSwitchPin = rPin;
    buzzerPin = buzzPin;
 
    pinMode(btnAcionarAlarme, INPUT_PULLUP);
    pinMode(reedSwitchPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
}

// Função para atualizar o estado do alarme
void loopAlarme() {
    if (digitalRead(btnAcionarAlarme) == LOW) {
        delay(200);

        alarmOn = !alarmOn;  // Alterna o estado
        if (alarmOn) {
            Serial.println("Sistema armado.");
            alarmeLigando();
        } else {
            Serial.println("Sistema desarmado.");
            alarmeDesligando();
        }
        delay(500);
    }

    // Verifica o estado do sensor Reed Switch
    if (alarmOn && digitalRead(reedSwitchPin) == HIGH) {
        Serial.println("Intrusão detectada! Alarme ativado.");
        mostrarNoLCD("--- ATENCAO! ---", "Alarme dispadado");
        alarmeTocando();
    } else {
        noTone(buzzerPin);  // Desativa o som quando não há intrusão
    }
}

void alarmeTocando() {
    tone(buzzerPin, 1500);
    delay(200);
    noTone(buzzerPin);
    delay(200);
}

void alarmeLigando() {
    mostrarNoLCD("Alarme ligado", "Pelo btn interno");
    tone(buzzerPin, 1500);
    delay(300);
    noTone(buzzerPin);
}

void alarmeDesligando() {
    mostrarNoLCD("Alarme desligado", "Pelo btn interno");
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
}
