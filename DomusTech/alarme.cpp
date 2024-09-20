#include <Arduino.h>
#include "alarme.h"

int btnAcionarAlarme;
int reedSwitchPin;
int buzzerPin;
bool alarmOn = false;
// teste
bool ligaDesliga = false;

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
    if (digitalRead(btnAcionarAlarme) == LOW || ligaDesliga == true) {
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

    // Verifica se o alarme foi ligado/desligado externamente (ligaDesliga)
    // if (ligaDesliga) {
    //     if (!alarmOn) {
    //         alarmOn = true;
    //         Serial.println("Sistema armado externamente.");
    //         alarmeLigando();
    //     }
    // } else {
    //     if (alarmOn) {
    //         alarmOn = false;
    //         Serial.println("Sistema desarmado externamente.");
    //         alarmeDesligando();
    //     }
    // }

    // Verifica o estado do sensor Reed Switch
    if (alarmOn && digitalRead(reedSwitchPin) == HIGH) {
        Serial.println("Intrusão detectada! Alarme ativado.");
        alarmeTocando();
    } else {
        noTone(buzzerPin);  // Desativa o som quando não há intrusão
    }
}

// teste
void ligaAlarme() {
    ligaDesliga = true;
}

void desligaAlarme() {
    ligaDesliga = false;
}


void alarmeTocando() {
    tone(buzzerPin, 1500);
    delay(200);
    noTone(buzzerPin);
    delay(200);
}

void alarmeLigando() {
    tone(buzzerPin, 1500);
    delay(300);
    noTone(buzzerPin);
}

void alarmeDesligando() {
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
}
