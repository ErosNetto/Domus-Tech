#include "alarme.h"
#include <Arduino.h>

int buttonPin;
int reedSwitchPin;
int buzzerPin;
bool alarmOn = false;

void setupAlarme(int bPin, int rPin, int buzzPin) {
    buttonPin = bPin;
    reedSwitchPin = rPin;
    buzzerPin = buzzPin;
    
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(reedSwitchPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    
    Serial.begin(115200);
}

void loopAlarme() {
    if (digitalRead(buttonPin) == LOW) {
        delay(200);
        alarmOn = !alarmOn;
        if (alarmOn) {
            Serial.println("Sistema armado.");
            alarmeLigando();
        } else {
            Serial.println("Sistema desarmado.");
            alarmeDesligando();
        }
        delay(500);
    }
    
    if (alarmOn && digitalRead(reedSwitchPin) == HIGH) {
        Serial.println("Intrusão detectada! Alarme ativado.");
        alarmeTocando();
    } else {
        noTone(buzzerPin);
    }
}

void alarmeTocando() {
    tone(buzzerPin, 2000);
    delay(300);
    noTone(buzzerPin);
    delay(300);
}

void alarmeLigando() {
    tone(buzzerPin, 2000);
    delay(300);
    noTone(buzzerPin);
}

void alarmeDesligando() {
    tone(buzzerPin, 2000);
    delay(150);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, 2000);
    delay(150);
    noTone(buzzerPin);
}
