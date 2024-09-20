#ifndef ALARME_H
#define ALARME_H

void setupAlarme(int buttonPin, int reedSwitchPin, int buzzerPin);
void loopAlarme();
void alarmeTocando();
void alarmeLigando();
void alarmeDesligando();

// Declaração da variável externa alarmOn
extern bool alarmOn;

// Declaração da função externa para mostrar no LCD
extern void mostrarNoLCD(const String& primeiraLinha, const String& segundaLinha);

#endif
