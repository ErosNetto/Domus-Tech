#ifndef ALARME_H
#define ALARME_H

void setupAlarme(int buttonPin, int reedSwitchPin, int buzzerPin);
void loopAlarme();
void alarmeTocando();
void alarmeLigando();
void alarmeDesligando();

#endif
