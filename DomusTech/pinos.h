#ifndef PINOS_H
#define PINOS_H

// Botões de configuração da ESP
const int btnResetWifi = 5;               // Botão de resetar o wifiManager da ESP
const int btnResetESP32 = 15;             // Botão de resetar a ESP

// Pinos do Alarme
const int btnAcionarAlarme = 14;          // Botão para ligar/desligar alarme
const int reedSwitchPin = 27;             // Sensor magnético (reed switch)
const int buzzerPin = 26;                 // Buzzer do alarme

// Pinos do Portão usando Servo Motor
const int botaoAbreFechaPortao = 32;      // Botão único para abrir/fechar portão
const int pinoServoPWM = 33;              // Fio laranja do motor

// Leds da casa
const int ledPin1 = 4;                    // Pino do LED 1
const int ledPin2 = 23;                   // Pino do LED 2
const int ledPin3 = 12;                   // Pino do LED 3

// Botões para ligar manualmente os leds   
const int btnLedPin1 = 18;                // Pino do botão para LED 1
const int btnLedPin2 = 19;                // Pino do botão para LED 2
const int btnLedPin3 = 13;                // Pino do botão para LED 3

#endif
