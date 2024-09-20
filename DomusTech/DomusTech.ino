#include <WiFi.h>                       // Biblioteca do WiFi
#include <WiFiManager.h>                // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h>          // Biblioteca do LCD
#include <ESPAsyncWebServer.h>          // Biblioteca para criar um servidor
#include <AsyncTCP.h>                   // Biblioteca para o AsyncWebServer

// Modularização
#include "wifi_config.h"                // Configurações do Wi-Fi
#include "pinos.h"                      // Configurações dos pinos de entrada e saida
#include "alarme.h"                     // Função do alarme
#include "portao.h"                     // Função do portao

extern void alarmeLigando();
extern void alarmeDesligando();

// Pinos do Display I2C (SDA = 21, SCL = 22)
LiquidCrystal_I2C lcd(0x27, 16, 2);     // Configura o LCD 16x2 com endereço I2C 0x27

WiFiManager wm;                         // Instância do WiFiManager
AsyncWebServer server(80);                      

void setup() {
    Serial.begin(115200);

    // Configura os pinos dos botões pra ESP como entrada e com pull-up interno
    pinMode(btnResetWifi, INPUT_PULLUP);
    pinMode(btnResetESP32, INPUT_PULLUP);

    // Configura os pinos de entrada e saida
    setupAlarme(btnAcionarAlarme, reedSwitchPin, buzzerPin);    // Parâmetros: btnAcionarAlarme, reedSwitchPin, buzzerPin
    setupPortao(botaoAbreFechaPortao, pinoServoPWM);            // Parâmetros: botao_unico, pinoServo

    // Configura o pino de saida para os Leds
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    pinMode(ledPin3, OUTPUT);

    // Configura os botões de ligação manual como entrada com pull-up interno
    pinMode(btnLedPin1, INPUT_PULLUP);
    pinMode(btnLedPin2, INPUT_PULLUP);
    pinMode(btnLedPin3, INPUT_PULLUP);

    // Inicialização
    lcd.init();                     // Inicializa o LCD
    lcd.backlight();                // Liga a luz de fundo do LCD
    mostrarNoLCD("   Domus Tech   ", "Carregando...");
    delay(500);

    digitalWrite(ledPin1, HIGH);    // Liga o LED
    digitalWrite(ledPin2, HIGH);    // Liga o LED
    digitalWrite(ledPin3, HIGH);    // Liga o LED
    delay(1000);
    digitalWrite(ledPin1, LOW);     // Desliga o LED
    digitalWrite(ledPin2, LOW);     // Desliga o LED
    digitalWrite(ledPin3, LOW);     // Desliga o LED
    delay(500);

    // Inicia as configurações do Wi-Fi
    configurarWiFi();
    delay(500);

    // Rota para controlar LEDs
    server.on("/led", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("pin") && request->hasParam("state")) {
            int pin = request->getParam("pin")->value().toInt();
            int ledState = request->getParam("state")->value().toInt();

            Serial.println("Recebendo o pino: " + String(pin));
            Serial.println("Recebendo o estado: " + String(ledState));
            
            pinMode(pin, OUTPUT); // Certifica que o pino está configurado como saída
            digitalWrite(pin, ledState); // Liga ou desliga o LED
            String state = (ledState == HIGH) ? "ON" : "OFF";
            request->send(200, "text/plain", "LED " + String(pin) + " is " + state);
        } else {
            request->send(400, "text/plain", "Missing parameters");
        }
    });

    server.on("/alarme", HTTP_GET, [&alarmOn](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int alarmState = request->getParam("state")->value().toInt();

            Serial.println("Recebendo o estado do alarme: " + String(alarmState));

            // Ativa ou desativa o alarme
            if (alarmState == 1) {
                alarmOn = true;  // Liga o alarme
                Serial.println("Alarme ativado remotamente.");
                alarmeLigando();
            } else if (alarmState == 0) {
                alarmOn = false; // Desliga o alarme
                Serial.println("Alarme desativado remotamente.");
                alarmeDesligando();
            }

            String state = (alarmOn) ? "ON" : "OFF";
            request->send(200, "text/plain", "Alarme está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'state' ausente");
        }
    });

    // Rota para obter o status dos dispositivos
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
        String status = "LED1: " + String(digitalRead(ledPin1)) + "\n";
        status += "LED2: " + String(digitalRead(ledPin2)) + "\n";
        status += "LED3: " + String(digitalRead(ledPin3)) + "\n";   
        status += (alarmOn) ? "Alarme está ligado" : "Alarme está desligado";

        request->send(200, "text/plain", status);
    });

    server.begin();
}

void loop() {
    loopAlarme();  // Controle do sistema de alarme
    loopPortao();  // Controle do sisetma de portão 

    // Verifica continuamente se o botão foi pressionado para resetar o Wi-Fi
    if (digitalRead(btnResetWifi) == LOW) {
        Serial.println("Botão pressionado. Resetando configurações de Wi-Fi...");
        mostrarNoLCD("Resetando Wi-Fi", "Reiniciando...");
        wm.resetSettings();
        delay(3000);        
        ESP.restart();
    }

    // Reinicia a ESP32
    if (digitalRead(btnResetESP32) == LOW) {
        Serial.println("Botão pressionado. Reiniciando...");
        mostrarNoLCD("Reiniciando...");
        delay(3000);        
        ESP.restart();
    }
}

void mostrarNoLCD(const String& primeiraLinha, const String& segundaLinha) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(primeiraLinha);
    lcd.setCursor(0, 1);
    lcd.print(segundaLinha);
}
