#include <WiFi.h>                       // Biblioteca do WiFi
#include <WiFiManager.h>                // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h>          // Biblioteca do LCD
#include <ESPAsyncWebServer.h>          // Biblioteca para criar um servidor
#include <AsyncTCP.h>                   // Biblioteca para o AsyncWebServer

// Modularização
#include "alarme.h"                     // Função alarme
#include "portao.h"                     // Função portao

// Pinos do Display I2C (SDA = 21, SCL = 22)
// Estes pinos ficam reservados para o I2C e não devem ser alterados
LiquidCrystal_I2C lcd(0x27, 16, 2);     // Configura o LCD 16x2 com endereço I2C 0x27

WiFiManager wm;                         // Instância do WiFiManager
AsyncWebServer server(80);                      

// Configurações do IP estático
IPAddress local_IP(192, 168, 18, 123);  // IP fixo desejado
IPAddress gateway(192, 168, 18, 1);     // Gateway na mesma sub-rede
IPAddress subnet(255, 255, 255, 0);     // Máscara de sub-rede padrão

// Botões de configuração da ESP
const int btnResetWifi = 5;             // Botão de resetar o wifiManager da ESP
const int btnResetESP32 = 15;           // Botão de resetar a ESP

// Pinos do Alarme
const int buttonPin = 14;               // Botão para ligar/desligar alarme
const int reedSwitchPin = 27;           // Sensor magnético (reed switch)
const int buzzerPin = 26;               // Buzzer do alarme

// Pinos do Portão
const int in1 = 32;                     // Pino IN1 da Ponte H
const int in2 = 33;                     // Pino IN2 da Ponte H
const int ena = 25;                     // Pino ENA da Ponte H (PWM para controle de velocidade)
const int limite_aberto = 34;           // Fim de curso do portão aberto
const int limite_fechado = 35;          // Fim de curso do portão fechado
const int botao_unico = 23;             // Botão único para abrir/parar/fechar portão

// Leds da casa
const int ledPin1 = 4;                  // Pino do LED 1
const int ledPin2 = 2;                  // Pino do LED 2
const int ledPin3 = 12;                 // Pino do LED 3

// Botões para ligar manualmente os leds
const int btnLedPin1 = 18;              // Pino do botão para LED 1
const int btnLedPin2 = 19;              // Pino do botão para LED 2
const int btnLedPin3 = 13;              // Pino do botão para LED 3

void setup() {
    Serial.begin(115200);

    // Configura o pino do botão como entrada com pull-up interno
    pinMode(btnResetWifi, INPUT_PULLUP);
    pinMode(btnResetESP32, INPUT_PULLUP);

    // Configura os pinos de entrada e saida
    setupAlarme(buttonPin, reedSwitchPin, buzzerPin);  // Parâmetros: buttonPin, reedSwitchPin, buzzerPin
    setupPortao(in1, in2, ena, limite_aberto, limite_fechado, botao_unico);  // Parâmetros: in1, in2, ena, limite_aberto, limite_fechado, botao_unico

    // Configura o pino de saida para os Leds
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    pinMode(ledPin3, OUTPUT);

    // Configura os botões de ligação manual como entrada com pull-up interno
    pinMode(btnLedPin1, INPUT_PULLUP);
    pinMode(btnLedPin2, INPUT_PULLUP);
    pinMode(btnLedPin3, INPUT_PULLUP);

    // Inicialização
    digitalWrite(ledPin1, HIGH); // Liga o LED
    digitalWrite(ledPin2, HIGH); // Liga o LED
    digitalWrite(ledPin3, HIGH); // Liga o LED
    delay(1000);
    digitalWrite(ledPin1, LOW);  // Desliga o LED
    digitalWrite(ledPin2, LOW);  // Desliga o LED
    digitalWrite(ledPin3, LOW);  // Desliga o LED
    delay(1000);

    lcd.init(); // Inicializa o LCD
    lcd.backlight(); // Liga a luz de fundo do LCD
    lcd.setCursor(0, 0);
    lcd.print("   Domus Tech   ");
    lcd.setCursor(0, 1);
    lcd.print("Carregando...");
    delay(2000);

    // Verifica se há redes Wi-Fi salvas no WiFiManager
    if (hasSavedNetworks()) {
        Serial.println("Wi-Fi salvo encontrado. Tentando conectar...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi salvo");
        lcd.setCursor(0, 1);
        lcd.print("Conectando...");
        delay(2000);

        WiFi.begin(); // Tenta conectar ao Wi-Fi salvo

        // Aguarda até conectar ou até o timeout de 10 segundos
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Conectado ao Wi-Fi! " + WiFi.SSID());
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("WiFi conectado!1");
            lcd.setCursor(0, 1);
            lcd.print(WiFi.SSID());
            delay(500);
            configurarIPFixo();
        } else {
            Serial.println("Falha ao conectar ao Wi-Fi salvo.");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Falha na conexao");
            delay(3000);
            lcd.setCursor(0, 1);
            lcd.print("Reiniciando...");
            delay(4000);
            ESP.restart(); // Reinicia a ESP32
        }
    } else {
        Serial.println("Nenhuma rede Wi-Fi salva foi encontrada.");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("  Nenhuma rede  ");
        lcd.setCursor(0, 1);
        lcd.print("  Wi-Fi  salva  ");
        delay(4000);
        iniciarModoConfiguracaoWiFi();
    }

    // TESTE
    // Rota para controlar LEDs
    server.on("/led", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("pin") && request->hasParam("state")) {
            int pin = request->getParam("pin")->value().toInt();
            int ledState = request->getParam("state")->value().toInt();

            Serial.println("Received pin: " + String(pin));
            Serial.println("Received state: " + String(ledState));
            
            pinMode(pin, OUTPUT); // Certifica que o pino está configurado como saída
            digitalWrite(pin, ledState); // Liga ou desliga o LED
            String state = (ledState == HIGH) ? "ON" : "OFF";
            request->send(200, "text/plain", "LED " + String(pin) + " is " + state);
        } else {
            request->send(400, "text/plain", "Missing parameters");
        }
    });

    // Rota para obter o status dos LEDs
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
        String status = "LED1: " + String(digitalRead(ledPin1));
        request->send(200, "text/plain", status);
    });

    server.begin();
}

void loop() {
    loopAlarme();  // Controle do sistema de alarme
    loopPortao();  // Controle do sistema de portão

    // Verifica continuamente se o botão foi pressionado para resetar o Wi-Fi
    if (digitalRead(btnResetWifi) == LOW) {
        Serial.println("Botão pressionado. Resetando configurações de Wi-Fi...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Resetando Wi-Fi");
        lcd.setCursor(0, 1);
        lcd.print("Reiniciando...");
        wm.resetSettings(); // Reseta as configurações de Wi-Fi salvas
        delay(3000);        
        ESP.restart();
    }

    // Reinicia a ESP32
    if (digitalRead(btnResetESP32) == LOW) {
        Serial.println("Botão pressionado. Reiniciando...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Reiniciando...");
        delay(3000);        
        ESP.restart();
    }

    // if (WiFi.status() == WL_CONNECTED) {
    //     scrollSSID(WiFi.SSID());
    // }
}

// Função para verificar se há redes Wi-Fi salvas no WiFiManager
bool hasSavedNetworks() {
    WiFi.mode(WIFI_STA); // Garante que estamos no modo estação
    WiFi.begin();        // Inicia o WiFi com credenciais salvas, se houver

    delay(500);
        
    // Verifica se temos um SSID salvo
    if (WiFi.SSID() != "") {
        return true;
    } else {
        return false;
    }
}

// Função para iniciar o modo de configuração de Wi-Fi
void iniciarModoConfiguracaoWiFi() {
    Serial.println("Entrando em modo de configuração Wi-Fi...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Entrando em modo");
    lcd.setCursor(0, 1);
    lcd.print("de config Wi-Fi");
    delay(4000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Por favor, faca");
    lcd.setCursor(0, 1);
    lcd.print("a configuracao");
    delay(500);
    
    // Inicia o WiFiManager para modo de configuração
    bool res = wm.autoConnect("Domus_Tech", "123123123");
    if (!res) {
        Serial.println("Falha na configuração do Wi-Fi");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Falha Config");
        delay(3000);
        lcd.setCursor(0, 1);
        lcd.print("Reiniciando...");
        delay(4000);
        ESP.restart();
    } else {
        Serial.println("Wi-Fi Configurado com sucesso! " + WiFi.SSID());
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi conectado!2");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.SSID());
        configurarIPFixo();
    }
}

// Função para configurar o IP fixo
void configurarIPFixo() {
    if (!WiFi.config(local_IP, gateway, subnet)) {
        delay(4000);
        Serial.println("Falha ao configurar o IP estático.");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Falha ao config");
        lcd.setCursor(0, 1);
        lcd.print("o IP estatico");
    } else {
        delay(4000);
        Serial.println("IP estático configurado: " + local_IP.toString());
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("IP atribuido:");
        lcd.setCursor(0, 1);
        lcd.print(local_IP.toString());
    }
}

// Função para rolar o nome do SSID no display
void scrollSSID(String ssid) {
    int len = ssid.length();
    // Se o SSID couber na tela, exibe diretamente
    if (len <= 16) {
        lcd.setCursor(0, 1);
        lcd.print(ssid);
    } else {
        // Exibe o SSID parado por 3 segundos antes de iniciar a rolagem
        lcd.setCursor(0, 1);
        lcd.print(ssid.substring(0, 16));
        delay(3000);
        
        ssid += "   ";
        for (int start = 0; start < len + 3; start++) {
            String scrollText = ssid.substring(start);
            if (scrollText.length() < 16) {
                scrollText += ssid.substring(0, 16 - scrollText.length());
            }
            lcd.setCursor(0, 1);
            lcd.print(scrollText);
            delay(300);
        }
    }
}