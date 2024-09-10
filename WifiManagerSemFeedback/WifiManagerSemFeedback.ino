#include <WiFiManager.h> // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h> // Biblioteca do LCD

LiquidCrystal_I2C lcd(0x27, 16, 2); // Configura o LCD 16x2

const int btnResetWifi = 5; // Pino do botão (GPIO 5)
const int btnResetESP32 = 15; // Pino do botão (GPIO 15)
WiFiManager wm; // Instância do WiFiManager

void setup() {
    Serial.begin(115200);

    // Configura os pinos dos botões como entrada com pull-up interno
    pinMode(btnResetWifi, INPUT_PULLUP);
    pinMode(btnResetESP32, INPUT_PULLUP);

    lcd.init(); // Inicializa o LCD
    lcd.backlight(); // Liga a luz de fundo do LCD
    lcd.setCursor(0, 0);
    lcd.print("   Domus Tech   ");
    lcd.setCursor(0, 1);
    lcd.print("Carregando...");
    delay(2000);

    bool res = wm.autoConnect("Domus-Tech", "123123123");

    if(!res) {
        Serial.println("Nenhuma rede Wi-Fi salva ou falha ao conectar.");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("  Nenhuma rede  ");
        lcd.setCursor(0, 1);
        lcd.print("  Wi-Fi salva  ");
    } else {
        Serial.println("Conectado ao Wi-Fi salvo!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi conectado!");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.SSID());
    }
}

void loop() {
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
        ESP.restart(); // Reinicia o ESP32 para aplicar as mudanças
    }

    // Reinicia a ESP32
    if (digitalRead(btnResetESP32) == LOW) {
        Serial.println("Botão pressionado. Reiniciando...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Reiniciando...");
        delay(5000);        
        ESP.restart(); // Reinicia o ESP32
    }
}
