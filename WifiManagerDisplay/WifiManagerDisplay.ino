#include <WiFiManager.h> // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h> // Biblioteca do LCD

LiquidCrystal_I2C lcd(0x27, 16, 2); // Configura o LCD 16x2

const int buttonPin = 5; // Pino do botão (GPIO 0)

WiFiManager wm; // Cria uma instância do WiFiManager

void setup() {
    Serial.begin(115200);

    // Configura o pino do botão como entrada com pull-up interno
    pinMode(buttonPin, INPUT_PULLUP);

    lcd.init(); // Inicializa o LCD
    lcd.backlight(); // Liga a luz de fundo do LCD

    // Exibe mensagem de conexão no display
    lcd.setCursor(0, 0);
    lcd.print("Conectando...");
    delay(500);

    // Tenta se conectar ao Wi-Fi
    bool res = wm.autoConnect("ESP32", "123123123");
    if (!res) {
        Serial.println("Falha na conexão");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Falha Conexao");
        delay(3000);
        ESP.restart(); // Reinicia em caso de falha
    } else {
        Serial.println("Conectado!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Conectado!");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.SSID()); // Exibe o nome da rede conectada
    }
}

void loop() {
    // Verifica se o WiFiManager está no modo de configuração
    if (wm.getConfigPortalActive()) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Config Wi-Fi..."); // Mostra no LCD que está no modo de configuração
    }

    // Verifica continuamente se o botão foi pressionado para resetar o Wi-Fi
    if (digitalRead(buttonPin) == LOW) {
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

    // Exibe o SSID (nome da rede) na segunda linha do LCD com rolagem, se necessário
    scrollSSID(WiFi.SSID());
}

// Função para rolar o nome do SSID no display
void scrollSSID(String ssid) {
    int len = ssid.length();
    if (len <= 16) {
        // Se o SSID couber na tela, exibe diretamente
        lcd.setCursor(0, 1);
        lcd.print(ssid);
    } else {
        ssid += "   ";
        // Caso contrário, faz rolar o nome
        for (int start = 0; start < len + 3; start++) {
            String scrollText = ssid.substring(start);
            if (scrollText.length() < 16) {
                scrollText += ssid.substring(0, 16 - scrollText.length());
            }
            lcd.setCursor(0, 1);
            lcd.print(scrollText);
            delay(500);
        }
    }
}
