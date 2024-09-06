#include <WiFiManager.h> // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h> // Biblioteca do LCD

LiquidCrystal_I2C lcd(0x27, 16, 2); // Configura o LCD 16x2

const int buttonPin = 5; // Pino do botão (GPIO 5)

WiFiManager wm; // Cria uma instância do WiFiManager

void setup() {
    Serial.begin(115200);

    // Configura o pino do botão como entrada com pull-up interno
    pinMode(buttonPin, INPUT_PULLUP);

    lcd.init(); // Inicializa o LCD
    lcd.backlight(); // Liga a luz de fundo do LCD

    // Exibe mensagem de configuração no display antes de iniciar a conexão
    lcd.setCursor(0, 0);
    lcd.print("Aguardando...");
    delay(500); // Aguarda para garantir que a mensagem seja visível

    // Inicia o modo de configuração se necessário
    bool res = wm.autoConnect("ESP32", "123123123");
    if (!res) {
        // Caso o modo de configuração esteja ativo, atualize o display
        if (wm.getConfigPortalActive()) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Config Wi-Fi...");
            // Aguarda um pouco para garantir que a mensagem seja visível
            delay(1000);
        } else {
            // Caso contrário, exibe mensagem de falha na conexão
            Serial.println("Falha na conexão");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Falha Conexao");
            delay(3000);
            ESP.restart(); // Reinicia em caso de falha
        }
    } else {
        // Caso contrário, exibe mensagem de conexão
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
        delay(1000);
    } else {
        // Exibe o SSID (nome da rede) na segunda linha do LCD com rolagem, se necessário
        if (WiFi.status() == WL_CONNECTED) {
            scrollSSID(WiFi.SSID());
        }
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
