#include <WiFiManager.h> // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h> // Biblioteca do LCD

LiquidCrystal_I2C lcd(0x27, 16, 2); // Configura o LCD 16x2

const int btnResetWifi = 5; // Pino do botão (GPIO 5)
const int btnResetESP32 = 15; // Pino do botão (GPIO 15)
WiFiManager wm; // Instância do WiFiManager

void setup() {
    Serial.begin(115200);

    // Configura o pino do botão como entrada com pull-up interno
    pinMode(btnResetWifi, INPUT_PULLUP);
    pinMode(btnResetESP32, INPUT_PULLUP);

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
        delay(500);

        WiFi.begin(); // Tenta conectar ao Wi-Fi salvo

        // Aguarda até conectar ou até o timeout de 10 segundos
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Conectado ao Wi-Fi!");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("WiFi conectado!");
            lcd.setCursor(0, 1);
            lcd.print(WiFi.SSID());
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

    if (WiFi.status() == WL_CONNECTED) {
        scrollSSID(WiFi.SSID());
    }
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
        Serial.println("Wi-Fi Configurado com sucesso!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WiFi conectado!");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.SSID());
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
