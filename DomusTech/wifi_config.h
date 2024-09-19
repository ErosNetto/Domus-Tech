#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <WiFi.h>
#include <WiFiManager.h>
#include <LiquidCrystal_I2C.h> // Para o display LCD

extern LiquidCrystal_I2C lcd; // Declaração do LCD para usar dentro deste arquivo
extern WiFiManager wm; // WiFiManager para gerenciar a conexão

// Inicia as configurações de Wi-Fi
void configurarWiFi() {
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
            yield();
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Conectado ao Wi-Fi! " + WiFi.SSID());
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("WiFi conectado!");
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
    // Configurações do IP estático
    IPAddress local_IP(192, 168, 18, 123);  // IP fixo
    IPAddress gateway(192, 168, 18, 1);     // Gateway na mesma sub-rede
    IPAddress subnet(255, 255, 255, 0);     // Máscara de sub-rede padrão

    if (!WiFi.config(local_IP, gateway, subnet)) {
        delay(3000);
        Serial.println("Falha ao configurar o IP estático.");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Falha ao config");
        lcd.setCursor(0, 1);
        lcd.print("o IP estatico");
    } else {
        delay(3000);
        Serial.println("IP estático configurado: " + local_IP.toString());
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("IP atribuido:");
        lcd.setCursor(0, 1);
        lcd.print(local_IP.toString());
    }
}

#endif
