#include "wifi_config.h"

// Definições das variáveis globais
extern WiFiManager wm; // Inicialize o WiFiManager

// Configurações do IP estático
IPAddress local_IP(192, 168, 18, 65);  // IP fixo
IPAddress gateway(192, 168, 18, 1);     // Gateway na mesma sub-rede
IPAddress subnet(255, 255, 255, 0);     // Máscara de sub-rede padrão

// Inicia as configurações de Wi-Fi
void configurarWiFi() {
    // Verifica se há redes Wi-Fi salvas no WiFiManager
    if (hasSavedNetworks()) {
        Serial.println("Wi-Fi salvo encontrado. Tentando conectar...");
        mostrarNoLCD("Wi-Fi salvo", "Conectando...");
        delay(2000);

        configurarIPFixo(); // Configura o IP fixo antes de tentar conectar

        WiFi.begin(); // Tenta conectar ao Wi-Fi salvo

        // Aguarda até conectar ou até o timeout de 10 segundos
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
            yield();
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("Conectado ao Wi-Fi! " + WiFi.SSID());
            mostrarNoLCD("WiFi conectado!", WiFi.SSID());
            delay(3000);
            ipTexto();
        } else {
            Serial.println("Falha ao conectar ao Wi-Fi salvo.");
            mostrarNoLCD("Falha na conexao", "Reiniciando...");
            delay(3000);
            ESP.restart(); // Reinicia a ESP32
        }
    } else {
        Serial.println("Nenhuma rede Wi-Fi salva foi encontrada.");
        mostrarNoLCD("  Nenhuma rede  ", "  Wi-Fi  salva  ");
        delay(3000);
        iniciarModoConfiguracaoWiFi();
    }
}

// Função para verificar se há redes Wi-Fi salvas no WiFiManager
bool hasSavedNetworks() {
    WiFi.mode(WIFI_STA); // Garante que estamos no modo estação
    WiFi.begin();        // Inicia o WiFi com credenciais salvas, se houver
    delay(200);

    // Verifica se temos um SSID salvo
    return (WiFi.SSID() != "");
}

// Função para iniciar o modo de configuração de Wi-Fi
void iniciarModoConfiguracaoWiFi() {
    Serial.println("Entrando em modo de configuração Wi-Fi...");
    mostrarNoLCD("Entrando em modo", "de config Wi-Fi");
    delay(3000);

    mostrarNoLCD("Por favor, faca", "a configuracao");
    delay(500);
    
    // Inicia o WiFiManager para modo de configuração
    bool res = wm.autoConnect("Domus_Tech", "123123123");
    if (!res) {
        Serial.println("Falha na configuração do Wi-Fi");
        mostrarNoLCD("Falha Config", "Reiniciando...");
        delay(4000);
        ESP.restart();
    } else {
        Serial.println("Wi-Fi Configurado com sucesso! " + WiFi.SSID());
        mostrarNoLCD("WiFi conectado!", WiFi.SSID());
        delay(3000);
        configurarIPFixo();
        delay(500);
        ipTexto();
    }
}

// Função para configurar o IP fixo
void configurarIPFixo() {
    if (!WiFi.config(local_IP, gateway, subnet)) {
        delay(500);
        Serial.println("Falha ao configurar o IP estático.");
        mostrarNoLCD("Falha ao config", "o IP estatico");
    }
}

void ipTexto() {
    Serial.println("IP estático configurado: " + local_IP.toString());
    mostrarNoLCD("IP atribuido:", local_IP.toString());
}