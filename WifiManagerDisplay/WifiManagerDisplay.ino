#include <WiFiManager.h> // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h> // Biblioteca do LCD

LiquidCrystal_I2C lcd(0x27, 16, 2); // Configura o LCD 16x2

void setup() {
    Serial.begin(115200);

    lcd.init(); // Inicializa o LCD
    lcd.backlight(); // Liga a luz de fundo do LCD
    
    WiFiManager wm;

    // Exibe mensagem de conexão no display
    lcd.setCursor(0, 0);
    lcd.print("Conectando");

    // Animação de "Conectando..." por 10 segundos
    for (int i = 0; i < 10; i++) {
        lcd.setCursor(10, 0);  // Posiciona o cursor após "Conectando"
        if (i % 3 == 0) {
            lcd.print(".  ");   // Um ponto, seguido por dois espaços para limpar o resto
        } else if (i % 3 == 1) {
            lcd.print(".. ");   // Dois pontos, seguido por um espaço
        } else if (i % 3 == 2) {
            lcd.print("...");   // Três pontos
        }
        delay(1000);            // Aguarda 1 segundo
    }

    lcd.clear(); // Limpa o display após 10 segundos

    // Tentativa de conexão automática
    bool res;
    res = wm.autoConnect("ESP32", "123123123");

    if (!res) {
        Serial.println("Falha Conexao");
        lcd.setCursor(0, 0);
        lcd.print("Falha Conexao");
        // ESP.restart(); // Reinicia em caso de falha, se necessário
    } else {
        Serial.println("Conectado!!!");
        
        // Exibe "Conectado!!!" na primeira linha do LCD
        lcd.setCursor(0, 0);
        lcd.print("Conectado!!!");

        // Exibe o SSID (nome da rede) na segunda linha do LCD com rolagem, se necessário
        scrollSSID(WiFi.SSID());
    }
}

void loop() {
    // Código principal que será executado repetidamente
}

// Função para rolar o nome do SSID no display
void scrollSSID(String ssid) {
    int len = ssid.length();
    if (len <= 16) {
        // Se o SSID couber na tela, exibe diretamente
        lcd.setCursor(0, 1);
        lcd.print(ssid);
    } else {
        // Caso contrário, faz rolar o nome
        for (int start = 0; start < len; start++) {
            String scrollText = ssid.substring(start);
            if (scrollText.length() < 16) {
                scrollText += ssid.substring(0, 16 - scrollText.length()); // Enrola o texto
            }
            lcd.setCursor(0, 1);
            lcd.print(scrollText);
            delay(500); // Ajusta a velocidade da rolagem
        }
    }
}
