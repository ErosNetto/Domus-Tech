#include <WiFi.h>                         // Biblioteca do WiFi
#include <WiFiManager.h>                  // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h>            // Biblioteca do LCD
#include <ESPAsyncWebServer.h>            // Biblioteca para criar um servidor
#include <AsyncTCP.h>                     // Biblioteca para o AsyncWebServer
#include <ESP32Servo.h>                   // Biblioteca para o Servo Motor

// ----- CONEXÃO DOS PINOS DA ESP ----- //
// Botões de configuração da ESP
const int btnResetWifi = 5;               // Botão de resetar o wifiManager da ESP
const int btnResetESP32 = 15;             // Botão de resetar a ESP

// Pinos do Alarme
const int btnAcionarAlarme = 14;          // Botão para ligar/desligar alarme
const int reedSwitchPin = 27;             // Sensor magnético (reed switch)
const int buzzerPin = 26;                 // Buzzer do alarme

// Pinos do Portão usando Servo Motor
const int btnAbreFechaPortao = 25;        // Botão único para abrir/fechar portão
const int pinServoMotor = 33;             // Fio laranja do motor

// Leds da casa
const int ledPin1 = 4;                    // Pino do LED 1
const int ledPin2 = 23;                   // Pino do LED 2
const int ledPin3 = 12;                   // Pino do LED 3

// Botões para ligar manualmente os leds   
const int btnLedPin1 = 18;                // Pino do botão para LED 1
const int btnLedPin2 = 19;                // Pino do botão para LED 2
const int btnLedPin3 = 13;                // Pino do botão para LED 3

// Pinos do Display I2C (SDA = 21, SCL = 22)
LiquidCrystal_I2C lcd(0x27, 16, 2);       // Configura o LCD 16x2 com endereço I2C 0x27

// Configurações do IP estático
IPAddress local_IP(192, 168, 18, 85);     // IP fixo
IPAddress gateway(192, 168, 18, 1);       // Gateway na mesma sub-rede
IPAddress subnet(255, 255, 255, 0);       // Máscara de sub-rede padrão

WiFiManager wm;                           // Instância do WiFiManager
AsyncWebServer server(80);

// Variáveis globais
bool alarmeAtivo = false;                 // Variável para controlar se o alarme está ligado ou desligado
bool ledState[] = {0, 0, 0};

// Função para escrever no LCD I2C
void mostrarNoLCD(const String& primeiraLinha, const String& segundaLinha) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(primeiraLinha);
    lcd.setCursor(0, 1);
    lcd.print(segundaLinha);
}

void setup() {
    Serial.begin(115200);

    // Configura os pinos dos botões pra ESP como entrada e com pull-up interno
    pinMode(btnResetWifi, INPUT_PULLUP);
    pinMode(btnResetESP32, INPUT_PULLUP);

    // Configura os pinos de entrada e saida
    setupAlarme();
    setupPortao();
    setupLeds();

    // Inicialização
    lcd.init();                     // Inicializa o LCD
    lcd.backlight();                // Liga a luz de fundo do LCD
    mostrarNoLCD("   Domus Tech   ", "Carregando...");
    delay(500);

    // Inicia as configurações do Wi-Fi
    configurarWiFi();
    delay(500);

    // Rota para controlar LEDs via requisição HTTP
    server.on("/led", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("ledNum") && request->hasParam("state")) {
            int ledNumero = request->getParam("ledNum")->value().toInt();
            int ledStateRequest = request->getParam("state")->value().toInt();
            int pin;

            if (ledNumero == 1) {
                pin = ledPin1;
            } else if (ledNumero == 2) {
                pin = ledPin2;
            } else if (ledNumero == 3) {
                pin = ledPin3;
            } else {
                request->send(400, "text/plain", "Número do LED invalido.");
                return;
            }

            Serial.println("\nRecebendo o pino: " + String(pin));
            Serial.println("Recebendo o estado: " + String(ledStateRequest));
            
            digitalWrite(pin, ledStateRequest); // Liga ou desliga o LED
            ledState[ledNumero - 1] = ledStateRequest; // Atualiza o estado do LED no array

            String state = (ledStateRequest == HIGH) ? "ON" : "OFF";
            request->send(200, "text/plain", "LED " + String(ledNumero) + " está " + state);
        } else {
            request->send(400, "text/plain", "Missing parameters");
        }
    });

    // Rota para ligar e desligar o alarme
    server.on("/alarme", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int alarmState = request->getParam("state")->value().toInt();

            Serial.println("\nRecebendo o estado do alarme: " + String(alarmState));

            // Ativa ou desativa o alarme
            if (alarmState == 1) {
                alarmeAtivo = true;
                Serial.println("\nAlarme ligado remotamente.");
                mostrarNoLCD("Alarme ligado", "remotamente");
                alarmeLigando();
            } else if (alarmState == 0) {
                alarmeAtivo = false;
                Serial.println("\nAlarme desligado remotamente.");
                mostrarNoLCD("Alarme desligado", "remotamente");
                alarmeDesligando();
            }

            String state = (alarmeAtivo) ? "ligado" : "desligado";
            request->send(200, "text/plain", "O alarme está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'state' ausente");
        }
    });

    // Rota para obter o status dos dispositivos
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
        String status = "";
        status += "LED1: " + String(digitalRead(ledPin1)) + "\n";
        status += "LED2: " + String(digitalRead(ledPin2)) + "\n";
        status += "LED3: " + String(digitalRead(ledPin3)) + "\n";   
        status += (alarmeAtivo) ? "Alarme está ligado" : "Alarme está desligado";

        Serial.println("\nEnviando status.");
        request->send(200, "text/plain", status);
    });

    server.begin();
}

void loop() {
    loopAlarme();   // Controle do sistema de alarme
    loopPortao();   // Controle do sistema de portão 
    loopLeds();     // Controle do sistema de leds

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
        mostrarNoLCD("Reiniciando...", "");
        delay(3000);        
        ESP.restart();
    }
}





// ----- CONFIGURAÇÃO DE WI-FI ----- //
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

// Função para escrever o IP da ESP no terminal e no LCD 
void ipTexto() {
    Serial.println("IP estático configurado: " + local_IP.toString());
    mostrarNoLCD("IP atribuido:", local_IP.toString());
}




// ----- CONFIGURAÇÃO DOS LEDS ----- //
void setupLeds() {
    // Configura os pinos de saida para os Leds
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    pinMode(ledPin3, OUTPUT);

    // Configura os botões de ligação manual como entrada com pull-up interno
    pinMode(btnLedPin1, INPUT_PULLUP);
    pinMode(btnLedPin2, INPUT_PULLUP);
    pinMode(btnLedPin3, INPUT_PULLUP);

    // Inicializa os LEDs como desligados
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
}

void loopLeds() {
    // Checa o botão do LED 1
    if (digitalRead(btnLedPin1) == LOW) {
        delay(200); // Debounce
        
        ledState[0] = !ledState[0]; // Alterna o estado do LED
        digitalWrite(ledPin1, ledState[0]);

        if (ledState[0] == 1) {
            Serial.println("\nLED: 1 ligado internamente.");
            mostrarNoLCD("LED: 1 ligado", "internamente");
        } else {
            Serial.println("\nLED: 1 desligado internamente.");
            mostrarNoLCD("LED: 1 desligado", "internamente");
        }
    }

    // Checa o botão do LED 2
    if (digitalRead(btnLedPin2) == LOW) {
        delay(200); // Debounce

        ledState[1] = !ledState[1]; // Alterna o estado do LED
        digitalWrite(ledPin2, ledState[1]);

        if (ledState[1] == 1) {
            Serial.println("\nLED: 2 ligado internamente.");
            mostrarNoLCD("LED: 2 ligado", "internamente");
        } else {
            Serial.println("\nLED: 2 desligado internamente.");
            mostrarNoLCD("LED: 2 desligado", "internamente");
        }
    }

    // Checa o botão do LED 3
    if (digitalRead(btnLedPin3) == LOW) {
        delay(200); // Debounce

        ledState[2] = !ledState[2]; // Alterna o estado do LED
        digitalWrite(ledPin3, ledState[2]);

        if (ledState[2] == 1) {
            Serial.println("\nLED: 3 ligado internamente.");
            mostrarNoLCD("LED: 3 ligado", "internamente");
        } else {
            Serial.println("\nLED: 3 desligado internamente.");
            mostrarNoLCD("LED: 3 desligado", "internamente");
        }
    }
}




// ----- CONFIGURAÇÃO DO ALARME ----- //
void setupAlarme() {
    // Configura os pinos
    pinMode(btnAcionarAlarme, INPUT_PULLUP);        // Pino de entrada com pull-up interno
    pinMode(reedSwitchPin, INPUT_PULLUP);           // Pino de entrada com pull-up interno
    pinMode(buzzerPin, OUTPUT);                     // Pino de saida
}

// Função para atualizar o estado do alarme
void loopAlarme() {
    if (digitalRead(btnAcionarAlarme) == LOW) {
        delay(200);

        alarmeAtivo = !alarmeAtivo;  // Alterna o estado
        if (alarmeAtivo) {
            Serial.println("\nAlarme ligado internamente.");
            mostrarNoLCD("Alarme ligado", "pelo btn interno");
            alarmeLigando();
        } else {
            Serial.println("\nAlarme desligado internamente.");
            mostrarNoLCD("Alarme desligado", "pelo btn interno");
            alarmeDesligando();
        }
        delay(500);
    }

    // Verifica o estado do sensor Reed Switch
    if (alarmeAtivo && digitalRead(reedSwitchPin) == HIGH) {
        Serial.println("Intrusão detectada! Alarme ativado.");
        mostrarNoLCD("--- ATENCAO! ---", "Alarme dispadado");
        alarmeTocando();
    } else {
        noTone(buzzerPin);  // Desativa o som quando não há intrusão
    }
}

// Som de disparo do alarme 
void alarmeTocando() {
    tone(buzzerPin, 1500);
    delay(200);
    noTone(buzzerPin);
    delay(200);
}

// Som de ligar o alarme
void alarmeLigando() {
    tone(buzzerPin, 1500);
    delay(300);
    noTone(buzzerPin);
}

// Som de desligar o alarme
void alarmeDesligando() {
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
}



// ----- CONFIGURAÇÃO DO PORTÃO ----- //
// Definições dos pinos e variáveis
Servo servoMotor;               // Instância do servo
const int posicaoAberto = 85;    // Ângulo para abrir o portão
const int posicaoFechado = 0;    // Ângulo para fechar o portão
bool portaoAberto = false;      // Estado inicial do portão (fechado)
 
// int pinServo = 13; // Defina o pino do servo
void setupPortao() {  
    pinMode(btnAbreFechaPortao, INPUT_PULLUP); 
    servoMotor.attach(pinServoMotor);

    // Inicializa o portão como fechado
    servoMotor.write(posicaoFechado); // Portão inicia fechado
}

void loopPortao() {
    if (digitalRead(btnAbreFechaPortao) == LOW) {
        delay(200);  // Debounce para evitar leituras múltiplas rápidas
        
        // Alterna entre abrir e fechar o portão
        if (portaoAberto) {
            fecharPortao();
        } else {
            abrirPortao();
        }

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(btnAbreFechaPortao) == LOW) {
            delay(10);  // Espera o botão ser solto
        }
    }
}

void abrirPortao() {
  Serial.println("Abrindo o portão...");
  servoMotor.write(posicaoAberto);         // Move o servo para a posição aberta
  portaoAberto = true;                     // Atualiza o estado para aberto
  delay(1000);                             // Espera para garantir que o portão abra
}

void fecharPortao() {
    Serial.println("Fechando o portão...");
    servoMotor.write(posicaoFechado);     // Move o servo para a posição fechada
    portaoAberto = false;                 // Atualiza o estado para fechado
    delay(1000);                          // Espera para garantir que o portão feche
}