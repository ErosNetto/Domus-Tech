#include <WiFi.h>                         // Biblioteca do WiFi
#include <WiFiManager.h>                  // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h>            // Biblioteca do LCD
#include <ESPAsyncWebServer.h>            // Biblioteca para criar um servidor
#include <AsyncTCP.h>                     // Biblioteca para o AsyncWebServer
#include <ESP32Servo.h>                   // Biblioteca para o Servo Motor


// ----- CONEXÃO DOS PINOS DA ESP ----- //
// Botões de configuração da ESP
const int btnResetWifi = 5;               // Botão de resetar o wifiManager da ESP
const int btnResetESP32 = 32;             // Botão de resetar a ESP

// Pinos do Alarme
const int btnAcionarAlarme = 13;          // Botão para ligar/desligar alarme
const int reedSwitchPin = 27;             // Sensor magnético (reed switch)
const int pinSensorMovimento = 17;        // Pino do sensor movimento (PIR HC-SR501)
const int buzzerPin = 26;                 // Buzzer do alarme
const int ledAlarmeAtivado = 16;          // LED que indica que o alarme está ativado
const int ledPIRDetectado = 12;           // LED que acende quando o PIR detecta movimento

// Pinos do Portão usando Servo Motor
const int btnAbreFechaPortao = 15;        // Botão único para abrir/fechar portão
const int pinServoMotor = 33;             // Fio laranja do motor

// Leds da casa
const int ledPin1 = 4;                    // Pino do LED 1
const int ledPin2 = 25;                   // Pino do LED 2
const int ledPin3 = 23;                   // Pino do LED 3

// Botões para ligar manualmente os leds   
const int btnLedPin1 = 14;                // Pino do botão para LED 1
const int btnLedPin2 = 19;                // Pino do botão para LED 2
const int btnLedPin3 = 18;                // Pino do botão para LED 3

// Pinos do Display I2C (SDA = 21, SCL = 22)
LiquidCrystal_I2C lcd(0x27, 16, 2);       // Configura o LCD 16x2 com endereço I2C 0x27

// Configurações do IP estático
IPAddress local_IP(192, 168, 18, 85);     // IP fixo
IPAddress gateway(192, 168, 18, 1);       // Gateway na mesma sub-rede
IPAddress subnet(255, 255, 255, 0);       // Máscara de sub-rede padrão

WiFiManager wm;                           // Instância do WiFiManager
AsyncWebServer server(80);

// Variáveis globais
bool ledState[] = {0, 0, 0};              // Variável para controlar se os leds estão ligados ou desligados
bool alarmeAtivo = false;                 // Variável para controlar se o alarme está ligado ou desligado
bool portaoAberto = false;                // Variável para controlar se o portão está aberto ou fechado

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




    // ----- CONFIGURAÇÃO para as requisições HTTP ----- //
    // Rota para controlar LEDs
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

            Serial.println("LED " + String(ledNumero) + (ledStateRequest == HIGH) ? ": ON" : ": OFF");
            
            digitalWrite(pin, ledStateRequest); // Liga ou desliga o LED
            ledState[ledNumero - 1] = ledStateRequest; // Atualiza o estado do LED no array

            String state = (ledStateRequest == HIGH) ? "ON" : "OFF";
            request->send(200, "text/plain", "LED " + String(ledNumero) + " está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'ledNum' e 'state' ausentes");
        }
    });

    // Rota para ligar e desligar o alarme
    server.on("/alarme", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int alarmState = request->getParam("state")->value().toInt();

            // Ativa ou desativa o alarme
            if (alarmState == 1 && !alarmeAtivo) {
                alarmeAtivo = true;
                Serial.println("\nAlarme ligado remotamente.");
                mostrarNoLCD("Alarme ligado", "remotamente");
                delay(1000);
                alarmeLigando();
                delay(1000);
            } else if (alarmState == 0 && alarmeAtivo) {
                alarmeAtivo = false;
                Serial.println("\nAlarme desligado remotamente.");
                mostrarNoLCD("Alarme desligado", "remotamente");
                alarmeDesligando();
                delay(1000);
            } else {
                request->send(400, "text/plain", "Ação inválida.");
                return;  // Sai da função após enviar a resposta
            }

            String state = alarmeAtivo ? "ligado" : "desligado";
            request->send(200, "text/plain", "O alarme está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'state' ausente");
        }
    });

    // Rota para controlar o portão
    server.on("/portao", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int portaoState = request->getParam("state")->value().toInt();

            if (portaoState == 1 && portaoAberto == false) {
                abrirPortao();
                Serial.println("\nPortão aberto remotamente.");
                mostrarNoLCD("Portão aberto", "remotamente");
            } else if (portaoState == 0 && portaoAberto == true) {
                fecharPortao();
                Serial.println("\nPortao fechado remotamente.");
                mostrarNoLCD("Portao fechado", "remotamente");
            } else {
                request->send(400, "text/plain", "Ação inválida.");
            }

            String state = (portaoState) ? "aberto." : "fechado.";
            request->send(200, "text/plain", "O portão está " + state);
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
        status += (alarmeAtivo) ? "Alarme está ligado\n" : "Alarme está desligado\n";
        status += (portaoAberto) ? "Portão está aberto" : "Portão está fechado";

        Serial.println("\nEnviando status.");
        request->send(200, "text/plain", status);
    });

    server.begin();
}

void loop() {
    loopLeds();     // Controle do sistema de leds
    loopAlarme();   // Controle do sistema de alarme
    loopPortao();   // Controle do sistema de portão 

    // Verifica continuamente se o botão foi pressionado para resetar o Wi-Fi
    if (digitalRead(btnResetWifi) == LOW) {
        Serial.println("\nBotão pressionado. Resetando configurações de Wi-Fi...");
        mostrarNoLCD("Resetando Wi-Fi", "Reiniciando...");
        wm.resetSettings();
        delay(3000);        
        ESP.restart();
    }

    // Reinicia a ESP32
    if (digitalRead(btnResetESP32) == LOW) {
        Serial.println("\nBotão pressionado. Reiniciando...");
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
        ledState[0] = !ledState[0];             // Alterna o estado do LED
        digitalWrite(ledPin1, ledState[0]);     // Liga ou desliga o led

        if (ledState[0] == 1) {
            Serial.println("\nLED: 1 ligado internamente.");
            mostrarNoLCD("LED: 1 ligado", "internamente");
        } else {
            Serial.println("\nLED: 1 desligado internamente.");
            mostrarNoLCD("LED: 1 desligado", "internamente");
        }
        delay(200); // Debounce
    }

    // Checa o botão do LED 2
    if (digitalRead(btnLedPin2) == LOW) {
        ledState[1] = !ledState[1];             // Alterna o estado do LED
        digitalWrite(ledPin2, ledState[1]);     // Liga ou desliga o led

        if (ledState[1] == 1) {
            Serial.println("\nLED: 2 ligado internamente.");
            mostrarNoLCD("LED: 2 ligado", "internamente");
        } else {
            Serial.println("\nLED: 2 desligado internamente.");
            mostrarNoLCD("LED: 2 desligado", "internamente");
        }
        delay(200); // Debounce
    }

    // Checa o botão do LED 3
    if (digitalRead(btnLedPin3) == LOW) {
        ledState[2] = !ledState[2];             // Alterna o estado do LED
        digitalWrite(ledPin3, ledState[2]);     // Liga ou desliga o led

        if (ledState[2] == 1) {
            Serial.println("\nLED: 3 ligado internamente.");
            mostrarNoLCD("LED: 3 ligado", "internamente");
        } else {
            Serial.println("\nLED: 3 desligado internamente.");
            mostrarNoLCD("LED: 3 desligado", "internamente");
        }
        delay(200); // Debounce
    }
}




// ----- CONFIGURAÇÃO DO ALARME ----- //
void setupAlarme() {
    // Configura os pinos
    pinMode(btnAcionarAlarme, INPUT_PULLUP);        // Pino de entrada com pull-up interno
    pinMode(reedSwitchPin, INPUT_PULLUP);           // Pino de entrada com pull-up interno
    pinMode(buzzerPin, OUTPUT);                     // Pino de saida
    pinMode(pinSensorMovimento, INPUT);             // Pino do sensor PIR
    pinMode(ledAlarmeAtivado, OUTPUT);              // LED do alarme ativado
    pinMode(ledPIRDetectado, OUTPUT);               // LED do sensor PIR
}

// Função para atualizar o estado do alarme
void loopAlarme() {
    // Verifica se o botão de acionamento do alarme foi pressionado
    if (digitalRead(btnAcionarAlarme) == LOW) {
        delay(200);  // Debounce para evitar leituras múltiplas rápidas

        alarmeAtivo = !alarmeAtivo;  // Alterna o estado do alarme
        Serial.println(alarmeAtivo ? "\nAlarme ligado internamente." : "\nAlarme desligado internamente.");
        mostrarNoLCD(alarmeAtivo ? "Alarme ligado" : "Alarme desligado", "internamente.");
        alarmeAtivo ? alarmeLigando() : alarmeDesligando();

        // Liga ou desliga o LED que indica se alarme está ativado
        digitalWrite(ledAlarmeAtivado, alarmeAtivo ? HIGH : LOW);

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(btnAcionarAlarme) == LOW) {
            delay(10);
        }
    }

    // Verifica o estado do sensor Reed Switch (portão/porta)
    if (alarmeAtivo && digitalRead(reedSwitchPin) == HIGH) {
        Serial.println("Sensor de portão ativado! Alarme disparado.");
        mostrarNoLCD("--- ATENCAO! ---", "Alarme disparado");
        alarmeTocando();
    } else {
        noTone(buzzerPin);
    }

    // Verifica o estado do sensor de movimento
    if (alarmeAtivo && digitalRead(pinSensorMovimento) == HIGH) {
        Serial.println("Movimento detectado pelo sensor PIR! Alarme disparado.");
        mostrarNoLCD("--- ATENCAO! ---", "Alarme disparado");
        digitalWrite(ledPIRDetectado, HIGH);            // Liga o LED quando o PIR detecta movimento
        alarmeTocando();                                // Opcional: você pode disparar o alarme também
    } else {
        digitalWrite(ledPIRDetectado, LOW);             // Desliga o LED se não houver detecção de movimento
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
Servo servoMotor;                // Instância do servo
const int posicaoAberto = 85;    // Ângulo para abrir o portão
const int posicaoFechado = 0;    // Ângulo para fechar o portão

void setupPortao() {  
    pinMode(btnAbreFechaPortao, INPUT_PULLUP);
    pinMode(pinServoMotor, OUTPUT);
    // fecharPortao();
}

// Função para atualizar o estado do portão
void loopPortao() {
    if (digitalRead(btnAbreFechaPortao) == LOW) {
        delay(200);  // Debounce para evitar leituras múltiplas rápidas
        
        // Alterna entre abrir e fechar o portão
        if (portaoAberto) {
            fecharPortao();
        } else {
            abrirPortao();
        }

        Serial.println(portaoAberto ? "\nPortão aberto internamente." : "\nPortão fechado internamente.");
        mostrarNoLCD(portaoAberto ? "Portao aberto" : "Portao fechado", "internamente.");

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(btnAbreFechaPortao) == LOW) {
            delay(10);
        }
    }
}

// Função para abrir o portão
void abrirPortao() {
    servoMotor.attach(pinServoMotor);
    for (int pos = posicaoFechado; pos <= posicaoAberto; pos++) {
        servoMotor.write(pos);          // Move o servo para a posição aberta
        delay(5);                       // Pequeno delay para suavizar o movimento
    }
    portaoAberto = true;                // Atualiza o estado para aberto
    delay(500);
    servoMotor.detach();                // Desligar o controle do servo motor 
}

// Função para fechar o portão
void fecharPortao() {
    servoMotor.attach(pinServoMotor);
    for (int pos = posicaoAberto; pos >= posicaoFechado; pos--) {
        servoMotor.write(pos);          // Move o servo para a posição fechada
        delay(5);                       // Pequeno delay para suavizar o movimento
    }
    portaoAberto = false;               // Atualiza o estado para fechado
    delay(500);
    servoMotor.detach();                // Desligar o controle do servo motor 
}
