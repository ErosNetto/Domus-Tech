#include <WiFi.h>                         // Biblioteca do WiFi
#include <WiFiManager.h>                  // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h>            // Biblioteca do LCD
#include <ESPAsyncWebServer.h>            // Biblioteca para criar um servidor
#include <AsyncTCP.h>                     // Biblioteca para o AsyncWebServer
#include <ESP32Servo.h>                   // Biblioteca para o Servo Motor


// ------------- CONEXÃO DOS PINOS DA ESP ------------- //
// ------------- Botões de configuração da ESP (entrada) ------------- 
const int btnResetWifi = 23;              // GPIO 23 - Botão de resetar o wifiManager (INPUT)
const int btnResetESP32 = 5;              // GPIO 5 - Botão de resetar a ESP (INPUT)

// ------------- Pinos do Alarme ------------- 
const int btnAcionarAlarme = 15;          // GPIO 15 - Botão para ligar/desligar alarme (INPUT)
const int reedSwitchPin = 14;             // GPIO 14 - Sensor magnético (reed switch) (INPUT)
const int pinSensorMovimento = 13;        // GPIO 13 - Sensor de movimento (PIR HC-SR501) (INPUT)
const int buzzerPin = 27;                 // GPIO 27 - Buzzer do alarme (OUTPUT)
const int ledAlarmeLigado = 12;           // GPIO 12 - LED que indica que o alarme está ativado (OUTPUT)
const int ledSensorPin = 26;           // GPIO 12 - LED que indica que o alarme está ativado (OUTPUT)

// ------------- Pinos do Portão usando Servo Motor ------------- 
const int btnAbreFechaPortao = 4;         // GPIO 4 - Botão único para abrir/fechar portão (INPUT)
const int pinServoMotor = 33;             // GPIO 33 - Pino para o servo motor (OUTPUT)

// ------------- Leds da casa (saida) ------------- 
const int ledPin1 = 18;                   // GPIO 18 - Pino do LED 1 (OUTPUT)
const int ledPin2 = 19;                   // GPIO 19 - Pino do LED 2 (OUTPUT)
const int ledPin3 = 25;                   // GPIO 25 - Pino do LED 3 (OUTPUT)

// ------------- Botões para ligar manualmente os LEDs (entrada) ------------- 
const int btnLedPin1 = 35;                // GPIO 35 - Pino do botão para LED 1 (INPUT)
const int btnLedPin2 = 32;                // GPIO 32 - Pino do botão para LED 2 (INPUT)
const int btnLedPin3 = 34;                // GPIO 34 - Pino do botão para LED 3 (INPUT)

// ------------- Pinos do Display I2C (SDA = 21, SCL = 22) ------------- 
LiquidCrystal_I2C lcd(0x27, 16, 2);       // Configura o LCD 16x2 com endereço I2C 0x27

// ------------- Configurações do IP estático -------------
IPAddress local_IP(192, 168, 18, 85);     // IP fixo
IPAddress gateway(192, 168, 18, 1);       // Gateway na mesma sub-rede
IPAddress subnet(255, 255, 255, 0);       // Máscara de sub-rede padrão

AsyncWebServer server(80);
WiFiManager wm;                           // Instância do WiFiManager

// ------------- Variáveis globais -------------
bool ledState[] = {0, 0, 0};              // Variável para controlar se os leds estão ligados ou desligados
bool alarmeLigado = false;                // Variável para controlar se o alarme está ligado ou desligado
bool alarmeAtivo = false;                 // Variável para controlar se o alarme foi ativado ou não
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
    setupLeds();
    setupAlarme();
    setupPortao();

    // Inicialização
    lcd.init();                     // Inicializa o LCD
    lcd.backlight();                // Liga a luz de fundo do LCD
    mostrarNoLCD("   Domus Tech   ", "Carregando...");
    delay(500);

    // Inicia as configurações do Wi-Fi
    configurarWiFi();
    delay(500);




    // ------------- CONFIGURAÇÃO para as requisições HTTP ------------- //
    // ------------- Rota para controlar LEDs ( http://192.168.18.85/led?ledNum=1&state=1 ) -------------
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

            Serial.println("LED " + String(ledNumero) + ((ledStateRequest == HIGH) ? ": ON" : ": OFF"));
            
            digitalWrite(pin, ledStateRequest); // Liga ou desliga o LED
            ledState[ledNumero - 1] = ledStateRequest; // Atualiza o estado do LED no array

            String state = (ledStateRequest == HIGH) ? "ON" : "OFF";
            request->send(200, "text/plain", "LED " + String(ledNumero) + " esta " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'ledNum' ou 'state' ausentes");
        }
    });

    // ------------- Rota para ligar e desligar o alarme ( http://192.168.18.85/alarme?state=1 ) -------------
    server.on("/alarme", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int alarmeState = request->getParam("state")->value().toInt();

            // Ativa ou desativa o alarme
            if (alarmeState == 1 && !alarmeLigado) {
                alarmeLigado = true;
                alarmeAtivo = false;
                mostrarNoLCD("Alarme ligado", "remotamente");
                somAlarmeLigando();
                Serial.println("\nAlarme ligado remotamente.");
            } else if (alarmeState == 0 && alarmeLigado) {
                alarmeLigado = false;
                alarmeAtivo = false; 
                mostrarNoLCD("Alarme desligado", "remotamente");
                pararSomAlarme();
                somAlarmeDesligando();
                pararSomAlarme();
                Serial.println("\nAlarme desligado remotamente.");
            } else {
                request->send(400, "text/plain", "Ação inválida.");
                return;  // Sai da função após enviar a resposta
            }

            digitalWrite(ledAlarmeLigado, alarmeLigado ? HIGH : LOW);

            String state = alarmeLigado ? "ligado" : "desligado";
            request->send(200, "text/plain", "O alarme está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'state' ausente");
        }
    });

    // ------------- Rota para controlar o portão ( http://192.168.18.85/portao?state=1 ) -------------
    server.on("/portao", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int portaoState = request->getParam("state")->value().toInt();

            if (portaoState == 1 && portaoAberto == false) {
                abrirPortao();
                Serial.println("\nPortão aberto remotamente.");
                mostrarNoLCD("Portao aberto", "remotamente");
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

    // ------------- Rota para obter o status dos dispositivos ( http://192.168.18.85/status ) -------------
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String status = "";
        status += "LED1: " + String(digitalRead(ledPin1)) + "\n";
        status += "LED2: " + String(digitalRead(ledPin2)) + "\n";
        status += "LED3: " + String(digitalRead(ledPin3)) + "\n";   
        status += (alarmeLigado) ? "Alarme esta ligado\n" : "Alarme esta desligado\n";
        status += (portaoAberto) ? "Portao esta aberto\n" : "Portao esta fechado\n";
        status += (alarmeAtivo) ? "Alarme disparado" : "Alarme nao esta disparado";

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




// ------------- CONFIGURAÇÃO DE WI-FI ------------- //
// Inicia as configurações de Wi-Fi
void configurarWiFi() {
    // Verifica se há redes Wi-Fi salvas no WiFiManager
    if (hasSavedNetworks()) {
        Serial.println("Wi-Fi salvo encontrado. Tentando conectar...");
        mostrarNoLCD("Wi-Fi salvo", "Conectando...");
        delay(2000);

        // configurarIPFixo(); // Configura o IP fixo antes de tentar conectar

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
        // configurarIPFixo();
        delay(500);
        ipTexto();
    }
}

// Função para configurar o IP fixo
// void configurarIPFixo() {
//     if (!WiFi.config(local_IP, gateway, subnet)) {
//         delay(500);
//         Serial.println("Falha ao configurar o IP estático.");
//         mostrarNoLCD("Falha ao config", "o IP estatico");
//     }
// }

// Função para escrever o IP da ESP no terminal e no LCD 
// void ipTexto() {
//     Serial.println("IP estático configurado: " + local_IP.toString());
//     mostrarNoLCD("IP atribuido:", local_IP.toString());
// }

void ipTexto() {
    // Exibe o IP atribuído pela ESP diretamente
    Serial.println("IP atribuído pela ESP: " + WiFi.localIP().toString());

    // Exibe o IP no LCD diretamente
    mostrarNoLCD("IP atribuido:", WiFi.localIP().toString());
}





// ------------- CONFIGURAÇÃO DOS LEDS ------------- //
void setupLeds() {
    // Configura os pinos de saida para os Leds
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    pinMode(ledPin3, OUTPUT);

    // Configura os botões de ligação manual como entrada com pull-up interno
    pinMode(btnLedPin1, INPUT);
    pinMode(btnLedPin2, INPUT);
    pinMode(btnLedPin3, INPUT);

    // Inicializa os LEDs como desligados
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    delay(500);
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




// ------------- CONFIGURAÇÃO DO ALARME ------------- //
void setupAlarme() {
    // Configura os pinos do alarme
    pinMode(btnAcionarAlarme, INPUT_PULLUP);    // Pino de entrada com pull-up interno
    pinMode(reedSwitchPin, INPUT_PULLUP);       // Pino de entrada com pull-up interno
    pinMode(buzzerPin, OUTPUT);                 // Pino de saída para o buzzer
    pinMode(pinSensorMovimento, INPUT);         // Pino de entrada do sensor PIR
    pinMode(ledAlarmeLigado, OUTPUT);           // LED do alarme ativado
    pinMode(ledSensorPin, OUTPUT);              // LED do sensor PIN ativado

    // Inicializa LEDs como LOW
    digitalWrite(ledAlarmeLigado, HIGH);
    delay(500);
    digitalWrite(ledAlarmeLigado, LOW);
}

// Função para atualizar o estado do alarme
void loopAlarme() {
    // Verifica se o botão de acionamento do alarme foi pressionado
    if (digitalRead(btnAcionarAlarme) == LOW) {
        delay(200);  // Debounce para evitar leituras múltiplas rápidas

        alarmeLigado = !alarmeLigado;  // Alterna o estado do alarme
        alarmeAtivo = false;
        Serial.println(alarmeLigado ? "\nAlarme ligado internamente." : "\nAlarme desligado internamente.");
        mostrarNoLCD(alarmeLigado ? "Alarme ligado" : "Alarme desligado", "internamente.");

        alarmeLigado ? somAlarmeLigando() : somAlarmeDesligando();
        // Liga ou desliga o LED que indica se o alarme está ativado
        digitalWrite(ledAlarmeLigado, alarmeLigado ? HIGH : LOW);

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(btnAcionarAlarme) == LOW) {
            delay(10);
        }
    }

    // Verifica os sensores
    if (alarmeLigado && !alarmeAtivo) {
        // Verifica o estado do sensor Reed Switch (portão/porta)
        if (digitalRead(reedSwitchPin) == HIGH) {
            Serial.println("Sensor de porta ou janela ativado! Alarme disparado.");
            mostrarNoLCD("--- ATENCAO! ---", "Alarme disparado");
            alarmeAtivo = true;
        }

        // Verifica o estado do sensor de movimento
        if (digitalRead(pinSensorMovimento) == HIGH) {
            Serial.println("Sensor de movimento ativado! Alarme disparado.");
            mostrarNoLCD("--- ATENCAO! ---", "Alarme disparado");
            alarmeAtivo = true;
        }
    }

    // Verifica se o sensor PIN foi acionado
    if (digitalRead(pinSensorMovimento) == HIGH) { 
        digitalWrite(ledSensorPin, HIGH);
    } else {
        digitalWrite(ledSensorPin, LOW);
    }

    // Verifica se o alarme foi ativado
    if (alarmeAtivo) {
        somAlarmeTocando();
    }
}

// Som de disparo do alarme 
void somAlarmeTocando() {
    tone(buzzerPin, 1500);
    digitalWrite(ledAlarmeLigado, HIGH);
    delay(200);
    noTone(buzzerPin);
    digitalWrite(ledAlarmeLigado, LOW);
    delay(200);
}

// Desliga o buzzer ou qualquer som do alarme
void pararSomAlarme() {
    noTone(buzzerPin);
}

// Som de ligar o alarme
void somAlarmeLigando() {
    tone(buzzerPin, 1500);
    delay(300);
    noTone(buzzerPin);
}

// Som de desligar o alarme
void somAlarmeDesligando() {
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
    delay(100);
    tone(buzzerPin, 1500);
    delay(150);
    noTone(buzzerPin);
}




// ------------- CONFIGURAÇÃO DO PORTÃO ------------- //
// Definições dos pinos e variáveis
Servo servoMotor;                // Instância do servo
const int posicaoAberto = -85;    // Ângulo para abrir o portão
const int posicaoFechado = 0;    // Ângulo para fechar o portão

void setupPortao() {  
    pinMode(btnAbreFechaPortao, INPUT_PULLUP);      // Pino de entrada com pull-up interno
    pinMode(pinServoMotor, OUTPUT);                 // Pino de saída para o servo motor
    fecharPortao();
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
    for (int pos = posicaoFechado; pos >= posicaoAberto; pos++) {
        servoMotor.write(pos);          // Move o servo para a posição aberta
        delay(2);                       // Pequeno delay para suavizar o movimento
    }
    portaoAberto = true;                // Atualiza o estado para aberto
    delay(1000);
    servoMotor.detach();                // Desligar o controle do servo motor 
}

// Função para fechar o portão
void fecharPortao() {
    servoMotor.attach(pinServoMotor);
    for (int pos = posicaoAberto; pos <= posicaoFechado; pos--) {
        servoMotor.write(pos);          // Move o servo para a posição fechada
        delay(2);                       // Pequeno delay para suavizar o movimento
    }
    portaoAberto = false;               // Atualiza o estado para fechado
    delay(1000);
    servoMotor.detach();                // Desligar o controle do servo motor 
}
