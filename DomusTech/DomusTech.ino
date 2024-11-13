#include <WiFi.h>                         // Biblioteca do WiFi
#include <WiFiManager.h>                  // Biblioteca do WiFiManager
#include <LiquidCrystal_I2C.h>            // Biblioteca do LCD
#include <ESPAsyncWebServer.h>            // Biblioteca para criar um servidor
#include <AsyncTCP.h>                     // Biblioteca para o AsyncWebServer
#include <ESP32Servo.h>                   // Biblioteca para o Servo Motor
#include <Preferences.h>                  // Biblioteca para amazenar dados na ESP32 utilizando NVS (Non-Volatile Storage)


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



// ------------- Instâncias e Configurações -------------
AsyncWebServer server(80);                // Configura o servidor para escutar requisições HTTP na porta 80
WiFiManager wm;                           // Instância do WiFiManager
Preferences preferences;                  // Instância do Preferences



// ------------- Variáveis globais -------------
bool ledState[] = {0, 0, 0};              // Variável para controlar se os leds estão ligados ou desligados
bool alarmeLigado = false;                // Variável para controlar se o alarme está ligado ou desligado
bool alarmeAtivo = false;                 // Variável para controlar se o alarme foi ativado ou não
bool sensorMovimentoAtivado = true;       // Variável para controlar se o sensor de movimento ficará ativado ou não
bool sensorPortaAtivado = true;           // Variável para controlar se o sensor de porta ficará ativado ou não
bool portaoAberto = false;                // Variável para controlar se o portão está aberto ou fechado
bool luzLCDLigado = true;                 // Variável para controlar se o display está ligado ou desligado



// Função para escrever no LCD I2C
void mostrarNoLCD(const String& primeiraLinha, const String& segundaLinha) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(primeiraLinha);
    lcd.setCursor(0, 1);
    lcd.print(segundaLinha); 
}

// Função para ligar a luz do LCD
void alteraLuzLCD() {
    if (luzLCDLigado) {
        lcd.backlight();
    } else {
        lcd.noBacklight();
    }
}



void setup() {
    Serial.begin(115200);

    // Configura os pinos dos botões pra ESP como entrada e com pull-up interno
    pinMode(btnResetWifi, INPUT_PULLUP);
    pinMode(btnResetESP32, INPUT_PULLUP);

    // Inicializar a NVS
    preferences.begin("config", false);

    // Carrega as variáveis salvas
    ledState[0] = preferences.getBool("ledState0", false);
    ledState[1] = preferences.getBool("ledState1", false);
    ledState[2] = preferences.getBool("ledState2", false);
    alarmeLigado = preferences.getBool("alarmeLigado", false);
    alarmeAtivo = preferences.getBool("alarmeAtivo", false);
    sensorMovimentoAtivado = preferences.getBool("sensorMovAtiv", true);
    sensorPortaAtivado = preferences.getBool("sensorPorAtiv", true);
    // portaoAberto = preferences.getBool("portaoAberto", false);
    luzLCDLigado = preferences.getBool("luzLCDLigado", true);

    // Configura os pinos de entrada e saída
    setupLeds();
    setupAlarme();
    setupPortao();

    // Inicializa o LCD
    lcd.init();

    // Verifica se a luz de fundo do LCD ficara ligada ou desligada  
    alteraLuzLCD();
    mostrarNoLCD("   Domus Tech   ", "Carregando...");
    delay(200);

    // Inicia as configurações do Wi-Fi
    configurarWiFi();
    delay(200);





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
            
            // Liga ou desliga o LED
            digitalWrite(pin, ledStateRequest);
            // Atualiza o estado do LED no array
            ledState[ledNumero - 1] = ledStateRequest;

            // Salva o stado dos leds na memória
            saveLEDsPreferences();

            String state = (ledStateRequest == HIGH) ? "ON" : "OFF";
            request->send(200, "text/plain", "LED " + String(ledNumero) + " esta " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'ledNum' ou 'state' ausentes.");
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
                delay(200);
                somAlarmeDesligando();
                delay(200);
                pararSomAlarme();
                Serial.println("\nAlarme desligado remotamente.");
            } else {
                request->send(400, "text/plain", "Ação inválida.");
                return;
            }

            digitalWrite(ledAlarmeLigado, alarmeLigado ? HIGH : LOW);

            // Salva o stado do alarme na memória
            saveAlarmePreferences();

            String state = alarmeLigado ? "ligado." : "desligado.";
            request->send(200, "text/plain", "O alarme está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'state' ausente.");
        }
    });

    // ------------- Rota para ativar ou desativar o sensor de movimento ( http://192.168.18.85/sensores?sensor=movimento&state=1 ou http://192.168.18.85/sensores?sensor=porta&state=1 ) -------------
    server.on("/sensores", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("sensor") && request->hasParam("state")) {
            String sensorType = request->getParam("sensor")->value();
            int sensorState = request->getParam("state")->value().toInt();

            if (sensorType == "movimento") {
                // Ativa ou desativa o sensor de movimento
                if (sensorState == 1) {
                    sensorMovimentoAtivado = true;
                    mostrarNoLCD("-- Sensor PIR --", "Ativado");
                    somAtivandoSensor();
                    Serial.println("\nSensor PIR ativado remotamente.");
                } else if (sensorState == 0) {
                    sensorMovimentoAtivado = false;
                    mostrarNoLCD("-- Sensor PIR --", "Desativado");
                    somDesativandoSensor();
                    Serial.println("\nSensor PIR desativado remotamente.");
                } else {
                    request->send(400, "text/plain", "Ação inválida.");
                    return;
                }

                // Salva o estado do sensor na memória
                saveOpcoesAlarmePreferences();

                String state = sensorMovimentoAtivado ? "ativado." : "desativado.";
                request->send(200, "text/plain", "O sensor de movimento PIR está " + state);
            } else if (sensorType == "porta") {
                // Ativa ou desativa o sensor de porta
                if (sensorState == 1) {
                    sensorPortaAtivado = true;
                    mostrarNoLCD("- Sensor Porta -", "Ativado");
                    somAtivandoSensor();
                    Serial.println("\nSensor de porta ativado remotamente.");
                } else if (sensorState == 0) {
                    sensorPortaAtivado = false;
                    mostrarNoLCD("- Sensor Porta -", "Desativado");
                    somDesativandoSensor();
                    Serial.println("\nSensor de porta desativado remotamente.");
                } else {
                    request->send(400, "text/plain", "Ação inválida.");
                    return;
                }

                // Salva o estado do sensor na memória
                saveOpcoesAlarmePreferences();

                String state = sensorPortaAtivado ? "ativado." : "desativado.";
                request->send(200, "text/plain", "O sensor de porta está " + state);
            } else {
                request->send(400, "text/plain", "Sensor inválido.");
                return;
            }
        } else {
            request->send(400, "text/plain", "Parâmetro 'sensor' ou 'state' ausentes.");
        }
    });

    // ------------- Rota para controlar o portão ( http://192.168.18.85/portao?state=1 ) -------------
    // Precisa arrumar a função do portão, pois ela buga no começo
    server.on("/portao", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int portaoState = request->getParam("state")->value().toInt();

            // Abre o portão caso ele esteja aberto
            if (portaoState == 1 && !portaoAberto) {
                abrirPortao();
                Serial.println("\nPortão aberto remotamente.");
                mostrarNoLCD("Portao aberto", "remotamente");
            // Fecha o portão caso ele esteja aberto
            } else if (portaoState == 0 && portaoAberto) {
                fecharPortao();
                Serial.println("\nPortao fechado remotamente.");
                mostrarNoLCD("Portao fechado", "remotamente");
            } else {
                request->send(400, "text/plain", "Ação inválida.");
                return;
            }

            // Salva o estado do portão na memória
            // savePortaoPreferences();

            String state = (portaoState) ? "aberto." : "fechado.";
            request->send(200, "text/plain", "O portão está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'state' ausente.");
        }
    });

    // ------------- Rota para controlar a luz de fundo do display LCD ( http://192.168.18.85/display?state=1 ) -------------
    server.on("/display", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("state")) {
            int displayState = request->getParam("state")->value().toInt();

            // Liga o display caso ele esteja desligado
            if (displayState == 1 && !luzLCDLigado) {
                luzLCDLigado = true;
                alteraLuzLCD();
                Serial.println("\nLuz LCD ligada remotamente.");
                mostrarNoLCD("LCD ligado", "remotamente");
            // Desliga o display caso ele esteja ligado
            } else if (displayState == 0 && luzLCDLigado) {
                luzLCDLigado = false;
                alteraLuzLCD();
                Serial.println("\nLuz LCD desligada remotamente.");
                mostrarNoLCD("LCD desligado", "remotamente");
            } else {
                request->send(400, "text/plain", "Ação inválida.");
                return;
            }

            // Salva o estado do display na memória
            saveLCDPreferences();

            String state = (displayState) ? "ligado." : "desligado.";
            request->send(200, "text/plain", "O display está " + state);
        } else {
            request->send(400, "text/plain", "Parâmetro 'state' ausente.");
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

    // ------------- Rota para obter o status dos dispositivos ( http://192.168.18.85/opcoes) -------------
    server.on("/opcoes", HTTP_GET, [](AsyncWebServerRequest *request) {
        String opcoes = "";
        opcoes += "SensorPIR: " + String(sensorMovimentoAtivado) + "\n";
        opcoes += "SensorPorta: " + String(sensorPortaAtivado) + "\n";
        opcoes += "LuzLCDSistema: " + String(luzLCDLigado);

        Serial.println("\nEnviando opções.");
        request->send(200, "text/plain", opcoes);
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

// Funções para salvar os estados atuais na NVS
void saveLEDsPreferences() {
    preferences.putBool("ledState0", ledState[0]);
    preferences.putBool("ledState1", ledState[1]);
    preferences.putBool("ledState2", ledState[2]);
}

void saveAlarmePreferences() {
    preferences.putBool("alarmeLigado", alarmeLigado);
    preferences.putBool("alarmeAtivo", alarmeAtivo);
}

void saveOpcoesAlarmePreferences() {
    preferences.putBool("sensorMovAtiv", sensorMovimentoAtivado);
    preferences.putBool("sensorPorAtiv", sensorPortaAtivado);
}

// void savePortaoPreferences() {
//     preferences.putBool("portaoAberto", portaoAberto);
// }

void saveLCDPreferences() {
    preferences.putBool("luzLCDLigado", luzLCDLigado);
}





// ------------- CONFIGURAÇÃO DE WI-FI ------------- //
// Inicia as configurações de Wi-Fi
void configurarWiFi() {
    // Verifica se há redes Wi-Fi salvas no WiFiManager
    if (temRedesSalvas()) {
        Serial.println("Wi-Fi salvo encontrado. Tentando conectar...");
        mostrarNoLCD("Wi-Fi salvo", "Conectando...");
        delay(2000);

        // configurarIPFixo(); // Configura o IP fixo antes de tentar conectar

        // Tenta conectar ao Wi-Fi salvo
        WiFi.begin();

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
            ESP.restart();
        }
    } else {
        Serial.println("Nenhuma rede Wi-Fi salva foi encontrada.");
        mostrarNoLCD("  Nenhuma rede  ", "  Wi-Fi  salva  ");
        delay(3000);
        iniciarModoConfiguracaoWiFi();
    }
}

// Função para verificar se há redes Wi-Fi salvas no WiFiManager
bool temRedesSalvas() {
    WiFi.mode(WIFI_STA);
    WiFi.begin();        
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

// Função para escrever o IP da ESP no terminal e no LCD (Com IP definido)
// void ipTexto() {
//     Serial.println("IP estático configurado: " + local_IP.toString());
//     mostrarNoLCD("IP atribuido:", local_IP.toString());
// }

// Função para escrever o IP da ESP no terminal e no LCD (Sem IP definido)
void ipTexto() {
    Serial.println("IP atribuído pela ESP: " + WiFi.localIP().toString());
    mostrarNoLCD("IP atribuido:", WiFi.localIP().toString());
}





// ------------- CONFIGURAÇÃO DOS LEDS ------------- //
void setupLeds() {
    // Configura os pinos de saída para os LEDs
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    pinMode(ledPin3, OUTPUT);

    // Configura os botões de ligação manual como entrada com pull-up interno
    pinMode(btnLedPin1, INPUT_PULLUP);
    pinMode(btnLedPin2, INPUT_PULLUP);
    pinMode(btnLedPin3, INPUT_PULLUP);

    // Inicializa os LEDs com o estado salvo
    digitalWrite(ledPin1, ledState[0] ? HIGH : LOW);
    digitalWrite(ledPin2, ledState[1] ? HIGH : LOW);
    digitalWrite(ledPin3, ledState[2] ? HIGH : LOW);
}

// Função para alterar o stado dos leds
void alteraLed(int ledIndex, int ledPin, const char* ledName) {
    // Alterna o estado do LED e salva o novo estado
    ledState[ledIndex] = !ledState[ledIndex];
    digitalWrite(ledPin, ledState[ledIndex]);

    // Exibe mensagem de estado no Serial e no LCD
    if (ledState[ledIndex]) {
        Serial.printf("\n%s ligado internamente.\n", ledName);
        mostrarNoLCD(ledName, "ligado internamente");
    } else {
        Serial.printf("\n%s desligado internamente.\n", ledName);
        mostrarNoLCD(ledName, "desligado internamente");
    }
}

void loopLeds() {
    bool stateChanged = false;

    // Checa e alterna o LED 1
    if (digitalRead(btnLedPin1) == LOW) {
        alteraLed(0, ledPin1, "LED 1");
        stateChanged = true;
        delay(200);
    }

    // Checa e alterna o LED 2
    if (digitalRead(btnLedPin2) == LOW) {
        alteraLed(1, ledPin2, "LED 2");
        stateChanged = true;
        delay(200);
    }

    // Checa e alterna o LED 3
    if (digitalRead(btnLedPin3) == LOW) {
        alteraLed(2, ledPin3, "LED 3");
        stateChanged = true;
        delay(200);
    }

    // Salva o estado dos leds
    if (stateChanged) {
        saveLEDsPreferences();
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
    digitalWrite(ledSensorPin, HIGH);
    delay(500);
    digitalWrite(ledAlarmeLigado, alarmeLigado ? HIGH : LOW);
    digitalWrite(ledSensorPin, LOW);
}

// Função para atualizar o estado do alarme
void loopAlarme() {
    // Verifica se o botão de acionamento do alarme foi pressionado
    if (digitalRead(btnAcionarAlarme) == LOW) {
        delay(200);

        alarmeLigado = !alarmeLigado;  // Alterna o estado do alarme
        alarmeAtivo = false;

        Serial.println(alarmeLigado ? "\nAlarme ligado internamente." : "\nAlarme desligado internamente.");
        mostrarNoLCD(alarmeLigado ? "Alarme ligado" : "Alarme desligado", "internamente.");

        alarmeLigado ? somAlarmeLigando() : somAlarmeDesligando();
        // Liga ou desliga o LED que indica se o alarme está ativado
        digitalWrite(ledAlarmeLigado, alarmeLigado ? HIGH : LOW);

        // Salva o estado do alarme
        saveAlarmePreferences();

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(btnAcionarAlarme) == LOW) {
            delay(10);
        }
    }

    // Verifica os sensores
    if (alarmeLigado && !alarmeAtivo) {
        // Verifica o estado do sensor Reed Switch (portão/porta)
        if (digitalRead(reedSwitchPin) == HIGH && sensorPortaAtivado) {
            Serial.println("Sensor de porta ativado! Alarme disparado.");
            mostrarNoLCD("-ALARME TOCANDO-", "Sensor porta");
            alarmeAtivo = true;
            saveAlarmePreferences();
        }

        // Verifica o estado do sensor de movimento
        if (digitalRead(pinSensorMovimento) == HIGH && sensorMovimentoAtivado) {
            Serial.println("Sensor de movimento ativado! Alarme disparado.");
            mostrarNoLCD("-ALARME TOCANDO-", "Sensor movimento");
            alarmeAtivo = true;
            saveAlarmePreferences();
        }
    }

    // Verifica e atualiza o LED do sensor de movimento
    if (digitalRead(pinSensorMovimento) == HIGH && sensorMovimentoAtivado) { 
        digitalWrite(ledSensorPin, HIGH);
    } else {
        digitalWrite(ledSensorPin, LOW);
    }

    // Verifica se o alarme foi ativado e dispara o som do alarme
    if (alarmeAtivo) {
        somAlarmeTocando();
    }
}

// Som de ligar o alarme
void somAlarmeLigando() {
    digitalWrite(buzzerPin, HIGH);         
    delay(300);
    digitalWrite(buzzerPin, LOW);
}

// Som de desligar o alarme
void somAlarmeDesligando() {
    digitalWrite(buzzerPin, HIGH);         
    delay(150);                             
    digitalWrite(buzzerPin, LOW);
    delay(100);
    digitalWrite(buzzerPin, HIGH);
    delay(150);                             
    digitalWrite(buzzerPin, LOW);
}

// Som de disparo do alarme 
void somAlarmeTocando() {
    digitalWrite(ledAlarmeLigado, HIGH);
    digitalWrite(buzzerPin, HIGH);      
    delay(200);                            
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledAlarmeLigado, LOW); 
    delay(200);                            
}

// Som de ativar um sensor
void somAtivandoSensor() {
    digitalWrite(buzzerPin, HIGH);         
    delay(800);                             
    digitalWrite(buzzerPin, LOW);
}

// Som de desativar um sensor
void somDesativandoSensor() {
    digitalWrite(buzzerPin, HIGH);         
    delay(1600);                             
    digitalWrite(buzzerPin, LOW);
}

// Função para parar qualquer som do alarme
void pararSomAlarme() {
    digitalWrite(buzzerPin, LOW);
}





// ------------- CONFIGURAÇÃO DO PORTÃO ------------- //
// Precisa arrumar a função do portão, pois ela buga no começo
// Definições dos pinos e variáveis
Servo servoMotor;                 // Instância do servo
const int posicaoAberto = 165;    // Ângulo para abrir o portão
const int posicaoFechado = 40;    // Ângulo para fechar o portão

void setupPortao() {  
    pinMode(btnAbreFechaPortao, INPUT_PULLUP);      // Pino de entrada com pull-up interno
    pinMode(pinServoMotor, OUTPUT);                 // Pino de saída para o servo motor
    // fecharPortao();
    servoMotor.detach();
}

// Função para atualizar o estado do portão
void loopPortao() {
    if (digitalRead(btnAbreFechaPortao) == LOW) {
        delay(200);
        
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
        delay(10);                      // Delay maior para suavizar o movimento
    }
    portaoAberto = true;                // Atualiza o estado para aberto
    delay(900);
    servoMotor.detach();                // Desliga o controle do servo motor 
}

// Função para fechar o portão
void fecharPortao() {
    servoMotor.attach(pinServoMotor);
    for (int pos = posicaoAberto; pos >= posicaoFechado; pos--) {
        servoMotor.write(pos);          // Move o servo para a posição fechada
        delay(10);                      // Delay maior para suavizar o movimento
    }
    portaoAberto = false;               // Atualiza o estado para fechado
    delay(900);
    servoMotor.detach();                // Desliga o controle do servo motor 
}
