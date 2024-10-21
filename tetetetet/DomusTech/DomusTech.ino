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

// ------------- Variáveis globais -------------
bool ledState[] = {0, 0, 0};              // Variável para controlar se os leds estão ligados ou desligados
bool alarmeLigado = false;                // Variável para controlar se o alarme está ligado ou desligado
bool alarmeAtivo = false;                 // Variável para controlar se o alarme foi ativado ou não
bool portaoAberto = false;                // Variável para controlar se o portão está aberto ou fechado


void setup() {
    Serial.begin(115200);
    Serial.println("Entrando no loop principal");

    // Configura os pinos dos botões pra ESP como entrada e com pull-up interno
    pinMode(btnResetWifi, INPUT_PULLUP);
    pinMode(btnResetESP32, INPUT_PULLUP);

    // Configura os pinos de entrada e saida
    setupLeds();
    setupAlarme();
    setupPortao();
}

void loop() {
    loopAlarme();   // Controle do sistema de alarme
    loopPortao();   // Controle do sistema de portão 
    loopLeds();     // Controle do sistema de leds

    // Verifica continuamente se o botão foi pressionado para resetar o Wi-Fi
    if (digitalRead(btnResetWifi) == LOW) {
        Serial.println("\nBotão pressionado. Resetando configurações de Wi-Fi...");
        // wm.resetSettings();
        delay(3000);        
        ESP.restart();
    }

    // Reinicia a ESP32
    if (digitalRead(btnResetESP32) == LOW) {
        Serial.println("\nBotão pressionado. Reiniciando...");
        delay(3000);        
        ESP.restart();
    }
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
        } else {
            Serial.println("\nLED: 1 desligado internamente.");
        }
        delay(200); // Debounce
    }

    // Checa o botão do LED 2
    if (digitalRead(btnLedPin2) == LOW) {
        ledState[1] = !ledState[1];             // Alterna o estado do LED
        digitalWrite(ledPin2, ledState[1]);     // Liga ou desliga o led

        if (ledState[1] == 1) {
            Serial.println("\nLED: 2 ligado internamente.");
        } else {
            Serial.println("\nLED: 2 desligado internamente.");
        }
        delay(200); // Debounce
    }

    // Checa o botão do LED 3
    if (digitalRead(btnLedPin3) == LOW) {
        ledState[2] = !ledState[2];             // Alterna o estado do LED
        digitalWrite(ledPin3, ledState[2]);     // Liga ou desliga o led

        if (ledState[2] == 1) {
            Serial.println("\nLED: 3 ligado internamente.");
        } else {
            Serial.println("\nLED: 3 desligado internamente.");
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
            alarmeAtivo = true;
        }

        // Verifica o estado do sensor de movimento
        if (digitalRead(pinSensorMovimento) == HIGH) {
            Serial.println("Sensor de movimento ativado! Alarme disparado.");
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
const int posicaoAberto = 85;    // Ângulo para abrir o portão
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
        // mostrarNoLCD(portaoAberto ? "Portao aberto" : "Portao fechado", "internamente.");

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(btnAbreFechaPortao) == LOW) {
            delay(10);
        }
    }
}

// Função para abrir o portão
void abrirPortao() {
    servoMotor.attach(pinServoMotor, 500, 2400);
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
    servoMotor.attach(pinServoMotor, 500, 2400);
    for (int pos = posicaoAberto; pos <= posicaoFechado; pos--) {
        servoMotor.write(pos);          // Move o servo para a posição fechada
        delay(2);                       // Pequeno delay para suavizar o movimento
    }
    portaoAberto = false;               // Atualiza o estado para fechado
    delay(1000);
    servoMotor.detach();                // Desligar o controle do servo motor 
}
