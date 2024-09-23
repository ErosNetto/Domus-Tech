// Pinos do Alarme
const int btnAcionarAlarme = 13;          // Botão para ligar/desligar alarme (INPUT)
const int pinSensorMovimento = 34;        // Pino do sensor movimento (PIR HC-SR501) (INPUT)
const int buzzerPin = 26;                 // Buzzer do alarme (OUTPUT)
const int ledAlarmeAtivado = 2;           // LED que indica que o alarme está ativado (OUTPUT)
const int ledPIRDetectado = 12;           // LED que acende quando o PIR detecta movimento (OUTPUT)

bool alarmeAtivo = false;                 // Variável para controlar se o alarme está ligado ou desligado

void setup() {
    Serial.begin(115200);

    // Configura os pinos
    pinMode(btnAcionarAlarme, INPUT_PULLUP);        // Pino de entrada com pull-up interno
    pinMode(buzzerPin, OUTPUT);                     // Pino de saida
    pinMode(pinSensorMovimento, INPUT);             // Pino do sensor PIR
    pinMode(ledAlarmeAtivado, OUTPUT);              // LED do alarme ativado
    pinMode(ledPIRDetectado, OUTPUT);               // LED do sensor PIR

    digitalWrite(ledAlarmeAtivado, HIGH);  // Acende o LED
    delay(1000);
    digitalWrite(ledAlarmeAtivado, LOW);  // Apaga o LED
}

// Função para atualizar o estado do alarme
void loop() {
    // Verifica se o botão de acionamento do alarme foi pressionado
    if (digitalRead(btnAcionarAlarme) == LOW) {
        delay(200);  // Debounce para evitar leituras múltiplas rápidas

        alarmeAtivo = !alarmeAtivo;  // Alterna o estado do alarme
        Serial.println(alarmeAtivo ? "\nAlarme ligado internamente." : "\nAlarme desligado internamente.");
        
        alarmeAtivo ? alarmeLigando() : alarmeDesligando();

        // Liga ou desliga o LED que indica se alarme está ativado
        digitalWrite(ledAlarmeAtivado, alarmeAtivo ? HIGH : LOW);

        // Aguarda até o botão ser solto antes de continuar
        while (digitalRead(btnAcionarAlarme) == LOW) {
            delay(10);
        }
    }


    // Verifica o estado do sensor de movimento
    if (alarmeAtivo && digitalRead(pinSensorMovimento) == HIGH) {
        Serial.println("Movimento detectado pelo sensor PIR! Alarme disparado.");
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