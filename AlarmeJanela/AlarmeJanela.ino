const int buttonPin = 14;
const int reedSwitchPin = 12;
const int buzzerPin = 13;

bool alarmOn = false;  // Apenas uma variável para o estado do sistema

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // Botão com pull-up
  pinMode(reedSwitchPin, INPUT_PULLUP);  // Reed Switch com pull-up
  pinMode(buzzerPin, OUTPUT);  // Buzzer como saída

  Serial.begin(115200);
}

void loop() {
  // Verifica se o botão foi pressionado
  if (digitalRead(buttonPin) == LOW) {
    delay(200);  // Debounce
    alarmOn = true;  // Alterna o estado do sistema
    if (alarmOn) {
      Serial.println("Sistema armado.");
      alarmeLigando();
    } else {
      Serial.println("Sistema desarmado.");
      alarmeDesligando();
    }
    delay(500);  // Evita múltiplas leituras por um único clique
  }

  // Se o sistema estiver armado, verificar o sensor magnético
  if (alarmOn && digitalRead(reedSwitchPin) == HIGH) {
    Serial.println("Intrusão detectada! Alarme ativado.");
    alarmeTocando();
  } else {
    noTone(buzzerPin);  // Desativa o buzzer se não houver intrusão
  }
}

void alarmeTocando() {
  tone(buzzerPin, 2000);
  delay(300);      
  noTone(buzzerPin);
  delay(300);   
}

void alarmeLigando() {
    tone(buzzerPin, 2000);
    delay(300);
    noTone(buzzerPin);
}

void alarmeDesligando() {
    tone(buzzerPin, 2000);
    delay(150);            
    noTone(buzzerPin);
    delay(100); 
    tone(buzzerPin, 2000);
    delay(150);             
    noTone(buzzerPin);
}
