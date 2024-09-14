// Definir os pinos da ponte H
const int in1 = 5;    // Pino para IN1 da Ponte H
const int in2 = 18;   // Pino para IN2 da Ponte H
const int ena = 19;   // Pino para ENA da Ponte H (PWM para controle de velocidade)

// Definir os pinos dos botões de fim de curso
const int limite_aberto = 12;  // Fim de curso do portão aberto
const int limite_fechado = 13; // Fim de curso do portão fechado

// Definir o pino do botão único de controle (abrir/parar/fechar)
const int botao_unico = 14;

// Estados do portão
enum EstadoPortao {
  PARADO,
  ABRINDO,
  FECHANDO
};

EstadoPortao estadoAtual = PARADO;  // Estado inicial do portão

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  // Configuração dos pinos de saída para o motor
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);

  // Configuração dos pinos dos botões de fim de curso
  pinMode(limite_aberto, INPUT_PULLUP);
  pinMode(limite_fechado, INPUT_PULLUP);

  // Configuração do botão único de controle
  pinMode(botao_unico, INPUT_PULLUP);

  // Inicializa o motor parado
  pararMotor();
  Serial.println("Sistema inicializado. Portão parado.");
}

void loop() {
  // Verifica o estado do botão único
  if (digitalRead(botao_unico) == LOW) {
    delay(200);  // Debounce simples
    Serial.println("Botão de controle único pressionado.");

    // Alterar o estado do portão a cada pressionamento
    if (estadoAtual == PARADO) {
      if (digitalRead(limite_fechado) == LOW) { // Se o portão estiver totalmente fechado
        Serial.println("Portão totalmente fechado. Abrindo portão.");
        abrirPortao();
        estadoAtual = ABRINDO;
      } else if (digitalRead(limite_aberto) == LOW) { // Se o portão estiver totalmente aberto
        Serial.println("Portão totalmente aberto. Fechando portão.");
        fecharPortao();
        estadoAtual = FECHANDO;
      }
    } 
    else if (estadoAtual == ABRINDO) {
      Serial.println("Portão em movimento (abrindo). Parando portão.");
      pararMotor();
      estadoAtual = PARADO;
    } 
    else if (estadoAtual == FECHANDO) {
      Serial.println("Portão em movimento (fechando). Parando portão.");
      pararMotor();
      estadoAtual = PARADO;
    }
  }

  // Verifica os limites do portão para parar o motor automaticamente
  if (estadoAtual == ABRINDO && digitalRead(limite_aberto) == LOW) {
    Serial.println("Portão completamente aberto. Parando motor.");
    pararMotor();
    estadoAtual = PARADO;
  }
  if (estadoAtual == FECHANDO && digitalRead(limite_fechado) == LOW) {
    Serial.println("Portão completamente fechado. Parando motor.");
    pararMotor();
    estadoAtual = PARADO;
  }
}

// Função para abrir o portão (girar o motor em uma direção)
void abrirPortao() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, 100); // Velocidade de 100
  Serial.println("Portão abrindo...");
}

// Função para fechar o portão (girar o motor na outra direção)
void fecharPortao() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, 100); // Velocidade de 100
  Serial.println("Portão fechando...");
}

// Função para parar o motor
void pararMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ena, 0);  // Parar o motor
  Serial.println("Motor parado.");
}
