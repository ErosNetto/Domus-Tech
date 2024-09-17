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
EstadoPortao ultimoEstado = PARADO; // Armazenar o último estado antes de parar

void setup() {
  Serial.begin(115200);

  // Configuração dos pinos
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(limite_aberto, INPUT_PULLUP);
  pinMode(limite_fechado, INPUT_PULLUP);
  pinMode(botao_unico, INPUT_PULLUP);

  pararMotor();
}

void loop() {
  // Verifica o estado do botão único
  if (digitalRead(botao_unico) == LOW) {
    delay(200);  // Debounce

    if (estadoAtual == PARADO) {
      handlePortaoParado();
    } else {
      pararMotor();
      ultimoEstado = estadoAtual;
      estadoAtual = PARADO;
    }
  }

  // Verifica limites do portão
  verificarLimites();
}

// Função para decidir o próximo movimento do portão
void handlePortaoParado() {
  if (digitalRead(limite_fechado) == LOW) {
    Serial.println("Portão totalmente fechado. Abrindo portão.");
    abrirPortao();
    estadoAtual = ABRINDO;
    ultimoEstado = ABRINDO;
  } else if (digitalRead(limite_aberto) == LOW) {
    Serial.println("Portão totalmente aberto. Fechando portão.");
    fecharPortao();
    estadoAtual = FECHANDO;
    ultimoEstado = FECHANDO;
  } else {
    Serial.println("Portão parado no meio do caminho.");
    if (ultimoEstado == ABRINDO) {
      Serial.println("Invertendo para fechar.");
      fecharPortao();
      estadoAtual = FECHANDO;
    } else if (ultimoEstado == FECHANDO) {
      Serial.println("Invertendo para abrir.");
      abrirPortao();
      estadoAtual = ABRINDO;
    }
  }
}

// Função para verifica os limites do portão
void verificarLimites() {
  if (estadoAtual == ABRINDO && digitalRead(limite_aberto) == LOW) {
    Serial.println("Portão completamente aberto.");
    pararMotor();
    estadoAtual = PARADO;
  } else if (estadoAtual == FECHANDO && digitalRead(limite_fechado) == LOW) {
    Serial.println("Portão completamente fechado.");
    pararMotor();
    estadoAtual = PARADO;
  }
}

// Função para abrir o portão
void abrirPortao() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, 80);
  Serial.println("Portão abrindo...\n");
}

// Função para fechar o portão
void fecharPortao() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, 80);
  Serial.println("Portão fechando...\n");
}

// Função para parar o motor
void pararMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ena, 0);
  Serial.println("Motor parado.\n");
}