#ifndef PORTAOUSANDOSERVO_H
#define PORTAOUSANDOSERVO_H

// Função para configurar o portão
void setupPortao(int pinoBotao, int pinoServo);

// Função para atualizar o estado do portão
void atualizarPortao();
void abrirPortao();
void fecharPortao();
void pararMotor();
void acionaPortao();

#endif
