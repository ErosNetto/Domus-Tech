#ifndef PORTAO_H
#define PORTAO_H

// Função para configurar o portão
void setupPortao(int pinoBotao, int pinoServo);

// Função para atualizar o estado do portão
void loopPortao();
void abrirPortao();
void fecharPortao();
void pararMotor();
void acionaPortao();

#endif
