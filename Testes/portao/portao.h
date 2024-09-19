#ifndef PORTAO_H
#define PORTAO_H

void setupPortao(int in1Pin, int in2Pin, int enaPin, int limiteAbertoPin, int limiteFechadoPin, int botaoUnicoPin);
void loopPortao();
void abrirPortao();
void fecharPortao();
void pararMotor();
void verificarLimites();

#endif
