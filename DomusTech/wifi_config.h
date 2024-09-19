#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <WiFi.h>
#include <WiFiManager.h>

// Declarações de funções
void configurarWiFi();
bool hasSavedNetworks();
void iniciarModoConfiguracaoWiFi();
void configurarIPFixo();
void ipTexto();

// Declaração da função externa para mostrar no LCD
extern void mostrarNoLCD(const String& primeiraLinha = "", const String& segundaLinha = "");

// Declarações de variáveis globais externas
extern WiFiManager wm;

#endif
