# Domus Tech: Automação Residencial

O **Domus Tech** é uma solução completa de automação residencial que permite o controle de diversos sistemas de sua casa de forma remota, através de um aplicativo móvel conectado via Wi-Fi. O projeto integra iluminação inteligente, controle de portão/garagem e um sistema de alarme para proporcionar mais segurança, praticidade e eficiência energética.

## Funcionalidades Principais

- **Controle Remoto**: Gerencie a iluminação, portão e sistemas de segurança de qualquer lugar pelo aplicativo.
- **Monitoramento em Tempo Real**: Acompanhe o status dos sistemas em tempo real, incluindo o estado do portão (aberto/fechado) e a ativação do alarme.
- **Iluminação Inteligente**: Controle a iluminação com integração de sensores de movimento, otimizando o consumo de energia.
- **Segurança Avançada**: O sistema de alarme com sensores de movimento e de abertura garante a proteção da sua casa, disparando uma sirene em caso de invasão.

## Benefícios

- **Praticidade**: Controle sua casa mesmo à distância, resolvendo problemas remotamente.
- **Eficiência Energética**: Reduza o consumo de energia através da automação de sistemas.
- **Conforto e Conveniência**: A automação proporciona mais comodidade no dia a dia, com total controle através de um único aplicativo.
- **Segurança**: Proteção da sua residência com sistemas integrados de alarme e monitoramento.

## Requisitos

### Funcionais:

- Controle e verificação de dispositivos via aplicativo.
- Automação de luzes com sensores de movimento.
- Sistema de alarme integrado.

### Não Funcionais:

- Confiabilidade, usabilidade, escalabilidade e eficiência energética.
- Baixa latência na resposta aos comandos.

## Materiais Utilizados

- **1x ESP32**: Microcontrolador principal do sistema, responsável pela comunicação via Wi-Fi e controle dos dispositivos.
- **1x Display LCD I2C 16x2**: Interface para exibir informações do sistema, como status dos dispositivos.
- **5x LEDs**: Indicadores visuais para sinalizar o estado dos sistemas, como iluminação ou alarme.
- **3x Resistores 10Ω**: Usados para proteger os Push Button.
- **5x Resistores 180Ω**: Usados para proteger os LEDs e outros componentes contra sobrecarga de corrente.
- **7x Push Button**: Botões físicos para interação direta com o sistema, permitindo ações manuais, como ligar/desligar dispositivos.
- **1x Sensor de Movimento (HCSR501)**: Detecta movimento em áreas específicas para automatizar a iluminação e melhorar a segurança.
- **1x Sensor Magnético Reed Switch**: Sensor de abertura/fechamento usado em portas e janelas para detectar invasões.
- **1x Módulo Buzzer Ativo 5V**: Emite alertas sonoros quando o sistema de alarme é acionado.
- **1x Servomotor**: Controla a abertura e o fechamento do portão/garagem de forma remota.
- **1x Protoboard**: Utilizado para montar e testar os circuitos de maneira prática, sem a necessidade de solda.
- **1x Jumpers**: Fios para realizar as conexões entre os componentes no protoboard.
