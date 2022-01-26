# Robo-IHM-WIFI
Projeto de um robô, controlado via Ethernet remotamente com uma IHM Nextion.

//// README EM ELABORAÇÃO
// Controle do robô 4 rodas
//
//  - Utiliza controle de velocidade PWM
//  - Envia e recebe pacote de dados com redundância na verificação de integridade de dados, para segurança
//  - Sensores de temperatura, acelerômetro e pressão
//  - Utiliza princípios de RTOS devido à complexidade do código, visando máximo de eficiência e resposta
//
// IHM
//
//  - Controle analógico de velocidade e curvas via joystick
//  - Tela IHM Nextion
//  - Geração de relatório com critérios (parado ou andando) para cartão SD com todos os sensores e nível de bateria
//     com digitação via tela do nome do arquivo
//  - Captura de relatórios via interface HTML pelo IP
//
//
// TODO:
//  - Controle PTZ de IP Câmera via pacote SOAP
