//  Definições de comunicação
#define Tamanho_mensagem_Enviada   8
#define Tamanho_mensagem_Recebida  17
#define headerByte                 0xEF
#define endByte                    0xFE

//  Definições de pinos
#define joystick_x_pino  A0
#define joystick_y_pino  A1

//  Definições de controle
#define joystick_zona_morta 10
#define joystickCentro      512

//  Definições de usuário
#define velocidadeMaxima    255

#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <SdFat.h>
#include <DS3231.h>

// Network
byte mac[] =        { 0x70, 0xB3, 0xD5, 0x0A, 0xC5, 0xD7 };
byte ip[] =         {192, 168, 0, 120};
byte gateway[] =    { 192, 168, 0, 10};
byte subnet[] =     { 255, 255, 0, 0 };

byte     server[] = { 192, 168, 0, 10 }; // Google
uint16_t port = 23;

bool iniciarTrajeto;
bool gravarDadosNoCartao;
bool ethernetStatusConexao;
bool motoresStatusMovimento;

byte mensagemRecebida[17];
byte mensagemEnvio[8];

int offset_x = 0; // subtracting the initial joystick x-location
int offset_y = 0; // subtracting the initial joystick y-location

uint16_t joystickZonaMortaACIMA;
uint16_t joystickZonaMortaABAIXO;

uint16_t contadorLeituraJoystick;
uint16_t contadorEthernetEnvio;
uint16_t contadorEthernetTimeout;
uint16_t contadorGravacaoRelatorio;
uint32_t tempoConectado;

float anguloX;
float anguloY;
float anguloZ;
float temperatura;
float pressao;
float corrente;
float tensao;

char fileName[80];

EthernetClient Controle;
SdFat SD;
DS3231  relogio(20, 21);

//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;


void setupTimer1() {
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 1000 Hz (16000000/((249+1)*64))
  OCR1A = 249;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void setup() {
  Serial.begin (57600);

  //  Inicialização pinos
  pinMode(10, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  setupTimer1();

  //  Inicialização rede / comunicação
  Ethernet.begin(mac, ip);//, gateway, gateway, subnet);
  Serial.print(F("My IP address: "));
  Serial.println(Ethernet.localIP());

  // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(4, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }

  //  Inicialização variáveis
  joystickZonaMortaACIMA = 512 + joystick_zona_morta;
  joystickZonaMortaABAIXO = 512 - joystick_zona_morta;


  offset_x = analogRead(joystick_x_pino); // initial joystick x-val
  offset_y = analogRead(joystick_y_pino); // initial joystick y-val

  mensagemEnvio[0] = headerByte;
  mensagemEnvio[7] = endByte;

  Controle.setTimeout(400);  // set the timeout duration for client.connect() and client.stop()
}

ISR(TIMER1_COMPA_vect) {
  contadorLeituraJoystick++;
  contadorEthernetEnvio++;
  contadorEthernetTimeout++;

  contadorGravacaoRelatorio++;
  tempoConectado++;
}

void loop() {

  if (iniciarTrajeto) {
    if (timerControle(&contadorEthernetEnvio, 10))
      ethernetStatusConexao = ethernetConectarEnviarMensagem();


    if (Controle.available()) {
      for (byte i = 0; i < 17; i++)
        mensagemRecebida[i] = Controle.read();

      contadorEthernetTimeout = 0;
      ethernetStatusConexao = true;

      if (mensagemRecebida[0] == headerByte && mensagemRecebida[16] == endByte) {                //  Verificação da integridade da mensagem recebida
        if (mensagemRecebida[15] = ethernetCRC8(mensagemRecebida, Tamanho_mensagem_Recebida)) {

          anguloX     = byteParaFloat(mensagemRecebida[1], mensagemRecebida[2]);
          anguloY     = byteParaFloat(mensagemRecebida[3], mensagemRecebida[4]);
          anguloZ     = byteParaFloat(mensagemRecebida[5], mensagemRecebida[6]);
          temperatura = byteParaFloat(mensagemRecebida[7], mensagemRecebida[8]);
          pressao     = byteParaFloat(mensagemRecebida[9], mensagemRecebida[10]) * 10;
          corrente    = byteParaFloat(mensagemRecebida[11], mensagemRecebida[12]);
          tensao      = byteParaFloat(mensagemRecebida[13], mensagemRecebida[14]);

          Serial.print("Angulo X: ");
          Serial.print(anguloX);
          Serial.print(", Angulo Y: ");
          Serial.print(anguloY);
          Serial.print(", Angulo Z: ");
          Serial.print(anguloZ);
          Serial.print(", Temperatura: ");
          Serial.print(temperatura);
          Serial.print(", Pressao: ");
          Serial.print(pressao);
          Serial.print(", Corrente: ");
          Serial.print(corrente);
          Serial.print(", Tensao: ");
          Serial.println(tensao);

        }
      }
    }

    if (timerControle(&contadorLeituraJoystick, 15))
      motoresStatusMovimento = leituraJoystick(&mensagemEnvio[1], &mensagemEnvio[2], &mensagemEnvio[3], &mensagemEnvio[4]);

    ethernetDetectaDesconexao(&contadorEthernetTimeout, 1000);

    cartaoEscreverRelatorio (gravarDadosNoCartao, motoresStatusMovimento, &contadorGravacaoRelatorio, 7000, 3000);

  }
}


bool ethernetConectarEnviarMensagem() {
  if (!Controle.connected()) {

    if (Controle.connect(server, port)) {
      Serial.println ("Conectado!");
      return true;
    } else {
      Serial.println ("Tentativa de reconexão!");
      delay(20);
      return false;
    }
  } else if (!Controle.write(mensagemEnvio, 8)) {
    Serial.println ("Mensagem não enviada!");
    return false;
  }
}

bool timerControle(uint16_t *contador, uint16_t tempo) {
  if (*contador > tempo) {
    *contador = 0;
    return true;
  } else
    return false;
}

byte ethernetCRC8(byte mensagem[], byte tamanhoMensagem) {
  byte crc = 0;

  for (byte i = 0; i < tamanhoMensagem; i++)
    crc ^= mensagem[i];

  return crc;
}

void ethernetDetectaDesconexao(uint16_t *contadorEthernetTimeout, uint16_t tempoTimeOut) {
  if (*contadorEthernetTimeout > tempoTimeOut) {
    *contadorEthernetTimeout = 0;
    Controle.stop();
    Serial.println ("Desconectado por TimeOut!");
  }
}

bool leituraJoystick(byte *byte0, byte *byte1, byte *byte2, byte *byte3) {
  int eixoX = analogRead(joystick_x_pino);
  int eixoY = analogRead(joystick_y_pino);

  int xVal = eixoX - offset_x;
  int yVal = eixoY - offset_y;


  if (eixoX < joystickZonaMortaACIMA && eixoX > joystickZonaMortaABAIXO)
    eixoX = joystickCentro;

  if (eixoY < joystickZonaMortaACIMA && eixoY > joystickZonaMortaABAIXO)
    eixoY = joystickCentro;

  byte pwmMotorEsquerda;
  byte pwmMotorDireita;
  bool dirMotorEsquerda;
  bool dirMotorDireita;

  int angulo;

  //  int eixoX = (analogRead(joystick_x_pino) - offset_x); // relative joystick x
  //  int eixoY = (analogRead(joystick_y_pino) - offset_y); // relative joystick y


  if (eixoX != joystickCentro || eixoY != joystickCentro) {

    float deg = 180 - (int(atan2(xVal, yVal) * (180.0 / PI)) + 90); // angle calc
    angulo = (int) deg;


    if (angulo >= 45 && angulo < 90) {                                            //  Movimento para frente-esquerda
      pwmMotorEsquerda = map(eixoX, joystickCentro, 1023, velocidadeMaxima, 0);
      pwmMotorDireita  = map(eixoY, joystickCentro, 1023, 0, velocidadeMaxima);

      dirMotorEsquerda = 1;
      dirMotorDireita  = 1;
    } else if (angulo > 90 && angulo <= 136) {                                           //  Movimento para frente-direita
      pwmMotorDireita  = map(eixoX, 0, joystickCentro, 0, velocidadeMaxima);
      pwmMotorEsquerda = map(eixoY, joystickCentro, 1023, 0, velocidadeMaxima);

      dirMotorEsquerda = 1;
      dirMotorDireita  = 1;
    } else if (angulo == 90) {                                                            //  Movimento para frente
      pwmMotorDireita  = map(eixoY, joystickCentro, 1023, 0, velocidadeMaxima);
      pwmMotorEsquerda = map(eixoY, joystickCentro, 1023, 0, velocidadeMaxima);

      dirMotorEsquerda = 1;
      dirMotorDireita  = 1;
    } else if (angulo >= 0 && angulo < 45) {                                              //  Movimento para esquerda-frente
      pwmMotorEsquerda = map(eixoY, joystickCentro, 1023, velocidadeMaxima, 0);
      pwmMotorDireita  = map(eixoX, joystickCentro, 1023, 0, velocidadeMaxima);

      dirMotorEsquerda = 0;
      dirMotorDireita  = 1;
    } else if (angulo >= 136 && angulo <= 180) {                                          //  Movimento para direita-frente
      pwmMotorDireita = map(eixoY, joystickCentro, 1023, velocidadeMaxima, 0);
      pwmMotorEsquerda  = map(eixoX, 0, joystickCentro, velocidadeMaxima, 0);

      dirMotorEsquerda = 1;
      dirMotorDireita  = 0;
    } else if (angulo <= -46 && angulo > -90) {                                            //  Movimento para trás-esquerda
      pwmMotorDireita = map(eixoX, joystickCentro, 1023, velocidadeMaxima, 0);
      pwmMotorEsquerda  = map(eixoY, 0, joystickCentro, velocidadeMaxima, 0);

      dirMotorEsquerda = 0;
      dirMotorDireita  = 0;
    } else if (angulo <= 270 && angulo >= 225) {                                           //  Movimento para trás-direita
      pwmMotorEsquerda  = map(eixoX, 0, joystickCentro, 0, velocidadeMaxima);
      pwmMotorDireita = map(eixoY, 0, joystickCentro, velocidadeMaxima, 0);

      dirMotorEsquerda = 0;
      dirMotorDireita  = 0;
    } else if (angulo == -90) {                                                            //  Movimento para trás
      pwmMotorDireita  = map(eixoY, 0, joystickCentro, velocidadeMaxima, 0);
      pwmMotorEsquerda = map(eixoY, 0, joystickCentro, velocidadeMaxima, 0);

      dirMotorEsquerda = 0;
      dirMotorDireita  = 0;
    } else if (angulo <= 0 && angulo >= -46) {                                              //  Movimento para esquerda-trás
      pwmMotorDireita = map(eixoY, 0, joystickCentro, 0, velocidadeMaxima);
      pwmMotorEsquerda  = map(eixoX, joystickCentro, 1023, 0, velocidadeMaxima);

      dirMotorEsquerda = 0;
      dirMotorDireita  = 1;
    } else if (angulo >= 180 && angulo <= 225) {                                           //  Movimento para direita-trás

      pwmMotorEsquerda = map(eixoY, 0, joystickCentro, 0, velocidadeMaxima);
      pwmMotorDireita = map(eixoX, 0, joystickCentro, velocidadeMaxima, 0);

      dirMotorEsquerda = 1;
      dirMotorDireita  = 0;
    }
    *byte0 = pwmMotorEsquerda;
    *byte1 = dirMotorEsquerda;
    *byte2 = pwmMotorDireita;
    *byte3 = dirMotorDireita;

    return true;
  } else {
    pwmMotorDireita  = 0;
    pwmMotorEsquerda = 0;

    *byte0 = pwmMotorEsquerda;
    *byte1 = dirMotorEsquerda;
    *byte2 = pwmMotorDireita;
    *byte3 = dirMotorDireita;
    return false;
  }



  //  Serial.print("Angulo joystick: ");
  //  Serial.print(angulo);
  //  Serial.print("Eixo X: ");
  //  Serial.print(eixoX);
  //  Serial.print("Eixo Y: ");
  //  Serial.println(eixoY);

  //  Serial.print("Angulo joystick: ");
  //  Serial.print(angulo);
  //  Serial.print(", ");
  //  Serial.print("Motor esquerda: ");
  //  Serial.print(pwmMotorEsquerda);
  //  Serial.print(";");
  //  Serial.print(dirMotorEsquerda);
  //  Serial.print(", ");
  //  Serial.print("Motor direita: ");
  //  Serial.print(pwmMotorDireita);
  //  Serial.print(";");
  //  Serial.print(dirMotorDireita);
  //  Serial.println();

}

void floatParaByte(float variavel, byte * byte0, byte * byte1) {
  int conversao = (int)(variavel * 100);

  *byte0 = highByte(conversao);
  *byte1 = lowByte(conversao);
}

float  byteParaFloat(byte byte0, byte byte1) {
  return (float)((int)((byte0 << 8) | (byte1))) / 100;
}

char *imprimirDoisDigitosData(uint8_t numero1, uint8_t numero2, uint16_t numero3) {
  static char Buff[2];
  sprintf(Buff, "%02d/%02d/%02d", numero1, numero2, numero3);
  return Buff;
}

char *imprimirDoisDigitosHoras(uint8_t numero1, uint8_t numero2, uint8_t numero3) {
  static char Buff[2];
  sprintf(Buff, "%02d:%02d:%02d", numero1, numero2, numero3);
  return Buff;
}

void cartaoEscreverRelatorio (bool gravar, bool escolhaDeTempo, uint16_t *contador, uint16_t tempo1, uint16_t tempo2) {

  if (gravar) {
    if ((*contador > tempo1 && escolhaDeTempo) || (*contador > tempo2 && !escolhaDeTempo)) {
      *contador = 0;

      relatorioEscreverDados(" ");
    }
  }
}

char * removeFromString(char str[]) {
  static char saida[100];
  uint8_t o = 0;
  uint8_t i = 0;

  for (i = 0; i < 100; i++) {
    if (str[i] == '\0') {
      saida[o] = '\0';
      break;
    }

    if (str[i] == '%' && str[i + 1] == '2' && str[i + 2] == '0') {
      saida[o] = ' ';
      i = i + 3;
      o = o + 1;
    }

    saida[o] = str[i];
    o++;
  }

  return saida;
}



//------------------------------------------------------------------------------
// Write data header.
void relatorioEscreverCabecalho() {
  uint16_t year;
  uint8_t day, month, hour, minute, second;

  if (!file.open(fileName, FILE_WRITE)) {
    Serial.println(F("Erro ao abrir arquivo"));
  }

  delay(10);

  relogioLeituraHora(&hour, &minute, &second);
  relogioLeituraData(&day, &month, &year);

  file.print("sep=;");                                                       //  Indicando qual o separador de colunas
  file.println();



  file.print("Descricao: ;Arquivo gerado as ");

  file.print(imprimirDoisDigitosHoras(hour, minute, second));

  file.print(F(" de "));

  file.print(imprimirDoisDigitosData(day, month, year));

  for (byte i = 0; i < 9; i++)
    file.print(F(";"));
  file.println();

  for (byte i = 0; i < 10; i++)
    file.print(F(";"));
  file.println();

  file.println(F(";Percurso:;;;;;;;Sensores:;;"));

  file.println(F(";Hora:;x:;y:;z:;Distancia:;Flag:;;Bateria;Temperatura;Motores"));

  if (!file.sync() || file.getWriteError()) {
    Serial.println(F("Erro ao escrever no arquivo"));
  }

  file.close();

}

//------------------------------------------------------------------------------
// Log a data record.
void relatorioEscreverDados(char* Flag) {
  uint16_t year;
  uint8_t day, month, hour, minute, second;

  if (!file.open(fileName, FILE_WRITE)) {
    Serial.println(F("Erro ao abrir arquivo"));
  }

  relogioLeituraHora(&hour, &minute, &second);
  relogioLeituraData(&day, &month, &year);

  file.timestamp(T_WRITE, year, month, day, hour, minute, second);
  file.timestamp(T_ACCESS, year, month, day, hour, minute, second);

  file.print(F(";"));

  file.print(imprimirDoisDigitosHoras(hour, minute, second));

  file.println(F(";Hora:;x:;y:;z:;Distancia:;Flag:;;Bateria;Temperatura;Motores"));

  file.print(F(";"));
  file.print(anguloX);
  file.print(F(";"));
  file.print(anguloY);
  file.print(F(";"));
  file.print(anguloZ);
  file.print(F(";"));
  file.print(" ");
  file.print(F(";"));
  file.print(*Flag);
  file.print(F(";"));
  file.print(F(";"));
  file.print(tensao);
  file.print(F(";"));
  file.print(temperatura);
  file.print(F(";"));
  file.print(hour);

  file.println();

  if (!file.sync() || file.getWriteError()) {
    Serial.println(F("Erro ao escrever no arquivo"));
  }
  file.close();

}


char relatorioNomearArquivo(char nomeArquivo[]) {
  char buff[4];
  uint16_t year;
  uint8_t day, month, hour, minute, second;

  strcpy(fileName, nomeArquivo);
  strcat(fileName, '\0');

  uint8_t BASE_NAME_SIZE;

  for (int i = 0; i < 99; i++) {
    if (fileName[i] == '\0') {
      BASE_NAME_SIZE = i;
      break;
    }
  }

  strcat(fileName, "00 ");

  relogioLeituraData(&day, &month, &year);
  relogioLeituraHora(&hour, &minute, &second);

  sprintf(buff, "%02d-%02d-%02d", day, month, year);

  strcat(fileName, buff);

  strcat(fileName, ".csv");

  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      Serial.println(F("Erro ao criar nome de arquivo"));
    }
  }
}

void relatorioCriarArquivo(char* nomeArquivo[]) {
  uint16_t year;
  uint8_t day, month, hour, minute, second;

  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
    Serial.println(F("Erro ao abrir arquivo"));
  }

  // set creation date time
  file.timestamp(T_CREATE, year, month, day, hour, minute, second);
  file.timestamp(T_WRITE, year, month, day, hour, minute, second);
  file.timestamp(T_ACCESS, year, month, day, hour, minute, second);

  file.close();
}

void relogioLeituraData(uint8_t *dia, uint8_t *mes, uint16_t *ano) {
  Time  data = relogio.getTime();

  *dia = data.date;
  *mes = data.mon;
  *ano = data.year;
}
void relogioLeituraHora(uint8_t *horas, uint8_t *minutos, uint8_t *segundos) {
  Time  hora = relogio.getTime();

  *horas =    hora.hour;
  *minutos =  hora.min;
  *segundos = hora.sec;
}
