//  Definições de comunicação
#define Tamanho_mensagem_Enviada   17
#define Tamanho_mensagem_Recebida  8
#define headerByte                 0xEF
#define endByte                    0xFE

//  Definições de pinos
#define MOTOR_A2_PIN    7
#define MOTOR_B2_PIN    8
#define MOTOR_A1_PIN    4
#define MOTOR_B1_PIN    9
#define PWM_MOTOR_2     5
#define PWM_MOTOR_1     6
#define ENABLE_MOTOR_1  A0
#define ENABLE_MOTOR_2  A1

#define SENSOR_TENSAO   A2
#define SENSOR_CORRENTE A3


//  Definições de usuário
#define Timer_leitura_corrente     10
#define Timer_leitura_tensao       500
#define Timer_leitura_temperatura  3000
#define Timer_leitura_pressao      1200
#define Timer_rede_Timeout         300
#define Timer_leitura_acelerometro 10


//  Configurações MPU6050
#define MPU_6050_addr 0x68

#define GYRO_LSB_2_DEGSEC  65.5     // [bit/(°/s)]
#define ACC_LSB_2_G        16384.0  // [bit/gravity]
#define RAD_2_DEG          57.29578 // [°/rad]
#define GYRO_OFFSET_NB_MES 3000     //
#define TEMP_LSB_2_DEGREE  340.0    // [bit/celsius]
#define TEMP_LSB_OFFSET    12412.0

float gyroXoffset, gyroYoffset, gyroZoffset;
long preInterval;
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
float accCoef, gyroCoef;


//  Bibliotecas
#include <SPI.h>
#include <Ethernet2.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface

// Network
byte mac[] =        { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] =         { 192, 168, 0, 10};
byte gateway[] =    { 192, 168, 0, 10};
byte subnet[] =     { 255, 255, 0, 0 };

byte     server[] = { 192, 168, 0, 120 }; // Google
uint16_t port = 23;

byte mensagemRecebida[Tamanho_mensagem_Recebida];
byte mensagemEnvio[Tamanho_mensagem_Enviada];

byte recebimentoMensagem;
bool statusConexao;


uint16_t contadorEthernetTimeout;
uint16_t contadorLeituraTemp;
uint16_t contadorLeituraPress;
uint16_t contadorLeituraCorr;
uint16_t contadorLeituraTensao;
uint16_t contadorLeituraAcel;
uint16_t counterAcel;

int motorAceleracaoEsquerda;
int motorAceleracaoDireita;
uint16_t contadorAcelMotor;


EthernetServer Controle(23);  // create a server at port 23

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

  setupTimer1();

  Serial.begin (57600);

  TCCR0B = TCCR0B & B11111000 | B00000010;    // define o set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  // isso afeta millis() e delay, então cuidado.

  pinMode(10, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  Ethernet.begin(mac, ip);//, gateway, gateway, subnet);
  //Zinho.setConnectionTimeout(100);
  Controle.begin();           // start to listen for clients
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  configAcelerometro();

  bmp.begin();
  //    Serial.println("Iniciado com sucesso");
  //  else {
  //    Serial.println("Erro ao iniciar");
  ////    while (1);
  //  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1,  OUTPUT);
  pinMode(PWM_MOTOR_2,  OUTPUT);
  pinMode(ENABLE_MOTOR_1,  OUTPUT);
  pinMode(ENABLE_MOTOR_2,  OUTPUT);

  pinMode(SENSOR_TENSAO, INPUT);
  pinMode(SENSOR_CORRENTE, INPUT);

  digitalWrite(ENABLE_MOTOR_1, HIGH);
  digitalWrite(ENABLE_MOTOR_2, HIGH);

  mensagemEnvio[0]  = headerByte;
  mensagemEnvio[16] = endByte;

}

ISR(TIMER1_COMPA_vect) {
  contadorEthernetTimeout++;
  contadorLeituraTemp++;
  contadorLeituraPress++;
  contadorLeituraCorr++;
  contadorLeituraTensao++;
  contadorLeituraAcel++;

  counterAcel++;
  contadorAcelMotor++;
}

void loop() {
  static uint8_t acelMotorEsquerda;
  static uint8_t acelMotorDireita;
  uint8_t pwmMotorEsquerda;
  uint8_t pwmMotorDireita;
  bool    direcaoMotorEsquerda;
  bool    direcaoMotorDireita;

  EthernetClient Zinho = Controle.available();

  if (Zinho) {
    if (Zinho.available()) {
      for (byte i = 0; i < Tamanho_mensagem_Recebida; i++)
        mensagemRecebida[i] = Zinho.read();
    }

    statusConexao = true;
    contadorEthernetTimeout = 0;


    if (mensagemRecebida[0] == headerByte && mensagemRecebida[7] == endByte) {                //  Verificação da integridade da mensagem recebida
      if (mensagemRecebida[6] = ethernetCRC8(mensagemRecebida, Tamanho_mensagem_Recebida)) {

        pwmMotorEsquerda =            mensagemRecebida[1];                                    //  Define valores às variáveis apenas se passar na condição de integridade
        direcaoMotorEsquerda = (bool) mensagemRecebida[2];
        pwmMotorDireita =             mensagemRecebida[3];
        direcaoMotorDireita  = (bool) mensagemRecebida[4];

      }
    }

    //  Envio de mensagem via ethernet

    mensagemEnvio[15] = ethernetCRC8(mensagemEnvio, Tamanho_mensagem_Enviada);                //  Faz cálculo de CRC para redundância de validação de integridade
    mensagemEnvio[16] = endByte;
    Controle.write(mensagemEnvio, Tamanho_mensagem_Enviada);                                  //  Responde ao recebimento da mensagem


    //  Ações após recebimento de mensagem


  }

  //  Realiza verificação de desconexão e, caso haja uma, para os motores
  statusConexao = ethernetDetectaDesconexao(&contadorEthernetTimeout, Timer_rede_Timeout);


  //  Realiza controle dos motores, caso esteja conectado, caso contrário, os motores estarão parados
  if (statusConexao) {
    motorAceleracao(pwmMotorEsquerda, &acelMotorEsquerda, pwmMotorDireita, &acelMotorDireita, 20000);

    motorGo(1, direcaoMotorEsquerda, acelMotorEsquerda, motorBreak(acelMotorEsquerda, acelMotorDireita));                                       //  Envia valor PWM aos motores apenas no recebimento da mensagem
    motorGo(2, direcaoMotorDireita,  acelMotorDireita,  motorBreak(acelMotorEsquerda, acelMotorDireita));
  }


  //  Realiza leitura dos sensores com um tempo pré determinado
  if (timerControle(&contadorLeituraAcel, Timer_leitura_acelerometro))
    leituraAcelerometro(&mensagemEnvio[1], &mensagemEnvio[2], &mensagemEnvio[3], &mensagemEnvio[4], &mensagemEnvio[5], &mensagemEnvio[6]);

  if (timerControle(&contadorLeituraTemp, Timer_leitura_temperatura))
    leituraTemperatura(&mensagemEnvio[7], &mensagemEnvio[8]);

  if (timerControle(&contadorLeituraPress, Timer_leitura_pressao))
    leituraPressao(&mensagemEnvio[9], &mensagemEnvio[10]);

  if (timerControle(&contadorLeituraCorr, Timer_leitura_corrente))
    leituraCorrente(&mensagemEnvio[11], &mensagemEnvio[12]);

  if (timerControle(&contadorLeituraTensao, Timer_leitura_tensao))
    leituraTensao(&mensagemEnvio[13], &mensagemEnvio[14]);

}


bool timerControle(uint16_t *contador, uint16_t tempo) {
  if (*contador > tempo) {
    *contador = 0;
    return true;
  } else
    return false;
}

bool ethernetDetectaDesconexao(uint16_t *contadorEthernetTimeout, uint16_t tempoTimeOut) {
  if (*contadorEthernetTimeout > tempoTimeOut) {
    *contadorEthernetTimeout = 0;
    analogWrite(PWM_MOTOR_1, 0);
    analogWrite(PWM_MOTOR_2, 0);
    digitalWrite(MOTOR_A1_PIN, HIGH);
    digitalWrite(MOTOR_B1_PIN, HIGH);
    digitalWrite(MOTOR_A2_PIN, HIGH);
    digitalWrite(MOTOR_B2_PIN, HIGH);

    Serial.println("Desconectado por TimeOut!");

    return false;
  }

  return true;
}

byte ethernetCRC8(byte mensagem[], byte tamanhoMensagem) {
  byte crc = 0;

  for (byte i = 0; i < tamanhoMensagem; i++)
    crc ^= mensagem[i];

  return crc;
}

void leituraTemperatura(byte *byte0, byte *byte1) {

  float temperatura = bmp.readTemperature();

  floatParaByte(temperatura, &(*byte0), &(*byte1));

  //  Serial.println(temperatura);

}

void leituraPressao(byte *byte0, byte *byte1) {

  float pressao = (bmp.readPressure() / 1000);

  floatParaByte(pressao, &(*byte0), &(*byte1));
  //  Serial.println(pressao);
}


void leituraTensao(byte *byte0, byte *byte1) {

  float tensao = (float)analogRead(A2) / 47.7777;

  floatParaByte(tensao, &(*byte0), &(*byte1));

}

void leituraCorrente(byte *byte0, byte *byte1) {
  static float   EMA_S;
  static uint8_t leiturasDadas;
  static float   leitura;

  if (leiturasDadas < 30) {
    leiturasDadas++;
    leitura = leitura + (float)analogRead(A3) - 511;
  } else {
    leitura = leitura / 30;
    EMA_S = (0.4 * leitura) + ((1 - 0.4) * EMA_S); //run the EMA
    leiturasDadas = 0;
    leitura = 0;
    float corrente = (float) EMA_S / 1023.0 * 5.0 / 0.1255;

    corrente = (float) fabs(double(corrente));

    floatParaByte(corrente, &(*byte0), &(*byte1));
  }

}

void configAcelerometro() {
  Wire.begin();
  Wire.beginTransmission(MPU_6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  float xyz[3] = {0, 0, 0};
  int16_t b;

  for (int i = 0; i < GYRO_OFFSET_NB_MES; i++) {
    Wire.beginTransmission(MPU_6050_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_6050_addr, 6);

    for (int j = 0; j < 3; j++) {
      b  = Wire.read() << 8;
      b |= Wire.read();
      xyz[j] += ((float)b) / GYRO_LSB_2_DEGSEC;
    }
  }
  gyroXoffset = xyz[0] / GYRO_OFFSET_NB_MES;
  gyroYoffset = xyz[1] / GYRO_OFFSET_NB_MES;
  gyroZoffset = xyz[2] / GYRO_OFFSET_NB_MES;

  accCoef = 1.0 - 0.98;
  gyroCoef = 0.98;
}

void leituraAcelerometro(byte *byte0, byte *byte1, byte *byte2, byte *byte3, byte *byte4, byte *byte5) {

  Wire.beginTransmission(MPU_6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_6050_addr, 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for (int i = 0; i < 7; i++) {
    rawData[i]  = Wire.read() << 8;
    rawData[i] |= Wire.read();
  }

  accX = ((float)rawData[0]) / ACC_LSB_2_G;
  accY = ((float)rawData[1]) / ACC_LSB_2_G;
  accZ = ((float)rawData[2]) / ACC_LSB_2_G;
  //  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / GYRO_LSB_2_DEGSEC - gyroXoffset;
  gyroY = ((float)rawData[5]) / GYRO_LSB_2_DEGSEC - gyroYoffset;
  gyroZ = ((float)rawData[6]) / GYRO_LSB_2_DEGSEC - gyroZoffset;

  float sgZ = (accZ >= 0) - (accZ < 0);
  angleAccX = atan2(accY, sgZ * sqrt(accZ * accZ + accX * accX)) * RAD_2_DEG;
  angleAccY = - atan2(accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;

  //  unsigned long Tnew = millis();
  //  float dt = (Tnew - preInterval) * 1e-3;
  //  preInterval = Tnew;

  //  unsigned long Tnew = millis();
  float dt = counterAcel * 1e-3;
  counterAcel = 0;

  angleX = (gyroCoef * (angleX + gyroX * dt)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * dt)) + (accCoef * angleAccY);
  angleZ += gyroZ * dt;


  floatParaByte(angleY, &(*byte0), &(*byte1));
  floatParaByte(angleX, &(*byte2), &(*byte3));
  floatParaByte(angleZ, &(*byte4), &(*byte5));

}

void motorAceleracao (uint8_t pwm1, uint8_t *pwmAceleracao1, uint8_t pwm2, uint8_t *pwmAceleracao2, uint16_t tempo) {
  static uint32_t oldMicros;

  if (micros() - oldMicros > tempo) {
    if (*pwmAceleracao1 < pwm1)
      *pwmAceleracao1 = *pwmAceleracao1 + 1;

    if (*pwmAceleracao1 > pwm1)
      *pwmAceleracao1 = *pwmAceleracao1 - 1;

    if (*pwmAceleracao2 < pwm2)
      *pwmAceleracao2++;

    if (*pwmAceleracao2 > pwm2)
      *pwmAceleracao2--;

    oldMicros = micros();
  }
}

bool motorBreak(uint8_t pwm1, uint8_t pwm2) {

  if (pwm1 > 0 && pwm2 > 0)
    return false;

  return true;
}

void motorGo(uint8_t motor, bool direc, byte pwm, bool breaks) {
  switch (motor) {
    case 1:
      if (!breaks) {
        if (direc) {
          digitalWrite(MOTOR_A1_PIN, LOW);
          digitalWrite(MOTOR_B1_PIN, HIGH);
        } else {
          digitalWrite(MOTOR_A1_PIN, HIGH);
          digitalWrite(MOTOR_B1_PIN, LOW);
        }

        analogWrite(PWM_MOTOR_1, pwm);
      } else {
        analogWrite(PWM_MOTOR_1, 0);
        digitalWrite(MOTOR_A1_PIN, HIGH);
        digitalWrite(MOTOR_B1_PIN, HIGH);
      }
      break;

    case 2:
      if (!breaks) {
        if (direc) {
          digitalWrite(MOTOR_A2_PIN, LOW);
          digitalWrite(MOTOR_B2_PIN, HIGH);
        } else {
          digitalWrite(MOTOR_A2_PIN, HIGH);
          digitalWrite(MOTOR_B2_PIN, LOW);
        }

        analogWrite(PWM_MOTOR_2, pwm);
      } else {
        analogWrite(PWM_MOTOR_2, 0);
        digitalWrite(MOTOR_A2_PIN, HIGH);
        digitalWrite(MOTOR_B2_PIN, HIGH);
      }
      break;
  }
}

void floatParaByte(float variavel, byte * byte0, byte * byte1) {
  long conversao = long ((variavel) * 100);

  *byte0 = highByte(conversao);
  *byte1 = lowByte(conversao);
}

float  byteParaFloat(byte byte0, byte byte1) {
  return (float)(long (((byte0 << 8) | (byte1)))) / 100;
}
