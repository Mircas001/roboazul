#include <AFMotor.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <GY521.h>
#include <NewPing.h>
#include <Servo.h>
#include <ArduinoLog.h>
#include <QTRSensors.h>
#include <math.h>
#include <EEPROM.h>

QTRSensors qtr;

#define LEDA 41
#define LEDB 42
#define TRIGGER_PIN 40
#define ECHO_PIN 39
#define CALIB_FLAG 34  // conecte no GND para calibrar os sensores QTR
#define LED1 38
#define LED2 37
#define LED3 36
#define LED4 35
#define MAX_DISTANCE 50
#define TCAADDR 0x70
#define DIREITA 0
#define ESQUERDA 1

#define INTERVALO_LEITURA 5
#define TEMPO_PRE90 1000
#define TEMPO_ORBITA 750
/*
Tabela
8V - 80 VEL
*/

#define VEL_NORMAL 98
#define VEL_RESISTENCIA 130
#define VEL_CURVA 190
#define VEL_CURVA_EXTREMA 220
#define DISTANCIA_OBSTACULO 7
#define DISTANCIA_PARADA 15
#define DISTANCIA_MINIMA_VIRADA 10
#define LIMIAR_LINHA 995

#define MAX_SATURACAO_SEGUIDA 50  // Número de leituras saturadas seguidas para considerar defeituoso
#define MAX_NORMAL_SEGUIDO 50     // Número de leituras normais seguidas para reviver

#define SERVO_PIN 10
#define MAX_FALHAS_CONTINUAS 10
#define MAX_BUFFER_CORES 10

#define KP 90.0f
#define KI 0.0f
#define KD 6.0f

#define FILTER_SAMPLES 5
float pitchSamples[FILTER_SAMPLES];
int sampleIndex = 0;

// Enumerações com descrições detalhadas
enum class Estado {
  SEGUINDO_LINHA,         // Estado normal de seguimento de linha
  RESOLVENDO_BIFURCACAO,  // Encontrou uma bifurcação na pista
  DESVIANDO_OBSTACULO,    // Detectou obstáculo e está desviando
  SALA_RESGATE,
  PARADO,        // Robô parado (emergência ou final de percurso)
  INICIALIZANDO  // Estado inicial durante boot
};

enum Codigos {
  OP_SEGUINDO_LINHA = 1,         // 0001 - Operando normalmente
  OP_RESOLVENDO_BIFURCACAO = 2,  // 0010 - Resolvendo bifurcação
  OP_DESVIANDO_OBSTACULO = 3,    // 0011 - Desviando de obstáculo
  OP_PARADO = 4,                 // 0100 - Parado
  ERRO_MPU = 5,                  // 0101 - Erro no sensor MPU6050
  ERRO_TCS_ESQUERDA = 6,         // 0110 - Erro no sensor de cor esquerdo
  ERRO_TCS_DIREITA = 7,          // 0111 - Erro no sensor de cor direito
  ERRO_QTR_SATURADO = 8,         // 1000 - Erro nos sensores de linha QTR
  ERRO_I2C = 9,                  // 1001 - Erro de comunicação I2C
  CALIBRANDO = 10,               // 1010 - Calibrando
  ERRO_TCS_CENTRAL = 11,         // 1011 - Erro no sensor de cor central
  OP_SALA_RESGATE = 12,          // 1100 - na sala de resgate
  OP_INICIALIZANDO = 15          // 1111 - Inicializando
};

enum Cores {
  PRETO,
  BRANCO,
  VERMELHO,
  VERDE,
  AZUL,
  CINZA,
  ERRO
};

struct CorSensor {
  Adafruit_TCS34725 tcs;      // Objeto do sensor de cor
  bool inicializado = false;  // Flag de inicialização
};

#define ANGULO_FRENTE   90
#define ANGULO_45_DIREITA   45
#define ANGULO_45_ESQUERDA  135
#define ANGULO_DIREITA  0
#define ANGULO_ESQUERDA 180
#define LIMIAR_PAREDE   35   // cm
#define LIMIAR_BURACO   70   // cm
#define DELAY_ANGULO    85
#define DELAY_ANDAR     250

NewPing sonarLongo(TRIGGER_PIN, ECHO_PIN, 400);

// Instanciação de objetos com logs detalhados
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
GY521 mpu(0x68);
bool mpuFuncionando = true;
const int MAX_TENTATIVAS_MPU = 3;
bool tcsFuncionando[2] = { false, false };
volatile Estado estadoAtual = Estado::INICIALIZANDO;
unsigned long ultimaAtualizacaoLED = 0;
const unsigned long INTERVALO_LED_MS = 200;
int ultimoCodigoMostrado = -1;

Servo servoUltrassonico;
// Por algum motivo, a primeira classe declarada sempre não funcionava, isso foi embora

AF_DCMotor motorTrasEsquerdo(3);
AF_DCMotor motorFrenteEsquerdo(4);
AF_DCMotor motorTrasDireito(2);
AF_DCMotor motorFrenteDireito(1);

CorSensor corSensores[3];
Cores bufferCores[MAX_BUFFER_CORES];
int indiceBuffer = 0;

unsigned int sensorValues[8];
int sensoresValidos = 8;  // ou inicialize conforme necessário
int validosDireita = 4;
int validosEsquerda = 4;

float media = 0.0;
float ultimaMedia = 0.0;

#define QTR_EEPROM_ADDR 0
#define QTR_EEPROM_MAGIC 0xA5A5
#define QTR_EEPROM_SIZE (2 + 2 + 8 * 2 * 2 + 2)  // magic + sensorCount + min/max for 8 sensors (on/off) + CRC

// ====== Configuração de limiares ======
#define LIMIAR_BRANCO 7000       // Branco puro (todos os canais acima)
#define LIMIAR_BRANCO_SUJO 3500  // Média mínima para branco sujo
#define TOL_BRANCO_SUJO 1750     // Diferença máxima entre canais para branco sujo
#define LIMIAR_PRETO_R 1600
#define LIMIAR_PRETO_G 1700
#define LIMIAR_PRETO_B 1600
#define TOLERANCIA_CINZA 300      // Diferença máxima entre canais para cinza
#define FATOR_PREDOMINANCIA 1.5f  // Quanto um canal deve ser maior para ser predominantew

// CRC-16-CCITT
uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

void saveQTRCalibrationToEEPROM() {
  int addr = QTR_EEPROM_ADDR;
  EEPROM.put(addr, QTR_EEPROM_MAGIC);
  addr += 2;
  EEPROM.put(addr, qtr.getType() == QTRType::RC ? 8 : 0);
  addr += 2;
  for (int i = 0; i < 8; i++) {
    EEPROM.put(addr, qtr.calibrationOn.minimum[i]);
    addr += 2;
    EEPROM.put(addr, qtr.calibrationOn.maximum[i]);
    addr += 2;
  }
  for (int i = 0; i < 8; i++) {
    EEPROM.put(addr, qtr.calibrationOff.minimum ? qtr.calibrationOff.minimum[i] : 0);
    addr += 2;
    EEPROM.put(addr, qtr.calibrationOff.maximum ? qtr.calibrationOff.maximum[i] : 0);
    addr += 2;
  }
  // CRC
  uint8_t buf[QTR_EEPROM_SIZE - 2];
  for (int i = 0; i < QTR_EEPROM_SIZE - 2; i++)
    buf[i] = EEPROM.read(QTR_EEPROM_ADDR + i);
  uint16_t crc = crc16(buf, QTR_EEPROM_SIZE - 2);
  EEPROM.put(addr, crc);
}

bool loadQTRCalibrationFromEEPROM() {
  qtr.calibrate();  // este é uma calibração temporária para garantir que os arrays estejam alocados, ou seja, só cria os arrays
  int addr = QTR_EEPROM_ADDR;
  uint16_t magic;
  EEPROM.get(addr, magic);
  addr += 2;
  if (magic != QTR_EEPROM_MAGIC)
    return false;
  uint16_t sensorCount;
  EEPROM.get(addr, sensorCount);
  addr += 2;
  if (sensorCount != 8)
    return false;
  uint16_t minOn[8], maxOn[8], minOff[8], maxOff[8];
  for (int i = 0; i < 8; i++) {
    EEPROM.get(addr, minOn[i]);
    addr += 2;
    EEPROM.get(addr, maxOn[i]);
    addr += 2;
  }
  for (int i = 0; i < 8; i++) {
    EEPROM.get(addr, minOff[i]);
    addr += 2;
    EEPROM.get(addr, maxOff[i]);
    addr += 2;
  }
  uint16_t crcStored;
  EEPROM.get(addr, crcStored);
  uint8_t buf[QTR_EEPROM_SIZE - 2];
  for (int i = 0; i < QTR_EEPROM_SIZE - 2; i++)
    buf[i] = EEPROM.read(QTR_EEPROM_ADDR + i);
  uint16_t crcCalc = crc16(buf, QTR_EEPROM_SIZE - 2);
  if (crcStored != crcCalc)
    return false;
  // Copy to QTR
  for (int i = 0; i < 8; i++) {
    qtr.calibrationOn.minimum[i] = minOn[i];
    qtr.calibrationOn.maximum[i] = maxOn[i];
    if (qtr.calibrationOff.minimum)
      qtr.calibrationOff.minimum[i] = minOff[i];
    if (qtr.calibrationOff.maximum)
      qtr.calibrationOff.maximum[i] = maxOff[i];
  }
  qtr.calibrationOn.initialized = true;
  qtr.calibrationOff.initialized = true;
  return true;
}

void setup() {
  Serial.begin(9600);

  pinMode(CALIB_FLAG, INPUT_PULLUP);  // Use pull-up so default is HIGH

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  ledCountTest();

  if (Serial) {
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  }

  Log.noticeln(F("============================================="));
  Log.noticeln(F("| Iniciando sistema do seguidor de linha    |"));
  Log.noticeln(F("| Feito em 07/08/25                         |"));
  Log.noticeln(F("| Log level: VERBOSE                        |"));
  Log.noticeln(F("============================================="));

  Log.verboseln(F("Inicializando servo..."));
  servoUltrassonico.attach(SERVO_PIN);
  servoUltrassonico.write(90);

  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);

  // Inicialização I2C e MPU6050
  Log.verboseln(F("Iniciando comunicação I2C..."));
  Wire.begin();

  Log.verboseln(F("Iniciando MPU..."));
  mpu.begin();
  for (int i = 0; i < MAX_TENTATIVAS_MPU; i++) {
    Log.verboseln("Tentativa %d de inicializar MPU...", i + 1);
    if (verificarMPU()) {
      break;
    }
    delay(100);
  }

  if (!mpuFuncionando) {
    Log.error("Falha ao inicializar MPU6050 após %d tentativas. Verifique as conexões e o sensor.", MAX_TENTATIVAS_MPU);
    ledBinOutput(ERRO_MPU);
  } else {
    Log.noticeln(F("MPU6050 inicializado com sucesso"));
  }

  Log.verboseln(F("Inicializando MPU6050..."));
  mpu.setAccelSensitivity(0);
  mpu.setGyroSensitivity(0);
  mpu.setThrottle(false);
  mpu.setNormalize(true);  // desativa normalização automática

  if (mpuFuncionando) {
    Log.verboseln(F("Calibrando MPU..."));
    mpu.calibrate(2000);
    Log.noticeln(F("MPU6050 inicializado e calibrado"));
  } else {
    Log.error(F("Falha na inicialização do MPU6050"));
  }

  // Configuração de pinos com logs
  Log.verboseln(F("Configurando pinos de LED..."));

  ledBinOutput(CALIBRANDO);

  // Calibração dos sensores QTR
  Log.verboseln(F("Iniciando calibração dos sensores QTR..."));
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 43, 44, 45, 46, 47, 48, 49, 50 }, 8);
  qtr.setEmitterPin(51);

  bool doCalib = digitalRead(CALIB_FLAG) == LOW;  // Calibra se o pino estiver LOW
  Log.verboseln("Calibração QTR: %s", doCalib);
  bool loaded = false;
  if (!doCalib) {
    loaded = loadQTRCalibrationFromEEPROM();
    if (loaded)
      Log.noticeln(F("QTR calibração carregada da EEPROM."));
    else
      Log.warningln(F("EEPROM inválida, calibrando QTR."));
  }
  if (doCalib || !loaded) {
    for (int i = 0; i < 1000; i++) {
      qtr.calibrate();
      if (i % 50 == 0)
        Log.verboseln("Calibração QTR: %d/1000", i);
      delay(20);
    }
    pararMotores();
    Log.noticeln(F("Calibração QTR concluída"));
    saveQTRCalibrationToEEPROM();
    Log.noticeln(F("Calibração QTR salva na EEPROM."));
    while (true) {
      ledBinOutput(OP_PARADO);
    }
  }

  ledBinOutput(OP_INICIALIZANDO);

  delay(500);

  // Configuração dos LEDs dos sensores de cor
  Log.verboseln(F("Configurando LEDs dos sensores de cor..."));
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);
  desligarLEDs();

  // Inicialização dos sensores de cor
  Log.verboseln(F("Inicializando sensores de cor..."));
  corSensores[0].tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
  corSensores[1].tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
  corSensores[2].tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

  tcaSelect(0);
  corSensores[0].inicializado = corSensores[0].tcs.begin();
  Log.verboseln("Sensor de cor 0 (esquerda) - %s", corSensores[0].inicializado ? "OK" : "FALHA");

  tcaSelect(1);
  corSensores[1].inicializado = corSensores[1].tcs.begin();
  Log.verboseln("Sensor de cor 1 (direita) - %s", corSensores[1].inicializado ? "OK" : "FALHA");

  tcaSelect(2);
  corSensores[2].inicializado = corSensores[2].tcs.begin();
  Log.verboseln("Sensor de cor 3 (especial) - %s", corSensores[2].inicializado ? "OK" : "FALHA");

  // Rotina inicial
  estadoAtual = Estado::INICIALIZANDO;
  Log.noticeln(F("Sistema inicializado com sucesso"));
  Log.noticeln(F("Estado inicial: SEGUINDO_LINHA"));
}

void loop() {
  Log.verboseln("%d", static_cast<int>(estadoAtual));

  switch (estadoAtual) {
    case Estado::SEGUINDO_LINHA:
      {
        ledBinOutput(OP_SEGUINDO_LINHA);

        int distancia = sonar.ping_cm();
        if (distancia < DISTANCIA_OBSTACULO && distancia != 0) {
          estadoAtual = Estado::DESVIANDO_OBSTACULO;
          return;
        }

        Cores corCentral = quickDetectarCor(2);

        if (corCentral == VERMELHO) {
          estadoAtual = Estado::PARADO;
          return;
        } else if (corCentral == CINZA) {
          estadoAtual = Estado::SALA_RESGATE;
          return;
        }
        lerSensores();
        media = calcularPosicaoLinha();

        if (media == 69) {
          estadoAtual = Estado::RESOLVENDO_BIFURCACAO;
          return;
        }

        // PID variables
        static float integral = 0.0;
        static float lastError = 0.0;
        float setpoint = 0.0;
        float error = setpoint - media;
        integral += error;
        float derivative = error - lastError;
        lastError = error;

        int correcaoPID = KP * error + KI * integral + KD * derivative;
        ultimaMedia = media;

        int velocidade1 = VEL_NORMAL - correcaoPID;

        int velocidade2 = VEL_NORMAL + correcaoPID;

        controleFino(velocidade1, velocidade2);
        // Remove delay para máxima velocidade
        break;
      }
    case Estado::INICIALIZANDO:
      {
        Log.warning(F("Estado INICIALIZANDO não deveria ser alcançado no loop"));
        estadoAtual = Estado::SEGUINDO_LINHA;
        return;
      }
    case Estado::RESOLVENDO_BIFURCACAO:
      {
        pararMotores();
        Log.noticeln(F("Iniciando RESOLVENDO_BIFURCACAO..."));
        ledBinOutput(OP_RESOLVENDO_BIFURCACAO);

        // Executa a resolução apenas uma vez
        static bool bifurcacaoResolvida = false;
        if (!bifurcacaoResolvida) {
          resolverBifurcacao();
          bifurcacaoResolvida = true;
          Log.noticeln(F("Bifurcação resolvida. Pronto para voltar a seguir linha"));
        }

        // Volta para SEGUINDO_LINHA apenas quando confirmado
        lerSensores();

        media = calcularPosicaoLinha();

        if (media != 69) {  // Só volta se não estiver mais na bifurcação
          estadoAtual = Estado::SEGUINDO_LINHA;
          bifurcacaoResolvida = false;
          Log.noticeln(F("Voltando para SEGUINDO_LINHA"));
        }
        return;
      }
    case Estado::DESVIANDO_OBSTACULO:
      {
        ledBinOutput(OP_DESVIANDO_OBSTACULO);
        Log.noticeln(F("Iniciando desvio de obstáculo..."));
        desviarObstaculo();
        Log.noticeln(F("Obstáculo desviado. Voltando para SEGUINDO_LINHA"));
        estadoAtual = Estado::SEGUINDO_LINHA;
        break;
      }
    case Estado::SALA_RESGATE:
      salaDeResgate();
      estadoAtual = Estado::SEGUINDO_LINHA;  // Volta a seguir linha após sair
      break;
    case Estado::PARADO:
      {
        Log.warning(F("ROBÔ PARADO!"));
        pararMotores();
        break;
      }
    default:
      {
        Log.warningln("PORQUE RAIOS ESTÁ NO DEFAULT? ISSO NÃO É POSSIVEL! O ESTADO ATUAL É: %d", estadoAtual);
      }
  }
  delay(INTERVALO_LEITURA);
}

void controlarMotores(int esq, int dir, int vel = VEL_NORMAL) {
  motorFrenteEsquerdo.setSpeed(vel);
  motorFrenteDireito.setSpeed(vel);
  motorTrasEsquerdo.setSpeed(vel);
  motorTrasDireito.setSpeed(vel);
  motorFrenteEsquerdo.run(esq ? FORWARD : BACKWARD);
  motorFrenteDireito.run(dir ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run(esq ? FORWARD : BACKWARD);
  motorTrasDireito.run(dir ? FORWARD : BACKWARD);
}

void controleFino(int esq, int dir) {
  int absEsq = constrain(abs(esq), 0, 255);
  int absDir = constrain(abs(dir), 0, 255);
  motorFrenteEsquerdo.setSpeed(absEsq);
  motorFrenteDireito.setSpeed(absDir);
  motorTrasEsquerdo.setSpeed(absEsq);
  motorTrasDireito.setSpeed(absDir);

  motorFrenteEsquerdo.run((esq > 0) ? FORWARD : BACKWARD);
  motorFrenteDireito.run((dir > 0) ? FORWARD : BACKWARD);
  motorTrasEsquerdo.run((esq > 0) ? FORWARD : BACKWARD);
  motorTrasDireito.run((dir > 0) ? FORWARD : BACKWARD);
}

void lerSensores() {
  qtr.readCalibrated(sensorValues);  // Lê todos os 8 sensores do QTR-8RC
}

float calcularPosicaoLinha() {
  int somaPonderada = 0;
  int total = 0;
  const int posicoes[8] = { -15000, -8000, -4000, -1000, 1000, 4000, 8000, 15000 };
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > LIMIAR_LINHA) {
      somaPonderada += posicoes[i];
      total++;
    }
  }
  if (total == 8) {
    pararMotores();
    andarTras();
    delay(100);
    pararMotores();
    return 69;  // Indica bifurcação ou linha complet
  }
  if (total == 0)
    return 0;
  return somaPonderada / total / 1000;
}

void resolverBifurcacao() {
  Log.verboseln("Resolverbifurcacao()");
  ligarLEDs();
  Cores corA = detectarCor(0);
  Cores corB = detectarCor(1);
  desligarLEDs();

  Log.verboseln("corA: %d | corB: %d", corA, corB);

  andarReto();
  delay(300);

  // Checa erro antes de seguir
  if (corA == ERRO || corB == ERRO) {
    Log.errorln(F("Falha na detecção de cor! corA: %d | corB: %d"), corA, corB);
    pararMotores();
    delay(500);  // Ou lidar de forma mais inteligente
    return;
  }

  if (corA == VERDE && corB != VERDE) {
    virar90(ESQUERDA);
  } else if (corA != VERDE && corB == VERDE) {
    virar90(DIREITA);
  } else if (corA == VERDE && corB == VERDE) {
    virar90(DIREITA);
    virar90(DIREITA);
  } else {
    Log.warningln(F("Condição de bifurcação inesperada. Executando andarReto."));
    andarReto();
    delay(1350);
  }
}

void desviarObstaculo() {
  pararMotores();
  delay(100);
  virar90(DIREITA);
  delay(100);
  andarReto();
  delay(TEMPO_ORBITA / 2);
  unsigned long ultimaVirada = millis();

  while (true) {
    lerSensores();
    float HaLinha = calcularPosicaoLinha();
    if (HaLinha != 0) {
      pararMotores();
      virar90(DIREITA);
      return;
    }

    andarReto();
    if (millis() - ultimaVirada >= TEMPO_ORBITA) {
      pararMotores();
      delay(100);
      virar90(ESQUERDA);
      delay(100);
      ultimaVirada = millis();
    }
    delay(10);
  }
}

void velocidadeMaxima() {
  motorFrenteDireito.setSpeed(255);
}

void atualizarBufferCor(Cores novaCor) {
  bufferCores[indiceBuffer] = novaCor;
  indiceBuffer = (indiceBuffer + 1) % MAX_BUFFER_CORES;
}

Cores classificarCor(uint16_t r, uint16_t g, uint16_t b) {
  // ====== Função genérica para classificar uma cor ======
  if (r < 2000 && g < 3000 && b < 2000)
    return PRETO;

  // 1. Branco puro
  if (r > LIMIAR_BRANCO && g > LIMIAR_BRANCO && b > LIMIAR_BRANCO)
    return BRANCO;

  // 2. Branco sujo (média alta e pouca diferença entre canais)
  int media = (r + g + b) / 3;
  if (media > LIMIAR_BRANCO_SUJO && abs((int)r - (int)g) < TOL_BRANCO_SUJO && abs((int)r - (int)b) < TOL_BRANCO_SUJO && abs((int)g - (int)b) < TOL_BRANCO_SUJO)
    return BRANCO;

  // 3. Preto
  if (r < LIMIAR_PRETO_R && g < LIMIAR_PRETO_G && b < LIMIAR_PRETO_B)
    return PRETO;

  // 4. Cinza
  if (abs((int)r - (int)g) < TOLERANCIA_CINZA && abs((int)r - (int)b) < TOLERANCIA_CINZA && abs((int)g - (int)b) < TOLERANCIA_CINZA)
    return CINZA;

  // 5. Cores predominantes
  if (g > r * FATOR_PREDOMINANCIA && g > b * FATOR_PREDOMINANCIA)
    return VERDE;
  if (r > g * FATOR_PREDOMINANCIA && r > b * FATOR_PREDOMINANCIA)
    return VERMELHO;
  if (b > r * FATOR_PREDOMINANCIA && b > g * FATOR_PREDOMINANCIA)
    return AZUL;

  // 6. Erro ou indefinido
  return ERRO;
}

// ====== detectarCor() com fallback ======
Cores detectarCor(uint8_t canal) {
  if (canal > 2 || !corSensores[canal].inicializado) {
    unsigned int sensorValues[8];
    qtr.readCalibrated(sensorValues);
    int soma = 0;
    for (int i = 0; i < 8; i++) soma += sensorValues[i];
    int media = soma / 8;
    if (media > 400 && media < 900)
      return VERDE;
    return PRETO;
  }

  tcaSelect(canal);
  uint16_t r, g, b, c;
  corSensores[canal].tcs.getRawData(&r, &g, &b, &c);
  Log.verboseln("R:%d G:%d B:%d", r, g, b);
  tcaSelect(2);  // deixa sempre no canal 2
  return classificarCor(r, g, b);
}

// ====== quickDetectarCor() sem tcaSelect ======
Cores quickDetectarCor(uint8_t canal) {
  uint16_t r, g, b, c;
  corSensores[canal].tcs.getRawData(&r, &g, &b, &c);
  return classificarCor(r, g, b);
}

bool verificarMPU() {
  static int tentativasFalhas = 0;
  if (mpu.begin()) {
    mpu.readGyro();
    if (mpu.getGyroX() == 0 && mpu.getGyroY() == 0 && mpu.getGyroZ() == 0)
      tentativasFalhas++;
    else {
      tentativasFalhas = 0;
      mpuFuncionando = true;
      return true;
    }
  } else
    tentativasFalhas++;

  if (tentativasFalhas >= MAX_TENTATIVAS_MPU) {
    mpuFuncionando = false;
    ledBinOutput(ERRO_MPU);
  }
  return false;
}

void virar90(int direcao) {
  if (mpuFuncionando)
    virarComGiro(90, direcao);
  else {
    virarForte(direcao);
    delay(700);
    pararMotores();
  }
}

void virarComGiro(float angulo, int direcao) {
  pararMotores();

  if (!verificarMPU()) {
    Serial.println("MPU não detectada! Girando sem correção...");
    virarForte(direcao);
    delay(angulo * 10);
    pararMotores();
    return;
  }

  mpu.read();
  float yawInicial = mpu.getYaw();
  float yawAtual = yawInicial;
  float giroAcumulado = 0;

  const float margem = 2.0;  // tolerância final

  // Definir o valor final do ângulo de destino
  float anguloDestino = yawAtual + angulo;

  // Corrigir o ângulo destino para garantir que esteja dentro de -180 a 180 graus
  if (anguloDestino > 180) anguloDestino -= 360;
  if (anguloDestino < -180) anguloDestino += 360;

  while (fabs(giroAcumulado) < fabs(angulo)) {
    mpu.read();
    float novoYaw = mpu.getYaw();

    // Calcular a diferença entre o novo yaw e o atual
    float delta = novoYaw - yawAtual;
    if (delta > 180)
      delta -= 360;
    if (delta < -180)
      delta += 360;

    giroAcumulado += delta;
    yawAtual = novoYaw;

    // Calcular o erro em relação ao ângulo de destino
    float erro = anguloDestino - yawAtual;
    int vel = map(fabs(erro), 0, angulo, VEL_CURVA, VEL_CURVA_EXTREMA);
    vel = constrain(vel, 50, VEL_CURVA);

    // Controlar os motores na direção correta
    if (direcao == DIREITA)
      controlarMotores(ESQUERDA, DIREITA, vel);
    else
      controlarMotores(DIREITA, ESQUERDA, vel);

    delay(10);
  }

  pararMotores();
  delay(80);
}

void pararMotores() {
  controlarMotores(1, 1, 0);
}
void andarReto() {
  controlarMotores(1, 1, VEL_NORMAL);
}
void andarRapido() {
  controlarMotores(1, 1, 200);
}
void andarTras() {
  controlarMotores(0, 0, VEL_NORMAL + 10);
}
void virar(int direcao) {
  controlarMotores(direcao, !direcao, VEL_CURVA);
}
void virarForte(int direcao) {
  controlarMotores(direcao, !direcao, VEL_CURVA_EXTREMA);
}

void vencerResistenciaInicial() {
  controlarMotores(1, 1, VEL_RESISTENCIA);
  delay(100);
}

void desligarLEDs() {
  digitalWrite(LEDA, LOW);
  digitalWrite(LEDB, LOW);
  delay(200);
}

void ligarLEDs() {
  digitalWrite(LEDA, HIGH);
  digitalWrite(LEDB, HIGH);
  delay(200);
}

void tcaSelect(uint8_t channel) {
  if (channel > 7)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void ledCountTest() {
  digitalWrite(LED1, HIGH);
  delay(125);
  digitalWrite(LED2, HIGH);
  delay(125);
  digitalWrite(LED3, HIGH);
  delay(125);
  digitalWrite(LED4, HIGH);
  delay(12);
}

void ledBinOutput(int code) {
  unsigned long agora = millis();
  if ((agora - ultimaAtualizacaoLED < INTERVALO_LED_MS) && (code == ultimoCodigoMostrado))
    return;

  digitalWrite(LED1, (code & 0b0001) ? HIGH : LOW);
  digitalWrite(LED2, (code & 0b0010) ? HIGH : LOW);
  digitalWrite(LED3, (code & 0b0100) ? HIGH : LOW);
  digitalWrite(LED4, (code & 0b1000) ? HIGH : LOW);

  ultimaAtualizacaoLED = agora;
  ultimoCodigoMostrado = code;
}

int lerDistancia(int angulo) {
  servoUltrassonico.write(angulo);
  delay(DELAY_ANGULO);
  int dist = sonarLongo.ping_cm();
  if (dist == 0) dist = 400;
  return dist;
}

bool temParedeNaFrente() {
  int distFrente = lerDistancia(ANGULO_FRENTE);
  return distFrente < LIMIAR_PAREDE;
}

void contornarTriangulo(int ladoParede) {
  // ladoParede: 0 = direita, 1 = esquerda
  pararMotores();
  delay(80);
  // Vira 45° para o lado da parede
  virarComGiro(45, ladoParede == 0 ? DIREITA : ESQUERDA);
  andarReto();
  delay(5000);  // Anda por 5 segundos
  pararMotores();
  delay(80);
  // Vira mais 45° para o mesmo lado (total 90°)
  virarComGiro(45, ladoParede == 0 ? DIREITA : ESQUERDA);
  andarReto();
  delay(250);  // Ajuste: anda um pouco para estabilizar
  pararMotores();
  delay(80);
}

bool temParedeColada(int dist) {
  return dist <= 15;
}

bool temBuraco(int dist) {
  return dist > LIMIAR_BURACO;
}

void salaDeResgate() {
  Log.noticeln(F("Entrando na sala de resgate..."));

  while (true) {
    // 1. Anda reto até chegar perto da parede
    andarReto();
    int distFrente = lerDistancia(ANGULO_FRENTE);
    if (distFrente < DISTANCIA_PARADA) {
      pararMotores();
      delay(200);
      
      // 2. Olha para os lados
      int distDir = lerDistancia(ANGULO_45_DIREITA);
      int distEsq = lerDistancia(ANGULO_45_ESQUERDA);
      Log.noticeln("Dist Esq: %d | Dist Dir: %d", distEsq, distDir);

      // 3. Decide para onde ir
      int direcaoEscolhida;
      if (distEsq > 100 && distEsq > distDir) {
        direcaoEscolhida = ESQUERDA;
        Log.noticeln(F("Ambiente aberto à esquerda. Indo para lá."));
      } else if (distDir > 100 && distDir > distEsq) {
        direcaoEscolhida = DIREITA;
        Log.noticeln(F("Ambiente aberto à direita. Indo para lá."));
      } else {
        direcaoEscolhida = (distEsq < distDir) ? ESQUERDA : DIREITA;
        Log.noticeln("Seguindo lado mais próximo: %s", direcaoEscolhida == ESQUERDA ? "Esquerda" : "Direita");
      }

      // 4. Virar e seguir
      virar90(direcaoEscolhida);
      andarReto();
      delay(500);

      // 5. Verifica linha
      lerSensores();
      float pos = calcularPosicaoLinha();
      if (pos != 0) {
        pararMotores();
        Log.noticeln(F("Linha encontrada! Saindo da sala de resgate."));
        return;
      }
    }

    delay(50); // Evita leitura excessiva
  }
}
