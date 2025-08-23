#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <GY521.h>
#include <NewPing.h>
#include <QTRSensors.h>
#include <EEPROM.h>

#define TRIGGER_PIN 40
#define ECHO_PIN 39
#define MAX_DISTANCE 50
#define TCAADDR 0x70

// Objetos
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
GY521 mpu(0x68);
QTRSensors qtr;
unsigned int sensorValues[8];

// Endereços da EEPROM
#define QTR_EEPROM_ADDR 0
#define QTR_EEPROM_MAGIC 0xA5A5
#define QTR_EEPROM_SIZE (2 + 2 + 8 * 2 * 2 + 2)  // magic + sensorCount + min/max + CRC

// Função CRC (igual à do seu código principal)
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

bool loadQTRCalibrationFromEEPROM() {
  qtr.calibrate();  // só para garantir que os arrays internos existam

  int addr = QTR_EEPROM_ADDR;
  uint16_t magic;
  EEPROM.get(addr, magic);
  addr += 2;

  if (magic != QTR_EEPROM_MAGIC) return false;

  uint16_t sensorCount;
  EEPROM.get(addr, sensorCount);
  addr += 2;

  if (sensorCount != 8) return false;

  uint16_t minOn[8], maxOn[8], minOff[8], maxOff[8];
  for (int i = 0; i < 8; i++) {
    EEPROM.get(addr, minOn[i]); addr += 2;
    EEPROM.get(addr, maxOn[i]); addr += 2;
  }
  for (int i = 0; i < 8; i++) {
    EEPROM.get(addr, minOff[i]); addr += 2;
    EEPROM.get(addr, maxOff[i]); addr += 2;
  }

  uint16_t crcStored;
  EEPROM.get(addr, crcStored);

  uint8_t buf[QTR_EEPROM_SIZE - 2];
  for (int i = 0; i < QTR_EEPROM_SIZE - 2; i++) {
    buf[i] = EEPROM.read(QTR_EEPROM_ADDR + i);
  }
  uint16_t crcCalc = crc16(buf, QTR_EEPROM_SIZE - 2);

  if (crcStored != crcCalc) return false;

  // Copia para dentro da lib QTR
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

// Sensores de cor
Adafruit_TCS34725 tcs[3] = {
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X)
};

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println(F("=== TESTE DE SENSORES ==="));

  // Inicializa MPU6050
  if (!mpu.begin()) {
    Serial.println(F("MPU6050 FALHOU!"));
  } else {
    Serial.println(F("MPU6050 OK"));
  }

  // Inicializa sensores de cor
  for (int i = 0; i < 3; i++) {
    tcaSelect(i);
    if (tcs[i].begin()) {
      Serial.print(F("TCS ")); Serial.print(i); Serial.println(F(" OK"));
    } else {
      Serial.print(F("TCS ")); Serial.print(i); Serial.println(F(" FALHOU"));
    }
  }

  // Inicializa QTR
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){43,44,45,46,47,48,49,50}, 8);
  qtr.setEmitterPin(51);
  Serial.println(F("QTR OK"));
  loadQTRCalibrationFromEEPROM();

}

void loop() {
  // Testa sonar
  int dist = sonar.ping_cm();
  Serial.print(F("Distancia Sonar: "));
  Serial.print(dist);
  Serial.println(F(" cm"));

  // Testa MPU6050
  mpu.read();
  Serial.print(F("Pitch: ")); Serial.print(mpu.getPitch());
  Serial.print(F(" | Roll: ")); Serial.print(mpu.getRoll());
  Serial.print(F(" | Yaw: ")); Serial.println(mpu.getYaw());

  // Testa sensores de cor
  for (int i = 0; i < 3; i++) {
    tcaSelect(i);
    uint16_t r, g, b, c;
    tcs[i].getRawData(&r, &g, &b, &c);
    Serial.print(F("TCS")); Serial.print(i);
    Serial.print(F(" -> R: ")); Serial.print(r);
    Serial.print(F(" G: ")); Serial.print(g);
    Serial.print(F(" B: ")); Serial.print(b);
    Serial.print(F(" C: ")); Serial.println(c);
  }

  // Testa QTR
  qtr.readCalibrated(sensorValues);
  Serial.print(F("QTR: "));
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  Serial.println(F("-----------------------------"));
}
