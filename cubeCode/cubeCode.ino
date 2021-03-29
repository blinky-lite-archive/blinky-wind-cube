#include <OneWire.h>
#include <SPI.h> 
#define BAUD_RATE 57600
#define CHECKSUM 64

struct TransmitData
{
  float mAX31855_A = 0;
  float mAX31855_B = 0;
  float dS18B20_A = 0;
  float dS18B20_B = 0;
  byte extraInfo[36];
};
struct ReceiveData
{
  byte extraInfo[56];
};
struct DS18B20
{
  int signalPin;
  int powerPin;
  byte chipType;
  byte address[8];
  OneWire oneWire;
  float temp = 0.0;
};
struct MAX31855
{
  int chipSelectPin;
  float temp = 0.0;
};

DS18B20 dS18B20_A;
DS18B20 dS18B20_B;

MAX31855 mAX31855_A;
MAX31855 mAX31855_B;
SPISettings spiSettings;

void setupPins(TransmitData* tData, ReceiveData* rData)
{
  dS18B20_A.signalPin = 3;
  dS18B20_A.powerPin = 2;
  pinMode(dS18B20_A.powerPin, OUTPUT);
  digitalWrite(dS18B20_A.powerPin, HIGH);    
  dS18B20_A.oneWire = OneWire(dS18B20_A.signalPin);
  dS18B20_A.chipType = initDS18B20(dS18B20_A.address, &dS18B20_A.oneWire);

  dS18B20_B.signalPin = 5;
  dS18B20_B.powerPin = 4;
  pinMode(dS18B20_B.powerPin, OUTPUT);
  digitalWrite(dS18B20_B.powerPin, HIGH);    
  dS18B20_B.oneWire = OneWire(dS18B20_B.signalPin);
  dS18B20_B.chipType = initDS18B20(dS18B20_B.address, &dS18B20_B.oneWire);

  mAX31855_A.chipSelectPin = 10;
  pinMode (mAX31855_A.chipSelectPin, OUTPUT);
  digitalWrite(mAX31855_A.chipSelectPin,HIGH);

  mAX31855_B.chipSelectPin = 6;
  pinMode (mAX31855_B.chipSelectPin, OUTPUT);
  digitalWrite(mAX31855_B.chipSelectPin,HIGH);

  int sizeOfextraInfo = sizeof(tData->extraInfo);
  for (int ii = 0; ii < sizeOfextraInfo; ++ii) tData->extraInfo[ii] = 0;

  spiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE1);
  SPI.begin();
}
void processNewSetting(TransmitData* tData, ReceiveData* rData, ReceiveData* newData)
{
}
boolean processData(TransmitData* tData, ReceiveData* rData)
{
  
  dS18B20_A.temp = getDS18B20Temperature(&dS18B20_A.oneWire, dS18B20_A.address, dS18B20_A.chipType);
  dS18B20_B.temp = getDS18B20Temperature(&dS18B20_B.oneWire, dS18B20_B.address, dS18B20_B.chipType);

  mAX31855_A.temp = getMAX31855Temperature(mAX31855_A.chipSelectPin, spiSettings);
  mAX31855_B.temp = getMAX31855Temperature(mAX31855_B.chipSelectPin, spiSettings);

  tData->dS18B20_A = dS18B20_A.temp;
  tData->dS18B20_B = dS18B20_B.temp;
  tData->mAX31855_A = mAX31855_A.temp;
  tData->mAX31855_B = mAX31855_B.temp;
  delay(2000);
  return true;
}
float getDS18B20Temperature(OneWire* ow, byte* addr, byte chipType)
{
  byte i;
  byte data[12];
  float celsius;
  ow->reset();
  ow->select(addr);
  ow->write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(750);     // maybe 750ms is enough, maybe not
  
  ow->reset();
  ow->select(addr);    
  ow->write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) data[i] = ow->read();
  int16_t raw = (data[1] << 8) | data[0];
  if (chipType) 
  {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10)  raw = (raw & 0xFFF0) + 12 - data[6];
  }
  else 
  {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  }
  celsius = (float)raw / 16.0;
  return celsius;
  
}

byte initDS18B20(byte* addr, OneWire* ow)
{
  byte type_s = 0;
  if ( !ow->search(addr)) 
  {
    ow->reset_search();
    delay(250);
    return 0;
  }
   
  // the first ROM byte indicates which chip
  switch (addr[0]) 
  {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return 0;
  } 
  return type_s;
}
float getMAX31855Temperature(int chipSelect, SPISettings spiSetup)
{
  uint8_t  dataBufRead[4];
  int bits[32];
  int iTemp = 0;
  float fTemp;
  int pow2 = 1; 

  SPI.beginTransaction(spiSetup);
  digitalWrite (chipSelect, LOW);
  SPI.transfer(&dataBufRead, 4);
  digitalWrite (chipSelect, HIGH);
  SPI.endTransaction();

  for (int ibyte = 0; ibyte < 4; ++ibyte)
  {
    for (int ibit = 0; ibit < 8; ++ibit)
    {
      bits[31 - (ibyte * 8 + 7 - ibit)] = ((dataBufRead[ibyte] >> ibit) % 2);
    }
  }
  iTemp = 0;
  pow2 = 1;
  for (int ibit = 18; ibit < 31; ++ibit)
  {
    iTemp = iTemp + pow2 * bits[ibit];
    pow2 = pow2 * 2;
  }
  if (bits[31] > 0)
  {
    iTemp = iTemp - 8192;
  }
  fTemp = ((float) iTemp) * 0.25;
  return fTemp;
}

const int commLEDPin = 15;
boolean commLED = true;

struct TXinfo
{
  int cubeInit = 1;
  int newSettingDone = 0;
  int checkSum = CHECKSUM;
};
struct RXinfo
{
  int newSetting = 0;
  int checkSum = CHECKSUM;
};

struct TX
{
  TXinfo txInfo;
  TransmitData txData;
};
struct RX
{
  RXinfo rxInfo;
  ReceiveData rxData;
};
TX tx;
RX rx;
ReceiveData settingsStorage;

int sizeOfTx = 0;
int sizeOfRx = 0;

void setup()
{
  setupPins(&(tx.txData), &settingsStorage);
  pinMode(commLEDPin, OUTPUT);  
  digitalWrite(commLEDPin, commLED);

  sizeOfTx = sizeof(tx);
  sizeOfRx = sizeof(rx);
  Serial1.begin(BAUD_RATE);
  delay(1000);
}
void loop()
{
  boolean goodData = false;
  goodData = processData(&(tx.txData), &settingsStorage);
  if (goodData)
  {
    tx.txInfo.newSettingDone = 0;
    if(Serial1.available() > 0)
    { 
      commLED = !commLED;
      digitalWrite(commLEDPin, commLED);
      Serial1.readBytes((uint8_t*)&rx, sizeOfRx);
      
      if (rx.rxInfo.checkSum == CHECKSUM)
      {
        if (rx.rxInfo.newSetting > 0)
        {
          processNewSetting(&(tx.txData), &settingsStorage, &(rx.rxData));
          tx.txInfo.newSettingDone = 1;
          tx.txInfo.cubeInit = 0;
        }
      }
      else
      {
        Serial1.end();
        for (int ii = 0; ii < 50; ++ii)
        {
          commLED = !commLED;
          digitalWrite(commLEDPin, commLED);
          delay(100);
        }

        Serial1.begin(BAUD_RATE);
        tx.txInfo.newSettingDone = 0;
        tx.txInfo.cubeInit = -1;
      }
    }
    Serial1.write((uint8_t*)&tx, sizeOfTx);
    Serial1.flush();
  }
}
