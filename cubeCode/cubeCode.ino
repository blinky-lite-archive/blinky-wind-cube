#define BAUD_RATE 9600
#define CHECKSUM 64
#define R0 9.85
#define V0 1010.0
int anemometerLEDPin = 3;
int anemometerPin = 4;
int windVanePin = A0;

volatile boolean anemometerLED = false;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long nowTime = 0;
volatile float fsamp = 16.0;
volatile float anemometerPulseInt = 10000.0;

float windDirR[16] = {33.0,  6.57, 8.2,  0.891, 1.0,    0.688,   2.2,   1.41,  3.9,   3.14,  16.0,  14.12, 120.0,  42.12,  64.9,  21.88};
float windDir[16] =  { 0.0, 22.5, 45.0, 67.5,  90.0, 112.5,    135.0, 157.5, 180.0, 202.5,  225.0, 247.5,  270.0, 292.5,  315.0, 337.5 };
float windVolts[16];

struct TransmitData
{
  float windSpeed = 0.0;
  float windDirection = 0.0;
  byte extraInfo[44];
};
struct ReceiveData
{
  int loopDelay = 1000;
  float faverageSamples = 16.0;
  byte extraInfo[48];
};

void anemometerPulseHandler()
{
  float deltaT;
  nowTime = millis();
  deltaT = (float) (nowTime - lastPulseTime);
  lastPulseTime = nowTime;
  anemometerLED = !anemometerLED;
  digitalWrite(anemometerLEDPin,anemometerLED);
  if (deltaT > 10000.0) return;
  anemometerPulseInt = anemometerPulseInt + (deltaT - anemometerPulseInt) / fsamp;
  return;
}

float windDirectionLookup(float sensV)
{
  float errorMin = abs(sensV - windVolts[0]);
  float error = 0.0;
  int iwindDir = 0;
  for (int ii = 1; ii < 16; ++ii)
  {
    error = abs(sensV - windVolts[ii]);
    if (errorMin > error)
    {
      errorMin = error;
      iwindDir = ii;
    }
  }
  return(windDir[iwindDir]);
}

void setupPins(TransmitData* tData, ReceiveData* rData)
{
  pinMode(anemometerLEDPin, OUTPUT);
  pinMode(anemometerPin, INPUT);
  pinMode(windVanePin, INPUT);
  attachInterrupt(anemometerPin, anemometerPulseHandler, RISING);
  digitalWrite(anemometerLEDPin,anemometerLED);
  lastPulseTime = millis();
  for (int ii = 0; ii < 16; ++ii) windVolts[ii] = V0 * windDirR[ii] / (R0 +  windDirR[ii]);
  fsamp = rData->faverageSamples;
}
void processNewSetting(TransmitData* tData, ReceiveData* rData, ReceiveData* newData)
{
  rData->loopDelay = newData->loopDelay;
  rData->faverageSamples = newData->faverageSamples;
  fsamp = rData->faverageSamples;
}
boolean processData(TransmitData* tData, ReceiveData* rData)
{
  tData->windSpeed = 666.7 / anemometerPulseInt;
  tData->windDirection = windDirectionLookup((float) analogRead(windVanePin));
  delay(rData->loopDelay);
  return true;
}


const int commLEDPin = 5;
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
  Serial.begin(BAUD_RATE);
  delay(1000);  
  int sizeOfextraInfo = sizeof(tx.txData.extraInfo);
  for (int ii = 0; ii < sizeOfextraInfo; ++ii) tx.txData.extraInfo[ii] = 0;

}
void loop()
{
  boolean goodData = false;
  goodData = processData(&(tx.txData), &settingsStorage);
  if (goodData)
  {
    tx.txInfo.newSettingDone = 0;
    if(Serial.available() > 0)
    { 
      commLED = !commLED;
      digitalWrite(commLEDPin, commLED);
      Serial.readBytes((char*)&rx, sizeOfRx);
      
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
        Serial.end();
        for (int ii = 0; ii < 50; ++ii)
        {
          commLED = !commLED;
          digitalWrite(commLEDPin, commLED);
          delay(100);
        }

        Serial.begin(BAUD_RATE);
        tx.txInfo.newSettingDone = 0;
        tx.txInfo.cubeInit = -1;
      }
    }
    Serial.write((uint8_t*)&tx, sizeOfTx);
    Serial.flush();
  }
}
