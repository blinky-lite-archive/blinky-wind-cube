#include <OneWire.h>
#define R0 9.85
#define V0 1010.0
int anemometerLEDPin = 8;
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

struct DS18B20
{
  int signalPin;
  int powerPin;
  byte chipType;
  byte address[8];
  OneWire oneWire;
  float temp = 0.0;
};
DS18B20 dS18B20_A;

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
}

void setup() 
{
  // initialize the digital pin as an output.
  pinMode(anemometerLEDPin, OUTPUT);
  pinMode(anemometerPin, INPUT);
  pinMode(windVanePin, INPUT);
  attachInterrupt(anemometerPin, anemometerPulseHandler, RISING);
  digitalWrite(anemometerLEDPin,anemometerLED);
  lastPulseTime = millis();
  for (int ii = 0; ii < 16; ++ii) windVolts[ii] = V0 * windDirR[ii] / (R0 +  windDirR[ii]);
 
  dS18B20_A.signalPin = 20;
  dS18B20_A.powerPin = 22;
  pinMode(dS18B20_A.powerPin, OUTPUT);
  digitalWrite(dS18B20_A.powerPin, HIGH);    
  dS18B20_A.oneWire = OneWire(dS18B20_A.signalPin);
  dS18B20_A.chipType = initDS18B20(dS18B20_A.address, &dS18B20_A.oneWire);

}

void loop() 
{
  float windSpeed = 2400.0 / anemometerPulseInt;
  float windDirection = windDirectionLookup((float) analogRead(windVanePin));
  dS18B20_A.temp = getDS18B20Temperature(&dS18B20_A.oneWire, dS18B20_A.address, dS18B20_A.chipType);
  Serial.print(windSpeed);
  Serial.print(',');
  Serial.print(windDirection);
  Serial.print(',');
  Serial.println(dS18B20_A.temp);
  delay(100);        
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
