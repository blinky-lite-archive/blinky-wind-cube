#include <SPI.h> 
#define CS0Pin 10
#define CS1Pin 6
#define blueLedPin 15

boolean blueLed = true; 
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE1); 

void setup() 
{
  pinMode (blueLedPin, OUTPUT);
  pinMode (CS0Pin, OUTPUT);
  pinMode (CS1Pin, OUTPUT);
  digitalWrite(blueLedPin,blueLed);
  digitalWrite(CS0Pin,1);
  digitalWrite(CS1Pin,1);
  Serial.begin(9600);
  SPI.begin();
}

void loop() 
{
  float fTemp1 = 0.0;
  float fTemp2 = 0.0;
  blueLed = !blueLed;
  digitalWrite(blueLedPin,blueLed);
  fTemp1 = readTemp(CS0Pin);
  fTemp2 = readTemp(CS1Pin);
  Serial.print(fTemp1);
  Serial.print(" ");
  Serial.println(fTemp2);
  delay(1000);  
}

float readTemp(int chipSelect)
{
  uint8_t  dataBufRead[4];
  int bits[32];
  int iTemp = 0;
  float fTemp;
  int pow2 = 1; 

  SPI.beginTransaction(spiSettings);
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
