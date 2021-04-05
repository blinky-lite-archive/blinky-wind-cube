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
}

void loop() 
{
  float windSpeed = 2400.0 / anemometerPulseInt;
  float windDirection = windDirectionLookup((float) analogRead(windVanePin));
  Serial.print(windSpeed);
  Serial.print(',');
  Serial.println(windDirection);
  delay(1000);        
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
