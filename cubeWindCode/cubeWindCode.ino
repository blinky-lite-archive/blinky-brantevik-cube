#include <SPI.h>
#include "RH_RF95.h"
#include "OneWire.h"

#define VBATPIN A7
#define R0 9.85
#define V0 1010.0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};
 
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int sigPower = 20;
int modemConfigIndex = 1;
float rfFreq = 433.550;
int loopDelay = 10000;

int commLEDPin = 13;
boolean commLED = false;

struct DS18B20
{
  int signalPin = 12;
  int powerPin = 11;
  byte chipType;
  byte address[8];
//  OneWire oneWire;
  float temp = 0.0;
};
DS18B20 dS18B20_A;
OneWire oneWireA(dS18B20_A.signalPin);

struct Radiopacket
{
  byte transAddr = 24;
  float windSpeed = 0.0;
  float windDirection = 0.0;
  float temp = 0.0;
  float measuredvbat = 0.0;
  byte extraInfo[2];
  byte endByte = 24;
};
Radiopacket radiopacket;
uint8_t sizeOfextraInfo = sizeof(radiopacket.extraInfo);
uint8_t sizeOfRadiopacket = sizeof(radiopacket);

int anemometerPin = 20;
int windVanePin = A0;

volatile unsigned long lastPulseTime = 0;
volatile unsigned long nowTime = 0;
volatile float fsamp = 64.0;
volatile float anemometerPulseInt = 10000.0;

float windDirR[16] = {33.0,  6.57, 8.2,  0.891, 1.0,    0.688,   2.2,   1.41,  3.9,   3.14,  16.0,  14.12, 120.0,  42.12,  64.9,  21.88};
float windDir[16] =  { 0.0, 22.5, 45.0, 67.5,  90.0, 112.5,    135.0, 157.5, 180.0, 202.5,  225.0, 247.5,  270.0, 292.5,  315.0, 337.5 };
float windVolts[16];

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
  if (deltaT > 10000.0) return;
  anemometerPulseInt = anemometerPulseInt + (deltaT - anemometerPulseInt) / fsamp;
}
void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(commLEDPin, OUTPUT);  
  digitalWrite(commLEDPin, commLED);
  delay(100);
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  rf95.init();
  rf95.setFrequency(rfFreq);
  rf95.setModemConfig(modeConfig[modemConfigIndex]); 

  rf95.setModeTx();
  rf95.setTxPower(sigPower, false);
  for (int ii = 0; ii < sizeOfextraInfo; ++ii) radiopacket.extraInfo[ii] = 0;

  pinMode(anemometerPin, INPUT);
  pinMode(windVanePin, INPUT);
  attachInterrupt(anemometerPin, anemometerPulseHandler, RISING);
  lastPulseTime = millis();
  for (int ii = 0; ii < 16; ++ii) windVolts[ii] = V0 * windDirR[ii] / (R0 +  windDirR[ii]);

  pinMode(VBATPIN, INPUT);
  
  pinMode(dS18B20_A.powerPin, OUTPUT);
  digitalWrite(dS18B20_A.powerPin, HIGH);
  delay(500);
  dS18B20_A.chipType = initDS18B20(dS18B20_A.address, &oneWireA);

//  Serial.begin(9600);
  delay(500);
}

void loop() 
{
  digitalWrite(commLEDPin, HIGH);
  digitalWrite(dS18B20_A.powerPin, HIGH); 
  delay(500);  
  dS18B20_A.temp = getDS18B20Temperature(&oneWireA, dS18B20_A.address, dS18B20_A.chipType);
  radiopacket.temp = dS18B20_A.temp;

  radiopacket.windDirection = windDirectionLookup((float) analogRead(windVanePin));
  radiopacket.windSpeed = 666.7 / anemometerPulseInt;
  radiopacket.measuredvbat = measureBattery();

  commLED = !commLED;
  radiopacket.extraInfo[sizeOfextraInfo - 2] = 0;
  if (commLED) radiopacket.extraInfo[sizeOfextraInfo - 2] = 1;
  radiopacket.extraInfo[sizeOfextraInfo - 1] = 0;
  if (!commLED) radiopacket.extraInfo[sizeOfextraInfo - 1] = 1;
  digitalWrite(commLEDPin, HIGH);
  rf95.send((uint8_t *)&radiopacket, sizeOfRadiopacket);
  digitalWrite(commLEDPin, LOW);
/* 
  Serial.print(radiopacket.windSpeed);
  Serial.print(',');
  Serial.print(radiopacket.temp);
  Serial.print(',');
  Serial.print(radiopacket.windDirection);
  Serial.print(',');
  Serial.println(radiopacket.measuredvbat);
*/ 
  delay(500);
  digitalWrite(dS18B20_A.powerPin, LOW); 
  digitalWrite(commLEDPin, LOW);
  
  delay(loopDelay); 
}
float measureBattery()
{
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
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
