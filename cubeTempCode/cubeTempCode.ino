#include <SPI.h>
#include "RH_RF95.h"
#include "OneWire.h"

#define VBATPIN A7
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
float rfFreq = 433.300;
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
  byte transAddr = 23;
  float hold1 = 0.0;
  float hold2 = 0.0;
  float temp = 0.0;
  float measuredvbat = 0.0;
  byte extraInfo[2];
  byte endByte = 23;
};
Radiopacket radiopacket;
uint8_t sizeOfextraInfo = sizeof(radiopacket.extraInfo);
uint8_t sizeOfRadiopacket = sizeof(radiopacket);

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
  Serial.print(radiopacket.temp);
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
