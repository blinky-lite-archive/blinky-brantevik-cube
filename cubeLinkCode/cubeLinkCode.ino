#include <SPI.h>
#include "RH_RF95.h"
#define BAUD_RATE 57600
#define CHECKSUM 64
#define TIMEOUTMILLIS 30000

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

RH_RF95::ModemConfigChoice modeConfig[] = {
      RH_RF95::ModemConfigChoice::Bw125Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw500Cr45Sf128, 
      RH_RF95::ModemConfigChoice::Bw31_25Cr48Sf512, 
      RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096};

RH_RF95 rf95(RFM95_CS, RFM95_INT);
struct Radiopacket
{
  byte transAddr = 25;
  float windSpeed = 0.0;
  float windDirection = 0.0;
  float temp = 0.0;
  float measuredvbat = 0.0;
  byte extraInfo[2];
  byte endByte = 25;
};
Radiopacket radiopacket;
uint8_t sizeOfRadioExtraInfo = sizeof(radiopacket.extraInfo);
uint8_t sizeOfRadiopacket = sizeof(radiopacket);

boolean pin9Value = false;
boolean pin12Value = true;
int modemConfigIndex = 1;
int deviceType = 0;
float deviceRfFreq[] = {433.300, 433.800};
byte deviceTransAddr[] = {23,25};

struct TransmitData
{
  float windSpeed = 0.0;
  float windDirection = 0.0;
  float temp = 0.0;
  float measuredvbat = 0.0;
  int   signalStrength = 0.0;
  int   deviceType = 0;
  byte extraInfo[28];
};
struct ReceiveData
{
  int loopDelay = 100;
  byte extraInfo[52];
};

void setupPins(TransmitData* tData, ReceiveData* rData)
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(9, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(9, pin9Value);
  digitalWrite(12, pin12Value);
  delay(100);
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  rf95.init();
  rf95.setFrequency(deviceRfFreq[0]);
  rf95.setModemConfig(modeConfig[modemConfigIndex]); 
  rf95.setModeRx();
//  Serial.begin(9600);
  delay(500);
}
void processNewSetting(TransmitData* tData, ReceiveData* rData, ReceiveData* newData)
{
  rData->loopDelay = newData->loopDelay;
}
boolean processData(TransmitData* tData, ReceiveData* rData)
{
  boolean waitForData = true;
  boolean timeOut = false;
  unsigned long lastWriteTime = millis();
  rf95.setFrequency(deviceRfFreq[tData->deviceType]);
  delay(1000);
  while (waitForData && !timeOut)
  {
    if (rf95.recv((uint8_t *)&radiopacket, &sizeOfRadiopacket))
    {
      if ((deviceTransAddr[tData->deviceType] == radiopacket.transAddr) && (deviceTransAddr[tData->deviceType] == radiopacket.endByte))
      {
        pin9Value = false;
        if (radiopacket.extraInfo[sizeOfRadioExtraInfo - 2] == 1) pin9Value = true;
        pin12Value = false;
        if (radiopacket.extraInfo[sizeOfRadioExtraInfo - 1] == 1) pin12Value = true;
        digitalWrite(9, pin9Value);
        digitalWrite(12, pin12Value);
        tData->windSpeed      = radiopacket.windSpeed;
        tData->windDirection  = radiopacket.windDirection;
        tData->temp           = radiopacket.temp;
        tData->measuredvbat   = radiopacket.measuredvbat;
        tData->signalStrength = rf95.lastRssi();
/*
        Serial.print(tData->windSpeed);
        Serial.print(',');
        Serial.print(tData->temp);
        Serial.print(',');
        Serial.print(tData->windDirection);
        Serial.print(',');
        Serial.print(tData->measuredvbat);
        Serial.print(',');
        Serial.println(tData->signalStrength);
*/  
        waitForData = false;
      }
    }
    if ((millis() - lastWriteTime) > TIMEOUTMILLIS)
    {
      timeOut = true;
      tData->windSpeed      = -100.0;
      tData->windDirection  = -100.0;
      tData->temp           = -100.0;
      tData->measuredvbat   = -100.0;
      for (int ii = 0; ii < 20; ++ii)
      {
        digitalWrite(9, pin9Value);
        digitalWrite(12, pin9Value);
        pin9Value = !pin9Value;
        delay(100);
      }
      digitalWrite(9, true);
      digitalWrite(12, true);
    }
  }
  tData->deviceType = tData->deviceType + 1;
  if (tData->deviceType > 1) tData->deviceType = 0;
  return true;
}


const int commLEDPin = 13;
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
