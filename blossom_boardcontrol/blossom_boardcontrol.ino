#include "Adafruit_PWMServoDriver/Adafruit_PWMServoDriver.h"
#include "Crc16/Crc16.h"
#include <Wire.h>

Adafruit_PWMServoDriver *boards = new Adafruit_PWMServoDriver[9];
Crc16 crc16;
const int PIN_CTS = 3;
const int PIN_RTS = 4;

void setup()
{
  // Start serial
  Serial.begin(9600);
  Serial.println("Starting up");
  
  // Setup pins
  pinMode(PIN_CTS, OUTPUT);
  pinMode(PIN_RTS, INPUT);
  
  // Setup breakout boards
  for (uint8_t i = 0; i < 9; i++)
  {
    // the default address is 0x40
    Adafruit_PWMServoDriver *pwm = new Adafruit_PWMServoDriver(0x40 + i);
    pwm->begin();
    pwm->setPWMFreq(1400);  // 1600? is the maximum PWM frequency
    boards[i] = *pwm;
  }
  
}

uint8_t *readData(uint16_t length)
{
  uint8_t *data = new uint8_t[length];
  for (uint8_t i = 0; i < length; i++)
  {
    uint8_t datam = Serial.read();
    data[i] = datam;
    crc16.updateCrc(datam);
  }
  return data;
}

void sendData(uint8_t *data)
{
  for (uint8_t board_idx = 0; board_idx < 9; board_idx++)
  {
    for (uint8_t channel = 0; channel < 16; channel++)
    {
      int index = board_idx * 16 + channel;
      boards[board_idx].setPWM(channel, 0, data[index] << 4);
    }
  }
}

boolean magicFound()
{
  if (Serial.peek() == 0xde) Serial.read();
  else return false;
  
  if (Serial.peek() == 0xad) Serial.read();
  else return false;
  
  if (Serial.peek() == 0xbe) Serial.read();
  else return false;
  
  if (Serial.peek() == 0xef) Serial.read();
  else return false;

  return true;
}

void loop()
{
  digitalWrite(PIN_CTS, HIGH);
  if (Serial.available())
  {
    if (magicFound())
    {
      digitalWrite(PIN_CTS, LOW);
      
      uint8_t lengthLow = Serial.read();
      uint16_t lengthHigh = Serial.read() << 8;
      uint16_t length = lengthHigh + lengthLow;

      crc16.clearCrc();
      uint8_t *data = readData(length);

      uint8_t crcLow = Serial.read();
      uint16_t crcHigh = Serial.read() << 8;
      uint16_t receivedCrc = crcHigh + crcLow;
      if (crc16.getCrc() == receivedCrc)
      {
        sendData(data);
        digitalWrite(PIN_CTS, HIGH);
      }
      
    }
  }
  
  
}
