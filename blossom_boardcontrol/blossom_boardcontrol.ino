#include <Adafruit_PWMServoDriver.h>
#include <crc16.h>

// Defines
#define RTSPIN 2
#define CTSPIN 3
#define NUM_BOARDS 9
#define CH_PER_BOARD 16
#define MAGIC_LENGTH 4

const byte magic[] = {0xde, 0xad, 0xbe, 0xef};
Adafruit_PWMServoDriver *boards = new Adafruit_PWMServoDriver[NUM_BOARDS];
crc16 crc;


void setup()
{
  // Start serial
  Serial.begin(115200);
  
  // Setup pins
  pinMode(RTSPIN, INPUT);
  pinMode(CTSPIN, OUTPUT);
  
  // Setup breakout boards
  for (uint8_t i = 0; i < NUM_BOARDS; i++)
  {
    // the default address is 0x40
    Adafruit_PWMServoDriver *pwm = new Adafruit_PWMServoDriver(0x40 + i);
    pwm->begin();
    pwm->setPWMFreq(1600);  // 1600? is the maximum PWM frequency
    boards[i] = *pwm;
  }
  
}

uint8_t *readData(uint16_t len)
{
  uint8_t *data = new uint8_t[len];
  for (uint8_t i = 0; i < len; i++)
  {
    // TODO check that Serial is available. while loop?
    uint8_t datam = Serial.read();
    data[i] = datam;
    crc.updateCrc(datam);
  }
  return data;
}

void sendData(uint8_t *data)
{
  for (uint8_t board_idx = 0; board_idx < NUM_BOARDS; board_idx++)
  {
    for (uint8_t channel = 0; channel < CH_PER_BOARD; channel++)
    {
      int index = board_idx * CH_PER_BOARD + channel;
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

uint16_t getTwoBytesSerial()
{
  uint8_t low = Serial.read();
  uint16_t high = Serial.read() << 8;
  uint16_t combined = high + low;
  return combined;
}

void loop()
{
  digitalWrite(CTSPIN, HIGH);
  if (Serial.available())
  {
    if (magicFound())
    {
      digitalWrite(CTSPIN, LOW);
      
      uint8_t lengthLow = Serial.read();
      uint16_t lengthHigh = Serial.read() << 8;
      uint16_t len = lengthHigh + lengthLow;

      crc.clearCrc();
      uint8_t *data = readData(len);
      uint16_t packetCrc = getTwoBytesSerial();
      
      if (crc.getCrc() == packetCrc)
      {
        sendData(data);
        digitalWrite(CTSPIN, HIGH);
      }
      
    }
  }
  
  
}

