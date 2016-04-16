#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// the default address is 0x40
Adafruit_PWMServoDriver* boards = new Adafruit_PWMServoDriver[9];

void setup()
{
//  Serial.begin(9600);
//  Serial.println("16 channel PWM test!");

  for (uint8_t i = 0; i < 9; i++)
  {
    Adafruit_PWMServoDriver* pwm = new Adafruit_PWMServoDriver(i);
    pwm->begin();
    pwm->setPWMFreq(1600);  // This is the maximum PWM frequency
    boards[i] = *pwm;
  }
  
}

void loop()
{
  // Fill with random data
  uint8_t* data = new uint8_t[144];
  uint8_t* fill = new uint8_t[3];
  fill[0] = 0;
  fill[1] = 120;
  fill[2] = 255;
  for (int i = 0; i < 144; i++)
  {
    data[i] = fill[i%3];
  }
  
  // Bit shift and send to boards
  for (uint8_t board_idx = 0; board_idx < 9; board_idx++)
  {
    for (uint8_t channel = 0; channel < 16; channel++)
    {
      boards[board_idx].setPWM(channel, 0, data[board_idx * 16 + channel] << 4);
    }
  }
  
  
  
}
