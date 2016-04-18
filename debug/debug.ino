#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// the default address is 0x40
Adafruit_PWMServoDriver* boards = new Adafruit_PWMServoDriver[9];

void setup()
{
  Wire.begin();
  
  Serial.begin(9600);
  Serial.println("Starting up");

  for (uint8_t i = 0; i < 9; i++)
  {
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40 + i);
    pwm.begin();
    pwm.setPWMFreq(800);  // This is the maximum PWM frequency
    boards[i] = pwm;
  }
  
}

void loop()
{
  // Fill with random data
  uint8_t* data = new uint8_t[144];
  uint8_t* fill = new uint8_t[3];
  fill[0] = 255;
  fill[1] = 150;
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
      int index = board_idx * 16 + channel;
      boards[board_idx].setPWM(channel, 0, data[index] << 4);
    }
  }
  
  delay(100);
  
}
