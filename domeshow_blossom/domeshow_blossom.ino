#include <dscom.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define DEBUG

#define BOARD_IDX 7

#define NUM_BOARDS 9
#define CH_PER_BOARD 16

Adafruit_PWMServoDriver* board;

DSCom dsc;

void setup() {
    Serial.begin(115200);
    Serial.write('B');
    
    uint8_t addresses[] = {0x42, 0x44, 0x41, 0x45, 0x40, 0x43, 0x46, 0x47, 0x48};
    
    // Setup breakout boards
    board = new Adafruit_PWMServoDriver(0x40 + BOARD_IDX);
    board->begin();
    board->setPWMFreq(1600);  // 1600? is the maximum PWM frequency
    
}

void loop() {
    dsc.read();
    if (dsc.isUpdated()) {
        #ifdef DEBUG
            Serial.println("WRITING");
        #endif
        for (uint8_t channel = 0; channel < CH_PER_BOARD; channel++) {
            uint16_t index = BOARD_IDX * CH_PER_BOARD + channel;
            board->setPWM(channel, 0, dsc.getData()[index] << 4);
        }
        #ifdef DEBUG
            Serial.println("Done writing");
        #endif
    }
}
