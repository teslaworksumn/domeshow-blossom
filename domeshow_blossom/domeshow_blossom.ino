#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <crc16.h>

#define DEBUG
#ifdef DEBUG
boolean messagewalk = false;  // Prevents spamming Serial monitor
#endif

#define BOARD_IDX 5

#define NUM_BOARDS 9
#define CH_PER_BOARD 16

#define MAGIC_LENGTH 4
const byte magic[] = {0xde, 0xad, 0xbe, 0xef};
byte magic_status = 0;

#define STATE_READY     0
#define STATE_READING   1
#define STATE_WRITING   2
byte state = STATE_READY;

Adafruit_PWMServoDriver* board;
crc16 crc;

byte *data;
uint16_t data_len;


void setup() {
    Serial.begin(115200);
    Serial.write('B');

    // Guess at size of data array
    data_len = NUM_BOARDS * CH_PER_BOARD;
    data = new byte[data_len];
    
    uint8_t addresses[] = {0x42, 0x44, 0x41, 0x45, 0x40, 0x43, 0x46, 0x47, 0x48};
    
    // Setup breakout boards
    board = new Adafruit_PWMServoDriver(0x40 + BOARD_IDX);
    board->begin();
    board->setPWMFreq(1600);  // 1600? is the maximum PWM frequency
    
}

void loop() {
    switch (state) {
        case STATE_READY:
            #ifdef DEBUG
                if (!messagewalk) Serial.println("READY");
            #endif
            messagewalk = true;
            //cts();
            if (Serial.available() > 1) {
                Serial.print("Peek: ");
                char b = Serial.peek();
                Serial.println(b);
                if (Serial.read() == magic[magic_status]) {
                    magic_status++;
                } else {
                    magic_status=0;
                }
                Serial.print("Magic status: ");
                Serial.println(magic_status);
                if (magic_status >= MAGIC_LENGTH) {
                    state = STATE_READING;
                    messagewalk = false;
                }
            }
            break;
        case STATE_READING:
            #ifdef DEBUG
                if (!messagewalk) Serial.println("READING");
            #endif
            messagewalk = true;
            if (Serial.available() > 2) {
                uint16_t len = getTwoBytesSerial();
                Serial.print("Length: ");
                Serial.println(len);
                readData(len);
                Serial.println("Data read");
                // Update len for future use (writing)
                data_len = len;
                uint16_t packetCrc = getTwoBytesSerial();
                Serial.print("Calculated CRC: ");
                uint16_t calculatedCrc = crc.XModemCrc(data, 0, data_len);
                Serial.println(calculatedCrc, HEX);
                Serial.print("Received CRC: ");
                Serial.println(packetCrc, HEX);
                messagewalk = false;
                if (calculatedCrc != packetCrc)
                {
                    Serial.println("CRC doesn't match");
                    state = STATE_READY;
                }
                else
                {
                    state = STATE_WRITING;
                }
            }
            break;
        case STATE_WRITING:
            #ifdef DEBUG
                if (!messagewalk) Serial.println("WRITING");
            #endif
            messagewalk = false;
            for (uint8_t channel = 0; channel < CH_PER_BOARD; channel++) {
                uint16_t index = BOARD_IDX * CH_PER_BOARD + channel;
                board->setPWM(channel, 0, data[index] << 4);
            }
            Serial.println("Done writing");
            state = STATE_READY;
            break;
        default:
            state = STATE_READY;
            messagewalk = false;
            break;
    }
}

uint16_t getTwoBytesSerial() {
    // Wait for serial bytes
    while (Serial.available() < 2) {}
    uint16_t high = Serial.read() << 8;
    uint16_t low = Serial.read();
    uint16_t combined = high | low;
    return combined;
}

void readData(uint16_t len) {
    // Resize data array if smaller than len
    // Should never happen
    if (len > data_len) {
        Serial.println("Resizing data array");
        byte* new_data = new byte[len];
        memcpy(data, new_data, data_len);
        data_len = len;
    }
    
    while (Serial.available() < len) {}
    
    // Read in data from serial
    Serial.readBytes(data, len);
    
}

void cts() {
    for (int i=MAGIC_LENGTH-1; i>=0; i--) {
        Serial.write(magic[i]);
    }
}


