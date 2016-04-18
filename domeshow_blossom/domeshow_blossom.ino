#include <Adafruit_PWMServoDriver.h>
#include <crc16.h>

#define DEBUG
#ifdef DEBUG
boolean messagewalk = false;  // Prevents spamming Serial monitor
#endif

#define RTSPIN 2
#define CTSPIN 3

#define NUM_BOARDS 9
#define CH_PER_BOARD 16

#define MAGIC_LENGTH 4
const byte magic[] = {0xde, 0xad, 0xbe, 0xef};
byte magic_status = 0;

#define STATE_READY     0
#define STATE_READING   1
#define STATE_WRITING   2
byte state = STATE_READY;

Adafruit_PWMServoDriver *boards = new Adafruit_PWMServoDriver[NUM_BOARDS];
crc16 crc;

uint8_t *data;
uint16_t data_len;


void setup() {
    pinMode(RTSPIN, INPUT);
    pinMode(CTSPIN, OUTPUT);
    Serial.begin(115200);
    
    // Setup breakout boards
    for (uint8_t i = 0; i < NUM_BOARDS; i++)
    {
        // the default (and highest) address is 0x40
        Adafruit_PWMServoDriver *pwm = new Adafruit_PWMServoDriver(0x40 - i);
        pwm->begin();
        pwm->setPWMFreq(1600);  // 1600? is the maximum PWM frequency
        boards[i] = *pwm;
    }
}

void loop() {
    switch (state) {
        case STATE_READY:
            #ifdef DEBUG
                if (!messagewalk) Serial.println("READY");
            #endif
            messagewalk = true;
            digitalWrite(CTSPIN, HIGH);
            if (Serial.available() > 0) {
                if (Serial.read() == magic[magic_status]) {
                    magic_status++;
                } else {
                    magic_status=0;
                }
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
            digitalWrite(CTSPIN, LOW);
            if (Serial.available() > 0) {
                crc.clearCrc();
                uint16_t len = getTwoBytesSerial();
                Serial.println(len);
                readData(len);
                Serial.println(*data);
                uint16_t packetCrc = getTwoBytesSerial();
                Serial.println(crc.getCrc());
                Serial.println(packetCrc);
                if (crc.getCrc() != packetCrc)
                {
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
            messagewalk = true;
            digitalWrite(CTSPIN, LOW);
            for (uint16_t i = 0; i < data_len; i++) {
                uint8_t board_idx = i / CH_PER_BOARD;
                uint8_t channel_idx = i % CH_PER_BOARD;
                boards[board_idx].setPWM(channel_idx, 0, data[i] << 4);
            }
            state = STATE_READY;
            break;
        default:
            state = STATE_READY;
            messagewalk = false;
            break;
    }
}

uint16_t getTwoBytesSerial() {
  uint8_t low = Serial.read();
  uint16_t high = Serial.read() << 8;
  uint16_t combined = high | low;
  return combined;
}

void readData(uint16_t len) {
    // Resize data array if smaller than len
    // Should never happen
    if (len > data_len) {
        uint8_t* new_data = new uint8_t[len];
        memcpy(data, new_data, data_len);
        data_len = len;
    }

    // Read in data from serial
    // and update CRC value
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t datam = Serial.read();
        data[i] = datam;
        crc.updateCrc(datam);
    }
}

