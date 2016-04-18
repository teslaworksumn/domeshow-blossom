#define DEBUG
#ifdef DEBUG
boolean messagewalk = false;
#endif

#define RTSPIN 2
#define CTSPIN 3
#define MAGIC_LENGTH 4
const byte magic[] = {0xde, 0xad, 0xbe, 0xef};
byte magic_status = 0;

#define STATE_READY     0
#define STATE_READING   1
#define STATE_WRITING   2
byte state = STATE_READY;

void setup() {
    pinMode(RTSPIN, INPUT);
    pinMode(CTSPIN, OUTPUT);
    Serial.begin(115200);
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
            break;
        case STATE_WRITING:
            #ifdef DEBUG
                if (!messagewalk) Serial.println("WRITING");
            #endif
            messagewalk = true;
            digitalWrite(CTSPIN, LOW);
            break;
        default:
            state = STATE_READY;
            messagewalk = false;
            break;
    }
}
