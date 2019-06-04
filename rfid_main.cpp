/*
 * Reading em4100 cards using Arduino Uno
 */

#include "em4100.h"
#include <HardwareSerial.h>
#include <string.h>

em4100_reader::State state;
uint8_t cardFacility;
uint32_t cardUid;
char card_number_str[9];

void setup() {
    Serial.begin(115200);
    Serial.println("Connect input to pin 8 and scan rfid tag...");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Enable interrupts and start the state machine.
    em4100.startRfidCapture();

    memset(card_number_str, 0, sizeof(card_number_str));
}

void loop() {
    em4100.processCaptureBuffer();
    state = em4100.getState();

    if (state == em4100_reader::ERROR_BUFFER_OVERFLOW) {
        Serial.println("ERROR: Buffer overflow!");
        em4100.startRfidCapture();
    }

    if (state == em4100_reader::MANCHESTER_DONE || state == em4100_reader::BIPHASE_DONE) {

        if (state == em4100_reader::MANCHESTER_DONE) {
            Serial.print("DECODED: MANCHESTER=");
        } else {
            Serial.print("DECODED: BIPHASE=");
        }

        if (em4100.parseData()) {
            cardFacility = em4100.getCardFacility();
            cardUid = em4100.getCardUid();

            sprintf(card_number_str, "0x%02X%06lX", cardFacility, cardUid);
            Serial.println(card_number_str);

        }

        // Restart capturing again.
        em4100.startRfidCapture();
    }

}
