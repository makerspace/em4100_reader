/*
 * Reading em4100 cards using Arduino Uno
 */

#include "em4100.h"
#include <HardwareSerial.h>

em4100_reader::State state;
uint8_t cardFacility;
uint32_t cardUid;

void setup() {
    Serial.begin(115200);
    Serial.println("Connect input to pin 8 and scan rfid tag...");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Enable interrupts and start the state machine.
    em4100.startRfidCapture();
}

void loop() {
    em4100.processCaptureBuffer();
    state = em4100.getState();

    if (state == em4100_reader::ERROR_BUFFER_OVERFLOW) {
        Serial.println("Buffer overflow!");
        em4100.startRfidCapture();
    }

    if (state == em4100_reader::MANCHESTER_DONE || state == em4100_reader::BIPHASE_DONE) {

        if (state == em4100_reader::MANCHESTER_DONE) {
            Serial.println("MANCHESTER_DONE!");
        } else {
            Serial.println("BIPHASE_DONE!");
        }

        if (em4100.parseData()) {
            cardFacility = em4100.getCardFacility();
            cardUid = em4100.getCardUid();

            // Print tag data to serial
            Serial.print("0x");
            for (int8_t i = 4; i >= 0; i -= 4) {
                uint8_t val = 0xF & (cardFacility >> i);
                Serial.print(char(val < 0xA ? '0' + val : 'A' + val - 0xA));
            }
            for (int8_t i = 28; i >= 0; i -= 4) {
                uint8_t val = 0xF & (cardUid >> i);
                Serial.print(char(val < 0xA ? '0' + val : 'A' + val - 0xA));
            }
            Serial.println("!");

        }
        // Restart capturing again.
        em4100.startRfidCapture();
    }

}
