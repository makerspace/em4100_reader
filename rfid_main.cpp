/*
 * Reading em4100 cards using Arduino Uno
 */

#include "em4100.h"
#include <HardwareSerial.h>
#include <string.h>

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

    Serial.setTimeout(50);
}

const char TERMINATOR_CHARS[] = "\n\r";
bool read_input(char *str_buffer, int buffer_size) {
    if (Serial.available()) {
      int chars_read = Serial.readBytes(str_buffer, buffer_size-1);

      // Check that it ends with a terminator char
      if (strchr(TERMINATOR_CHARS, str_buffer[chars_read-1]) != NULL) {
          str_buffer[chars_read-1] = '\0'; // Remove termination char
          return true;
      }
    }
    return false;
}

void loop() {
    char card_number_str[9];
    em4100.processCaptureBuffer();
    state = em4100.getState();

    if (state == em4100_reader::ERROR_BUFFER_OVERFLOW) {
        Serial.println("ERROR: Buffer overflow!");
        em4100.startRfidCapture();
    }

    char read_str[20];
    // Read command
    if (read_input(read_str, sizeof(read_str))) {
        if (strcmp(read_str, "?") == 0) {
            Serial.println("EM4100 Reader");
        }

        // Restart capturing again since we screwed with timing
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
