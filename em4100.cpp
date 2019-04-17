/*
 * em4100.c - Reader for em4100 chips.
 * Copyright (c) 2019 Johan Kristensen.
 *
 * Inspired by EM4100 library by Søren Thing Andersen (https://github.com/Sthing/AccessThing).
 */

#include "Arduino.h"
#include "em4100.h"

em4100_reader em4100;

/* Overflow interrupt vector */
ISR(TIMER1_OVF_vect) {
    em4100.onTimerOverflow();
}

/* ICR interrupt vector */
ISR(TIMER1_CAPT_vect) {
    em4100.onTimerCapture();
}

inline void start_capture(capture_data* cap) {
    cap->active = true;
    cap->count = 0;
    cap->parity_idx = 4;
    cap->parity = 0;
}

inline void stop_capture(capture_data* cap) {
    cap->active = false;
}

inline void save_data(capture_data* cap, const uint8_t level) {
    const uint8_t pos = cap->count / 2;
    cap->parity ^= level;
    if (pos < cap->parity_idx) {
        const uint8_t byte_pos = pos / 8;
        cap->data[byte_pos] <<= 1;
        cap->data[byte_pos] |= level;
    } else {
        const uint8_t byte_pos = (pos - 1) / 8;
        if (pos == 44) {
            // Capture done
            if (level != 0) {
                // Stop bit error
                stop_capture(cap);
                cap->count = 0;
            } else {
                cap->parity ^= (cap->data[byte_pos] << 1);
                if (cap->parity & 0x1e) {
                    // Column parity error
                    stop_capture(cap);
                    cap->count = 0;
                }
            }
        } else if (cap->parity & 0x1) {
            // Parity error, reset
            stop_capture(cap);
        } else {
            cap->count -= 2;
            cap->parity_idx += 4;
        }
    }
}

em4100_reader::em4100_reader() :
        manchester { false, 0, 0 }, biphase { false, 0, 0 }, bit_period(0), short_min_period(0), short_max_period(0), long_min_period(
                0), long_max_period(0), short_sum_period(0), long_sum_period(0), short_periods(0), long_periods(0), consecutive_periods(
                0), cardFacility(0), cardUid(0) {
    pinMode(EM4100_INPUT_PIN, INPUT); // ICP1 pin (digital pin 8 on Arduino) as input
}

em4100_reader::State em4100_reader::getState() {
    return state;
}

/*
 * Starts the capture state machine.
 */
void em4100_reader::startRfidCapture() {
    state = STATE_INIT;
    PRR &= ~(1 << PRTIM1);             // Make sure Timer1 is enabled (Power Reduction Register)
    TCCR1A = 0;                        // Normal counting mode
    TCCR1B = TIMER1_PRESCALE_BITS;     // Set prescale bits
    TCCR1B |= _BV(ICES1);              // Edge Select: Trigger on rising edge
    TIMSK1 |= _BV(ICIE1) | _BV(TOIE1); // enable input capture interrupt and overflow interrupt for timer 1
}

// Called from ISR(TIMER1_OVF_vect)
void em4100_reader::onTimerOverflow() {
    // When tag is present timer won't overflow
    resetCapture();
}

void em4100_reader::resetCapture() {
    stop_capture(&manchester);
    stop_capture(&biphase);
    bit_period = 0x0;
}

void em4100_reader::update_short_period_data(const uint16_t time_v) {
    if (time_v < short_min_period) {
        short_min_period = time_v;
        short_sum_period = short_min_period + short_max_period;
        updateBitPeriod();
    } else if (time_v > short_max_period) {
        short_max_period = time_v;
        short_sum_period = short_min_period + short_max_period;
        updateBitPeriod();
    }
}
void em4100_reader::update_long_period_data(const uint16_t time_v) {
    if (time_v < long_min_period) {
        long_min_period = time_v;
        long_sum_period = long_min_period + long_max_period;
        updateBitPeriod();
    } else if (time_v > long_max_period) {
        long_max_period = time_v;
        long_sum_period = long_min_period + long_max_period;
        updateBitPeriod();
    }
}

// Called from ISR(TIMER1_CAPT_vect)
void em4100_reader::onTimerCapture() {
    // Always reset the timer counter
    TCNT1 = 0;
    // Toggle bit to trigger on the other edge next time
    TCCR1B ^= _BV(ICES1);

    // ICR1 holds the captured timer value
    const uint16_t timer_value = ICR1;

    if (timer_value < 10) { // ie 40 micro seconds
        // This is always an error, start over.
        // Probably no card is present.
        resetCapture();
        capture_buffer_read = capture_buffer_write = 0;
        return;
    }

    if (((capture_buffer_read - capture_buffer_write) & CAPTURE_BUFFER_INDEX_MASK) != 1) {
        const uint8_t index = capture_buffer_write & CAPTURE_BUFFER_INDEX_MASK;
        duration_buffer[index] = timer_value;
        level_buffer[index] = digitalRead(EM4100_INPUT_PIN) ^ 1; // Invert level because we read level after the captured periods end
        ++capture_buffer_write;
    } else {
        capture_buffer_read = capture_buffer_write = 0;
        state = ERROR_BUFFER_OVERFLOW;
        TIMSK1 = 0;
    }
}

void em4100_reader::processCaptureBuffer() {
    for (; (capture_buffer_read - capture_buffer_write) & CAPTURE_BUFFER_INDEX_MASK; ++capture_buffer_read) {
        const uint8_t index = capture_buffer_read & CAPTURE_BUFFER_INDEX_MASK;
        const uint16_t t_v = duration_buffer[index];
        const uint8_t level = level_buffer[index];

        const uint16_t bit_period_slack = bit_period / 4;
        if (t_v <= bit_period + bit_period_slack && t_v >= bit_period - bit_period_slack) {
            update_short_period_data(t_v);
            ++short_periods;

            if (manchester.active) {
                if ((manchester.count & 1) == 1) {
                    save_data(&manchester, level);
                }
                manchester.count += 1;
            }
            if (biphase.active) {
                if ((biphase.count & 1) == 1) {
                    save_data(&biphase, 0);
                }
                biphase.count += 1;
            }

            if (consecutive_periods < 0) {
                consecutive_periods = 1;
            } else {
                ++consecutive_periods;
                if (consecutive_periods >= 16 && level == 1) {
                    if (!manchester.active) {
                        // Valid header found for Manchester encoding
                        start_capture(&manchester);
                    }
                }
            }
        } else if (t_v <= 2 * bit_period + bit_period_slack && t_v >= 2 * bit_period - bit_period_slack) {
            update_long_period_data(t_v);
            ++long_periods;

            if (manchester.active) {
                if ((manchester.count & 1) == 1) {
                    // phase error
                    stop_capture(&manchester);
                } else {
                    save_data(&manchester, level);
                    manchester.count += 2;
                }
            }
            if (biphase.active) {
                if ((biphase.count & 1) == 1) {
                    // phase error
                    biphase.active = false;
                } else {
                    save_data(&biphase, 1);
                    biphase.count += 2;
                }
            }

            if (consecutive_periods >= 0) {
                consecutive_periods = -1;
            } else {
                --consecutive_periods;
                if (consecutive_periods <= -9) {
                    if (!biphase.active) {
                        // Valid header found for biphase encoding, start saving
                        start_capture(&biphase);
                    }
                }
            }
        } else if (long_periods == 0 && t_v <= (bit_period + bit_period_slack) / 2
                && t_v >= (bit_period - bit_period_slack) / 2) {
            long_min_period = short_min_period;
            long_max_period = short_max_period;
            long_sum_period = short_sum_period;
            long_periods = short_periods;
            short_min_period = short_max_period = t_v;
            short_sum_period = 2 * t_v;
            short_periods = 1;
            if (consecutive_periods >= 9) {
                //TODO: This is a valid biphase header
            }

            consecutive_periods = 1;

        } else {
            /* Period not compatible with previous periods or 0, reset*/
            resetCapture();
            manchester.count = biphase.count = 0;
            initializeBitPeriod(t_v);
            consecutive_periods = 1;
        }

        if (manchester.count / 2 >= 44) {
            stop_capture(&manchester);
            stop_capture(&biphase);
            TIMSK1 = 0;
            state = MANCHESTER_DONE;
        } else if (biphase.count / 2 >= 44) {
            stop_capture(&manchester);
            stop_capture(&biphase);
            TIMSK1 = 0;
            state = BIPHASE_DONE;
        }
    }
}

boolean em4100_reader::parseData() {
    switch (state) {
    case MANCHESTER_DONE:
        return parseDataBuffer(manchester.data);
    case BIPHASE_DONE:
        return parseDataBuffer(biphase.data);
    default:
        break;
    }
    return false;
}

/*
 * Results of decoding are stored in cardFacility and cardUid.
 */
boolean em4100_reader::parseDataBuffer(uint8_t* data) {
    cardFacility = data[0];
    cardUid = ((uint32_t) data[1] << 24) | ((uint32_t) data[2] << 16) | ((uint32_t) data[3] << 8)
            | ((uint32_t) data[4]);
    return true;
}

/*
 * Returns the found facility code.
 */
uint8_t em4100_reader::getCardFacility() const {
    return cardFacility;
}

/*
 * Returns the found UID.
 */
uint32_t em4100_reader::getCardUid() const {
    return cardUid;
}

