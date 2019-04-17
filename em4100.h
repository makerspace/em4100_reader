/*
 * em4100.h - Reader for em4100 chips.
 * Copyright (c) 2019 Johan Kristensen.
 *
 * Inspired by EM4100 library by Søren Thing Andersen (https://github.com/Sthing/AccessThing).
 *
 */

#ifndef EM4100_h
#define EM4100_h

#include "Arduino.h"

constexpr const int EM4100_INPUT_PIN = 8;    // ICP1 alias Arduino pin 8

constexpr inline bool isPowerOf2(uint8_t n) {
    return n != 0 && (n & (n - 1)) == 0;
}

template<typename T>
constexpr inline T volatile_read(const T& value) {
    return const_cast<volatile const T&>(value);
}

// Setup for Timer1 prescaler
constexpr const int TIMER1_PRESCALE = 64;   // prescale factor (each tick 4 us @16MHz)
constexpr const int TIMER1_PRESCALE_BITS = B011; // see Table 15-5 in the data sheet.

typedef struct {
    bool active;
    uint8_t count;
    uint8_t parity_idx;
    uint8_t parity; // bits: [4,1]=column parity, [0]=row parity
    uint8_t data[8];
} capture_data;

class em4100_reader {
public:
    enum State {
        STATE_INIT, ERROR_BUFFER_OVERFLOW, MANCHESTER_DONE, BIPHASE_DONE
    };
    static constexpr const uint8_t INVALID_COUNT = ~UINT8_C(0);

    em4100_reader();
    State getState();
    void startRfidCapture();
    void resetCapture();
    void onTimerOverflow();
    void onTimerCapture();
    void processCaptureBuffer();

    uint8_t getManchesterCount() const {
        return const_cast<volatile uint8_t&>(manchester.count);
    }
    uint8_t getBiphaseCount() const {
        return const_cast<volatile uint8_t&>(biphase.count);
    }
    boolean parseData();
    uint8_t getCardFacility() const;
    uint32_t getCardUid() const;
    inline void update_short_period_data(const uint16_t time_v);
    inline void update_long_period_data(const uint16_t time_v);
    void updateBitPeriod() {
        bit_period = (short_sum_period + long_sum_period / 2) / 4;
    }

    uint16_t getBitPeriod() const {
        return bit_period;
    }
    uint16_t get_short_min_period() const {
        return short_min_period;
    }
    uint16_t get_short_max_period() const {
        return short_max_period;
    }
    uint16_t get_long_min_period() const {
        return long_min_period;
    }
    uint16_t get_long_max_period() const {
        return long_max_period;
    }

    void initializeBitPeriod(const uint16_t time_v) {
        bit_period = time_v;
        short_min_period = short_max_period = time_v;
        short_sum_period = short_min_period + short_max_period;
        long_min_period = long_max_period = 2 * time_v;
        long_sum_period = long_min_period + long_max_period;
        short_periods = 1;
        long_periods = 0;
    }
private:
    static constexpr const uint8_t CAPTURE_BUFFER_SIZE = 64;
    static_assert(isPowerOf2(CAPTURE_BUFFER_SIZE), "CAPTURE_BUFFER_SIZE must be power of two");
    static constexpr const uint8_t CAPTURE_BUFFER_INDEX_MASK = CAPTURE_BUFFER_SIZE - 1;

    // Variables used by the state machine capturing the data
    uint16_t duration_buffer[CAPTURE_BUFFER_SIZE];
    uint8_t level_buffer[CAPTURE_BUFFER_SIZE];
    volatile uint8_t capture_buffer_read;
    volatile uint8_t capture_buffer_write;

    volatile State state;
    capture_data manchester;
    capture_data biphase; // TODO: Biphase tag never tested find card to test with.
    uint16_t bit_period;
    uint16_t short_min_period;
    uint16_t short_max_period;
    uint16_t long_min_period;
    uint16_t long_max_period;
    uint16_t short_sum_period;
    uint16_t long_sum_period;
    uint16_t short_periods;
    uint16_t long_periods;
    int8_t consecutive_periods;

    uint8_t cardFacility;  // First 8 bits of tag data
    uint32_t cardUid;      // Last 32 bits of tag data

    boolean parseDataBuffer(uint8_t* data);
};
extern em4100_reader em4100;
#endif
