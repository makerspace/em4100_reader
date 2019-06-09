# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile

CFLAGS_STD ?= -std=gnu11
CXXFLAGS_STD ?= -std=gnu++11

#ARDUINO_DIR = <YOUR ARDUINO INSTALL DIR>
BOARD_TAG    = uno
MONITOR_BAUDRATE = 115200

include /usr/share/arduino/Arduino.mk
