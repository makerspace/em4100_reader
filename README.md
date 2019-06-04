[![Build Status](https://travis-ci.org/makerspace/em4100_reader.svg?branch=master)](https://travis-ci.org/makerspace/em4100_reader)

# EM4100 rfid tag reader for Arduino Uno.

Connect receiver signal to pin 8 on your Arduino Uno.

Requires installed Arduino IDE and Arduino Makefile project

To install on recent Ubuntu distros:

```
sudo apt install arduino arduino-mk
```

If Arduino IDE is installed in non standard location uncomment Makefile line

```
#ARDUINO_DIR = <YOUR ARDUINO INSTALL DIR>
```

and add the path of your installation.

To build:

```
make
```

To flash the Arduino UNO:

```
make upload
```
