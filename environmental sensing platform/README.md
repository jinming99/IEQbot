# Arduino code and dweet data pulling code for sensor stations

## Introduction

This foleder contains files for arduino megas and dweet data pulling tool for sensor station.

- */Mega:* Arduino code for sensor station with possibly sensors of K30, TGS2620, SEN0177, AM2315, MCP9808, SI1145, DS18S20 and requires wifi shield 101. To use sensor stations with dweet, please also modify the numbering of the code so staions can upload data with their own number as an identifier.

- */Scripts/pull_data.py:* Python script for pulling station (type single) sensor data from dweet and store it locally. To run it, simply use

```bash
cd Scripts
python pull_data.py &
```

- */Scripts/realtime_process.py:* Python script running on the robot, you can see the results in real time on robot screen if you run on the robot:

```bash
cd Scripts
python realtime_process.py &
```

## Libraries

We ultilize several open sourced library in our arduino code, to compile the arduino code, you will need to have these library installed in your computer. Here are some links to retrieve them:

- *K30* Library included under the Mega folder

- *Adafruit_SI1145_Library* [Github](https://github.com/adafruit/Adafruit_SI1145_Library.git)

- *Adafruit_AM2315* [Github](https://github.com/adafruit/Adafruit_AM2315.git)

- *Adafruit_MCP9808* [Github](https://github.com/adafruit/Adafruit_MCP9808_Library.git)

- *OneWire* [pjrc](https://www.pjrc.com/teensy/td_libs_OneWire.html)

- *Maxim Temperature Integrated Circuits* [Github](https://github.com/milesburton/Arduino-Temperature-Control-Library.git)

- *Wifi101* [Github](https://github.com/arduino-libraries/WiFi101.git)
