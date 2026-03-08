# Automated Ultrasonic Humidifier — ESP32-S3

## Project Structure
```
humidifier/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   └── main.c
└── components/
    └── piezo/              ← BM52O5221-1 UART driver
```

## External Component Required: hd44780
This project uses the `hd44780` component from esp-idf-lib.
Add it to your project using the IDF Component Manager:

  Option A — idf_component.yml (recommended):
    Create main/idf_component.yml with:
      dependencies:
        esp-idf-lib/hd44780: ">=0.0.1"

  Option B — copy manually:
    Clone https://github.com/UncleRus/esp-idf-lib
    Copy components/hd44780 into your project's components/ folder

## Pin Assignments
| Signal          | GPIO | Notes                          |
|----------------|------|--------------------------------|
| DHT20 SDA       | 20   | I2C sensor data                |
| DHT20 SCL       | 21   | I2C sensor clock               |
| LCD RS          | 38   | Register select                |
| LCD E           | 37   | Enable                         |
| LCD D4          | 36   |                                |
| LCD D5          | 35   |                                |
| LCD D6          | 48   |                                |
| LCD D7          | 47   |                                |
| Piezo UART TX   | 17   | ESP TX → module RX             |
| Piezo UART RX   | 18   | module TX → ESP RX             |
| Button          | 4    | Other leg to GND               |
| Status LED      | 2    | ~330Ω series resistor          |

## LCD Display Layout
```
Row 0: Temp: 22.1 C
Row 1: Hum:65.2% ON 40%
```
State shown: ON/OFF = humidifier running, last number = threshold

## Button
Each press steps the humidity threshold up by 5%: 20→25→...→70→20

## Build & Flash
```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```
