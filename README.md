# Automated Ultrasonic Humidifier — ESP32-S3

## Project Structure
```
humidifier/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   └── main.c              ← Core logic (Kien), init (Iyeneobong)
└── components/
    ├── lcd_i2c/            ← HD44780 via PCF8574 I2C expander
    ├── dht20/              ← DHT20 humidity sensor driver
    └── piezo/              ← BM52O5221-1 PWM driver + water level
```

## Pin Assignments
| Signal           | GPIO | Notes                            |
|-----------------|------|----------------------------------|
| I2C SDA          | 8    | Shared: LCD + DHT20              |
| I2C SCL          | 9    | Shared: LCD + DHT20              |
| Piezo PWM        | 5    | To piezo driver board PWM input  |
| Piezo water sens | 6    | HIGH = water present             |
| Fan PWM          | 7    | To MOSFET gate (drives 12V fan)  |
| Button           | 4    | Other leg to GND                 |
| Status LED       | 2    | Series resistor ~330Ω            |

> **Change pins** in `main/main.c` at the top (`PIN_*` defines).

## LCD I2C Address
Default is `0x27`. If the LCD shows nothing, try `0x3F`.
Change `LCD_I2C_ADDR_DEFAULT` in `components/lcd_i2c/lcd_i2c.h`.

## Build & Flash (VS Code with ESP-IDF extension)
```bash
# In VS Code: Ctrl+Shift+P → "ESP-IDF: Set Espressif device target" → esp32s3
# Then:
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## LCD Display Layout
```
Row 0: H:65.2%  T:22.1C
Row 1: Set:40% ON  FULL
```
- `H` = current humidity %, `T` = temperature °C
- `Set` = target threshold, `ON/OFF` = humidifier state
- `FULL/EMPT` = water tank status

## Button Behavior
Each press cycles the humidity threshold:
`20% → 25% → 30% → ... → 70% → 20% (wraps)`

## Safety Features
1. Atomizer will **not** start if water tank is empty
2. Running atomizer **stops immediately** if water runs out mid-cycle
3. Fan starts **before** atomizer, stops **after** (clears residual mist)

## If You Get the BMN31K522 Library Later
Replace `piezo_init`, `piezo_set_active`, and `piezo_water_present`
in `components/piezo/piezo.c` with the library calls.
The rest of `main.c` stays unchanged.
