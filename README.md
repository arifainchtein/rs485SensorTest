# rs485SensorTest

Bench test for RS485 Modbus-RTU sensors on the **ChinampaSensorBoard**. Readings appear on the USB
serial monitor (115200 baud) and on the board's six 4-digit displays.

## Wiring

Both sensors connect **in parallel** to the RS485 terminal **J2**. Power both from the **12 V supply
that feeds J1**, with their black wires on the same supply negative as the board (common ground).

| Signal   | DO probe (Renke)    | Soil probe          | Connect to                 |
|----------|---------------------|---------------------|----------------------------|
| V+       | Brown (10–30 V)     | Red (3.3–24 V)      | 12 V supply +              |
| GND      | Black               | Black               | 12 V supply − (= J1 pin 2) |
| RS485 A  | Yellow (or green)   | Yellow              | J2 pin 1 (A)               |
| RS485 B  | Blue                | **Green**           | J2 pin 2 (B)               |

- DO colours are from the Renke manual, section 2.1. Soil colours are from the probe's cable.
- **Careful with green:** it is **B** on the soil probe, but **A** on DO cables that have no yellow wire.
- The DO probe will **not** run from the board's 5 V terminals (J7/J12). The soil probe can, if preferred.
- No reply (`Err 1`)? Swap A and B first.
- Jumper **J14** adds the 120 Ω termination resistor. Only needed with long cables.

## Board pins (ChinampaSensorBoard)

| Function             | Pin / address                                    |
|----------------------|--------------------------------------------------|
| RS485 RX / TX        | GPIO16 / GPIO17 (MAX485E U1)                     |
| RS485 DE + /RE       | GPIO26                                           |
| I²C                  | GPIO21 (SDA) / GPIO22 (SCL), via BSS138 shifter  |
| Displays DS3 / DS4   | HT16K33 at 0x70                                  |
| Displays DS7 / DS8   | HT16K33 at 0x71                                  |
| Displays DS1 / DS2   | HT16K33 at 0x72                                  |

## Sensor 1: Soil probe (7-in-1)

Source: *Soil Multi-parameter Sensor Manual V2.2* (English translation, `~/Downloads/SoilRS485.pdf`).

- **Defaults:** address **2**, **9600** baud, 8N1
- **Power:** 3.3–24 V DC; 3 mA idle, 25 mA measuring, 35 mA max
- **Ranges:** temperature −40–80 °C (±0.5 °C); moisture 0–100 % (±3 %); EC 0–20000 µS/cm; pH 3–9
- **Settling:** temperature, moisture and EC under 5 s; **first pH reading takes more than 5 minutes**
- **Use:** push the probe fully into the soil; avoid stones and hard clods; don't pull it out by the cable

Read: function 0x03, registers 0x0000–0x0007.

| Register | Value                | Conversion                               |
|----------|----------------------|------------------------------------------|
| 0x0000   | Temperature          | signed 16-bit ÷ 10 → °C                  |
| 0x0001   | Moisture             | ÷ 10 → %                                 |
| 0x0002   | Electrical conductivity | µS/cm, as is                          |
| 0x0003   | Salt                 | as is (manual gives no unit)             |
| 0x0004   | Nitrogen             | mg/kg                                    |
| 0x0005   | Phosphorus           | mg/kg                                    |
| 0x0006   | Potassium            | mg/kg                                    |
| 0x0007   | pH                   | ÷ 10                                     |

Settings (written with function 0x10):

| Register | Setting   | Values                                     |
|----------|-----------|--------------------------------------------|
| 0x0080   | Address   | 1–247                                      |
| 0x0081   | Baud rate | 1200 … 38400                               |
| 0x0082   | Parity    | 1 = odd, 2 = even                          |
| 0x0083   | Data bits | 8, 9                                       |
| 0x0084   | Stop bits | 1, 2                                       |

Address 0 is a broadcast: with only one sensor on the bus, it answers whatever its address is.

## Sensor 2: Dissolved oxygen probe (Renke RS-LDOS-N01-2-20)

The salt-water version. Source: Renke *RS-LDO-N01-1 Fluorescence Dissolved Oxygen Transmitter User
Manual V2.0*, same family and register map
([PDF](https://robu.in/wp-content/uploads/2024/06/18-RS-LDO-N01-1-Dissolved-Oxygen-Transmitter-Instruction-Manual-1.pdf)).

- **Defaults:** address **1**, **4800** baud, 8N1
- **Power:** 10–30 V DC
- **Range:** 0–20 mg/L (0–200 % saturation); resolution 0.01 mg/L, 0.1 %, 0.1 °C; accuracy ±3 % FS, ±0.5 °C
- **Response time:** up to 60 s. Working temperature 0–40 °C. IP68.
- **Fluorescent cap:** lasts about 1 year. Don't scratch or knock it, and keep it free of sediment.
  Remove the black rubber cover before measuring.
- **Cleaning:** about every 30 days, with tap water and a soft cloth (mild detergent for grease).
  Never use organic solvents.

Read: function 0x03 (or 0x04). Values are **32-bit floats, big-endian** (first register = high word).

| Registers       | Value          | Notes                                                    |
|-----------------|----------------|----------------------------------------------------------|
| 0x0000–0x0001   | Saturation     | The manual's example returns a fraction (0.918 = 91.8 %) |
| 0x0002–0x0003   | DO             | mg/L                                                     |
| 0x0004–0x0005   | Temperature    | °C                                                       |

Settings (function 0x06 or 0x10):

| Register | Setting           | Values                                                         |
|----------|-------------------|----------------------------------------------------------------|
| 0x1010   | Calibration       | 1 = zero point, 2 = 100 % saturation point                     |
| 0x1020   | Salinity          | ppt, whole number. 0 on the fresh-water model                  |
| 0x1022   | Air pressure      | kPa × 100 (default 10133 = 101.33 kPa)                         |
| 0x07D0   | Address           | 1–254                                                          |
| 0x07D1   | Baud rate         | 0 = 2400, 1 = 4800, 2 = 9600, 3 = 19200, 4 = 38400, 5 = 57600, 6 = 115200, 7 = 1200 |

**Salinity:** the salt-water version probably ships with salinity set to about 35 ppt. For the fresh-water
fish tank, check the value the sketch prints and send `do salinity 0`.

### Calibration (from the manual)

1. **Zero oxygen solution:** dissolve 5 g anhydrous sodium sulfite in 95 g distilled water (5 %), stir and
   leave it 1 hour. A trace of cobalt chloride speeds it up.
2. **100 % saturation:** bubble air through distilled water for 1 hour, then let it stand 30 minutes. Or
   shake a little water hard in a closed container for 30 s and hold the cap about 1 cm above the water
   (moist, no droplets).
3. In 100 % air, wait for a stable reading, then write **2** to 0x1010.
4. In the zero solution, stir gently, wait for a stable reading, then write **1** to 0x1010.
5. Let the probe reach the temperature of each environment before calibrating.

The sketch has no calibration command yet.

## Displays

Each enabled sensor gets a page. Pages rotate every 6 s and start with the sensor's name on every
display for 1 s. Send `displays` to see which display is which.

| Display | Soil page              | DO page             |
|---------|------------------------|---------------------|
| DS1     | Temperature °C         | DO mg/L             |
| DS2     | Moisture %             | Saturation %        |
| DS3     | EC µS/cm               | Water temperature   |
| DS4     | pH                     | Salinity ("S 35")   |
| DS7     | N / P / K in turn      | –                   |
| DS8     | Salt                   | "do"                |

On a failed read the page shows **Err** and a code:

| Code | Meaning                                                    |
|------|------------------------------------------------------------|
| 1    | No reply: check A/B, power, address, baud rate             |
| 2    | Short reply                                                |
| 3    | Bad checksum: noise, wrong baud rate, or A/B swapped       |
| 4    | Sensor rejected the request (Modbus exception)             |
| 5    | Reply came from a different address                        |
| 6    | Unexpected reply format                                    |

## Serial commands

`<s>` is `soil` or `do`.

| Command              | What it does                                                    |
|----------------------|-----------------------------------------------------------------|
| `read [<s>]`         | Read now: all enabled sensors, or one                           |
| `run` / `stop`       | Start / stop reading every 3 s                                  |
| `<s> on` / `off`     | Include or skip the sensor                                      |
| `<s> addr <n>`       | Address to query                                                |
| `<s> baud <b>`       | Baud rate the ESP32 uses for that sensor                        |
| `<s> find`           | Ask a lone sensor its address at every baud rate                |
| `<s> setaddr <n>`    | Write a new address into the sensor                             |
| `do salinity <ppt>`  | Set the DO probe's salinity compensation                        |
| `do pressure <kPa>`  | Set the DO probe's air pressure                                 |
| `raw on` / `off`     | Show / hide the Modbus frames                                   |
| `wiring`             | Print the cable colours                                         |
| `displays`           | Show each display's name for 3 s                                |
| `help`               | List commands and each sensor's address and baud                |
