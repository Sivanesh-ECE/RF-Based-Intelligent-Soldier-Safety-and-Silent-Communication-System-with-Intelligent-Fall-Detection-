# RF-Based-Intelligent-Soldier-Safety-and-Silent-Communication-System-with-Intelligent-Fall-Detection-
# RF-Based Intelligent Soldier Safety and Silent Communication System

## 1. Project Overview

Two independent embedded units:

- **Soldier Unit (Transmitter)** — ESP32, reads buttons + MPU6050, sends silent
  tactical messages and emergency fall alerts over RF via an HT12E encoder.
- **Base Station (Receiver)** — Arduino UNO, decodes RF data via HT12D,
  displays messages, drives LEDs, and sounds an active buzzer alarm.

---

## 2. Pin Configuration

### 2.1 Soldier Unit — ESP32 DevKit V1

| Function                     | ESP32 Pin |
|-------------------------------|-----------|
| OLED SDA                      | GPIO 21   |
| OLED SCL                      | GPIO 22   |
| MPU6050 SDA                   | GPIO 21 (shared I2C bus) |
| MPU6050 SCL                   | GPIO 22 (shared I2C bus) |
| Button 1 – Need Backup        | GPIO 32   |
| Button 2 – Enemy Spotted      | GPIO 33   |
| Button 3 – Need Medical Help  | GPIO 25   |
| Button 4 – Mission Completed  | GPIO 26   |
| HT12E D8 (MSB)                | GPIO 13   |
| HT12E D9                      | GPIO 12   |
| HT12E D10                     | GPIO 14   |
| HT12E D11 (LSB)                | GPIO 27   |
| HT12E TE (active LOW)          | GPIO 4    |
| Status LED (optional)          | GPIO 2 (onboard) |

HT12E address pins A0–A7: tie all to **GND** (fixed address, must match HT12D
address pins on the receiver).

HT12E DOUT (pin 17) → 433 MHz RF TX module DATA/ATAD pin.
RF TX VCC → 5 V (or 3.3 V per module rating), GND → common ground.

### 2.1.1 Troubleshooting: OLED and MPU6050 both not working

The OLED and MPU6050 share **one I2C bus** (GPIO21 = SDA, GPIO22 = SCL). If
**both** stop responding at the same time, the cause is almost always the
shared bus itself, not two coincidentally broken parts. Check in this order:

1. **SDA/SCL not swapped** — OLED SDA and MPU6050 SDA must both land on
   GPIO21; OLED SCL and MPU6050 SCL must both land on GPIO22.
2. **Common ground** — ESP32 GND, OLED GND, and MPU6050 GND must all share
   the same ground rail. A missing ground on just one board can silence
   the entire bus.
3. **`Wire.begin()` pin order** — must be `Wire.begin(I2C_SDA_PIN,
   I2C_SCL_PIN)`, i.e. `Wire.begin(21, 22)` on ESP32 (SDA first, then SCL).
4. **Reseat every jumper** on the breadboard header rows — a very common
   failure point when several wires are crowded into one row.
5. **Run an I2C scanner sketch** (search "Arduino I2C scanner", works as-is
   on ESP32 with `Wire.begin(21, 22)`). Expect to see `0x3C` (OLED) and
   `0x68` (MPU6050). Finding **neither** confirms a bus/wiring/ground issue
   rather than a code or component fault; finding only one means you have
   two separate problems, not one shared cause.
6. **Check MPU6050's AD0 pin** is tied to GND (or left floating) so its
   address stays at `0x68` and doesn't drift/float onto the bus.
7. **Power budget** — try powering both modules from 5V instead of the
   ESP32's 3.3V pin if multiple devices share that rail; an underpowered
   rail can brown out both I2C devices simultaneously.

The firmware itself already treats this defensively: `initializeOLED()`
and `initializeMPU()` each check their own return value and report
failure independently on the self-test screen and over Serial (115200
baud) — but a bus-level short/miswiring will make *both* report failure
together, which is the signature described above.

### 2.2 Base Station — Arduino UNO R3

| Function                    | UNO Pin |
|------------------------------|---------|
| HT12D VT (valid transmission)| D2      |
| HT12D D8 (MSB)                | D3      |
| HT12D D9                      | D4      |
| HT12D D10                     | D5      |
| HT12D D11 (LSB)                | D6      |
| Green LED                     | D7      |
| Red LED                       | D8      |
| Active buzzer module IN       | D9      |
| OLED SDA                      | A4 (hardware I2C) |
| OLED SCL                      | A5 (hardware I2C) |

HT12D address pins A0–A7: tie all to **GND** (must match transmitter).
RF RX module DATA output → HT12D DIN (pin 14).

### 2.3 Active Buzzer Module

An active buzzer module has its own internal oscillator, so it only needs
a digital HIGH/LOW to turn on/off — no PWM, tone frequency, or coupling
network required. Most 5 V active buzzer modules include their own driver
transistor, so D9 connects directly to the module's IN/signal pin:

```
Arduino D9  ──► Active Buzzer Module IN
Arduino 5V  ──► Active Buzzer Module VCC
Arduino GND ──► Active Buzzer Module GND
```

The receiver code pulses D9 on/off (`SIREN_TOGGLE_MS`) to create an
attention-grabbing pulsed alarm pattern for medical-help and soldier-down
events. Note that active buzzers have a fixed pitch set by their internal
oscillator — pitch cannot be varied like it could with `tone()` on a
passive buzzer or speaker, only the on/off pulsing pattern.

---

## 3. Connection Diagram (textual)

```
SOLDIER UNIT (ESP32)                         BASE STATION (Arduino UNO)
─────────────────────                        ──────────────────────────
MPU6050 ──I2C(21,22)── ESP32                  HT12D ──parallel bits(D3-D6)── UNO
OLED    ──I2C(21,22)── ESP32                  HT12D VT ──D2── UNO
4x Buttons ──GPIO(32,33,25,26)── ESP32        OLED ──I2C(A4,A5)── UNO
HT12E ──GPIO(13,12,14,27,4)── ESP32           Green LED ──D7── UNO
HT12E DOUT ──► RF TX DATA                     Red LED ──D8── UNO
RF TX (433 MHz) ▓▓▓▓▓ radiates ▓▓▓▓▓ ► RF RX (433 MHz)
                                               RF RX DATA ──► HT12D DIN
                                               UNO D9 ──► Active Buzzer Module IN
```

Both HT12E and HT12D must share the **same address bits** (all tied to GND
in this design) or the receiver will silently ignore all frames.

---

## 4. Flowchart

### 4.1 Soldier Unit (Transmitter)

```
[Power On]
   │
   ▼
[Initialize OLED, MPU6050, HT12E pins]
   │
   ▼
[Self-Test: OLED / MPU6050 / RF] ──fail──► [Display SYSTEM ERROR]
   │ pass
   ▼
[Display READY screen: ID / STATUS / RF]
   │
   ▼
[Loop] ──► [Read Buttons] ──pressed──► [Send RF code] ─► [Display Sending → Sent] ─┐
   │                                                                                │
   ├──► [Read MPU6050: accel + gyro] ─► [Compute total accel, pitch, roll, gyro]   │
   │           │                                                                    │
   │      impact ≥ threshold?                                                      │
   │           │ yes                                                               │
   │      within fusion window: orientation Δ AND gyro spike?                      │
   │           │ yes                                                               │
   │           ▼                                                                   │
   │   [SOLDIER DOWN: send RF 1111, display EMERGENCY] ◄──────────────────────────┘
   │           │
   └───────────┴──► back to [Loop]
```

### 4.2 Base Station (Receiver)

```
[Power On]
   │
   ▼
[Initialize OLED, HT12D pins, LEDs, Alarm pin]
   │
   ▼
[Display "COMMAND CENTER / Waiting..."]
   │
   ▼
[Loop] ──► [Poll HT12D VT pin]
              │ valid pulse (debounced)
              ▼
        [Read 4-bit data word] ──► [Decode command]
              │
              ▼
   ┌─────────────────────────────────────────────┐
   │  Need Backup / Enemy Spotted / Mission Done  │──► Green LED, display message
   │  Need Medical Help                           │──► Red LED, alarm ON, display
   │  Soldier Down                                │──► Red LED, alarm ON, EMERGENCY screen
   └─────────────────────────────────────────────┘
              │
              ▼
   [Timers expire (non-blocking)] ──► LED/alarm reset, return to Waiting (unless Emergency)
```

---

## 5. Working Explanation

**Silent messaging:** The soldier presses one of four buttons. The ESP32
debounces the input, loads the corresponding 4-bit code onto the HT12E data
lines, and pulses the TE (Transmit Enable) pin LOW for ~250 ms so the HT12E
repeatedly transmits the encoded word through the RF link. The OLED shows
"Sending…" then "Message Sent" before returning to the ready screen — all
using `millis()` timers so the button loop is never blocked.

**Fall detection (sensor fusion, not a timer):** Every 20 ms the ESP32 reads
the MPU6050's accelerometer and gyroscope. It computes total acceleration
magnitude (to catch a hard impact), plus pitch/roll from the accelerometer
and angular velocity from the gyroscope (to catch a sudden orientation
change). A "Soldier Down" event is only declared when a high-acceleration
impact **and** a correlated sudden rotation/orientation change both occur
within a short time window. This avoids false positives from a soldier
diving into cover or standing still (which a simple "wait 5 seconds"
inactivity timer would misinterpret), while still reacting immediately —
there is no waiting period before the alert is sent.

**Receiving and alerting:** The Arduino UNO continuously polls the HT12D's
VT (Valid Transmission) pin. When a debounced, valid pulse is detected, the
4-bit data lines are read and mapped back to a message. Routine messages
light the green LED; medical/emergency messages light the red LED and
pulse an active buzzer module on pin D9 on/off using `digitalWrite()` to
create an attention-grabbing alarm pattern. Since active buzzers have
their own internal oscillator, no PWM tone generation is needed — just
on/off switching.

---

## 6. Circuit Diagram Note

A schematic-style circuit diagram (component symbols, exact wire routing)
is best produced in a dedicated tool such as Fritzing, EasyEDA, or KiCad
using the pin tables above — these tools aren't available in this text
environment, but the pin configuration and connection diagram above contain
everything needed to build the schematic directly.

---

## 7. Libraries Required

**ESP32 (Soldier Unit):**
- `Wire.h`
- `Adafruit_GFX.h`
- `Adafruit_SSD1306.h`
- `Adafruit_MPU6050.h`
- `Adafruit_Sensor.h`

**Arduino UNO (Base Station):**
- `Wire.h`
- `Adafruit_GFX.h`
- `Adafruit_SSD1306.h`

Install via Arduino IDE Library Manager. For ESP32, add the ESP32 board
package URL in Boards Manager if not already installed.

---

## 8. Tuning Notes

- `IMPACT_THRESHOLD_G` (1.8g), `ORIENTATION_DELTA_DEG` (30°), and
  `GYRO_THRESHOLD_DPS` (120 dps) have been lowered from the original
  conservative defaults (2.5g / 45° / 200 dps) to values more typical of
  what a chest/belt-mounted prototype actually produces during a real
  fall — gear and body movement absorb some impact, so overly high
  thresholds tend to miss genuine falls. Still walk/run/sit/fall test on
  your specific mounting position and adjust further if you see missed
  falls (lower the thresholds) or false triggers during normal movement
  like jogging or sitting down quickly (raise them slightly).
- `FUSION_WINDOW_MS` (300 ms) controls how tightly the impact and
  orientation change must be correlated in time; too short may miss falls
  where rotation lags the impact slightly, too long may allow unrelated
  events to combine into a false alarm.
- **MPU6050 runtime health check:** the transmitter now verifies every
  sensor read (checking the library's success flag and screening for
  NaN values), not just the startup self-test. After 5 consecutive bad
  reads (`MPU_FAILURE_LIMIT`), fall detection is disabled and "FALL DET:
  OFFLINE" appears on the ready screen so the soldier/operator is aware.
  The unit retries reconnecting every 5 seconds (`MPU_RECOVERY_RETRY_MS`)
  and automatically re-enables fall detection once the sensor responds
  again — no manual reset needed for a loose-wire glitch.
- HT12E/HT12D address pins must match exactly between transmitter and
  receiver, or no data will be decoded.

---

## 9. Troubleshooting: RF Link — Soldier Unit Shows "Sending" but Base Station Never Receives

If the soldier unit's OLED confirms "Sending... / Message Sent" on every
button press, but the base station never displays anything, the fault is
almost always in the RF/encoder-decoder link itself, not the firmware.
The ESP32 has no way to confirm actual over-the-air delivery (HT12E has no
feedback line), so "Message Sent" only means the button press and encode
step worked — it says nothing about whether the receiver picked it up.
Check in this order:

1. **Antenna present on both modules.** Cheap 433 MHz TX/RX modules have
   very short range, sometimes just centimeters, without a proper antenna.
   Solder a **17.3 cm** straight wire (quarter-wavelength for 433 MHz) to
   the antenna pad/pin on **both** the TX and RX modules. If you've been
   testing with no antenna at all, this alone is the most likely fix.
2. **HT12E and HT12D address pins match exactly.** Both encoder and
   decoder have address pins (A0–A7). If even one differs between
   transmitter and receiver, the HT12D never asserts VT and silently
   ignores every transmission. Both are tied to GND in this design —
   verify with a multimeter that none has drifted to floating/HIGH from a
   bad solder joint or bent header pin.
3. **Oscillator resistor values match the HT12E/HT12D pair.** Each chip
   needs an external resistor across its OSC1/OSC2 pins that sets the bit
   rate, and the decoder's resistor must be ratioed correctly to the
   encoder's per the datasheet (commonly encoder ≈ 1 MΩ, decoder ≈ 51 kΩ,
   roughly a 1:20 ratio). If the encoder and decoder modules were bought
   separately rather than as a matched TX/RX kit, check the resistor
   value on each board — a mismatch means the decoder's timing window
   never lines up with the incoming bits, so it never validates a word
   even though the encoder is transmitting correctly.
4. **RF module wiring and power.**
   - HT12E **DOUT (pin 17)** → RF TX **DATA** pin (sometimes silkscreened
     "ATAD" on cheap boards — printed backwards, easy to misread).
   - RF RX **DATA OUT** → HT12D **DIN (pin 14)**.
   - Add a decoupling capacitor (100 nF + 10–100 µF) across the RF TX
     module's VCC/GND if not already present; a starved supply rail
     measurably weakens transmit power.
5. **Isolate the RF link from firmware with a quick hardware-only test.**
   Connect an LED (with a ~1 kΩ series resistor) directly from the
   HT12D's **VT pin** to GND. Press a button on the soldier unit:
   - LED never flickers → the fault is 100% in the RF/encoder-decoder
     link (antenna, address pins, oscillator resistors) — the Arduino
     firmware isn't the problem.
   - LED flickers but the Arduino still doesn't react → the fault is in
     the receiver's debounce/read logic instead, not the RF link.

**Suggested order to check:** antenna first (fastest fix, most likely
cause) → address pins → LED-on-VT test → oscillator resistors → RF module
wiring/power.
