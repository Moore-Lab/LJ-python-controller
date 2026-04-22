# LabJack T7 Controller — Usage Guide

## Contents

1. [Requirements](#requirements)
2. [Installation](#installation)
3. [Running the GUI](#running-the-gui)
4. [Connection bar](#connection-bar)
5. [Tabs](#tabs)
   - [AIN — Analog Inputs](#ain--analog-inputs)
   - [DAC — Analog Outputs](#dac--analog-outputs)
   - [DIO — Digital I/O](#dio--digital-io)
   - [Extended Features](#extended-features)
   - [PT100](#pt100)
   - [Thermocouple](#thermocouple)
   - [Registers](#registers)
   - [MQTT Rules](#mqtt-rules)
   - [Console](#console)
6. [CLI shortcuts](#cli-shortcuts)
7. [PT100 calibration workflow](#pt100-calibration-workflow)
8. [Thermocouple calibration workflow](#thermocouple-calibration-workflow)
9. [xsphere-slow-control integration](#xsphere-slow-control-integration)

---

## Requirements

| Package | Purpose |
|---------|---------|
| `labjack-ljm` | LabJack LJM driver (install from [labjack.com/ljm](https://labjack.com/ljm)) |
| `paho-mqtt` | MQTT publish rules tab (optional — all other tabs work without it) |

Python 3.10 or newer required.

---

## Installation

```bash
# Install the labjack_t7 package from this repo
pip install -e .

# Optional: MQTT publish rules
pip install paho-mqtt
```

The LabJack LJM library must also be installed at the OS level before `labjack-ljm` will work. Download the installer for your platform from the LabJack website and run it first.

---

## Running the GUI

```bash
python gui.py
```

All arguments are optional. The GUI opens in a disconnected state and you can connect manually, or use `--connect` to connect automatically on start.

```
python gui.py --help
```

---

## Connection bar

The connection bar is always visible at the top of the window.

| Field | Description |
|-------|-------------|
| **Type** | Connection method: `ANY` (auto), `USB`, `TCP`, or `WIFI` |
| **ID** | Device identifier: serial number, IP address, or `ANY` to use the first found device |
| **Connect / Disconnect** | Open or close the connection |
| **Status** | Shows `Connected — T7 SN#XXXXXX` in green when connected |
| **Poll interval (s)** | Interval used by the global poll, PT100 poll, and TC poll |
| **Start Poll / Stop Poll** | Read all AIN and DIO channels repeatedly at the set interval |

Connecting via USB with a single device attached:

```
Type: USB    ID: ANY
```

Connecting to a specific device by serial number:

```
Type: ANY    ID: 470012345
```

Connecting over Ethernet:

```
Type: TCP    ID: 192.168.1.100
```

---

## Tabs

### AIN — Analog Inputs

Reads and configures the 14 analog input channels (AIN0–AIN13).

| Column | Description |
|--------|-------------|
| **Value (V)** | Last measured voltage |
| **Range (V)** | Input range: ±10 V, ±1 V, ±0.1 V, or ±0.01 V. Use the smallest range that won't clip your signal for best resolution. |
| **Resolution** | Resolution index 0–8. 0 = automatic. Higher values give more bits but slower acquisition. |
| **EF Index** | Extended Feature index to apply to this channel (0 = normal voltage read). See the [Extended Features](#extended-features) tab for the index list. |
| **Read** | Single one-shot read of this channel |
| **Config** | Apply the current range, resolution, and EF index to the device |

**Read All AIN** reads all 14 channels in a single batch call.  
**Configure All AIN** sends configuration for every channel in sequence.

> Setting an EF index here is a shortcut for common use: it writes `AIN#_EF_INDEX` and then any read of that channel returns the EF result. For full control over EF parameters (CONFIG_A–D) use the Extended Features tab.

---

### DAC — Analog Outputs

Controls the two analog output channels, DAC0 and DAC1 (0–5 V).

- Type a value in the text field, or drag the slider.
- The slider and text field stay in sync.
- **Set DACx** writes the current value to the device.
- **Zero (0 V)** immediately writes 0.0 V.

> The T7 DAC outputs are not isolated. They share the T7 ground. Maximum output current is approximately 20 mA per channel.

---

### DIO — Digital I/O

Reads and writes the digital I/O pins arranged in three ports:

| Port | Pins | Notes |
|------|------|-------|
| FIO | FIO0–FIO7 | 3.3 V logic, 5 V tolerant inputs |
| EIO | EIO0–EIO7 | Requires LJTick or terminal block access |
| CIO | CIO0–CIO3 | Shared with SPI/I2C in some configurations |

Each pin shows:

- **Direction dropdown** — `IN` (read) or `OUT` (write). Setting to `OUT` and clicking the checkbox will write that value to the pin.
- **LED indicator** — Green when high, grey when low (updated on read).
- **High checkbox** — Desired output state when direction is OUT.
- **R button** — Read the current state of this pin.

**Read All DIO** reads every pin in sequence.

> Writing a pin (direction = OUT) calls `eWriteName(handle, "FIO0", value)`, which automatically configures that pin as an output in the T7.

---

### Extended Features

Direct access to the T7's AIN Extended Feature (EF) system, which implements signal processing on-chip: RTD resistance, thermocouple linearisation, averaging, and more.

**Configuration fields:**

| Field | Register written |
|-------|-----------------|
| Channel | selects which AIN# all registers apply to |
| EF Index | `AIN#_EF_INDEX` |
| Config A | `AIN#_EF_CONFIG_A` |
| Config B | `AIN#_EF_CONFIG_B` |
| Config C | `AIN#_EF_CONFIG_C` |
| Config D | `AIN#_EF_CONFIG_D` |

**Apply EF Config** writes all five registers at once.  
**Read EF** reads `READ_A` through `READ_D` and displays the results.

The right panel lists the most commonly used EF indices for quick reference.

**Key EF indices:**

| Index | Function |
|-------|----------|
| 0 | Off — normal voltage read |
| 1 | Slope + Offset linear transform |
| 10 | Binary average |
| 20–27 | Thermocouple types E, J, K, N, R, S, T, C |
| 40 | RTD (PT100/PT1000) resistance and temperature |
| 44 | General resistance |

For EF 40 (RTD), `READ_A` returns temperature in Kelvin and `READ_B` returns resistance in ohms. `CONFIG_A` sets which AIN channel provides the current source return; `CONFIG_B` selects 0 = PT100, 1 = PT1000.

---

### PT100

Dedicated tab for 4-wire PT100 resistance measurement and temperature conversion.

#### Measurement methods

**EF (AIN_EF_INDEX = 40, RTD mode)**

Uses the T7's built-in RTD extended feature. The LJTick module or external hardware provides the excitation current. The GUI reads `AIN#_EF_READ_B` (resistance in Ω) directly. Apply the EF configuration once via the Extended Features tab before using this mode — set EF Index = 40, CONFIG_A = current source AIN channel, CONFIG_B = 0 for PT100.

**Manual (R = V / I_excitation)**

For custom excitation circuits. Reads the raw voltage from the specified AIN channel and divides by the entered excitation current (in amperes) to get resistance.

```
R = V_ain / I_excitation
```

Set the excitation current to match your hardware (e.g., 1 mA = 0.001 A is typical for PT100).

#### Temperature conversion

Resistance is converted to temperature using the **Callendar–Van Dusen equation** (IEC 60751):

For T ≥ 0 °C:
```
R(T) = R₀ · (1 + A·T + B·T²)
```

For T < 0 °C:
```
R(T) = R₀ · (1 + A·T + B·T² + C·(T−100)·T³)
```

The GUI solves this numerically (Newton–Raphson) from R to T. Standard IEC 60751 coefficients are pre-filled:

| Coefficient | Value |
|-------------|-------|
| A | 3.9083 × 10⁻³ |
| B | −5.775 × 10⁻⁷ |
| C | −4.183 × 10⁻¹² |
| R₀ | 100 Ω |

All four fields are editable — enter your own calibration coefficients from a traceable calibration certificate to improve accuracy.

**Start Poll / Stop Poll** repeats the measurement at the global poll interval.

---

### Thermocouple

Reads thermocouple signals and optionally converts to temperature.

#### Measurement modes

**Raw EMF (for calibration)**

Reads the raw junction voltage from the selected AIN channel and reports it in millivolts. This is the correct mode for building your own calibration — you collect (EMF, reference temperature) pairs and fit your own polynomial.

- **Differential** — enables differential mode by setting `AIN#_NEGATIVE_CH` to the next channel (ch+1). Recommended for thermocouple measurements to reject common-mode noise. AIN0 uses AIN1 as the negative input, AIN2 uses AIN3, etc.
- **Reference junction source** — either the T7's internal temperature sensor (`TEMPERATURE_DEVICE_K`) or a PT100 on another AIN channel (assumes 1 mA excitation, uses CVD conversion).

**EF Temperature (K, standard tables)**

Sets `AIN#_EF_INDEX` to the NIST polynomial for the selected TC type and reads `AIN#_EF_READ_A` (temperature in K). Uses the T7's built-in cold-junction compensation. Convenient for quick checks but not for custom calibration.

#### Supported thermocouple types

E, J, K, N, R, S, T, C

**Start Poll / Stop Poll** repeats the measurement at the global poll interval.

---

### Registers

Direct read/write access to any named T7 register by name.

- Type any valid LJM register name in the **Register** field and click **Read**.
- Enter a value and click **Write** to write it.
- The **Quick access** buttons pre-fill common register names and immediately read them.

Common registers:

| Register | Description |
|----------|-------------|
| `SERIAL_NUMBER` | Device serial number |
| `HARDWARE_VERSION` | Hardware revision |
| `FIRMWARE_VERSION` | Firmware version |
| `TEMPERATURE_DEVICE_K` | T7 internal temperature sensor (K) |
| `TEMPERATURE_AIR_K` | T7 air temperature sensor (K) |
| `DAC0`, `DAC1` | Analog output voltages |
| `AIN0` … `AIN13` | Analog input voltages |
| `FIO_STATE` | FIO port state bitmask (bits 0–7) |
| `EIO_STATE` | EIO port state bitmask |
| `AIN0_EF_INDEX` | EF index for AIN0 |
| `AIN0_EF_READ_A` | EF result A for AIN0 |

Any register listed in the [LabJack T7 Modbus map](https://labjack.com/pages/support?doc=/datasheets/t-series-datasheet/31-modbus-map-t-series-datasheet/) can be used here.

---

### MQTT Rules

Publish any T7 register value to an MQTT broker on a configurable schedule. Each rule is independent and runs at its own frequency.

#### Connecting to a broker

1. Enter the broker **Host** and **Port** (default: `localhost:1883`).
2. Set a **Client ID** (must be unique on the broker).
3. Click **Connect**. The status indicator turns green when connected.

#### Adding a rule

| Field | Description |
|-------|-------------|
| **Register / channel** | Any named T7 register (e.g. `AIN0`, `TEMPERATURE_DEVICE_K`, `FIO_STATE`) |
| **MQTT topic** | The topic to publish to (e.g. `xsphere/sensors/labjack_t7/ain/ain0`) |
| **Interval (s)** | How often to read and publish. Rules are independent — AIN0 can publish every 0.5 s while temperature publishes every 10 s. |

Click **Add Rule** to create the rule. It starts immediately if the device is connected.

#### Rule table

Each row shows the current state of a rule:

| Column | Description |
|--------|-------------|
| ID | Auto-assigned rule number |
| Register | Register being read |
| Topic | MQTT topic being published to |
| Interval s | Publish frequency |
| Enabled | ✓ running, ✗ paused |
| Last Value | Most recent value read from the device |
| Published | Total number of publishes since the rule was created |

#### Payload format

Every message is published as JSON:

```json
{"value": 1.234567, "register": "AIN0", "ts": 1713700000.123}
```

- `value` — the float value read from the register
- `register` — the register name
- `ts` — Unix timestamp (seconds since epoch)

QoS 1 is used for all publishes.

#### Controls

- **Enable / Disable Selected** — pause or resume a single rule
- **Remove Selected** — permanently delete a rule
- **Enable All / Disable All** — bulk toggle
- **Remove All** — clear all rules

> Rules are not persisted between sessions. If you have a standard set of rules, consider wrapping the launch command in a shell script that also sends MQTT messages to configure things, or simply re-add them each session.

---

### Console

A scrolling log of every device operation, error, and result. Also accepts typed commands:

| Command | Action |
|---------|--------|
| `read <register>` | Read a named register and log the result |
| `write <register> <value>` | Write a value to a named register |
| `connect [id]` | Connect to the device (optionally set the identifier first) |
| `disconnect` | Disconnect from the device |

Press **Enter** or click **Run** to execute.

---

## CLI shortcuts

Every button in the GUI has a corresponding command-line argument that pre-populates fields and executes the action automatically after the device connects.

```
python gui.py [--connect ID] [--connection-type TYPE] [OPTIONS]
```

### Connection

| Argument | Short | Description |
|----------|-------|-------------|
| `--connect <id>` | `-c` | Connect on start. `id` = serial number, IP address, or `ANY` |
| `--connection-type <t>` | `-t` | `ANY` \| `USB` \| `TCP` \| `WIFI` (default: `ANY`) |

### AIN

| Argument | Description |
|----------|-------------|
| `--read-ain <ch> [<ch>...]` | Read one or more AIN channels after connect (e.g. `--read-ain AIN0 AIN1`) |

### DAC

| Argument | Description |
|----------|-------------|
| `--write-dac <ch> <v>` | Write a DAC voltage after connect (e.g. `--write-dac DAC0 2.5`) |

### DIO

| Argument | Description |
|----------|-------------|
| `--write-dio <ch> <v>` | Write a digital output after connect (e.g. `--write-dio FIO0 1`) |

### Registers

| Argument | Description |
|----------|-------------|
| `--read-register <name>` | Read a named register after connect |
| `--write-register <name> <v>` | Write a named register after connect |

### PT100

| Argument | Default | Description |
|----------|---------|-------------|
| `--pt100-channel <ch>` | `AIN0` | Pre-select the PT100 AIN channel |
| `--pt100-excitation <A>` | `0.001` | Excitation current for manual mode (amperes) |
| `--pt100-r0 <Ω>` | `100.0` | Reference resistance R₀ |

### Thermocouple

| Argument | Default | Description |
|----------|---------|-------------|
| `--tc-channel <ch>` | `AIN0` | Pre-select the thermocouple AIN channel |
| `--tc-type <type>` | `K` | TC type: `E` `J` `K` `N` `R` `S` `T` `C` |

### Polling

| Argument | Default | Description |
|----------|---------|-------------|
| `--poll` | off | Auto-start the global AIN+DIO poll immediately after connect |
| `--poll-interval <s>` | `1.0` | Poll interval in seconds (applies to all polls) |

### Examples

```bash
# Connect to the first USB device and start polling at 2 Hz
python gui.py --connect ANY --connection-type USB --poll --poll-interval 0.5

# Connect over TCP and immediately set DAC0 to 1.8 V
python gui.py --connect 192.168.1.100 --connection-type TCP --write-dac DAC0 1.8

# Open with PT100 on AIN2, manual mode, 0.5 mA excitation
python gui.py --connect ANY --pt100-channel AIN2 --pt100-excitation 0.0005

# Read three AIN channels on connect
python gui.py --connect ANY --read-ain AIN0 AIN1 AIN2

# Pre-select K-type thermocouple on AIN4
python gui.py --connect ANY --tc-channel AIN4 --tc-type K

# Read device serial number on connect
python gui.py --connect ANY --read-register SERIAL_NUMBER

# Set a digital output on connect
python gui.py --connect ANY --write-dio FIO3 1
```

---

## PT100 calibration workflow

**Goal:** determine accurate calibration coefficients (A, B, C, R₀) for your specific PT100.

### Hardware setup (EF mode with LJTick)

1. Connect the PT100 in 4-wire configuration to the LJTick module.
2. Wire the LJTick to the T7 AIN header (e.g. AIN0/AIN1 for voltage, AIN2 for current return).
3. In the **Extended Features** tab, set EF Index = 40, CONFIG_B = 0 (PT100), and click **Apply EF Config**.

### Hardware setup (manual mode)

1. Drive a known, stable excitation current through the PT100 (e.g. 1 mA from an external source or the T7 DAC through a precision resistor).
2. Connect the voltage sense wires to a T7 AIN channel in single-ended or differential mode.
3. Select **Manual (R = V / I_excitation)** in the PT100 tab and enter your excitation current.

### Measurement steps

1. Immerse the PT100 in a calibrated reference bath or dewar at a known temperature.
2. Record the displayed **Resistance (Ω)** and the reference temperature.
3. Repeat at several temperatures spanning your range of interest.
4. Fit the Callendar–Van Dusen equation to your (R, T) dataset to obtain calibrated A, B, C, R₀.
5. Enter the fitted coefficients in the PT100 tab; the displayed temperature will now use your calibration.

---

## Thermocouple calibration workflow

**Goal:** collect raw EMF vs. reference temperature data for your own calibration polynomial.

### Hardware setup

1. Connect the thermocouple to a T7 AIN channel. For best noise rejection use two adjacent channels in differential mode (e.g. AIN0 as + and AIN1 as −, enabled by the **Differential** checkbox).
2. For the reference junction, either:
   - Use the T7's internal temperature sensor (acceptable for rough measurements near room temperature).
   - Use a PT100 on another AIN channel with a well-characterised calibration (enter its AIN channel in **Reference junction**).

### Measurement steps

1. Set **Measurement mode** to **Raw EMF (for calibration)**.
2. Place the thermocouple junction at a series of known temperatures (e.g. ice bath 0 °C, boiling water 100 °C, calibrated oil bath, LN₂ at 77 K, etc.).
3. At each temperature, record:
   - The displayed **EMF (mV)**
   - The reference temperature from your primary standard
4. Use your collected (EMF, T) pairs to fit a polynomial or use the NIST inverse coefficients for your TC type.

### Quick check with EF mode

Switch to **EF Temperature (K, standard tables)** to compare the T7's built-in NIST conversion against your reference. This is useful for sanity-checking your setup but should not be used as the final calibration.

---

## xsphere-slow-control integration

The `labjack_t7` package in this repo is a plugin for [xsphere-slow-control](references/xsphere-slow-control). The GUI is a standalone test tool; the plugin is used for automated slow-control operation.

### Wiring the plugin into xsphere

In `slowcontrol/core/service.py`, add to `SlowControlService.__init__()`:

```python
from labjack_t7 import LabJackT7Controller, LabjackT7Config, AinChannelConfig

lj_cfg = LabjackT7Config(
    connection_type="USB",
    ain_channels=[
        AinChannelConfig("AIN0", range_v=0.1),   # PT100 voltage
        AinChannelConfig("AIN2", range_v=0.01),  # thermocouple EMF
    ],
    dio_read_channels=["FIO0"],
    read_internal_temp=True,
    poll_interval_s=1.0,
)
self._controllers.append(
    LabJackT7Controller(self._config, self._mqtt, lj_cfg)
)
```

### MQTT topics published by the plugin

| Topic | Payload |
|-------|---------|
| `xsphere/sensors/labjack_t7/ain/ain0` | `{"value": 0.012, "unit": "V"}` |
| `xsphere/sensors/labjack_t7/dio/fio0` | `{"value": 1}` |
| `xsphere/sensors/labjack_t7/temperature` | `{"value": 295.1, "unit": "K"}` |
| `xsphere/status/labjack_t7` | `{"connected": true, "serial": "470012345", "error": null, …}` |

### MQTT commands accepted by the plugin

| Topic | Payload |
|-------|---------|
| `xsphere/commands/labjack_t7/write_dac` | `{"channel": "DAC0", "value": 2.5}` |
| `xsphere/commands/labjack_t7/write_dio` | `{"channel": "FIO0", "value": 1}` |
| `xsphere/commands/labjack_t7/read` | `{"channels": ["AIN0", "AIN1"]}` |
| `xsphere/commands/labjack_t7/configure` | `{"poll_interval_s": 2.0, "ain_channels": [{"name": "AIN0", "range_v": 0.1}]}` |
| `xsphere/commands/labjack_t7/reset` | `{}` |
