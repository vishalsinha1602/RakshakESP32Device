# RAKSHAK – ESP32 GPS Emergency Alert Device

RAKSHAK is an ESP32-based personal safety and emergency alert device designed to detect an emergency button press, obtain the user's GPS location, display device status on an OLED, and send the location through a connected Wi-Fi network.

## Features

- 🚨 Emergency push button
- 📍 NEO-M8N GPS for real-time location
- 📺 0.96-inch OLED display
- 📡 ESP32 Wi-Fi connectivity
- 🔴 Emergency/status LED with 220 Ω resistor
- 🔋 3.7 V 18650 Li-ion battery
- 🔌 TP4056 battery charging module
- ⚡ DC-DC step-up/step-down power module
- 🔘 Physical power switch
- 📱 Location sharing through the configured alert system

## Hardware

| Component | Purpose |
|---|---|
| ESP32 Dev Module | Main controller and Wi-Fi |
| NEO-M8N GPS Module | GPS coordinates |
| GPS Antenna | GPS signal reception |
| 0.96" OLED Display | Status, GPS and device information |
| Emergency Push Button | Activates emergency mode |
| LED | Visual emergency/status indication |
| 220 Ω Resistor | LED current limiting |
| TP4056 Charging Module | Li-ion battery charging/protection |
| 18650 3.7 V 2000 mAh Battery | Portable power source |
| DC-DC Converter | Converts battery voltage to required output |
| Power Switch | Turns the device ON/OFF |

## Circuit Diagram

<img width="1546" height="1454" alt="circuitDaigram" src="https://github.com/user-attachments/assets/85eac0af-711b-4db1-b23b-85549527e8e0" />


> Place the circuit diagram image in the repository root with the filename `circuit-diagram.png`.

### Interactive Circuit

The complete interactive circuit can be viewed in Cirkit Designer:

**[Open the interactive circuit](https://app.cirkitdesigner.com/project/2fc0ab2d-cccd-4329-8e30-1d043b0ca875?view=interactive_preview)**

> GitHub README files generally do not support arbitrary interactive `<iframe>` embeds reliably. A direct link to the Cirkit Designer project is therefore recommended.

## Main Connections

### GPS → ESP32

| GPS Pin | ESP32 |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| TX | ESP32 RX |
| RX | ESP32 TX |

The exact GPIO numbers should match the pins configured in the ESP32 firmware.

### OLED → ESP32

| OLED Pin | ESP32 |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SCL | I2C SCL |
| SDA | I2C SDA |

The I2C GPIO numbers should match the firmware configuration.

### Emergency Button

The emergency button is connected to a digital GPIO input. When pressed, the ESP32 enters the emergency procedure and handles the location/alert workflow.

### LED

The LED is connected through a **220 Ω current-limiting resistor** and is used as a visual status indicator.

## Power System

The power path is:

```text
18650 Battery
      │
      ▼
   Power Switch
      │
      ▼
 DC-DC Converter
      │
      ├──► ESP32
      ├──► OLED
      └──► GPS
```

The TP4056 module is used for charging the 18650 battery.

### Battery percentage sensing

The ESP32 estimates battery charge from the 18650 cell voltage. Add a **100 kΩ / 100 kΩ voltage divider** from the cell side of the TP4056 (before the DC-DC converter) to the ESP32:

```text
18650 B+ ── 100 kΩ ──┬── GPIO 34 (ADC)
                     ├── 100 kΩ ── 18650 B- / ESP32 GND
                     └── 100 nF ── 18650 B- / ESP32 GND
```

Connect the cell negative and ESP32 ground together. Do not connect the cell positive directly to GPIO 34: a charged 18650 reaches 4.2 V, above the ESP32 input limit. The ADC pin must have this divider installed before using the battery reading; an unconnected ADC pin can float and produce unreliable values.

The displayed percentage is an estimate from cell voltage and can shift with load and temperature. A MAX17048 fuel-gauge module is recommended if a more accurate state-of-charge reading is needed.

**Important:** Verify the DC-DC converter output voltage with a multimeter before connecting it to the ESP32, GPS, or OLED.

## Software

The firmware is intended for the **Arduino IDE** and an ESP32 board package.

Typical libraries used by this type of project include:

- `WiFi.h`
- `TinyGPSPlus.h`
- `Wire.h`
- `Adafruit_GFX.h`
- `Adafruit_SSD1306.h`

Install the required libraries through the Arduino IDE Library Manager before compiling.

## Basic Working Flow

```text
Power ON
   │
   ▼
ESP32 starts
   │
   ├──► Connect to configured Wi-Fi
   │
   ├──► Start GPS
   │
   └──► Start OLED
          │
          ▼
     Wait for GPS fix
          │
          ▼
    Display device status
          │
          ▼
   Emergency button pressed?
       │           │
      NO          YES
       │           │
       │           ▼
       │      Get latest GPS
       │           │
       │           ▼
       │      Send emergency
       │       location/alert
       │
       └───────────┘
```

## Repository Structure

```text
RAKSHAK/
│
├── README.md
├── circuit-diagram.png
├── RAKSHAK.ino
│
└── images/
    └── ...
```

## Safety Notes

- Do not connect the raw 3.7 V battery directly to a 3.3 V-only component unless the component and board are designed for it.
- Check the polarity of the 18650 battery before connecting it.
- Confirm the DC-DC converter output voltage before powering the circuit.
- Use a suitable protected/rechargeable Li-ion cell and an appropriate charging circuit.
- Keep battery wiring insulated to prevent accidental short circuits.
- The emergency alert system should be tested thoroughly before relying on it in a real emergency.

## Interactive Circuit

**Cirkit Designer:**  
https://app.cirkitdesigner.com/project/2fc0ab2d-cccd-4329-8e30-1d043b0ca875?view=interactive_preview

## Project Status

🚧 **Development in progress**

The hardware and firmware can be modified as the project evolves.

## License

Add your preferred open-source license here, for example **MIT License**.
