# 🌦️ ESP32-S3 Weather Station

A battery-powered, Wi-Fi-connected weather station built around the **ESP32-S3**
microcontroller.  Sensor data is published periodically to an **MQTT broker**
in JSON format.  Firmware updates are delivered **OTA** via GitHub Releases.

---

## ✨ Features

| Feature | Status |
|---|---|
| Provisioning via captive-portal Wi-Fi AP | ✅ v0.1.0 |
| INI-based runtime configuration on LittleFS | ✅ v0.1.0 |
| FreeRTOS task architecture + deep-sleep | 🚧 v0.2.0 (Increment 1) |
| Sensor drivers (BME280, AS5600, INA3221 …) | 🚧 v0.3.0 (Increment 2) |
| MQTT JSON publishing | 🚧 v0.4.0 (Increment 3) |
| OTA via GitHub Releases API | 🚧 v0.5.0 (Increment 4) |

---

## 🔩 Hardware

### Microcontroller
- **ESP32-S3-DevKitC-1** (or compatible ESP32-S3 board)

### Sensors

| Sensor | Interface | Measured quantity |
|---|---|---|
| BME280 | I²C (0x76) | Temperature, Humidity, Pressure |
| AS5600 | I²C (0x36) | Wind direction (magnetic encoder) |
| Tipping-bucket rain gauge | GPIO19 (reed switch) | Rainfall accumulation |
| Cup anemometer | GPIO20 (reed switch) | Wind speed |
| PMSA003I / PMS5003 | UART (RX=17, TX=18) | PM1.0 / PM2.5 / PM10 |
| CCS811 | I²C (0x5A), WAKE=GPIO41 | eCO₂, TVOC |
| MICS6814 | ADC (GPIO5, 6, 7) | CO, NO₂, NH₃ |
| INA3221 | I²C (0x40) | Voltage, current (3 channels) |

### Power
| Signal | GPIO |
|---|---|
| 5 V MOSFET enable (active LOW) | GPIO11 |
| 3.3 V MOSFET enable (active LOW) | GPIO4 |

### I²C Bus
| Signal | GPIO |
|---|---|
| SDA | GPIO8 |
| SCL | GPIO9 |

> **Tip:** All I²C devices share the same SDA/SCL lines.
> Make sure each device has the correct pull-up resistors (4.7 kΩ to 3.3 V is recommended).

---

## 🛠️ Software Requirements

| Tool | Version |
|---|---|
| [PlatformIO Core](https://platformio.org/) | ≥ 6.x |
| PlatformIO IDE (VS Code extension) | optional but recommended |
| Git | any recent version |
| Python | ≥ 3.8 (used by PlatformIO) |

---

## 🚀 Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/CarloFalco/WeatherStation.git
cd esp32-weather-station
```

### 2. Build the firmware

```bash
pio run
```

### 3. Upload the LittleFS filesystem image

This uploads `data/index.html` and the template `data/config.ini` to the device.

```bash
pio run -t uploadfs
```

### 4. Upload the firmware

```bash
pio run -t upload
```

### 5. First-time configuration

1. On first boot (no `config.ini`) the device starts a Wi-Fi Access Point:
   - **SSID:** `WeatherStation-Setup`
   - **Password:** *(open – no password)*
2. Connect to this network from your phone or laptop.
3. A captive-portal page will open automatically.
   If it does not, navigate to **http://192.168.4.1** in your browser.
4. Fill in your Wi-Fi, MQTT, and GitHub credentials, then press **Save & Restart**.
5. The device restarts, connects to your network, and begins normal operation.

### 6. Factory reset

Hold **GPIO0** (BOOT button) for **3 seconds** while the device is running to
erase the stored configuration and re-enter the setup wizard.

---

## 📁 Project Structure

```
esp32-weather-station/
├── .github/workflows/      # CI/CD – release automation
├── data/                   # LittleFS filesystem image
│   ├── config.ini          # Runtime configuration template
│   └── index.html          # Captive-portal setup page
├── docs/
│   └── Doxyfile            # Doxygen configuration
├── include/
│   ├── config.h            # Pin & address definitions (hardware constants)
│   └── version.h           # Firmware version (SemVer)
├── progress/               # Development progress logs (Markdown)
├── src/
│   ├── main.cpp            # Application entry point
│   ├── config/             # INI file parser / AppConfig singleton
│   ├── connectivity/       # Wi-Fi & MQTT managers (Increment 1–3)
│   ├── ota/                # OTA update manager (Increment 4)
│   ├── power/              # PowerManager / deep-sleep (Increment 1)
│   ├── provisioning/       # Captive-portal setup wizard
│   └── sensors/            # Per-sensor driver classes (Increment 2)
├── .gitignore
├── CHANGELOG.md
├── partitions.csv          # Custom OTA + LittleFS partition table
├── platformio.ini
└── README.md
```

---

## 📖 Documentation

Generate Doxygen HTML docs:

```bash
cd docs
doxygen Doxyfile
# Open docs/html/index.html in your browser
```

---

## 🔖 Versioning

This project follows [Semantic Versioning](https://semver.org/).
Each increment milestone corresponds to a tagged GitHub Release.

| Tag | Milestone |
|---|---|
| v0.1.0 | Increment 0 – Scaffold & provisioning |
| v0.2.0 | Increment 1 – FreeRTOS, deep-sleep, Wi-Fi, RTC |
| v0.3.0 | Increment 2 – Sensor drivers |
| v0.4.0 | Increment 3 – MQTT JSON publishing |
| v0.5.0 | Increment 4 – OTA via GitHub Releases |

---

## 📜 License

MIT – see [LICENSE](LICENSE) for details.

