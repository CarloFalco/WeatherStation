# WeatherStation V2

Stazione meteorologica a batteria basata su **ESP32-S3**, con trasmissione
dati via **LoRa (SX1276, 868 MHz)** verso un gateway e aggiornamenti
firmware **OTA via LoRa** (in roadmap). Autonomia attesa: > 12 mesi grazie a
deep sleep e invio periodico (default: ogni 10 minuti).

> Sviluppo a piccoli increment validati su hardware: vedi
> [docs/architecture.md](docs/architecture.md) per l'architettura e la roadmap,
> e [docs/progress/](docs/progress/) per il log di avanzamento.

## Hardware

| Componente | Ruolo | Collegamento |
|------------|-------|--------------|
| ESP32-S3 Dev Module | MCU (deep sleep, RTC, ULP) | — |
| SX1276 868 MHz | Radio LoRa | SPI: SCK 12, MISO 13, MOSI 11, CS 10, RST 9, DIO0 8 |
| BME280 | Temperatura / umidità / pressione | I2C `0x76` |
| AS5600 | Direzione vento (banderuola) | I2C `0x36` |
| INA3221 | Correnti pannello / batteria / carico | I2C `0x40` |
| Pluviometro (reed) | Pioggia a impulsi | GPIO 6 (wake-on-rain) |
| Anemometro (reed) | Velocità vento a impulsi | GPIO 7 |

Il pinout completo è definito in [include/config.h](include/config.h).

## Requisiti software

- [PlatformIO](https://platformio.org/) (CLI o estensione VS Code)
- Piattaforma `espressif32`, framework Arduino (installati da PlatformIO)

## Compilazione e flash

```bash
# Compila
pio run

# Flash via USB (la porta viene rilevata automaticamente)
pio run -t upload

# Monitor seriale (115200 baud)
pio device monitor

# Upload del filesystem LittleFS (config.ini) — da Increment 1
pio run -t uploadfs
```

## Versioning e release

Il progetto segue [SemVer](https://semver.org/): la serie V2 parte da
`v2.0.0` (i tag `v0.x` appartengono al prototipo precedente). Ogni increment
completato produce un tag `vX.Y.Z` e una GitHub Release con changelog e
binario del firmware, generata automaticamente dalla CI
([.github/workflows/release.yml](.github/workflows/release.yml)).

Changelog completo: [CHANGELOG.md](CHANGELOG.md).
