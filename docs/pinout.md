# Pinout — ESP32-S3-DevKitC-1

> Fonte di verità: `src/config.h`. Questo file è la vista leggibile; se divergono, vince config.h.

## Nodo esterno

| GPIO | Funzione        | Periferica     | Direzione | Vincoli                                    |
|------|-----------------|----------------|-----------|--------------------------------------------|
| 11   | SPI MOSI        | SX1276         | OUT       | bus SPI condiviso, non riusare              |
| 13   | SPI MISO        | SX1276         | IN        |                                             |
| 12   | SPI SCK         | SX1276         | OUT       |                                             |
| 10   | LoRa NSS (CS)   | SX1276         | OUT       |                                             |
| 9    | LoRa RESET      | SX1276         | OUT       |                                             |
| 8    | LoRa DIO0 (IRQ) | SX1276         | IN        | interrupt-capable, NON spostare             |
| 4    | I²C SDA         | BME280 @0x76   | I/O       | pull-up 4.7k esterni                        |
| 5    | I²C SCL         | BME280 @0x76   | I/O       |                                             |
| 48   | Data WS2812     | LED RGB        | OUT       | pin del LED onboard sul DevKitC-1           |
| 1    | ADC batteria    | Partitore 1:2  | IN        | ADC1 (ADC2 inutilizzabile con Wi-Fi attivo) |

## Pin da NON usare (ESP32-S3)
- GPIO 0: strapping (boot mode)
- GPIO 19/20: USB D-/D+ (USB-JTAG)
- GPIO 26–32: flash/PSRAM interne — MAI
- GPIO 45/46: strapping (VDD_SPI / boot)