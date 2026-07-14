# Pinout — ESP32-S3-DevKitC-1

> Fonte di verità: `src/config.h`. Questo file è la vista leggibile; se divergono, vince config.h.

# Configurazione ESP32-S3

## UART

| GPIO | Funzione |
|--------|--------|
| 43     | UART0 TX |
| 44     | UART0 RX |

# USB nativa

| GPIO | Funzione |
|--------|--------|
| GPIO19 | USB D− |
| GPIO20 | USB D+ |

## ADC1

| GPIO | Canale |
|--------|--------|
| GPIO1 | ADC1_CH0 |
| GPIO2 | ADC1_CH1 |
| GPIO3 | ADC1_CH2 |
| GPIO4 | ADC1_CH3 |
| GPIO5 | ADC1_CH4 |
| GPIO6 | ADC1_CH5 |
| GPIO7 | ADC1_CH6 |
| GPIO8 | ADC1_CH7 |
| GPIO9 | ADC1_CH8 |
| GPIO10 | ADC1_CH9 |

## ADC2

| GPIO      | Canale |
|-----------|--------|
| GPIO11    | ADC2_CH0 |
| GPIO12    | ADC2_CH1 |
| GPIO13    | ADC2_CH2 |
| GPIO14    | ADC2_CH3 |
| GPIO15    | ADC2_CH4 |
| GPIO16    | ADC2_CH5 |
| GPIO17    | ADC2_CH6 |
| GPIO18    | ADC2_CH7 |
| GPIO19    | ADC2_CH8 |
| GPIO20    | ADC2_CH9 |

## SPI (consigliato)

Configurazione tipica:

| Segnale | GPIO |
|--------|--------|
| MOSI | GPIO11 |
| MISO | GPIO13 |
| SCK | GPIO12 |
| CS | GPIO10 |

## I²C (consigliato)

Configurazione tipica:

| Segnale | GPIO |
|--------|--------|
| SDA | GPIO8 |
| SCL | GPIO9 |

## Pin da NON usare (ESP32-S3)
- GPIO 0: strapping (boot mode)
- GPIO 19/20: USB D-/D+ (USB-JTAG) — liberi da funzioni di progetto dal
  passaggio dei reed switch a GPIO 6/7: l'USB nativo è utilizzabile
- GPIO 26–32: flash/PSRAM interne — MAI
- GPIO 33–37: sul modulo **N16R8** (PSRAM octal) sono collegati alla PSRAM
  interna — NON utilizzabili come GPIO esterni (e 33+ non è RTC-capable:
  niente wake dal deep sleep)
- GPIO 45/46: strapping (VDD_SPI / boot)


# Configurazione di questo progetto

| GPIO | Funzione        | Periferica     | Direzione | Vincoli                                    |
|------|-----------------|----------------|-----------|--------------------------------------------|
| 11   | SPI MOSI        | SX1276         | OUT       | bus SPI condiviso, non riusare                 |
| 13   | SPI MISO        | SX1276         | IN        |                                                |
| 12   | SPI SCK         | SX1276         | OUT       |                                                |
| 10   | LoRa NSS (CS)   | SX1276         | OUT       |                                                |
| 9    | LoRa RESET      | SX1276         | OUT       |                                                |
| 8    | LoRa DIO0 (IRQ) | SX1276         | IN        | interrupt-capable, NON spostare                |
| 4    | I²C SDA         | BME280 @0x76   | I/O       | pull-up 4.7k esterni                           |
| 5    | I²C SCL         | BME280 @0x76   | I/O       | Sensore temperatura e umidita                  |
| 4    | I²C SDA         | INA3221 @0x40  | I/O       |                                                |
| 5    | I²C SCL         | INA3221 @0x40  | I/O       | Misurazione corrente e tensione PV/BAT/Load    |
| 4    | I²C SDA         | AS5600 @0x36  | I/O        |                                                |
| 5    | I²C SCL         | AS5600 @0x36  | I/O        | Sensore direzione del vento                    |
| 48   | Data WS2812     | LED RGB        | OUT       | pin del LED onboard sul DevKitC-1 (`LED_BUILTIN`) |
| 6    | Pluviometro     | Reed switch    | IN (pull-up) |                                             |
| 7    | Anemometro      | Reed switch    | IN (pull-up) | conteggio impulsi solo durante la veglia     |





