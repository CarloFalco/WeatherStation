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

| GPIO | Funzione        | Periferica     | Direzione       | Vincoli                                        |
|------|-----------------|----------------|---------------- |------------------------------------------------|
| 11   | SPI MOSI        | SX1276         | OUT             | bus SPI condiviso, non riusare                 |
| 13   | SPI MISO        | SX1276         | IN              |                                                |
| 12   | SPI SCK         | SX1276         | OUT             |                                                |
| 10   | LoRa NSS (CS)   | SX1276         | OUT             |                                                |
| 9    | LoRa RESET      | SX1276         | OUT             |                                                |
| 8    | LoRa DIO0 (IRQ) | SX1276         | IN              | interrupt-capable, NON spostare                |
| 4    | I²C SDA         | BME280 @0x76   | I/O             | pull-up 4.7k esterni                           |
| 5    | I²C SCL         | BME280 @0x76   | I/O             | Sensore temperatura e umidita                  |
| 4    | I²C SDA         | INA3221 @0x40  | I/O             |                                                |
| 5    | I²C SCL         | INA3221 @0x40  | I/O             | Misurazione corrente e tensione PV/BAT/Load    |
| 4    | I²C SDA         | AS5600 @0x36   | I/O             |                                                |
| 5    | I²C SCL         | AS5600 @0x36   | I/O             | Sensore direzione del vento                    |
| 48   | Data WS2812     | LED RGB        | OUT             | pin del LED onboard sul DevKitC-1 (`LED_BUILTIN`) |
| 6    | Pluviometro     | Reed switch    | IN (pull-up)    |                                             |
| 7    | Anemometro      | Reed switch    | IN (pull-up)    | conteggio impulsi solo durante la veglia     |
| 1    | Umidita Terreno | Sonda capacitiva | IN (analogico) | **ADC1**_CH0 (obbligatorio ADC1). Sonda alimentata a **3.3 V**: a 5 V l'uscita arriva a ~4.2 V e danneggia l'ADC |
| 17   | Tasto reset     | Switch verso GND | IN (pull-up)  | RTC-capable ⇒ sorgente di wake EXT1 (condivisa col pluviometro) |
| 18   | Alimentazione ramo 3.3V commutato | NPN + IRF9540 | OUT, HIGH = acceso (hold in deep sleep) | Pull-down 10k ⇒ spento se non pilotato. Alimenta AS5600 + sonda umidità |

## Alimentazioni

| Ramo | Dispositivi | Note |
|------|-------------|------|
| 3.3 V sempre acceso | BME280, INA3221, SX1276 | Consumo a riposo ~2.3 µA in totale: commutarli non darebbe risparmio e creerebbe back-powering dai pin dell'ESP32 |
| **3.3 V commutato (GPIO 18)** | **AS5600, sonda umidità terreno** | ~6.5 mA a riposo: sono il 99% del budget di sleep. Spento durante il deep sleep dal firmware |
| 5 V | Pluviometro, anemometro (reed) | ⚠️ vedi avvertenza sotto |

> ⚠️ **I reed switch non vanno alimentati.** Un reed è un contatto passivo:
> il firmware configura GPIO 6 e 7 come `INPUT_PULLUP` (3.3 V interni) e si
> aspetta che il contatto li porti **a GND**. Se sui pin arriva un livello
> a 5 V (contatto verso 5 V, oppure pull-up esterno a 5 V) si superano i
> limiti d'ingresso dell'ESP32-S3: si danneggia il pin e, in deep sleep, i
> diodi di clamp ri-alimentano parzialmente la scheda. Cablaggio corretto:
> un capo del reed al GPIO, l'altro a **GND**, nessuna alimentazione.
> Se i sensori installati sono moduli attivi con uscita a 5 V serve un
> level shifter (o un partitore) prima del GPIO.

> **Nota sull'IRF9540**: non è logic-level (soglia fino a −4 V, Rds(on)
> specificata a −10 V) e pilotato su un ramo a 3.3 V lavora appena sopra
> soglia. Con ~6.5 mA di carico funziona, ma va misurata la tensione del
> ramo commutato sotto carico — dettagli in `docs/power-budget.md`.





