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
| 5 V | Pluviometro, anemometro (reed) | ⚠️ pull-up a 5 V: vedi sotto |

## Scheda v1.0 (schematico: `docs/WeatherStationV2.pdf`, PCB: `docs/WeatherStationV2_PCB.pdf`)

Verifica dello schematico rispetto a `include/config.h` (2026-10-21): tutti i
pin coincidono. Note per componente:

| Blocco | Esito | Note |
|--------|-------|------|
| LoRa CN1 | ✔ | DIO0 su **GPIO 8** (net `GPIO08`) — non GPIO 7, che è l'anemometro |
| Bus I2C | ✔ | Pull-up R1/R3 4.7k su **+3V3SUP** (via JP1/JP2): spariscono col ramo spento, nessuna alimentazione fantasma verso l'AS5600 |
| Ramo commutato | ✔ | GPIO 18 → R4 1k → 2N3904 (R5 10k a GND sulla base) → gate IRF9540 (R2 5k1 a +3V3). HIGH = acceso, pin flottante = spento. J1 seleziona MOSFET o bypass: per il risparmio deve stare sul MOSFET |
| Tasto reset U1 | ✔ | GPIO 17, pull-up R12 10k a +3V3, contatto verso GND |
| Sonda umidità RJ7 | ✔ | Alimentata da +3V3SUP: uscita entro il range dell'ADC |
| **Pluviometro RJ3** | ⚠️ | **R9 10k da GPIO 6 a +5V** |
| **Anemometro RJ4** | ⚠️ | **R8 10k da GPIO 7 a +5V** |

> ⚠️ **R8 e R9 tirano GPIO 6/7 a 5 V.** I GPIO dell'ESP32-S3 non sono 5 V
> tolerant (VIH max = VDD + 0.3 V ≈ 3.6 V). Con il reed aperto — cioè quasi
> sempre — la 10k inietta ~0.12 mA per pin nei diodi di clamp verso il 3.3 V,
> anche durante il deep sleep: è fuori specifica e, con due pin, aggiunge
> ~0.25 mA al consumo di sleep (≈ 15× il budget di ~15 µA: autonomia stimata
> da ~315 a ~200 giorni).
>
> **Correzione consigliata** (taglio pista + ponticello, o revisione PCB):
> - **R9 (pluviometro, GPIO 6) → +3V3** (sempre acceso): il pull-up serve
>   anche in deep sleep per il wake-on-rain;
> - **R8 (anemometro, GPIO 7) → +3V3SUP** (commutato): l'anemometro non è
>   una sorgente di wake, e così se le coppe si fermano con il magnete sul
>   reed (contatto chiuso) non scorrono 0.33 mA tutta la notte.
>
> Il pin 5 dei connettori RJ3/RJ4 (+5V) non serve a un reed passivo a due
> fili. Se invece i sensori sono moduli attivi (es. a effetto Hall) che
> richiedono 5 V, la loro uscita va riportata a 3.3 V con un partitore o un
> level shifter.

> **Nota sull'IRF9540**: non è logic-level (soglia fino a −4 V, Rds(on)
> specificata a −10 V) e pilotato su un ramo a 3.3 V lavora appena sopra
> soglia. Con ~6.5 mA di carico funziona, ma va misurata la tensione del
> ramo commutato sotto carico — dettagli in `docs/power-budget.md`.





