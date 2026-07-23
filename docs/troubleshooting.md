# Troubleshooting (append-only: aggiungi, non cancellare)

Ogni problema risolto diventa una voce: sintomo → causa → fix (+ commit).
Consultare SEMPRE questo file prima di diagnosticare un problema nuovo.

## 1. LED_BUILTIN "97" dei requisiti non è un GPIO valido
- **Sintomo:** i requisiti indicavano `LED_BUILTIN 97`, ma l'ESP32-S3 arriva a GPIO 48.
- **Causa:** refuso nei requisiti originali.
- **Fix:** `STATUS_LED_PIN = LED_BUILTIN` (macro del core Arduino per la board), validato su hardware. Commit `b5af24b`.

## 2. Default I2C dell'ESP32-S3 in conflitto con i pin LoRa
- **Sintomo:** i default Arduino S3 sono SDA=8/SCL=9, che collidono con LORA_DIO0=8 e LORA_RST=9.
- **Causa:** pinout LoRa scelto prima di considerare i default I2C.
- **Fix:** bus I2C rimappato su SDA=4/SCL=5 con `Wire.begin(I2C_SDA, I2C_SCL)` in main.cpp. Mai usare `Wire.begin()` senza argomenti in questo progetto.

## 3. `BME280_ADDRESS` ridefinita: warning di build
- **Sintomo:** `warning: "BME280_ADDRESS" redefined` — la libreria Adafruit definisce già quella macro (con default 0x77!).
- **Fix:** la nostra macro si chiama `BME280_I2C_ADDR` (0x76) in config.h. Commit `cacd955`.

## 4. Fattore pluviometro del vecchio prototipo sbagliato di 100×
- **Sintomo:** `CONVERSIONE_PLUVIOMETRO = 47.43` ⇒ 47 mm di pioggia per basculata (impossibile).
- **Causa:** errore di unità: cm² trattati come 10⁻⁶ m² invece di 10⁻⁴.
- **Fix:** 3 cm³ / 63.25 cm² = **0.4743 mm/basculata** (`[rain] mm_per_pulse` in config.ini). Commit `9dc3173`.

## 5. INA3221: mappatura canali diversa dal commento del vecchio codice
- **Sintomo:** tensioni pannello/carico scambiate; "corrente pannello sempre 0" (in realtà era il carico, alimentato via USB durante i test).
- **Causa:** il commento del vecchio prototipo diceva CH1=pannello/CH3=carico; il cablaggio reale è l'opposto.
- **Fix:** mappatura verificata su hardware: **CH1 = carico, CH2 = batteria, CH3 = pannello**. Commit `02271a1`.

## 6. INA3221: correnti 1000× troppo piccole
- **Sintomo:** batteria in carica a ~600 mA letta come −0.8 mA.
- **Causa:** µV/mΩ dà già mA; il `/1000` extra era un residuo del vecchio `getCurrent()` che restituiva ampere.
- **Fix:** rimosso il `/1000` in `Ina3221Driver::currentMa`. Commit `890e38e`.

## 7. INA3221: shunt reale da 100 mΩ, non 10
- **Sintomo:** dopo il fix №6, correnti 10× troppo grandi (−6088 mA con 610 mA reali).
- **Causa:** il vecchio codice impostava 10 mΩ; il modulo monta shunt da 100 mΩ (61 mV misurati / 610 mA = 100 mΩ).
- **Fix:** `[power] shunt_mohm = 100` (default aggiornato anche nel codice). Ricordarsi `uploadfs` dopo aver cambiato config.ini. Commit `c50fa02`.

## 8. INA3221: corrente pannello sempre ~0 nonostante il pannello eroghi
- **Sintomo:** 660 mA reali dal pannello ma ~80 µV sullo shunt CH3.
- **Causa:** cablaggio — la corrente del pannello non attraversa IN+/IN− del canale 3 (solo il ramo di tensione è collegato).
- **Fix:** hardware, non firmware: instradare il positivo del pannello attraverso lo shunt del CH3. (Stato: da sistemare sulla scheda.)

## 9. Ricevitore LoRa: un pacchetto ricevuto e poi silenzio
- **Sintomo:** DebugLora riceveva una singola stringa, poi più nulla.
- **Causa:** ricevitore a **868.0 MHz**, stazione a **868.1 MHz**: con BW 125 kHz un offset di 100 kHz è fuori aggancio (il pacchetto singolo era passato per caso/deriva).
- **Fix:** `LORA_FREQ 868.1E6` + sync word esplicita + `enableCrc()` + JSON doc 512 B. I parametri radio dei due nodi vanno SEMPRE confrontati con docs/lora-protocol.md. Commit `0999c6c`.

## 10. Deep sleep: la porta COM sparisce a ogni ciclo (solo USB nativo)
- **Sintomo:** monitor seriale che cade quando la stazione dorme.
- **Causa:** con l'USB *nativo* dell'S3 la porta si ri-enumera a ogni wake-up; non è un crash.
- **Fix:** usare il connettore UART (bridge) per il monitor durante lo sviluppo.

## 11. Trasferimento OTA sempre fallito dopo ~240 chunk
- **Sintomo:** con un firmware reale (425 kB, 2365 chunk) il trasferimento
  si interrompeva sempre attorno al chunk 240, con CRC non corrispondente.
- **Causa:** `kSessionTimeoutMs = 120000` (2 minuti), dimensionato sul file
  di test da 4 kB. A ~0.5 s/chunk servono **~20 minuti**: il tetto scattava
  a 120.3 s misurati, cioè esattamente al timeout.
- **Fix:** `session_timeout_s` in `config.ini` (default 1800 s) + ripresa
  del trasferimento da RTC RAM, così una sessione interrotta non riparte
  da zero. Commit successivo a `489620f`.

## 12. OTA: chunk falliti a raffica su link ottimo (RSSI -35 dBm)
- **Sintomo:** `OTA: chunk N failed after 8 attempts` pur con la base a
  pochi metri.
- **Causa (concomitanti):** (a) `LoRaLink::receiveRaw` usava il `receive()`
  bloccante di RadioLib, che ascolta in finestre **RX-single** da ~102 ms
  con brevi buchi in mezzo: un chunk il cui preambolo cadeva nel buco era
  perso; (b) la base, dopo `LoRa.endPacket()`, restava in standby fino al
  successivo `parsePacket()` e perdeva la richiesta seguente, che la
  stazione invia pochi ms dopo.
- **Fix:** RX **continuo** con polling di DIO0 in `receiveRaw`; sulla base
  `LoRa.receive()` subito dopo `endPacket()` e log spostato dopo il riarmo.

## 13. ArduinoJson: CRC-32 letto come 0 (default `| 0` con valori unsigned)
- **Sintomo:** `ota_done` con `CRC 0x00000000` pur avendo ricevuto dati.
- **Causa:** `doc["crc"] | 0` — il letterale `0` è un `int`, quindi
  ArduinoJson converte con `as<int>()` e un valore sopra `0x7FFFFFFF`
  (metà dei CRC possibili) finisce fuori range e torna 0. Lo stesso bug
  era lato stazione su `offer.crc`: avrebbe fatto fallire ~metà degli
  aggiornamenti con un mismatch inspiegabile.
- **Fix:** default unsigned ovunque (`| 0UL`), in stazione, DebugOta e
  DebugLora. Regola: per campi `uint32_t` in JSON usare sempre `| 0UL`.

## 14. (Workflow) PowerShell 5.1: virgolette doppie nei messaggi di commit
- **Sintomo:** `git commit -m @'...'@` fallisce con `pathspec ... did not match` se il messaggio contiene `"..."`.
- **Causa:** bug di quoting di PowerShell 5.1 verso gli eseguibili nativi (le `"` interne spezzano gli argomenti).
- **Fix:** niente virgolette doppie nei messaggi di commit, oppure `git commit -F file`.
