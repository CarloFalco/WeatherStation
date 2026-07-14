Crea un nuovo sensore per la stazione meteo: $ARGUMENTS

Segui questo processo, un passo alla volta:

1. Leggi `docs/pinout.md` e `include/config.h` per i pin/indirizzi disponibili;
   se il pin non è assegnato, proponi un'assegnazione rispettando la sezione
   "Pin da NON usare" (e ricordando che su N16R8 i GPIO 33-37 sono della PSRAM).
   Per il wake dal deep sleep servono pin RTC-capable (GPIO 0-21).
2. Crea `src/sensors/<Nome>.{h,cpp}` che implementa `ISensor`
   (`name()` / `begin()` / `read(JsonObject&)`), sul modello di
   `Bme280Sensor` (I2C) o `RainGauge` (impulsi). Il sensore scrive da sé i
   propri campi JSON; in caso di guasto NON scrive nulla (campi assenti =
   sensore non disponibile, il ricevitore lo tollera).
3. Eventuale logica pura (conversioni, curve, validazioni) va in
   `src/logic/` SENZA header Arduino, con unit test in
   `test/test_<nome>/test_main.cpp` eseguibili con `pio test -e native`.
4. Eventuali ISR: `IRAM_ATTR`, corpo minimo (contatore + timestamp debounce),
   stato condiviso in RTC RAM se deve sopravvivere al deep sleep.
5. Parametri di calibrazione → `AppConfig` (nuova sezione/chiave in
   `data/config.ini` + parsing + save + printTo) — MAI hardcoded.
6. Registra il sensore in `main.cpp` (configure + `sensors.add(...)`).
7. Aggiorna: `docs/pinout.md` (tabella progetto), `docs/lora-protocol.md`
   (nuovi campi JSON con unità), `CHANGELOG.md`, `include/version.h` e il
   file `docs/progress/progress_step_XX.md` dell'increment.
8. Esegui `./scripts/check.sh` e correggi finché non è verde. Poi fermati
   e chiedi la validazione su hardware prima di taggare la release.
