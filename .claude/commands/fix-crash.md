Analizza questo crash/malfunzionamento della stazione meteo: $ARGUMENTS

Processo obbligatorio:

1. Se c'è un backtrace non decodificato (solo indirizzi hex), chiedi l'output
   del monitor con `pio device monitor` (il filtro `esp32_exception_decoder`
   è già attivo in platformio.ini) prima di procedere.
2. Consulta `docs/troubleshooting.md`: il sintomo è già noto? Molti problemi
   di questo progetto sono già stati diagnosticati lì (INA3221, LoRa,
   deep sleep/USB, I2C...).
3. Ricorda il contesto specifico di questo firmware:
   - ogni ciclo riparte da `setup()` dopo il deep sleep (loop() mai eseguito);
   - lo stato persistente vive in RTC RAM (`g_rtcState`) e si azzera solo
     al power-on;
   - il quick path pioggia (wake EXT0) salta seriale/sensori/radio;
   - la config runtime viene da `/config.ini` su LittleFS: un valore strano
     può venire dal filesystem non aggiornato (`uploadfs`), non dal codice.
4. Formula le 2-3 cause più probabili in ordine, ciascuna con: file/riga
   sospetta, meccanismo, e quale log o test la confermerebbe.
5. NON modificare codice finché l'ipotesi non è confermata (dai log o
   dall'utente).
6. Dopo il fix: `./scripts/check.sh` verde + nuova voce in
   `docs/troubleshooting.md` (append-only) + commit con messaggio `fix:`.
