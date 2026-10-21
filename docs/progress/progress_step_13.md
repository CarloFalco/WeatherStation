# Step 13 – Scheda v1.0: verifica dello schematico

**Data:** 2026-10-21
**Versione:** v3.0.0-alpha.5

Carlo ha stampato e montato la prima scheda (schematico e PCB in `docs/`).

## Attività svolte

- Verifica dello schematico blocco per blocco contro `include/config.h`:
  tutti i pin coincidono, nessuna modifica di pinout nel firmware. Nella
  descrizione testuale il DIO0 del LoRa era indicato come GPIO 7: lo
  schematico (net `GPIO08`) conferma GPIO 8, quindi era un refuso.
- Confermati come corretti: pull-up I2C sul ramo commutato (+3V3SUP),
  polarità del ramo commutato (GPIO 18 HIGH = acceso, flottante = spento),
  sonda umidità alimentata a 3.3 V, tasto reset con pull-up a +3V3.
- **Rilevato** sulla scheda: R8/R9 (pull-up di anemometro e pluviometro)
  collegate a **+5V**. Fuori specifica per l'ESP32-S3 e ~0.25 mA di consumo
  continuo in sleep. Correzione consigliata documentata in `pinout.md`.
- Firmware: guardia sugli ingressi di wake prima del deep sleep (attesa del
  rilascio, esclusione di un ingresso bloccato basso).

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Escludere dal wake un ingresso bloccato, non andare in errore | Il timer sveglia comunque la stazione: si perde al massimo il wake-on-rain per un ciclo, mai la telemetria. Un loop di risvegli invece scaricherebbe la batteria in ore. |
| Attesa del rilascio solo se il pin è basso | Nel caso normale (pin alto) il costo è una lettura digitale: il quick path pioggia resta rapido. |
| R8 → +3V3SUP, R9 → +3V3 (e non entrambe su +3V3) | Il pluviometro deve poter svegliare la stazione, quindi il suo pull-up resta alimentato; l'anemometro no, e con il pull-up sul ramo commutato un reed rimasto chiuso non consuma nulla durante il sonno. |

## Note per la validazione

1. Con la scheda montata: ciclo completo (telemetria + ACK), lettura di
   tutti i sensori, ramo +3V3SUP a 0 V durante il deep sleep (con J1 sul
   MOSFET).
2. Tasto reset: pressione breve = nessun effetto; pressione di 3 s = reset.
3. Pluviometro: una basculata in deep sleep deve essere contata **una sola
   volta**. Tenendo il contatto chiuso a lungo (simulazione di guasto) la
   stazione deve stampare `Rain gauge input stuck low` e continuare a
   dormire col timer invece di risvegliarsi in continuazione.

## TODO

- [ ] Correggere R8/R9 sulla scheda (o nella revisione PCB).
- [ ] Misurare la tensione di +3V3SUP sotto carico (IRF9540 non logic-level).
- [ ] Misurare la corrente di sleep reale (la devkit aggiunge il LED di
      alimentazione e la corrente a riposo del suo regolatore).
- [ ] OTA stage 2: scrittura in partizione, riavvio, rollback.
