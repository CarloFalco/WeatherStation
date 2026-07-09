# Step 06 – Monitor energia INA3221 (PowerMonitor)

**Data:** 2026-07-09
**Versione:** v2.6.0

## Attività svolte

- Validato l'Increment 5 su hardware; pubblicata release `v2.5.0`.
  Nota anemometro: 1 impulso/giro — il fattore `mps_per_hz` resta da
  tarare (dipende dal raggio delle coppette).
- Rielaborata la base di codice fornita dall'utente (`INA3221.{h,cpp}`,
  derivata dalla libreria Beast Devices, MIT) nei nuovi moduli:
  - `Ina3221Driver`: accesso a registri essenziale (config, bus/shunt
    voltage, mask/enable, manufacturer ID per il probe);
  - `PowerMonitor` (ISensor): canali 1 = pannello, 2 = batteria,
    3 = carico → `vbat`, `soc`, `ibat`, `ipan`, `iload`;
  - `vbToSoc()` conservata: curva di scarica LiPo a 21 punti (mV → DOD),
    interpolazione lineare, clamp 0–100.
- Campo `soc` [%] aggiunto al protocollo (`docs/lora-protocol.md`):
  la percentuale batteria viaggia nel JSON verso la base, come richiesto.
- Sezione `[power]` in `config.ini`: `shunt_mohm` (default 10, come nel
  codice originale).

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Conversioni single-shot invece di continuous | In continuous l'INA3221 assorbe ~350 µA fissi, da solo ~3× il budget medio dell'intera stazione: in triggered fa un set di conversioni per ciclo e torna in power-down da solo. |
| Lettura shunt con segno (int16, complemento a due) | L'ADC shunt è bidirezionale: il vecchio codice leggeva unsigned e correggeva a valle con soglie e differenze tra canali (`getAll`); con il segno nativo quei workaround spariscono. |
| `ibat` positiva = carica (inversione del raw) | Convenzione già fissata nel protocollo; con il cablaggio attuale lo shunt legge la scarica come positiva. |
| `vbat < 100 mV ⇒ letture scartate` | Sostituisce il fix `voltage = 1000` del vecchio codice: meglio nessun dato che un dato inventato (i campi assenti sono già gestiti dal protocollo). |
| Curva `vbToSoc` mantenuta identica | È una calibrazione già validata sul campo dall'utente; solo ripulita (tipi, lround, costanti). |
| AVG=16, CT=1.1 ms (~106 ms per set) | Compromesso rumore/durata veglia adeguato per grandezze lente come correnti di carica. |

## Note per la validazione

- JSON atteso ora completo, es.:
  `{"type":"data",...,"vbat":3.98,"soc":87,"ibat":-52,"ipan":120,"iload":35}`
- Verifiche di plausibilità:
  - `vbat` ≈ tensione batteria reale (multimetro);
  - `soc` coerente con la curva (es. 3.8 V ≈ 60%);
  - `iload` positiva (decine di mA con l'ESP sveglio);
  - `ipan` > 0 con pannello illuminato, ~0 al buio;
  - `ibat` positiva in carica (pannello attivo), negativa in scarica —
    se i segni risultano invertiti rispetto alla realtà va tolta
    l'inversione in `PowerMonitor::read()` (dipende dal verso di
    cablaggio degli shunt).
- Senza INA3221 sul bus: warning al boot, JSON senza campi energia.

## TODO per lo step successivo (Increment 7 – LoRa)

- [ ] **Validazione hardware step 6** come da note sopra (segno `ibat`
      in particolare).
- [ ] `ITelemetryLink` + `LoRaLink` (RadioLib, SX1276): init radio con i
      parametri `[lora]`, invio del JSON, gestione errori TX.
- [ ] Preparare il ricevitore di debug lato utente (parametri in
      docs/lora-protocol.md).
