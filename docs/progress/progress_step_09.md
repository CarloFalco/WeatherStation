# Step 09 – Workflow tooling: loop autonomo e repository auto-esplicativo

**Data:** 2026-07-09
**Versione:** (nessun bump: tooling e documentazione, in release con v2.8.0)

Implementate le voci a priorità ALTA di `workflow-esp32-claude-code.md`
(CLAUDE.md e pinout.md già scritti dall'utente; protocolli già coperti da
`docs/lora-protocol.md`).

## Attività svolte

- **§1.2 – env `native`** in platformio.ini (unity, `test_build_src`,
  filtro su `src/logic/`): unit test eseguibili sull'host in secondi,
  senza hardware. Verificato: MinGW g++ 13.2 presente sul PC.
- **`src/logic/`** creata con la prima estrazione di logica pura:
  `battery_soc.{h,cpp}` (`logic::vbatToSoc`, ex `PowerMonitor::vbToSoc`),
  zero header Arduino; `PowerMonitor` ora delega.
- **Test nativi** `test/test_battery_soc/` (6 test: clamp, punti noti
  della curva, interpolazione, monotonicità, range) — tutti verdi.
- **§5.1 – `scripts/check.sh`**: gate unico (test native + build station,
  `--fast` per saltare i test); trova `pio` anche fuori dal PATH (Windows).
- **§3.7 – `docs/troubleshooting.md`**: popolato con gli **11 problemi
  reali** già risolti nel progetto (INA3221 ×4, LoRa freq, I2C default,
  macro BME280, pluviometro ×100, deep sleep/USB, quoting PowerShell).
- **§6.1/6.2 – slash command** `.claude/commands/new-sensor.md` (adattato
  al pattern ISensor del progetto, non al generico "driver") e
  `fix-crash.md` (con i gotcha specifici: quick path, RTC RAM, uploadfs).
- **§1.3** – `monitor_filters` esteso con `time`.
- **CLAUDE.MD riallineato al progetto reale**: il template citava env
  inesistenti (`nodo_esterno`/`gateway`), secrets.h, MQTT, `src/log.h` —
  ora comandi, regole e gotcha corrispondono a questo repository.
- **`docs/pinout.md` corretto**: la tabella di progetto indicava reed su
  GPIO 35/36, ma la fonte di verità (config.h, validata su HW) usa
  **GPIO 19/20**; su N16R8 i GPIO 33–37 sono della PSRAM octal e non sono
  RTC-capable (niente wake-on-rain). Documentato il vincolo: USB nativo
  non disponibile → flash/monitor via connettore UART.

## Decisioni progettuali e motivazioni

| Decisione | Motivazione |
|-----------|-------------|
| Prima estrazione in `logic/` = curva SoC | È l'unica logica pura non banale già esistente; il pattern è tracciato per le prossime (encode payload, filtri sensori). |
| check.sh senza clang-format/pio check | Sono priorità MEDIA della guida; il gate parte minimo e veloce, si estende poi. |
| Slash command `/new-sensor` invece di `/new-driver` | Nel nostro design i "driver" sono le classi ISensor: il comando codifica il processo reale (config.ini, protocollo, progress file, validazione HW). |
| pinout.md allineato a config.h (19/20) e non viceversa | Regola dichiarata nel file stesso ("vince config.h") + cablaggio validato su hardware + 35/36 tecnicamente inutilizzabili su N16R8. |

## TODO per lo step successivo

- [ ] Domanda aperta per Carlo: confermare che i reed restino su GPIO 19/20
      (pinout.md diceva 35/36 — refuso o ricablaggio previsto?).
- [ ] Validazione hardware Increment 8 (ACK) ancora in sospeso → poi tag v2.8.0.
- [ ] Priorità MEDIA della guida quando torneranno utili: `.clang-format`,
      `pio check` in check.sh, `.claude/settings.json`, CI con check.sh.
- [ ] Increment 9 – ottimizzazione consumi e stima autonomia.
