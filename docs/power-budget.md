# Power budget e stima autonomia

Data: 2026-07-15 · fw 2.9.0 · Stato: **stima analitica**, da verificare con
misure reali (procedura in fondo). Requisito: autonomia > 12 mesi.

## Duty cycle (build di produzione, `wake_interval_s = 600`)

| Fase | Durata | Corrente media stimata | Carica |
|------|--------|------------------------|--------|
| Boot + init (80 MHz) | ~0.5 s | ~35 mA | 18 mAs |
| Sensori (BME forced + INA single-shot + AS5600) | ~0.4 s | ~35 mA | 14 mAs |
| Campionamento vento | 5 s | ~30 mA | 150 mAs |
| TX LoRa (SF7, ~160 B, 14 dBm) | ~0.13 s | ~120 mA | 16 mAs |
| Finestra ACK (RX) | ≤ 0.6 s | ~45 mA | 27 mAs |
| **Totale veglia** | **~6.6 s** | — | **~225 mAs ≈ 0.063 mAh/ciclo** |

144 cicli/giorno → **~9 mAh/giorno di veglia**. (La build dev con delay USB
2 s e log attivi sale a ~13 mAh/giorno: usare `station-release` sul campo.)

## Sleep (la parte che decide l'autonomia)

Topologia hardware (dal 2026-07-16): ramo 3.3 V **commutato** da GPIO 18
(NPN → IRF9540, pull-down 10k = spento a riposo) che alimenta **AS5600 +
sonda di umidità**; BME280, INA3221 e SX1276 restano sul ramo sempre acceso.

| Componente | Corrente in deep sleep | mAh/giorno |
|------------|------------------------|------------|
| ESP32-S3 deep sleep + RTC | ~10–15 µA | ~0.3 |
| SX1276 in sleep mode | ~0.2 µA | ~0 |
| BME280 (auto-sleep dopo forced) | ~0.1 µA | ~0 |
| INA3221 (power-down dopo single-shot) | ~2 µA | ~0.05 |
| AS5600 + sonda umidità (ramo commutato **spento**) | **0 µA** | **0** |
| *(prima: AS5600 in LPM3, ramo sempre acceso)* | *~1500 µA* | *~36* |
| *(prima ancora: AS5600 in modalità default)* | *~6500 µA* | *~156* |

⚠️ **Condizione necessaria: i pull-up I2C devono stare sul ramo commutato.**
L'AS5600 è sul ramo commutato ma condivide SDA/SCL con BME280 e INA3221 che
sono sempre alimentati. Se i pull-up restano sul ramo sempre acceso, a rail
spento iniettano corrente nei diodi ESD dell'AS5600 (≈ (3.3−0.6)/4.7k ≈
570 µA per linea, fino a ~1.1 mA) alimentandolo parzialmente e vanificando
buona parte del risparmio. Il firmware fa la sua parte (`Wire.end()` e
SDA/SCL come input prima di dormire), ma i pull-up esterni può spostarli
solo l'hardware.

## Scenari di autonomia (batteria 3000 mAh, senza sole)

| Scenario | Consumo/giorno | Autonomia |
|----------|----------------|-----------|
| fw < 2.9.0 (AS5600 default) | ~165 mAh | ~18 giorni ✗ |
| fw 2.9.0 (AS5600 LPM3, rail fisso) | ~45 mAh | ~66 giorni |
| **fw ≥ 3.0.0-alpha.4 (rail commutato)** | **~9.5 mAh** | **~315 giorni** ✓ |
| ...con pull-up I2C sul ramo sbagliato | ~35 mAh | ~85 giorni |

Col **pannello solare** (capacità osservata ~660 mA in pieno sole) il
requisito > 12 mesi è ora raggiunto con ampio margine anche nelle settimane
invernali più buie.

## Raccomandazioni hardware (in ordine di impatto)

1. **Load switch (P-MOSFET high-side) sul rail dei sensori**, comandato da
   un GPIO (predisposto: `SENSOR_POWER_PIN` = GPIO 18): azzera il
   contributo dell'AS5600 in sleep (36 → ~0.5 mAh/giorno). È il singolo
   intervento che porta l'autonomia da "dipende dal sole" a "> 12 mesi
   anche al buio". Vedi sotto quali dispositivi mettere sul ramo commutato.
2. **DevKit vs deploy**: la devkit ha bridge USB-seriale e LED di potenza
   sempre alimentati (mA persi non conteggiati sopra, dipendono dalla
   revisione). Per l'installazione definitiva: alimentare il rail 3V3
   direttamente da un buck/LDO a bassa quiescente (< 5 µA) o rimuovere i
   componenti superflui.
3. Se serve ridurre ancora la veglia: `sample_window_s = 3` (−90 mAs/ciclo)
   e/o `wake_interval_s = 900`.

## Ottimizzazioni firmware attive (fw 2.9.0)

- CPU a **80 MHz** in veglia (`CPU_FREQ_MHZ`, ~metà della corrente vs 240).
- Build `station-release`: niente `delay(2000)` USB (−2 s/ciclo) e log
  `log_d/log_i` eliminati a compile-time (`CORE_DEBUG_LEVEL=2`).
- AS5600 forzato in **LPM3** a ogni boot (registro CONF volatile).
- SX1276 in sleep mode (~0.2 µA) prima di ogni deep sleep.
- INA3221 in single-shot: power-down automatico tra i cicli.
- Quick path pioggia: wake EXT0 → conteggio → sleep in ~0.5 s senza radio
  né sensori.

## Che cosa mettere sul ramo commutato (GPIO 18)

Domanda aperta dell'hardware. Il criterio è semplice: **commutare ciò che
consuma quando dorme**, non ciò che è comodo raggruppare.

| Dispositivo | Assorbimento a riposo | Vale la pena commutarlo? |
|-------------|----------------------|--------------------------|
| **AS5600** (banderuola) | ~1500 µA (LPM3) | **Sì** — da solo è ~80% del budget di sleep |
| **Sonda umidità terreno** | ~5000 µA quando alimentata | **Sì** — è il consumo singolo più grande |
| Regolatore/boost 5V (se presente) | 0.5–2 mA di quiescente | **Sì**, se esiste: spesso basta pilotarne il pin EN, senza MOSFET |
| INA3221 | ~2 µA (power-down automatico) | No — risparmio nullo |
| BME280 | ~0.1 µA (auto-sleep) | No — risparmio nullo |
| SX1276 | ~0.2 µA (sleep mode) | No — risparmio nullo, e complica l'OTA |

Controindicazioni a staccare INA3221 / BME280 / SX1276:

- **Back-powering**: i pin dell'ESP32 restano alimentati; SPI (SX1276) e I2C
  (con pull-up sul ramo sempre acceso) inietterebbero corrente nei diodi ESD
  dei chip spenti, alimentandoli parzialmente e vanificando lo spegnimento.
  Servirebbe portare a livello basso tutte le linee condivise prima del
  distacco — complessità in cambio di ~2 µA.
- **INA3221**: gli ingressi IN+/IN− restano collegati ai rail vivi di
  pannello e batteria. Il chip è progettato per questo (ingressi fino a 26 V
  indipendenti da VS), quindi non si danneggia, ma non si guadagna nulla.

**Topologia consigliata**: un ramo commutato che porta **AS5600 + sonda di
umidità + i pull-up I2C**, lasciando BME280, INA3221 e SX1276 sul ramo
sempre alimentato. Prima del deep sleep il firmware chiude il bus
(`Wire.end()`) e lascia SDA/SCL come input: senza pull-up sul ramo acceso
non resta alcun percorso di alimentazione fantasma.

**Stato**: adottata (2026-07-16). Realizzazione: GPIO 18 → resistenza di
pull-down 10k + transistor NPN → gate di un **IRF9540** (P-MOSFET high-side)
sul ramo 3.3 V dedicato. Il pull-down garantisce il ramo spento quando il
pin non è pilotato (reset, MCU non programmato).

> ⚠️ **Verifica sull'IRF9540.** Non è un MOSFET logic-level: la Rds(on) è
> specificata a Vgs = −10 V e la soglia arriva fino a −4 V. Commutando il
> ramo 3.3 V si ha Vgs = −3.3 V, cioè funzionamento appena sopra soglia.
> Con il carico in gioco (~6.5 mA) la caduta resta comunque piccola anche
> con qualche ohm di Rds, ma **va misurata la tensione sul ramo commutato a
> valle del MOSFET, sotto carico**: se scende sotto ~3.0 V l'AS5600 (min
> 2.7 V) lavora al limite e conviene un P-MOSFET logic-level (AO3401,
> DMG3415, IRLML6402: soglia ~−0.9 V, stesso schema di pilotaggio).

## Procedura di misura (per validare le stime)

1. **Sleep**: multimetro in serie alla batteria (portata µA/mA), stazione in
   deep sleep, USB staccato. Atteso ~1.5 mA (LPM3); se molto di più,
   cercare il colpevole staccando un sensore alla volta.
2. **Veglia**: canale carico dell'INA3221 (`iload` nel JSON/debug) durante
   il ciclo, oppure multimetro in modalità max-hold.
3. **Bilancio reale**: `ibat` medio su 24 h (il gateway futuro potrà
   integrarlo dai dati di telemetria: positivo di giorno, negativo di notte).
