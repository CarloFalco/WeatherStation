/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "WeatherStation V2", "index.html", [
    [ "Hardware", "index.html#autotoc_md1", null ],
    [ "Requisiti software", "index.html#autotoc_md2", null ],
    [ "Compilazione e flash", "index.html#autotoc_md3", null ],
    [ "Versioning e release", "index.html#autotoc_md4", null ],
    [ "Changelog", "md_CHANGELOG.html", [
      [ "[3.0.0-alpha.2] – 2026-07-16 — OTA stage 1: fix dal primo test su firmware reale", "md_CHANGELOG.html#autotoc_md6", [
        [ "Fixed", "md_CHANGELOG.html#autotoc_md7", null ],
        [ "Added", "md_CHANGELOG.html#autotoc_md8", null ]
      ] ],
      [ "[3.0.0-alpha.1] – 2026-07-15 — OTA stage 1: trasferimento a chunk via LoRa", "md_CHANGELOG.html#autotoc_md9", [
        [ "Added", "md_CHANGELOG.html#autotoc_md10", null ],
        [ "Roadmap", "md_CHANGELOG.html#autotoc_md11", null ]
      ] ],
      [ "[2.9.0] – 2026-07-15 — Increment 9: ottimizzazione consumi e stima autonomia", "md_CHANGELOG.html#autotoc_md12", [
        [ "Added", "md_CHANGELOG.html#autotoc_md13", null ],
        [ "Changed", "md_CHANGELOG.html#autotoc_md14", null ]
      ] ],
      [ "[2.8.0] – 2026-07-09 — Increment 8: protocollo ACK e consegna affidabile", "md_CHANGELOG.html#autotoc_md15", [
        [ "Tooling (step 09, in release con questa versione)", "md_CHANGELOG.html#autotoc_md16", null ],
        [ "Added", "md_CHANGELOG.html#autotoc_md17", null ],
        [ "Changed", "md_CHANGELOG.html#autotoc_md18", null ]
      ] ],
      [ "[2.7.0] – 2026-07-09 — Increment 7: trasmissione LoRa", "md_CHANGELOG.html#autotoc_md19", [
        [ "Added", "md_CHANGELOG.html#autotoc_md20", null ],
        [ "Fixed (durante la validazione dell'Increment 6)", "md_CHANGELOG.html#autotoc_md21", null ]
      ] ],
      [ "[2.6.0] – 2026-07-09 — Increment 6: monitor energia INA3221", "md_CHANGELOG.html#autotoc_md22", [
        [ "Added", "md_CHANGELOG.html#autotoc_md23", null ],
        [ "Changed", "md_CHANGELOG.html#autotoc_md24", null ]
      ] ],
      [ "[2.5.0] – 2026-07-09 — Increment 5: anemometro e banderuola", "md_CHANGELOG.html#autotoc_md25", [
        [ "Added", "md_CHANGELOG.html#autotoc_md26", null ]
      ] ],
      [ "[2.4.0] – 2026-07-09 — Increment 4: pluviometro e wake-on-rain", "md_CHANGELOG.html#autotoc_md27", [
        [ "Added", "md_CHANGELOG.html#autotoc_md28", null ],
        [ "Fixed", "md_CHANGELOG.html#autotoc_md29", null ]
      ] ],
      [ "[2.3.0] – 2026-07-08 — Increment 3: interfaccia sensori e BME280", "md_CHANGELOG.html#autotoc_md30", [
        [ "Added", "md_CHANGELOG.html#autotoc_md31", null ]
      ] ],
      [ "[2.2.0] – 2026-07-08 — Increment 2: PowerManager e deep sleep", "md_CHANGELOG.html#autotoc_md32", [
        [ "Added", "md_CHANGELOG.html#autotoc_md33", null ]
      ] ],
      [ "[2.1.0] – 2026-07-08 — Increment 1: configurazione runtime e protocollo LoRa", "md_CHANGELOG.html#autotoc_md34", [
        [ "Added", "md_CHANGELOG.html#autotoc_md35", null ],
        [ "Changed", "md_CHANGELOG.html#autotoc_md36", null ]
      ] ],
      [ "[2.0.0] – 2026-07-08 — Increment 0: scaffold del progetto", "md_CHANGELOG.html#autotoc_md37", [
        [ "Added", "md_CHANGELOG.html#autotoc_md38", null ],
        [ "Notes", "md_CHANGELOG.html#autotoc_md39", null ]
      ] ]
    ] ],
    [ "WeatherStation V2 – Architettura software", "md_docs_2architecture.html", [
      [ "Visione d'insieme", "md_docs_2architecture.html#autotoc_md41", null ],
      [ "Principi guida", "md_docs_2architecture.html#autotoc_md42", null ],
      [ "Struttura dei moduli", "md_docs_2architecture.html#autotoc_md43", [
        [ "Perché le interfacce <tt>ISensor</tt> e <tt>ITelemetryLink</tt>", "md_docs_2architecture.html#autotoc_md44", null ]
      ] ],
      [ "Gestione energia", "md_docs_2architecture.html#autotoc_md45", null ],
      [ "Formato dati (bozza, da consolidare nell'increment LoRa)", "md_docs_2architecture.html#autotoc_md46", null ],
      [ "Roadmap degli increment", "md_docs_2architecture.html#autotoc_md47", null ]
    ] ],
    [ "WeatherStation V2 – Protocollo LoRa (v1.1)", "md_docs_2lora-protocol.html", [
      [ "Parametri radio", "md_docs_2lora-protocol.html#autotoc_md49", null ],
      [ "Messaggio di telemetria (stazione → base)", "md_docs_2lora-protocol.html#autotoc_md50", null ],
      [ "ACK (base → stazione)", "md_docs_2lora-protocol.html#autotoc_md51", null ],
      [ "Trasferimento OTA (base → stazione) — stage 1 in fw 3.0.0-alpha.1", "md_docs_2lora-protocol.html#autotoc_md52", [
        [ "1. Offerta (campo <tt>ota</tt> nell'ACK)", "md_docs_2lora-protocol.html#autotoc_md53", null ],
        [ "2. Richiesta chunk (stazione → base, JSON)", "md_docs_2lora-protocol.html#autotoc_md54", null ],
        [ "3. Chunk (base → stazione, <strong>binario</strong>, non JSON)", "md_docs_2lora-protocol.html#autotoc_md55", null ],
        [ "4. Esito (stazione → base)", "md_docs_2lora-protocol.html#autotoc_md56", null ],
        [ "5. Ripresa del trasferimento", "md_docs_2lora-protocol.html#autotoc_md57", null ],
        [ "Timing e ritrasmissioni (lato stazione, fw ≥ 3.0.0-alpha.2)", "md_docs_2lora-protocol.html#autotoc_md58", null ],
        [ "Roadmap stage successivi", "md_docs_2lora-protocol.html#autotoc_md59", null ]
      ] ]
    ] ],
    [ "Namespaces", "namespaces.html", [
      [ "Namespace List", "namespaces.html", "namespaces_dup" ],
      [ "Namespace Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", null ],
        [ "Functions", "namespacemembers_func.html", null ]
      ] ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", null ],
        [ "Functions", "functions_func.html", null ],
        [ "Variables", "functions_vars.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Functions", "globals_func.html", null ],
        [ "Variables", "globals_vars.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"Anemometer_8cpp.html",
"main_8cpp.html#a30f8dc961c6ffc04fdc55d8fbae90533"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';