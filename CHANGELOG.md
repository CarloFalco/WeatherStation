# Changelog

Formato basato su [Keep a Changelog](https://keepachangelog.com/it/1.1.0/);
il progetto segue il [Semantic Versioning](https://semver.org/).

## [2.0.0] – 2026-07-08 — Increment 0: scaffold del progetto

### Added
- Progetto PlatformIO per ESP32-S3 (framework Arduino, LittleFS,
  partizioni 4MB OTA-ready con doppio slot applicativo).
- `include/config.h`: pinout completo (LoRa SX1276, sensori I2C, reed switch).
- `include/version.h`: versionamento SemVer del firmware.
- `src/main.cpp`: boot banner con versione e causa di wake-up, heartbeat LED.
- Architettura documentata in `docs/architecture.md` (moduli, interfacce
  `ISensor`/`ITelemetryLink`, roadmap increment).
- CI GitHub Actions: build e release automatica con binario su push di tag.
- README, CHANGELOG, `.gitignore`, log di avanzamento `progress_step_00.md`.

### Notes
- La serie V2 riparte da `v2.0.0`; i tag `v0.x` appartengono al vecchio
  prototipo, rimosso con il commit di fresh start.
