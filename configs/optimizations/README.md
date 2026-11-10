# Historische PSO-Einstellungen (nur Herkunftsnachweis)

`ltv_pso.json` und `nmpc_pso.json` sind die V10/V11-Originale der Offline-PSO-Einstellungen. Sie werden **nicht** geladen. Die gültige Konfiguration liegt in `configs/optimization/` (`pso.yaml`, `pso_extended.yaml`, `*_tuning*.yaml`).

Der Ordner bleibt aus zwei Gründen erhalten:

- `pso.yaml` nennt ihn als `provenance.origin`.
- `tests/unit/test_configuration_defaults_consistency.py` prüft, dass `pso.yaml` diese Originalwerte unverändert reproduziert.

Nicht bearbeiten.
