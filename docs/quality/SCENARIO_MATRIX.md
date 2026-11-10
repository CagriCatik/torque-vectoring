# Verpflichtende Referenz- und Stressszenarien

Stand: 25.10.2026. Alle Werte sind Testspezifikationen, nicht Ergebnisbehauptungen. Simulationsgeschwindigkeit in m/s, Reifenkräfte in N, Winkel intern in rad. Das definierte Gültigkeitsfenster der jeweiligen Modellversion muss bei jeder Ausführung mit gespeichert werden.

| ID | Manöver | Eingang / Vorgehen | Verbindlicher Check | Kriterien |
|---|---|---|---|---|
| S01 | Statische Radlast | vx=vy=r=0, ebene Fahrbahn | Sum(Fz)=m*g; Schwerpunkt-Momentenbilanz, keine negative Normalkraft | Q01 Q05 Q08 |
| S02 | Geradeaus / Symmetrie | vx=20, Lenkwinkel=0, vier gleiche Radmomente | Sum(Fy)=0, Giermoment=0 bis auf Rundungsfehler, symmetrische Radlasten | Q04 Q05 Q08 |
| S03 | Freilauf | vx=20, alle T_drive=T_brake=0, Widerstände aktiv | Geschwindigkeit sinkt, Energiebilanz mit Widerständen, kein künstliches Motor-Giermoment | Q05 Q06 Q08 |
| S04 | Symmetrische Beschleunigung | vx=15, vier gleiche positive Drehmomente | Kappa steigt aus Raddrehzahl konsistent; Fx positiv; symmetrisch kein TV-Giermoment | Q04 Q06 Q08 |
| S05 | Asymmetrische Raddrehmomente | vx=15, T_rechts != T_links | Giermoment-Signum stimmt mit Spurweite und Kraftdifferenz; Radleistungsbilanz | Q05 Q06 Q11 |
| S06 | Symmetrisches Bremsen | vx=20, vier gleiche Bremsmomente | Fx bremsend, Radschlupf negativ, keine ungewollte Gierrate, Last nach vorn | Q04 Q05 Q06 |
| S07 | Linearbereich Kreisfahrt | 10/20/30 m/s, kleine Lenkung, |ay|<=0.2*g | Stationäre Gierrate und ay innerhalb 5 % des unabhängigen Einspurmodells, ohne Reifen-Saturation | Q02 Q05 Q09 |
| S08 | Lenksprung transient | vx=15 und 25 m/s, Lenkwinkel-Zeitprofil | Kennwerte Peak, Responsezeit, Gierrate, ay und Schrittweitenkonvergenz | Q05 Q07 Q09 Q10 |
| S09 | Combined-Slip Raster | Fz mehrfach, alpha und kappa mit positiven/negativen Werten | Pure-Slip-Limits; force cap; Vergleich gegen unabhängiges Reifen-Kennfeld mit Hold-out | Q02 Q03 Q08 |
| S10 | Low-mu / Split-mu | mu links ungleich mu rechts und S08/S06 | Kraftgrenzen pro Rad; TV-Allocator-Saturation; keine instabilen Zustände | Q03 Q06 Q07 Q11 |
| S11 | Niedriggeschwindigkeit | vx fällt von 3 m/s auf 0; optional Rückwärts | Definierte 0-m/s-Regel oder expliziter Outside-Envelope-Fehler; keine NaN/Inf | Q04 Q07 Q08 |
| S12 | Grenzbereich Lenken | Mehrfach dt=0.02/0.01/0.002/0.001 s; Start vx=18, Lenkwinkel bis 10 Grad | In physikalisch begründetem Arbeitsfenster dt-Konvergenz der Hauptzustände NRMSE<=2 %, sonst transparenter Unsupported-Status | Q03 Q05 Q07 |
| S13 | TV gegen Baseline | identisches Manöver mit/ohne TV bei gleichen Randbedingungen | Auf nachweislich erreichbarem Sollprofil > 20 % kleinere Gierraten-Tracking-NRMSE ohne Constraint-Verletzung | Q06 Q11 |
| S14 | Messvalidierung Kreisfahrt | Fahrversuch oder unabhängiger geeigneter Fahrzeugmessdatensatz | Stationärer Vergleich bei dokumentierten Fahrzeug-/Reifen-/Straßenbedingungen | Q01 Q05 Q10 |
| S15 | Messvalidierung Lenksprung | unabhängiger transienter Fahrzeugmessdatensatz | Zeitreihen- und Peakvergleich, Zeitversatz und Unsicherheit | Q01 Q07 Q10 |
| S16 | Reifen-/GUI-Vektorskalierung | bekannte Fx/Fy-Vektoren je Rad und Lenkwinkel | Pfeilwinkel, Pfeillängen und Kammkreisnutzung passen zu exportierten numerischen Werten | Q03 Q12 |

## Ausführungsprotokoll

Zu jedem Szenario mindestens: Szenario-ID, JSON-Konfiguration, Modellrevision, Reifendatensatzrevision, integrator/dt, Logdatei, Referenzdataset/Quelle, Ist/Soll-Vergleich, PASS/FAIL, validiertes Betriebsfenster und Review-Signatur.

Im bestehenden V2-Projekt sind diese Szenarien *nicht* alle implementiert. Insbesondere die S04/S05/S06/S13 mit tatsächlichen Raddrehmomenten erfordern die neue Radrotationsdynamik. S14/S15 erfordern unabhängige Fahrzeugmessdaten.

## V4 - Zusätzliche Manöverprüfungen (nicht normzertifiziert)

| ID | Manöver | Geschwindigkeiten | Nachweis | Erwartetes Prüfergebnis |
|---|---|---|---|---|
| M01 | Kreisfahrt, Lenkung offen | 15 m/s, R_soll=90 m | `exports_maneuvers/circle_open/result.json` | Nachweis Gierratenstationarität und gemessener Radius, kein Radius-Following-Gate |
| M02 | Kreisfahrt, Bahnführung geschlossen | 12, 15, 20 m/s, R_soll=90 m | `validation/maneuver_verification_v4.json` | Stationäre Radiusabweichung <1 m und keine Reifenlimitverletzung |
| M03 | Doppelter Spurwechsel | 12, 17, 21 m/s, Offset 3,5 m, Übergänge 42 m | `validation/maneuver_verification_v4.json` | Rückkehr in Ausgangsspur, max. Sollbahnfehler <1 m, Lenkkraft innerhalb Grenzen |
| M04 | Lenksprung | 17 m/s, 1,6 deg mit 0,4 s Übergang | `exports_maneuvers/step_steer/result.json` | Endliche transiente Gierratenreaktion |
| M05 | Sinuslenken | 17 m/s, 1,2 deg, 0,45 Hz | `exports_maneuvers/sine_steer/result.json` | Endliche transiente Gierratenreaktion |
| M06 | Schrittweitenverfeinerung | M02 (15 m/s), M03 (17 m/s), 40 ms vs. 15 ms | `validation/maneuver_verification_v4.json` | Max. normierter Zustands-RMSE <2 % |

**Wichtig:** Bei ISO 4138:2021 wird stationäres Kreisfahrverhalten über festgelegte Open-Loop-Verfahren bestimmt; das geschlossene Kreisfolgemanöver M02 ist bewusst ein zusätzliches Projektmanöver. ISO 3888-1:2018 definiert eine Teststrecke für einen schweren doppelten Spurwechsel; unser M03 verwendet keine nachgewiesene ISO-Strecken- und Fahrzeugabmessungsprüfung. Daher darf kein Ergebnis aus M01-M06 als ISO-Zertifizierung oder experimentelle Validierung ausgegeben werden. Die projektspezifischen PASS-Prüfungen fließen nicht automatisch als 8/10 in Q10 ein.
