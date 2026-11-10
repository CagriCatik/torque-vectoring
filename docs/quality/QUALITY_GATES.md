# Qualitäts- und Freigabekriterien — 4-Rad-Fahrdynamik / Pacejka / Torque Vectoring

**Version:** 1.0 (Kriterienbaseline, 25.10.2026)  
**Ziel:** Jede der 13 Dimensionen **mindestens 8/10**, ohne kritischen Fehler.  
**Geltung:** Full-scale Pkw mit vier einzelnen Reifen, nicht RC-Modell.  
**Status:** Dies ist eine **Soll-Spezifikation**. Durch das Festlegen der Kriterien wird die bestehende Simulation nicht validiert.

## Bewertungssystem

Jede Dimension hat genau fünf überprüfbare Teilanforderungen. Jede erhält:

- **0 Punkte:** nicht implementiert, nicht nachgewiesen oder Test fehlgeschlagen.
- **1 Punkt:** teilweise erfüllt, aber nicht vollständig gegen vorab definiertes Akzeptanzmaß nachgewiesen.
- **2 Punkte:** vollständig erfüllt, reproduzierbar dokumentiert und automatisch oder durch nachvollziehbare Referenzmessung nachgewiesen.

`SCORE(Qxx) = Summe der fünf Teilpunkte`, Wertebereich 0–10.  
**PASS** nur, wenn `SCORE(Qxx) >= 8` **UND** alle mit `KRITISCH` markierten Teilanforderungen **2 Punkte** erhalten.  
**GLOBAL PASS** nur, wenn alle 13 Dimensionen PASS sind; Durchschnitt, gewichteter Mittelwert oder ein Gesamtscore **ersetzen keinen** Einzelpass.

Ein fehlender Bericht / Datensatz / Quellenbeleg zählt als 0, nicht als "wahrscheinlich bestanden". Bei nicht durchgeführtem Audit wird der Score als `NOT_EVALUATED` angezeigt (nicht als faktische Implementierungsbewertung); das Gate gilt bis zum Nachweis als nicht bestanden. Scores sind niemals aus den gleichen Testdaten abzuleiten, auf die das Modell angepasst wurde.

### Unbedingt vor Beginn der Bewertung einfrieren

1. Fahrzeug- und Reifenkonfiguration, Referenzdatenquelle und Gültigkeitsbereich.
2. Trainings-/Identifikations-, Kalibrier- und Hold-out-Datensatztrennung.
3. Szenarien für Geradeauslauf, Antrieb, Bremsen, Kreisfahrt, Lenksprung, Slalom, asymmetrische Radmomente, niedrige Geschwindigkeit und niedrigen Reibwert.
4. Zulässige Fehlermaße, Normierung, Zeitschritte und Längeneinheiten.
5. Berichtsschema und Prüfer/Review-Datum.

**Wichtig:** Die nachfolgend genannten Fehlergrenzen (z. B. 10 % NRMSE, 2 % numerische Konvergenz, 20 % TV-Verbesserung) sind **projektspezifische vorgeschlagene Engineering-Akzeptanzwerte**, **keine** von ISO vorgegebenen universellen Grenzwerte. Grenzwerte müssen im spezifizierten Betriebsbereich und mit passender Referenzmessung nachgewiesen werden.

**NRMSE-Definition:** `RMSE(y_sim-y_ref)/scale_ref`, wobei `scale_ref` pro Messgröße **vor** dem Versuch festgelegt ist, beispielsweise dem 95-%-Quantil des Betrags der Referenzgröße plus fest definierter Untergrenze für Werte nahe null. Beide Signale müssen auf einer gemeinsamen Zeitbasis und mit definierten Verzögerungsregeln verglichen werden.

## Einzelkriterien
### Q01 — Parameter und Geltungsbereich

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q01.1 | Fahrzeugmasse, Trägheitsmomente, Radstand, Spurweiten und Schwerpunktlage sind mit Quelle, Einheit und Unsicherheit dokumentiert. | JA | 0–2 |
| Q01.2 | Reifenabmessung, Nenndruck, nomineller Reibwert, Referenznormalkraft und Herkunft aller Reifenparameter sind dokumentiert. | JA | 0–2 |
| Q01.3 | Zulässige Bereiche für v, alpha, kappa, Fz, mu und Temperatur beziehungsweise explizit nicht modellierte Temperatur sind festgelegt. | Nein | 0–2 |
| Q01.4 | Eingaben werden gegen Einheiten, Vorzeichen, Plausibilitätsbereiche und NaN/Inf validiert. | Nein | 0–2 |
| Q01.5 | Jeder Lauf protokolliert Parameterdatei, Version, Git-Revision und Gültigkeitsbereich. | Nein | 0–2 |

### Q02 — Pacejka Pure Slip

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q02.1 | Fx(kappa,Fz) und Fy(alpha,Fz) werden mit unabhängig erfassten Mess- oder belastbaren Benchmarkdaten identifiziert; Trainings- und Testdaten sind getrennt. | JA | 0–2 |
| Q02.2 | Normierter RMSE im Hold-out fuer Fx und Fy jeweils <= 10 % pro dokumentiertem Betriebsbereich. | JA | 0–2 |
| Q02.3 | Abweichung der Peak-Kraft jeweils <= 10 %; Initialsteifigkeiten dFx/dkappa und dFy/dalpha jeweils <= 15 %. | Nein | 0–2 |
| Q02.4 | Mindestens drei Fz-Stuetzstellen und symmetrische/antrieb-bremsende bzw. positive/negative Schlupfbereiche werden getestet. | Nein | 0–2 |
| Q02.5 | Kurven fuer Nullschlupf, kleine Winkel, Peak und nach dem Peak sind stetig und ohne unerklaerte Kraftspruenge. | Nein | 0–2 |

### Q03 — Combined Slip und Kammscher Kreis

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q03.1 | Grenzkreis/optional Reibellipse mit Fz- und mu-Abhaengigkeit ist als Grenzmodell dokumentiert; Visualisierung zeigt den Ausnutzungsgrad je Rad. | Nein | 0–2 |
| Q03.2 | Kombinierte Fx/Fy-Kennfelder werden gegen unabhängige Combined-Slip-Referenzdaten verglichen, Hold-out-NRMSE je Komponente <= 10 %. | JA | 0–2 |
| Q03.3 | Pure-Slip-Grenzfaelle alpha=0 und kappa=0 reproduzieren die entsprechenden Pure-Slip-Kurven. | JA | 0–2 |
| Q03.4 | Keine Ueberschreitung der definierten Kraftgrenze > 1e-6 relativ, fuer alle gueltigen Testfaelle. | Nein | 0–2 |
| Q03.5 | Beim Uebergang zwischen Antrieb, Bremsung und Querfuehrung keine Spruenge oder unphysikalischen Vorzeichenwechsel. | Nein | 0–2 |

### Q04 — Radkinematik und Schlupf

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q04.1 | Alle vier Radpositionen und lokale Geschwindigkeiten werden aus vx, vy und Gierrate mit unabhaengiger Referenzrechnung geprueft. | JA | 0–2 |
| Q04.2 | Rad- und Fahrzeugkoordinaten sind dokumentiert; Ruecktransformation erhaelt Norm und Vorzeichen bis 1e-10 numerisch. | Nein | 0–2 |
| Q04.3 | Schraeglaufwinkel alpha mit konsistenter Vorzeichenkonvention; 4-Rad-/Ackermann-Lenkung oder klar dokumentierte Parallel-Lenkungsannahme. | Nein | 0–2 |
| Q04.4 | Kappa wird aus Raddrehzahl und Laengsgeschwindigkeit berechnet; Sonderfall Stillstand, Rollen und Anfahren sind definiert. | JA | 0–2 |
| Q04.5 | Bei Fahrt rueckwaerts und nahe 0 m/s entstehen keine NaN/Inf und kein unerkannter Modellwechsel; ungueltige Bereiche werden explizit abgewiesen. | Nein | 0–2 |

### Q05 — Fahrzeugdynamik und Radlast

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q05.1 | Newton-Euler-Bilanzen fuer Laengs-, Quer- und Gierdynamik stimmen gegen unabhängige Handrechnung <= 1e-6 relativ. | JA | 0–2 |
| Q05.2 | Statische Radlasten summieren sich auf m*g (ebene Strasse ohne Zusatzlast) und stimmen mit Schwerpunkt-Momentenbilanz. | Nein | 0–2 |
| Q05.3 | Laengs- und Querradlastverlagerung haben korrekte Vorzeichen und reproduzierbare Referenzfaelle. | Nein | 0–2 |
| Q05.4 | Rollsteifigkeitsverteilung, Nick-/Wanknaeherung und Radabheben sind entweder korrekt modelliert oder klar als Modellgrenze gekennzeichnet. | JA | 0–2 |
| Q05.5 | Grenzfaelle Geradeausfahrt, Freilauf, konstante Kreisfahrt und Lastwechsel liefern reproduzierbare, physikalisch sinnvolle Ergebnisse. | Nein | 0–2 |

### Q06 — Radrotation und Momentenantrieb

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q06.1 | Pro Rad gilt Iw*omega_dot = T_drive - T_brake - R_eff*Fx mit dokumentierten Vorzeichen und Einheiten. | JA | 0–2 |
| Q06.2 | Vier Radgeschwindigkeiten omega_i sind echte dynamische Zustaende; Schlupf wird nicht als primaere Stellgroesse vorgegeben. | JA | 0–2 |
| Q06.3 | Antriebs- und Bremsmomentgrenzen, Verzoegerung und negative/positive Leistung sind korrekt erfasst. | Nein | 0–2 |
| Q06.4 | Drehmoment-Sprung, Schlupfaufbau und Radblockieren sind mit unabhaengigen Grenzfaellen getestet. | Nein | 0–2 |
| Q06.5 | Leistungs-/Energiebilanz von Antriebsstrang, Rotation und Fahrzeug einschliesslich modellierter Verluste liegt im Testfenster innerhalb 2 %. | Nein | 0–2 |

### Q07 — Numerische Robustheit

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q07.1 | Zweites numerisches Referenzverfahren oder hochaufgeloeste Integrationsreferenz ist implementiert und automatisiert abrufbar. | Nein | 0–2 |
| Q07.2 | Ueber alle als gueltig deklarierten Normal-, Lastwechsel- und Grenzmanöver NRMSE der Hauptzustaende zwischen dt und dt/10 <= 2 %. | JA | 0–2 |
| Q07.3 | Keine NaN/Inf, explodierenden Zustandswerte oder unerkannten Instabilitaeten in allen Grenz-/Stressszenarien. | JA | 0–2 |
| Q07.4 | Nahe Nullgeschwindigkeit, Radstillstand, Lastabheben und Kraeftesaturation werden als Ereignis robust behandelt oder sauber abgewiesen. | Nein | 0–2 |
| Q07.5 | Zeitstempel, Integrator-Optionen, Toleranzen und dt werden gespeichert; Ergebnisse sind bei gleichen Eingaben reproduzierbar. | Nein | 0–2 |

### Q08 — Physikalische Invarianten

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q08.1 | Kraft- und Momentenbilanzen werden pro Zeitschritt dokumentiert, mit Residuen <= 1e-6 relativ fuer algebraische Bilanzgleichungen. | JA | 0–2 |
| Q08.2 | Keine negativen Fz im gueltigen Kontaktzustand; Radabheben wird mit definierter Kontaktdynamik oder Fail-Fast behandelt. | JA | 0–2 |
| Q08.3 | Reifenkraftgrenzen sowie Reifen-/Rad-Leistungsvorzeichen sind in allen Testfaellen konsistent. | Nein | 0–2 |
| Q08.4 | Kinetische Energie wird bei Freilauf ohne externe Energiequelle nicht spontan erhoeht, unter Beruecksichtigung von Lage-/Verlusttermen. | Nein | 0–2 |
| Q08.5 | Impuls-/Energie-Residualberichte und automatische Alarme existieren je Szenario. | Nein | 0–2 |

### Q09 — Unabhaengige theoretische Benchmarks

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q09.1 | Lineares Einspurmodell ist unabhaengig implementiert oder analytisch nachgerechnet (kein Code-Sharing der Kernformeln). | JA | 0–2 |
| Q09.2 | Im linearen Bereich (z. B. |ay| <= 0.2g) Abweichung stationaerer Gierrate und Querbeschleunigung <= 5 % bei geeignetem Referenzfall. | JA | 0–2 |
| Q09.3 | Kreisfahrt: ay~=v*r und r~=v/R werden innerhalb der dokumentierten stationaeren Annahmen verglichen. | Nein | 0–2 |
| Q09.4 | Statische und dynamische Radlastreferenzen bestehen fuer symmetrische und asymmetrische Manöver. | Nein | 0–2 |
| Q09.5 | Mindestens zwei unabhängige Referenzmanöver mit tabellarischen Ergebnissen, Toleranzen und automatischen Regressionen. | Nein | 0–2 |

### Q10 — Experimentelle Validierung

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q10.1 | Nicht zur Parameteridentifikation genutzte Reifen- und/oder Fahrzeugmessdaten sind mit Herkunft, Messunsicherheit und Konfiguration vorhanden. | JA | 0–2 |
| Q10.2 | Mindestens stationaeres Kreisfahrmanöver plus transienter Lenkeingriff, passende Geschwindigkeit/Beladung und dokumentierte Wiederholungen. | JA | 0–2 |
| Q10.3 | Projektinterne Zielwerte: NRMSE fuer Gierrate und Querbeschleunigung <= 10 % gegen Messdaten im definierten Arbeitsbereich. | Nein | 0–2 |
| Q10.4 | Projektinterne Zielwerte: Peak-Abweichung <= 10 % und Zeitversatz <= 50 ms oder begruendete dynamikgerechte Grenze. | Nein | 0–2 |
| Q10.5 | Keine Parameternachstimmung auf Testdaten, Unsicherheit ausgewiesen; Ergebnis getrennt nach Arbeitsbereichen statt nur Gesamtscore. | Nein | 0–2 |

### Q11 — Torque Vectoring und Regelung

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q11.1 | Giermoment-Sollwert, Zustandsbeobachtung und Regelungsstrategie sind mathematisch spezifiziert; Baseline ohne TV existiert. | JA | 0–2 |
| Q11.2 | Allocator berechnet vier zulässige Antriebs-/Bremsmomente unter Aktuator-, Reib- und Radlastgrenzen. | JA | 0–2 |
| Q11.3 | Bei geeigneten machbaren Referenzmanövern verbessert TV den Gierraten-Tracking-NRMSE gegen Baseline um >= 20 %. | Nein | 0–2 |
| Q11.4 | Bei unmoeglichen Anforderungen, niedrigem mu und Aktuatorsaettigung degradieren die Ergebnisse definiert und stabil. | Nein | 0–2 |
| Q11.5 | Alle Tests nutzen Drehmoment als Eingang; keine direkte, nicht-kausale Schlupf- oder Radkraftvorgabe im Controllerpfad. | Nein | 0–2 |

### Q12 — 2D-Visualisierung und Telemetrie

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q12.1 | Alle vier Raeder mit Fx, Fy, Fres, Fz, alpha, kappa und Reifen-Ausnutzung eindeutig beschriftet. | Nein | 0–2 |
| Q12.2 | Pfeilrichtung, Koordinatentransformation und Pfeillaengenverhaeltnisse gegen bekannte Vektoren <= 1 % Darstellungsfehler. | JA | 0–2 |
| Q12.3 | Kammscher Kreis zeigt echte numerische Kraftgrenzen und nicht nur dekorative Skalierung. | JA | 0–2 |
| Q12.4 | Interaktives Szenario und CSV/SVG/PNG Export stimmen innerhalb der Rundung mit internem Zustand ueberein. | Nein | 0–2 |
| Q12.5 | Legende, Masseinheiten, Skalen und Kennzeichnung DEMO / CALIBRATED / VALIDATED sind erkennbar; GUI-Regressionstest vorhanden. | Nein | 0–2 |

### Q13 — Softwarearchitektur, Tests und Reproduzierbarkeit

| Check | Test / Akzeptanznachweis | Kritisch | Punkte |
|---|---|---|---|
| Q13.1 | Reifenmodell, Chassisdynamik, Regler, Szenarien, Auswertung und GUI haben getrennte Module; Domain importiert keine GUI. | JA | 0–2 |
| Q13.2 | Automatische Tests enthalten Unit-, Integrations-, Invarianten- und Grenztests; 100 % der definierten kritischen Gates laufen in CI. | JA | 0–2 |
| Q13.3 | Core-Line-Coverage >= 90 % plus explizite kritische Branch-Tests; alle Tests bestehen ohne unerwartete Skips. | Nein | 0–2 |
| Q13.4 | Kommandozeilenlauf mit fixer Konfiguration produziert reproduzierbare JSON/CSV-Ergebnisse und maschinenlesbaren Gate-Report. | Nein | 0–2 |
| Q13.5 | README, Parameter-/Quellenverzeichnis, Validierungsgrenzen, Changelog, feste Dependency-Versionen und Tests zum frischen Setup sind vorhanden. | Nein | 0–2 |

## Harte Release-Sperren (zusaetzlich zu den 13 Einzel-Gates)

- **BLOCKER B1 — Messbezug fehlt:** Q02/Q03/Q10 duerfen ohne passende Referenz-/Hold-out-Daten nicht auf >= 8 gesetzt werden. Ein synthetischer Datensatz aus demselben Modell ist keine unabhaengige Validierung.
- **BLOCKER B2 — Numerische Divergenz:** Schlaegt der dt-Konvergenztest in einem als gueltig definierten Arbeitsbereich fehl, ist Q07 nicht PASS und die Freigabe gesperrt. Die bestehende V2 zeigt ca. 15,6 % Abweichung im aggressiven Test; dies muss behoben oder ein physikalisch begruendeter engerer Bereich explizit deklariert werden.
- **BLOCKER B3 — Keine Radmomentdynamik:** Q06/Q11 duerfen nicht auf PASS stehen, wenn direkt `kappa_i` als Reglerkommando gesetzt wird.
- **BLOCKER B4 — Physikalische Verletzung:** Massive Verletzungen der Kraefte-, Radlast-, Drehimpuls- oder Energiebilanz unter gueltigen Modellannahmen sperren Q05/Q08.
- **BLOCKER B5 — Ungepruefte Modellbehauptung:** Ohne Messvalidierung wird das Resultat als `SIMULATOR / UNVALIDATED` ausgewiesen. Es darf nicht als realer Fahrzeugnachweis bezeichnet werden.

### Stand nach dem Audit V2 vom 25.10.2026

Ein automatischer Gate-Audit nach diesem neuen Raster wurde **noch nicht** ausgefuehrt. Die vorliegenden Befunde sind ein **Defizit-Backlog**, keine frei vergebenen neuen Zahlen-Scores:

- Q01/Q02: Reifenparameter sind illustrative Konstanten ohne identifizierten Reifen/Datensatz.
- Q03: Combined Slip benutzt eine kreisfoermige radiale Projektion, kein identifiziertes Combined-Slip-Kennfeld.
- Q04/Q07: `max(2.0, vx_w)`-Regularisierung und ein Grenzmanöver mit ca. 15,6 % dt-Abweichung.
- Q05: Quasi-statische Lastverlagerung, Radabheben durch Clamping statt Kontaktdynamik.
- Q06/Q11: Vier vorgegebene Schlupfwerte statt radmomentgetriebener Rotation, kein TV-Controller.
- Q09/Q10: Keine unabhaengige Einspur-Benchmark-/Fahrzeugmessvalidierung.
- Q12/Q13: 2D-Visualisierung, Export und 19 Basis-Tests bereits vorhanden, aber noch keine Abnahme aller obigen Checks.

**Praezisierung:** Ein Core-Softwaretest kann PASS erreichen, obwohl das Reifenmodell nicht experimentell validiert ist. Das gesamte Projekt ist dann dennoch NICHT FREIGEGEBEN.

## Engineering-Reihenfolge

1. **Definition einfrieren:** Q01, Szenario-/Testmatrix, Mess-/Referenzdatensatz, Einheiten und Modellgrenzen.
2. **Physikalischer Kern:** Q04, Q05, Q08, Q09 als nachvollziehbare Referenzmodelle und Invarianten, bevor neue Komplexitaet eingefuehrt wird.
3. **Reifenkalibrierung:** Q02 und Q03 mit getrennten Identifikations- und Validierungsdaten.
4. **Radmomentphysik:** Q06, dann Q07 mit reproduzierbaren Grenztests (dt/10).
5. **TV-Regler:** Q11 mit Baseline- und Gegenprobe inkl. Saettigungen und niedrigem Reibwert.
6. **Realitaetsabgleich:** Q10 gegen Messungen, erst danach eine fahrzeugbezogene Freigabe.
7. **Querschnitt:** Q12 und Q13 bei jedem Pull Request; alle Gates im CI.

## Strukturvorgabe

`core/tyres`, `core/vehicle`, `core/wheels`, `control/allocator`, `scenarios`, `validation`, `telemetry`, `ui` und `tests` sind getrennt zu halten. Domainmodule duerfen nicht von GUI, Plotting oder Datei-I/O abhaengen. Die Abhaengigkeitsrichtung ist immer UI/Adapter -> Application -> Domain. Fehlerprotokollierung und Numerik-Konfiguration sind explizite Ports.

Jede neue Funktion enthaelt: (a) physikalische Hypothese, (b) Einheiten und Vorzeichen, (c) mindestens einen Referenz- und einen Fehlertest, (d) Messbarkeit in Telemetrie, (e) zugeordnete Q-ID. Ohne Zuordnung keine Akzeptanz.

## Externe fachliche Referenzen

- ISO 19364:2016 — Validierung stationaeres Kreisfahrverhalten: https://www.iso.org/standard/64701.html
- ISO 22140:2021 — Validierung transienten Querfahrverhaltens: https://committee.iso.org/standard/72680.html
- ISO 7401:2011 — Transiente Testverfahren: https://committee.iso.org/standard/54144.html
- Pacejka/Bakker, Magic Formula Tyre Model, Vehicle System Dynamics (1992): https://doi.org/10.1080/00423119208969994

Die ISO-Dokumente liefern den methodischen Rahmen. Eine ISO-konforme Validierung wird mit diesem Dokument **nicht** behauptet.

## Scorecard verwenden

Die Datei `qa/quality_scorecard.csv` enthaelt 65 Teilpruefungen. `points` leer bedeutet **ohne Nachweis**. Werte 0, 1 oder 2 werden nur nach Pruefung samt `evidence_path` eingetragen. `qa/grade.py` erzeugt den Gate-Report und liefert bei Nichtbestehen einen Nicht-Null-Exitcode fuer CI.

### Hinweis zum manuellen Scoring

Alle fuenf Teilpruefungen einer Dimension muessen explizit mit 0, 1 oder 2 bewertet sein. Ein leeres Feld wird als `NOT_EVALUATED` ausgegeben und sperrt das Gate. Fuer 1 oder 2 Punkte ist ein nachvollziehbarer Beleg in `evidence_path` zwingend. Der CLI-Checker prueft die Anwesenheit dieser Angabe, **nicht** automatisch deren fachliche Echtheit; die inhaltliche Freigabe erfordert einen Review.

Die verbindlichen Standardmanöver sind in [`qa/SCENARIO_MATRIX.md`](qa/SCENARIO_MATRIX.md) definiert. Jeder Prüfnachweis wird einer Scenario-ID und einer Check-ID zugeordnet.

## V9 status addendum (25 October 2026)

The project remains **BLOCKED** under the existing 13-gate policy. New V9 MPC uses a consistent 40 ms sampling interval and adds predicted-linear-state / yaw-moment / yaw-moment-slew constraints. It passes 98 software tests, but the small six-candidate offline PSO does not meet Q11, and both 19/21 m/s holdout manoeuvres trigger explicit MPC infeasibility. Q10 has no experimental evidence. See `V9_ENGINEERING_REPORT.md` and `validation_v9/quality_v9.json`. Do not interpret synthetic data as measured physical validation.
