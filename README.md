# Automatisierte_Bewässerungsanlage
Automatisierte Gartenbewässerungsanlage mit Arduino GIGA R1 WiFi,  Sensor und IoT Cloud Verknüpfung. Automatisch sowie manuell steuerbar

## Funktionsbeschreibung
Das entwickelte Programm realisiert ein stabiles, automatisiertes Bewässerungssystem mit einem Arduino GIGA R1 WiFi. 
Es überwacht die Bodenfeuchtigkeit in sechs Zonen, steuert jeweils eine Pumpe zur Bewässerung und kontrolliert mithilfe eines Ultraschall-Entfernungssensor den 
Wasserstand im Regenfass. Zusätzlich wird das System mit der Arduino IoT Cloud verbunden, um Messwerte und Zustände online zu überwachen und manuell zu steuern.

Das System bietet folgende Kernfunktionen:

-Automatische und manuelle Steuerung von sechs Bewässerungszonen

-Messung von Bodenfeuchtigkeit

-Steuerung über die Arduino IoT Cloud

## Programmstrukur
Das Programm folgt der typischen Arduino-Struktur:

•	setup(): Initialisierung von Hardware, Schwellwertberechnung (Feuchtigkeitsgrenze) und Aufbau der Cloud-Verbindung.

•	loop(): Zyklische Programmschleife, welche Sensorwerte einliest, Pumpen steuert, Cloud-Daten aktualisiert und Eingaben aus der Cloud verarbeitet.

Die Programmlogik basiert auf verschiedenen Zeitintervallen, um Messungen und Steuerungen abgestuft auszuführen.


## Hardware
-1x Arduino GIGA R1 WiFi

-6x Kapazitive Feuchtigkeitssensoren

-6x Wasserpumpen 12V DC + Relais

-1x Ultraschall-Entfernungssensor (Wasserstand)

-1x Wasserventil + Relais

-1x Regenfass

## Pin-Belegung

Komponenten und Pins

-Sensoren (A0, A3, A2, A1, A4, A6)   

-Pumpen (26, 5, 7, 40, 30, 32)

-Ultraschall-Entfernungssensor (3 (Trig), 4 (ECHO))

-Wasserventil (36)

## Konfiguration

**Die Feuchtigkeitssensorgrenzen der sechs Zonen können im Code direkt angepasst werden:**


const int sensorPins[6] = {A0, A3, A2, A1, A4, A6}; // Zeile 9 Heidelbeere, Traubenbaum, Baum, Beet1, Beet2, Johannisbeere.

const int thresholdOn[6] = {342, 506, 417, 445, 445, 340}; // Zeile 16 Heidelbeere, Traubenbaum, Baum, Beet1, Beet2, Johannisbeere 





**WLAN-Zugangsdaten in der Datei Sketch Secrets hinterlegen**

#define SECRET_SSID "DeinWLANName"

#define SECRET_PASS "DeinPasswort"

Diese Datei wird automatisch durch #include "arduino_secrets.h" im Hauptprogramm eingebunden

## Installation

**Arduino Cloud Thing erstellen**

**Code herunterladen**

**WLAN konfigurieren**


## Arduino Cloud Variablen

**Sensoren (Read Only Funktion):**
- water_level
- moisture_zone_0 - moisture_zone_5
- pump_0_status - pump_5_status
- water_pump_status, system_ready

**Steuerung (Read & Write Funktion):**
- automatic_mode
- manual_pump_0 - manual_pump_5
- manual_water_pump
