/*
  Arduino GIGA R1 WiFi - Bewässerungssystem für Arduino Cloud
*/


#include "thingProperties.h" //Diese Datei enthält alle Variablen für die Arduino Cloud

// HARDWARE - PINS 
const int sensorPins[6] = {A0, A3, A2, A1, A4, A6};   // Feuchtigkeitssensoren
const int pumpPins[6]   = {26, 5, 7, 40, 30, 32};     // Bewässerungspumpen
const int TRIG_PIN = 3;      // Ultraschall Trigger
const int ECHO_PIN = 4;      // Ultraschall Echo  
const int RELAY_PIN = 36;    // Wasserventil (Wasserpumpe)

// BEWÄSSERUNG - Feuchtigkeitsgrenzen 
const int threshold[6] = {342, 506, 417, 445, 445, 340};  // Einschalt- und Ausschalt-Feuchtigkeitsgrenze

// WASSERSTANDSKONTROLLE - Parameter (in cm)
const float SENSOR_HEIGHT = 107.5;// Der Ultraschall-Entfernungssensor ist 107.5cm hoch.
const float BARREL_HEIGHT = 93.0; //Wassertank ist 93cm hoch.
const float MIN_FILL_LEVEL = 25.0;// Bei weniger als 25cm Wasser = nachfüllen.
const float MAX_FILL_LEVEL = 72.0;// Bei 72cm Wasser = Nachfüllung stoppen.
const int NUM_READINGS = 3;// Anzahl der aufeinanderfolgenden Messungen des Ultraschallsensors. Der Durchschnitt dieser Werte erhöht die Messgenauigkeit.
const float MAX_DISTANCE = 400.0;// Wenn Sensor über 400cm misst, wird diese Messung ignoriert, weil sie als ungültig gilt.

// WiFi-Stabilität 
unsigned long lastCloudUpdate = 0; // Speichert die Zeit (in ms), wann zuletzt Daten an die Cloud gesendet wurden. 0 = noch kein Update erfolgt.
const unsigned long CLOUD_UPDATE_INTERVAL = 3000;// Wie oft Daten an Cloud gesendet werden (Regelmässige Datenübertragung) 3000ms= 3s
bool cloudConnected = false;// Merk sich, ob Verbindung zur Cloud besteht (Verbindungsstatus)
unsigned long lastConnectionCheck = 0;// Wann die Verbindung zuletzt geprüft wurde (Zeitsteuerung für Verbindungscheck)
const unsigned long CONNECTION_CHECK_INTERVAL = 10000;// Wie oft die Verbindung überprüft wird (Stabilitätskontrolle) 10000ms= 10s

// STATUS-VARIABLEN (Hier wird gespeichert, was das System gerade macht)
bool zoneNeedsWater[6] = {false,false,false,false,false,false};//Status jeder Zone: false = genug Wasser, true = muss bewässert werden
bool pumpRunning[6] = {false,false,false,false,false,false};// Welche Pumpen sind aktiv? false = aus, true = läuft
bool waterPumpActive = false;// Status der Nachfüllpumpe (false = aus, true = aktiv)
bool systemReady = true; // Systemstatus: true = bereit, false = gestoppt (z. B. kein Wasser)
float currentWaterLevel = 0.0; // Aktueller Wasserstand in cm (wird später per Ultraschallsensor gemessen)

// Zeitsteuerungen für Messungen (Legt fest, wie oft Sensoren und Wasserstand gemessen werden) 
const unsigned long SENSOR_READ_INTERVAL = 3000; // Alle 3 Sekunden: Feuchtigkeitssensoren auslesen
const unsigned long WATER_MEASURE_INTERVAL = 3000; // Alle 3 Sekunden: Wasserstand im Regenfass messen
unsigned long lastSensorReadTime = 0; // Zeitpunkt der letzten Feuchtigkeitsmessung (0 = noch nie)
unsigned long lastWaterMeasurement = 0; // Zeitpunkt der letzten Wasserstandsmessung (0 = noch nie)

// Namen der Pflanzen für jede Bewässerungszone

// Die folgende Liste zeigt die Bezeichnungen im Code.
// Die Nummerierung entspricht NICHT den realen Zonen laut Installationsplan.
// plantNames [0] = Heidelbeere
// plantNames [1] = Traubenbaum
// plantNames [2] = Baum
// plantNames [3] = Beet 1
// plantNames [4] = Beet 2
// plantNames [5] = Johannisbeeren
// Hinweis: Diese Zuordnung dient nur der Software-Logik.
// Zweck: Die Namen werden im seriellen Monitor und in der Cloud angezeigt.
const String plantNames[6] = {
  "Heidelbeere","Traubenbaum","Baum","Beet 1","Beet 2","Johannisbeeren"
};

// MANUELLER MODUS
// Diese Variable bestimmt, ob das System im manuellen oder automatischen Modus läuft.
// false = Automatik (Standard)
// true  = Manuell (Pumpen können über die Cloud/App direkt geschaltet werden)
bool manualMode = false;

// SETUP-BEREICH 
// Wird nur einmal beim Start des Arduino ausgeführt.
// Hier wird alles vorbereitet: Hardware, Cloud und erste Messung.
void setup() {
  Serial.begin(115200); // Serielle Kommunikation starten (115200 Baud= Übertragungsgeschwindigkeit)
  delay(1000); // kurze Pause, damit alles stabil startet 1000ms

  // Hardware initialisieren
  initializeHardware(); // Richtet Sensoren, Pumpen und Relais ein
  
  // Cloud-Verbindung
  Serial.println("Starte Cloud-Verbindung..."); // Meldung im Monitor, dass jetzt die Verbindung zur Arduino Cloud gestartet wird
  initProperties(); //Lädt Cloud-Variablen (moisture_zone_0, pump_0_status usw,)
  ArduinoCloud.begin(ArduinoIoTPreferredConnection); // Startet WLAN + Cloud-Verbindung
  
  // Kurzes Warten, aber nicht blockieren
  unsigned long startTime = millis(); //Startzeit merken (Zeitpunkt des Schleifenbeginns)
  while (!ArduinoCloud.connected() && millis() - startTime < 15000) { // Schleife läuft max. 15 s
    ArduinoCloud.update(); // Verbindung versuchen aufzubauen
    delay(200); //kurze Pause, um System nicht zu überlasten
    Serial.print("."); // Fortschrittspunkt im Monitor anzeigen
  }
  
  cloudConnected = ArduinoCloud.connected(); // Überprüfen, ob die Verbindung zur Cloud erfolgreich hergestellt wurde. true= verbunden, false= nicht verbunden
  if (cloudConnected) {
    Serial.println("\nCloud verbunden!"); // Meldung im Monitor: Verbindung erfolgreich
  } else {
    Serial.println("\nCloud offline - System laeuft lokal"); // Keine Cloud-Verbindung, System arbeitet im Offline-Modus weiter
  }

  // Ersten Wasserstand messen
  currentWaterLevel = measureWaterLevel(); // Wasserstand in cm ermitteln und speichern
  Serial.println("System bereit!"); // Statusmeldung im seriellen Monitor: System läuft jetzt
}

void loop() {
  // Cloud-Update - Non-blocking
  if (ArduinoCloud.connected()) { //Prüfen, ob Cloud verbunden ist
    ArduinoCloud.update(); // Daten mit der Cloud synchronisieren
    if (!cloudConnected) {  // Wenn vorher getrennt war
      cloudConnected = true; // Status auf "verbunden" setzen
      Serial.println("Cloud wieder verbunden!"); // Meldung im seriellen Monitor
    }
  } else { // Wenn Cloud aktuell NICHT verbunden ist
    if (cloudConnected) { // Wenn sie vorher verbunden war
      cloudConnected = false; // Status auf "getrennt" setzen
      Serial.println("Cloud getrennt - System laeuft weiter"); // Info: Offline-Modus aktiv
    }
  }

  unsigned long now = millis(); // aktuelle Systemzeit in Millisekunden speichern

  // Sensoren lesen und Zonen aktualisieren
  if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    readSensorsAndUpdateZones();// Sensoren auslesen & Zustand (trocken/feucht) aktualisieren
    if (!manualMode) { // Nur im Automatikmodus aktiv
      controlIrrigationPumps(); // Pumpen entsprechend den Zonen steuern
    }
    lastSensorReadTime = now; // Zeitstempel aktualisieren
  }

  // Wasserstand messen und Pumpe steuern. Alle 3 Sekunden misst der Ultraschallsensor den Wasserstand im Fass. Wenn der Pegel zu niedrig ist, wird automatisch Leitungswasser nachgefüllt
  // bis der Maximalwert erreicht ist.
  if (now - lastWaterMeasurement >= WATER_MEASURE_INTERVAL) { // Aktuellen Wasserstand messen
    currentWaterLevel = measureWaterLevel();
    if (!manualMode) { // Nur im Automatikmodus aktiv
      controlWaterPump();  // Nachfüllpumpe ein-/ausschalten
    }
    lastWaterMeasurement = now;   // Zeitstempel aktualisieren
  }

  // Cloud-Variablen aktualisieren. Alle 3 Sekunden werden die aktuellen Messwerte, Pumpenstatus usw. an die Arduino Cloud gesendet. 
  // Dadurch siehst du in der App oder Weboberfläche immer den aktuellen Zustand deines Systems.
  if (now - lastCloudUpdate >= CLOUD_UPDATE_INTERVAL) {
    updateCloudVariables(); // Werte in die Cloud schreiben
    lastCloudUpdate = now;  // Zeitstempel aktualisieren
  }

  delay(100); // kleine Pause, um CPU-Last zu verringern
}
// Diese Funktion wird einmal im Setup aufgerufen. Sie sorgt dafür, dass alle Pumpen, Sensoren und Relais korrekt als Ein-/Ausgänge definiert werden und im sicheren Ausgangszustand (AUS) starten.
void initializeHardware() {
  // Pumpen initialisieren
  for (int i = 0; i < 6; i++) { // Alle 6 Zonen durchlaufen
    pinMode(pumpPins[i], OUTPUT); // Pin als Ausgang definieren
    digitalWrite(pumpPins[i], LOW);  // Pumpe beim Start ausschalten (LOW = AUS)
    pumpRunning[i] = false; // internen Status auf "aus" setzen
  }

  // Ultraschall und Wasserpumpe
  pinMode(TRIG_PIN, OUTPUT); // Trigger-Pin sendet Ultraschall-Impuls
  pinMode(ECHO_PIN, INPUT);  // Echo-Pin empfängt das Signal
  pinMode(RELAY_PIN, OUTPUT); // Relais für Nachfüllpumpe als Ausgang
  digitalWrite(RELAY_PIN, LOW); // Nachfüllpumpe beim Start ausschalten
  
  Serial.println("Hardware initialisiert"); // Bestätigung: alles bereit
  Serial.println("Pumpen schalten direkt beim Schwellenwert"); // Hinweis: ohne Verzögerung
}
// Cloud-Daten aktualisieren. Diese Funktion sendet die aktuellen Messwerte, Pumpen- und Systemzustände an die Arduino Cloud, damit sie in der App oder im Webinterface angezeigt werden.
// Wird nur ausgeführt, wenn das System mit der Cloud verbunden ist.
void updateCloudVariables() {
  // Nur aktualisieren wenn verbunden
  if (!cloudConnected) return; // Abbrechen, falls keine Verbindung besteht
  
  // Jede Zone hat ihren eigenen Sensor (0–5). Die Messwerte werden in die Cloud-Variablen geschrieben, damit du sie in der App live sehen kannst.
  moisture_zone_0 = analogRead(sensorPins[0]);
  moisture_zone_1 = analogRead(sensorPins[1]);
  moisture_zone_2 = analogRead(sensorPins[2]);
  moisture_zone_3 = analogRead(sensorPins[3]);
  moisture_zone_4 = analogRead(sensorPins[4]);
  moisture_zone_5 = analogRead(sensorPins[5]);

  // Aktueller Wasserstand (cm) und prozentualer Füllstand
  water_level = currentWaterLevel; // gemessener Wert in cm

  // Zeigt in der Cloud an, welche Pumpen aktuell laufen (true = EIN, false = AUS)
  pump_0_status = pumpRunning[0];
  pump_1_status = pumpRunning[1];
  pump_2_status = pumpRunning[2];
  pump_3_status = pumpRunning[3];
  pump_4_status = pumpRunning[4];
  pump_5_status = pumpRunning[5];

  // Diese Variablen geben Auskunft über den Gesamtzustand.
  water_pump_status = waterPumpActive; // Nachfüllpumpe aktiv?
  system_ready = systemReady; // System bereit oder gestoppt?
  automatic_mode = !manualMode; // true = Automatikmodus, false = manuell
}
// Feuchtigkeitssensoren lesen und Zonenstatus aktualisieren.
// Diese Funktion liest die analogen Werte aller 6 Bodenfeuchtigkeitssensoren. Je nach gemessenem Wert entscheidet sie, ob eine Zone bewässert werden muss (trocken) oder nicht (feucht genug).
// Die Entscheidung basiert auf den festgelegten Feuchtigkeitsgrenzen in "threshold[]".
void readSensorsAndUpdateZones() {
  Serial.println("\nSensor-Update:"); // Überschrift im seriellen Monitor
  for (int zone = 0; zone < 6; zone++) {  // Alle 6 Zonen nacheinander durchlaufen
    int raw = analogRead(sensorPins[zone]); // Aktuellen Messwert des Sensors lesen

    // Direktes Schalten bei Feuchtigkeitsgrenze
    if (!manualMode) { // Nur wenn Automatikmodus aktiv ist
      if (raw >= threshold[zone]) { // Messwert über Grenzwert: zu trocken
        // Zu trocken - Pumpe einschalten
        if (!zoneNeedsWater[zone]) {
          zoneNeedsWater[zone] = true; // Markiert Zone als "muss bewässert werden"
          Serial.println("Zone " + String(zone) + " (" + plantNames[zone] + ") braucht Wasser!");
        }
      } else { // Messwert unter Grenzwert: Boden ist feucht genug
        // Feucht genug - Pumpe ausschalten
        if (zoneNeedsWater[zone]) {
          zoneNeedsWater[zone] = false; // Markiert Zone als "feucht genug"
          Serial.println("Zone " + String(zone) + " (" + plantNames[zone] + ") gesaettigt");
        }
      }
    }
    // Messwert und Status im seriellen Monitor anzeigen
    Serial.print("Zone " + String(zone) + ": " + String(raw)); // Aktueller Sensorwert
    Serial.print(" (Schwelle: " + String(threshold[zone]) + ") -> ");  // Grenzwert anzeigen
    Serial.println(zoneNeedsWater[zone] ? "BEDARF" : "OK"); // Textausgabe je nach Zustand
  }
}
// Steuerung der Bewässerungspumpen
// Diese Funktion schaltet die Pumpen für jede Zone ein oder aus, abhängig davon, ob die jeweilige Zone Wasser benötigt.
// Zusätzlich wird überprüft, ob im Wassertank genug Wasser vorhanden ist.
// Bei zu niedrigem Wasserstand wird die Bewässerung gestoppt, um Trockenlauf zu verhindern.
void controlIrrigationPumps() {
  // Sicherheitscheck: Genug Wasser vorhanden?
  if (currentWaterLevel <= MIN_FILL_LEVEL) { // Wasserstand zu niedrig
    Serial.println("Zu wenig Wasser - Bewaesserung gestoppt!");
    // Alle Pumpen ausschalten, falls sie noch laufen
    for (int zone = 0; zone < 6; zone++) {
      if (pumpRunning[zone]) {
        stopIrrigationPump(zone); // Sofort stoppen
      }
    }
    systemReady = false; // System meldet: nicht betriebsbereit
    return;  // Funktion beenden (keine weitere Aktion)
  }

  systemReady = true; // System ist betriebsbereit

  // Pumpen entsprechend Zonen-Bedarf steuern
  for (int zone = 0; zone < 6; zone++) {
    if (zoneNeedsWater[zone] && !pumpRunning[zone]) {
      startIrrigationPump(zone);
  // Zone ist feucht genug: Pumpe ausschalten    
    } else if (!zoneNeedsWater[zone] && pumpRunning[zone]) {
      stopIrrigationPump(zone);
    }
  }
}
// Pumpe für eine Zone einschalten
// Setzt den zugehörigen Pumpen-Pin auf HIGH (ein) und aktualisiert den internen Status. Gibt eine Meldung aus.
void startIrrigationPump(int zone) {
  digitalWrite(pumpPins[zone], HIGH); // Pumpe EIN
  pumpRunning[zone] = true; // Status merken
  Serial.println("Pumpe " + String(zone) + " (" + plantNames[zone] + ") EIN");
}
// Pumpe für eine Zone ausschalten
// Setzt den zugehörigen Pumpen-Pin auf LOW (aus) und aktualisiert den internen Status. Gibt eine Meldung aus.
void stopIrrigationPump(int zone) {
  digitalWrite(pumpPins[zone], LOW); // Pumpe AUS
  pumpRunning[zone] = false; // Status merken
  Serial.println("Pumpe " + String(zone) + " (" + plantNames[zone] + ") AUS");
}
// Wasserstand messen (Ultraschallsensor)
// Führt mehrere Messungen durch, filtert ungültige Werte, bildet den Durchschnitt und berechnet daraus den Wasserstand in cm (SENSOR_HEIGHT - Distanz). 
// Der Wert wird auf [0, BARREL_HEIGHT] begrenzt. Bei Messfehler bleibt der letzte gültige Wert erhalten.
float measureWaterLevel() {
  float validDistances[NUM_READINGS]; // Zwischenspeicher gültiger Distanzen (cm)
  int validCount = 0;  // Anzahl gültiger Messungen

  // Mehrere Messungen für Genauigkeit
  for (int i = 0; i < NUM_READINGS; i++) {
    // Ultraschall-Trigger-Puls senden (10 Mikrosekunden HIGH)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  // Echo-Laufzeit messen (Timeout 25 ms, sonst 0)
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0) continue; // Timeout = Messung ignorieren
    // Zeit = Distanz umrechnen (Schallgeschwindigkeit ca. 343 m/s)
    // Distanz (cm) = (Dauer (µs) * 0.0343) / 2  (hin & zurück)
    float distance = (duration * 0.0343) / 2.0;
    // Nur plausible Werte übernehmen (zwischen 0 und MAX_DISTANCE)
    if (distance > 0 && distance <= MAX_DISTANCE) {
      validDistances[validCount] = distance;
      validCount++;
    }
    delay(30); // kurze Pause zwischen Messungen
  }

  // Wenn keine gültigen Messungen, alten Wert zurückgeben
  if (validCount == 0) {
    Serial.println("Ultraschall-Messfehler");
    return currentWaterLevel;
  }

  // Durchschnitt berechnen
  float avgDistance = 0;
  for (int i = 0; i < validCount; i++) {
    avgDistance += validDistances[i];
  }
  avgDistance /= validCount;

  // Wasserstand in Zentimetern berechnen und begrenzen
  float waterLevel = SENSOR_HEIGHT - avgDistance;
  waterLevel = constrain(waterLevel, 0, BARREL_HEIGHT);

  return waterLevel; // Berechneten Wasserstand zurückgeben
}
// Steuerung der Nachfüllpumpe (Wasserpumpe)
// Überwacht den aktuellen Wasserstand im Regenfass und schaltet die Nachfüllpumpe automatisch ein oder aus.
// Wenn der Wasserstand unter MIN_FILL_LEVEL fällt, wird Wasser nachgefüllt. 
// Sobald MAX_FILL_LEVEL erreicht ist, wird die Pumpe gestoppt.
void controlWaterPump() {
   // Normale automatische Steuerung
  if (!waterPumpActive && currentWaterLevel <= MIN_FILL_LEVEL) {
    startWaterPump(); // Nachfüllung starten
  } else if (waterPumpActive && currentWaterLevel >= MAX_FILL_LEVEL) {
    stopWaterPump(); // Nachfüllung beenden
  }
}
// Nachfüllpumpe einschalten
// Aktiviert das Relais, startet die Wasserzufuhr und setzt den internen Status auf aktiv.
void startWaterPump() {
  digitalWrite(RELAY_PIN, HIGH); // Relais einschalten
  waterPumpActive = true; // Status aktualisieren
  Serial.println("WASSERNACHFUELLUNG EIN");
}
// Nachfüllpumpe ausschalten
// Deaktiviert das Relais, stoppt die Wasserzufuhr und setzt den internen Status auf inaktiv.
void stopWaterPump() {
  digitalWrite(RELAY_PIN, LOW); // Relais ausschalten
  waterPumpActive = false; // Status aktualisieren
  Serial.println("WASSERNACHFUELLUNG AUS");
}

// Cloud Callback: Moduswechsel (Automatik / Manuell)
// Wird automatisch von der Arduino Cloud aufgerufen, wenn der Benutzer in der App oder im Webinterface den Betriebsmodus ändert.
// Im manuellen Modus werden alle automatisch laufenden Pumpen und die Nachfüllpumpe sofort gestoppt, damit keine Überschneidung zwischen Automatik und Handsteuerung entsteht.
void onAutomaticModeChange() {
  manualMode = !automatic_mode; // Invertiert den Cloud-Status (true = manuell, false = automatisch)
  Serial.println(manualMode ? "MANUELLER MODUS" : "AUTOMATIK-MODUS");
  
  // Wenn manueller Modus aktiv ist, alle laufenden Pumpen stoppen
  if (manualMode) {  // Wenn manueller Modus aktiv ist
    for (int i = 0; i < 6; i++) {   // Alle sechs Bewässerungszonen prüfen
      if (pumpRunning[i]) { // Wenn eine Zonenpumpe aktuell läuft
        stopIrrigationPump(i); // Pumpe für diese Zone ausschalten
      }
    }
     // Auch die Nachfüllpumpe ausschalten, falls aktiv
    if (waterPumpActive) { // Wenn die Nachfüllpumpe aktiv ist
      stopWaterPump(); // Nachfüllpumpe ausschalten
    }
  }
}
// Cloud Callback: Manuelle Steuerung der Zonenpumpen
// Diese Funktionen werden ausgelöst, wenn der Benutzer in der Arduino Cloud eine bestimmte Zonenpumpe manuell ein- oder ausschaltet. 
// Sie sind nur aktiv, wenn der manuelle Modus aktiv ist.
void onManualPump0Change() {
  if (manualMode && manual_pump_0 != pumpRunning[0]) { // Nur ausführen, wenn manueller Modus aktiv ist und sich der Status geändert hat
    if (manual_pump_0) startIrrigationPump(0); // Wenn Cloud-Schalter EIN, Pumpe 0 starten
    else stopIrrigationPump(0); // Wenn Cloud-Schalter AUS, Pumpe 0 stoppen
  }
}

void onManualPump1Change() {
  if (manualMode && manual_pump_1 != pumpRunning[1]) { // Prüft, ob manueller Modus aktiv und Status unterschiedlich ist
    if (manual_pump_1) startIrrigationPump(1);  // Cloud-Schalter EIN, Pumpe 1 einschalten
    else stopIrrigationPump(1); // Cloud-Schalter AUS, Pumpe 1 ausschalten
  }
}

void onManualPump2Change() {
  if (manualMode && manual_pump_2 != pumpRunning[2]) { // Gilt für Zone 2
    if (manual_pump_2) startIrrigationPump(2);
    else stopIrrigationPump(2);
  }
}

void onManualPump3Change() {
  if (manualMode && manual_pump_3 != pumpRunning[3]) { // Gilt für Zone 3
    if (manual_pump_3) startIrrigationPump(3);
    else stopIrrigationPump(3);
  }
}

void onManualPump4Change() {
  if (manualMode && manual_pump_4 != pumpRunning[4]) {  // Gilt für Zone 4
    if (manual_pump_4) startIrrigationPump(4);
    else stopIrrigationPump(4);
  }
}

void onManualPump5Change() {
  if (manualMode && manual_pump_5 != pumpRunning[5]) { // Gilt für Zone 5
    if (manual_pump_5) startIrrigationPump(5);
    else stopIrrigationPump(5);
  }
}
// Cloud Callback: Manuelle Steuerung der Nachfüllpumpe.
// Wird ausgelöst, wenn der Benutzer in der Arduino Cloud die Nachfüllpumpe manuell ein- oder ausschaltet.
// Diese Funktion ist nur aktiv, wenn der manuelle Modus aktiv ist.
void onManualWaterPumpChange() {
  if (manualMode && manual_water_pump != waterPumpActive) { // Nur ausführen, wenn manueller Modus aktiv und Status unterschiedlich ist
    if (manual_water_pump) startWaterPump();  // Cloud-Schalter EIN, Nachfüllpumpe starten
    else stopWaterPump(); // Cloud-Schalter AUS, Nachfüllpumpe stoppen
  }
}
