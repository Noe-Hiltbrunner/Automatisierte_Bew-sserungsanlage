/*
  Arduino GIGA R1 WiFi - Bewässerungssystem für Arduino Cloud
  OHNE Hysterese - Direkte Echtzeitsteuerung
*/

#include "thingProperties.h"

// ======== HARDWARE - PINS ========
const int sensorPins[6] = {A0, A3, A2, A1, A4, A6};   // Feuchtigkeitssensoren
const int pumpPins[6]   = {26, 5, 7, 40, 30, 32};     // Bewässerungspumpen
const int TRIG_PIN = 3;      // Ultraschall Trigger
const int ECHO_PIN = 4;      // Ultraschall Echo  
const int RELAY_PIN = 36;    // Magnetventil (Wasserpumpe)

// ======== BEWÄSSERUNG - Schwellenwerte ========
const int threshold[6] = {342, 506, 417, 445, 445, 340};  // Einschalt- und Ausschalt-Schwellenwert

// ======== WASSERSTANDSKONTROLLE - Parameter (in cm) ========
const float SENSOR_HEIGHT = 107.5;
const float BARREL_HEIGHT = 93.0;
const float MIN_FILL_LEVEL = 25.0;
const float MAX_FILL_LEVEL = 72.0;
const int NUM_READINGS = 3;
const float MAX_DISTANCE = 400.0;

// ======== WiFi-Stabilität ========
unsigned long lastCloudUpdate = 0;
const unsigned long CLOUD_UPDATE_INTERVAL = 3000;
bool cloudConnected = false;
unsigned long lastConnectionCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 10000;

// ======== STATUS-VARIABLEN ========
bool zoneNeedsWater[6] = {false,false,false,false,false,false};
bool pumpRunning[6] = {false,false,false,false,false,false};
bool waterPumpActive = false;
bool systemReady = true;
float currentWaterLevel = 0.0;

// ======== INTERVALLE ========
const unsigned long SENSOR_READ_INTERVAL = 15000;
const unsigned long WATER_MEASURE_INTERVAL = 2000;
unsigned long lastSensorReadTime = 0;
unsigned long lastWaterMeasurement = 0;

// Namen der Zonen
const String plantNames[6] = {
  "Heidelbeere","Traubenbaum","Baum","Beet 1","Beet 2","Johannisbeeren"
};

// ======== MANUELLER MODUS ========
bool manualMode = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Hardware initialisieren
  initializeHardware();
  
  // Cloud-Verbindung
  Serial.println("Starte Cloud-Verbindung...");
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  // Kurzes Warten, aber nicht blockieren
  unsigned long startTime = millis();
  while (!ArduinoCloud.connected() && millis() - startTime < 15000) {
    ArduinoCloud.update();
    delay(200);
    Serial.print(".");
  }
  
  cloudConnected = ArduinoCloud.connected();
  if (cloudConnected) {
    Serial.println("\nCloud verbunden!");
  } else {
    Serial.println("\nCloud offline - System laeuft lokal");
  }

  // Ersten Wasserstand messen
  currentWaterLevel = measureWaterLevel();
  Serial.println("System bereit! (OHNE Hysterese)");
}

void loop() {
  // Cloud-Update - Non-blocking
  if (ArduinoCloud.connected()) {
    ArduinoCloud.update();
    if (!cloudConnected) {
      cloudConnected = true;
      Serial.println("Cloud wieder verbunden!");
    }
  } else {
    if (cloudConnected) {
      cloudConnected = false;
      Serial.println("Cloud getrennt - System laeuft weiter");
    }
  }

  unsigned long now = millis();

  // Sensoren lesen und Zonen aktualisieren
  if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    readSensorsAndUpdateZones();
    if (!manualMode) {
      controlIrrigationPumps();
    }
    lastSensorReadTime = now;
  }

  // Wasserstand messen und Pumpe steuern
  if (now - lastWaterMeasurement >= WATER_MEASURE_INTERVAL) {
    currentWaterLevel = measureWaterLevel();
    if (!manualMode) {
      controlWaterPump();
    }
    lastWaterMeasurement = now;
  }

  // Cloud-Variablen aktualisieren
  if (now - lastCloudUpdate >= CLOUD_UPDATE_INTERVAL) {
    updateCloudVariables();
    lastCloudUpdate = now;
  }

  delay(100);
}

void initializeHardware() {
  // Pumpen initialisieren
  for (int i = 0; i < 6; i++) {
    pinMode(pumpPins[i], OUTPUT);
    digitalWrite(pumpPins[i], LOW);
    pumpRunning[i] = false;
  }

  // Ultraschall und Wasserpumpe
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
  Serial.println("Hardware initialisiert");
  Serial.println("ACHTUNG: System arbeitet OHNE Hysterese!");
  Serial.println("Pumpen schalten direkt beim Schwellenwert");
}

void updateCloudVariables() {
  // Nur aktualisieren wenn verbunden
  if (!cloudConnected) return;
  
  // Feuchtigkeitswerte
  moisture_zone_0 = analogRead(sensorPins[0]);
  moisture_zone_1 = analogRead(sensorPins[1]);
  moisture_zone_2 = analogRead(sensorPins[2]);
  moisture_zone_3 = analogRead(sensorPins[3]);
  moisture_zone_4 = analogRead(sensorPins[4]);
  moisture_zone_5 = analogRead(sensorPins[5]);

  // Wasserstand
  water_level = currentWaterLevel;
  water_level_percent = (currentWaterLevel / BARREL_HEIGHT) * 100;

  // Pumpenstatus
  pump_0_status = pumpRunning[0];
  pump_1_status = pumpRunning[1];
  pump_2_status = pumpRunning[2];
  pump_3_status = pumpRunning[3];
  pump_4_status = pumpRunning[4];
  pump_5_status = pumpRunning[5];

  // System-Status
  water_pump_status = waterPumpActive;
  system_ready = systemReady;
  automatic_mode = !manualMode;
}

void readSensorsAndUpdateZones() {
  Serial.println("\nSensor-Update:");
  for (int zone = 0; zone < 6; zone++) {
    int raw = analogRead(sensorPins[zone]);

    // OHNE Hysterese: Direktes Schalten beim Schwellenwert
    if (!manualMode) {
      if (raw >= threshold[zone]) {
        // Zu trocken - Pumpe einschalten
        if (!zoneNeedsWater[zone]) {
          zoneNeedsWater[zone] = true;
          Serial.println("Zone " + String(zone) + " (" + plantNames[zone] + ") braucht Wasser!");
        }
      } else {
        // Feucht genug - Pumpe ausschalten
        if (zoneNeedsWater[zone]) {
          zoneNeedsWater[zone] = false;
          Serial.println("Zone " + String(zone) + " (" + plantNames[zone] + ") gesaettigt");
        }
      }
    }

    Serial.print("Zone " + String(zone) + ": " + String(raw));
    Serial.print(" (Schwelle: " + String(threshold[zone]) + ") -> ");
    Serial.println(zoneNeedsWater[zone] ? "BEDARF" : "OK");
  }
}

void controlIrrigationPumps() {
  // Sicherheitscheck: Genug Wasser vorhanden?
  if (currentWaterLevel <= MIN_FILL_LEVEL) {
    Serial.println("Zu wenig Wasser - Bewaesserung gestoppt!");
    for (int zone = 0; zone < 6; zone++) {
      if (pumpRunning[zone]) {
        stopIrrigationPump(zone);
      }
    }
    systemReady = false;
    return;
  }

  systemReady = true;

  // Pumpen entsprechend Zonen-Bedarf steuern
  for (int zone = 0; zone < 6; zone++) {
    if (zoneNeedsWater[zone] && !pumpRunning[zone]) {
      startIrrigationPump(zone);
    } else if (!zoneNeedsWater[zone] && pumpRunning[zone]) {
      stopIrrigationPump(zone);
    }
  }
}

void startIrrigationPump(int zone) {
  digitalWrite(pumpPins[zone], HIGH);
  pumpRunning[zone] = true;
  Serial.println("Pumpe " + String(zone) + " (" + plantNames[zone] + ") EIN");
}

void stopIrrigationPump(int zone) {
  digitalWrite(pumpPins[zone], LOW);
  pumpRunning[zone] = false;
  Serial.println("Pumpe " + String(zone) + " (" + plantNames[zone] + ") AUS");
}

float measureWaterLevel() {
  float validDistances[NUM_READINGS];
  int validCount = 0;

  // Mehrere Messungen für Genauigkeit
  for (int i = 0; i < NUM_READINGS; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0) continue;

    float distance = (duration * 0.0343) / 2.0;
    if (distance > 0 && distance <= MAX_DISTANCE) {
      validDistances[validCount] = distance;
      validCount++;
    }
    delay(30);
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

  // Wasserstand berechnen und begrenzen
  float waterLevel = SENSOR_HEIGHT - avgDistance;
  waterLevel = constrain(waterLevel, 0, BARREL_HEIGHT);

  return waterLevel;
}

void controlWaterPump() {
  // Normale Steuerung
  if (!waterPumpActive && currentWaterLevel <= MIN_FILL_LEVEL) {
    startWaterPump();
  } else if (waterPumpActive && currentWaterLevel >= MAX_FILL_LEVEL) {
    stopWaterPump();
  }
}

void startWaterPump() {
  digitalWrite(RELAY_PIN, HIGH);
  waterPumpActive = true;
  Serial.println("WASSERNACHFUELLUNG EIN");
}

void stopWaterPump() {
  digitalWrite(RELAY_PIN, LOW);
  waterPumpActive = false;
  Serial.println("WASSERNACHFUELLUNG AUS");
}

// ======== CLOUD CALLBACK FUNKTIONEN ========
void onAutomaticModeChange() {
  manualMode = !automatic_mode;
  Serial.println(manualMode ? "MANUELLER MODUS" : "AUTOMATIK-MODUS");
  
  // Im manuellen Modus alle Automatik-Pumpen stoppen
  if (manualMode) {
    for (int i = 0; i < 6; i++) {
      if (pumpRunning[i]) {
        stopIrrigationPump(i);
      }
    }
    if (waterPumpActive) {
      stopWaterPump();
    }
  }
}

void onManualPump0Change() {
  if (manualMode && manual_pump_0 != pumpRunning[0]) {
    if (manual_pump_0) startIrrigationPump(0);
    else stopIrrigationPump(0);
  }
}

void onManualPump1Change() {
  if (manualMode && manual_pump_1 != pumpRunning[1]) {
    if (manual_pump_1) startIrrigationPump(1);
    else stopIrrigationPump(1);
  }
}

void onManualPump2Change() {
  if (manualMode && manual_pump_2 != pumpRunning[2]) {
    if (manual_pump_2) startIrrigationPump(2);
    else stopIrrigationPump(2);
  }
}

void onManualPump3Change() {
  if (manualMode && manual_pump_3 != pumpRunning[3]) {
    if (manual_pump_3) startIrrigationPump(3);
    else stopIrrigationPump(3);
  }
}

void onManualPump4Change() {
  if (manualMode && manual_pump_4 != pumpRunning[4]) {
    if (manual_pump_4) startIrrigationPump(4);
    else stopIrrigationPump(4);
  }
}

void onManualPump5Change() {
  if (manualMode && manual_pump_5 != pumpRunning[5]) {
    if (manual_pump_5) startIrrigationPump(5);
    else stopIrrigationPump(5);
  }
}

void onManualWaterPumpChange() {
  if (manualMode && manual_water_pump != waterPumpActive) {
    if (manual_water_pump) startWaterPump();
    else stopWaterPump();
  }
}
