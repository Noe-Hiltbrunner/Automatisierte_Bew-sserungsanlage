// Arduino IoT Cloud: Thing Properties
// Deklariert Cloud-Variablen, Callback-Prototypen und
// initialisiert die Properties mit Rechten und Triggern.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "Secrets.h" // enthält SECRET_SSID und SECRET_PASS

// Thing-ID der Arduino Cloud
// Hinweis: Die Thing-ID der Arduino Cloud muss im Code angegeben werden. Sie ist in der Arduino IoT Cloud im Bereich „Things“ unter „Metadata“ zu finden und muss exakt zu dem jeweiligen Projekt passen.
const char THING_ID[] = "1ea616f1-fa86-4fcc-a223-6e25e65ee455"; // Deine Thing ID

// WLAN-Verbindungshandler
WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_SSID, SECRET_PASS);

// Cloud-Variablen
// Datentypen müssen exakt zu den Properties im Dashboard passen.

// Messwerte
float water_level; // Wasserstand in cm
float water_level_percent; // Füllstand in Prozent
int moisture_zone_0; // Bodenfeuchte Zone 0 
int moisture_zone_1; // Bodenfeuchte Zone 1
int moisture_zone_2; // Bodenfeuchte Zone 2
int moisture_zone_3; // Bodenfeuchte Zone 3
int moisture_zone_4; // Bodenfeuchte Zone 4
int moisture_zone_5; // Bodenfeuchte Zone 5

// Bedienung
bool automatic_mode; // true = Automatik, false = Manuell
bool manual_pump_0; // manueller Schalter Zone 0
bool manual_pump_1; // manueller Schalter Zone 1
bool manual_pump_2; // manueller Schalter Zone 2
bool manual_pump_3; // manueller Schalter Zone 3
bool manual_pump_4; // manueller Schalter Zone 4
bool manual_pump_5; // manueller Schalter Zone 5
bool manual_water_pump; // manueller Schalter Nachfüllpumpe

// Statusanzeigen
bool pump_0_status; // Status Zone 0
bool pump_1_status; // Status Zone 1
bool pump_2_status; // Status Zone 2
bool pump_3_status; // Status Zone 3
bool pump_4_status; // Status Zone 4
bool pump_5_status; // Status Zone 5
bool water_pump_status; // Status Nachfüllpumpe
bool system_ready; // System bereit oder gestoppt
 
// Callback-Prototypen (werden im Hauptsketch definiert)
void onAutomaticModeChange();
void onManualPump0Change();
void onManualPump1Change();
void onManualPump2Change();
void onManualPump3Change();
void onManualPump4Change();
void onManualPump5Change();
void onManualWaterPumpChange();

// Properties registrieren
// READ       = nur lesen in der Cloud
// READWRITE  = lesen und schreiben in der Cloud
// ON_CHANGE  = nur senden, wenn sich der Wert ändert

void initProperties() {
  // Messwerte
  ArduinoCloud.addProperty(water_level, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(water_level_percent, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_0, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_1, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_2, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_3, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_4, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_5, READ, ON_CHANGE, NULL);

  // Bedienung
  ArduinoCloud.addProperty(automatic_mode, READWRITE, ON_CHANGE, onAutomaticModeChange);
  ArduinoCloud.addProperty(manual_pump_0, READWRITE, ON_CHANGE, onManualPump0Change);
  ArduinoCloud.addProperty(manual_pump_1, READWRITE, ON_CHANGE, onManualPump1Change);
  ArduinoCloud.addProperty(manual_pump_2, READWRITE, ON_CHANGE, onManualPump2Change);
  ArduinoCloud.addProperty(manual_pump_3, READWRITE, ON_CHANGE, onManualPump3Change);
  ArduinoCloud.addProperty(manual_pump_4, READWRITE, ON_CHANGE, onManualPump4Change);
  ArduinoCloud.addProperty(manual_pump_5, READWRITE, ON_CHANGE, onManualPump5Change);
  ArduinoCloud.addProperty(manual_water_pump, READWRITE, ON_CHANGE, onManualWaterPumpChange);

  // Statusanzeigen
  ArduinoCloud.addProperty(pump_0_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_1_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_2_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_3_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_4_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_5_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(water_pump_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(system_ready, READ, ON_CHANGE, NULL);
}
