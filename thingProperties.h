#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "Secrets.h"

// === Thing-ID von Arduino Cloud ===
const char THING_ID[] = "1ea616f1-fa86-4fcc-a223-6e25e65ee455"; // Deine Thing ID

// WLAN Verbindung
WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_SSID, SECRET_PASS);

// === Variablen für die IoT Cloud ===
float water_level;
float water_level_percent;
int moisture_zone_0;
int moisture_zone_1;
int moisture_zone_2;
int moisture_zone_3;
int moisture_zone_4;
int moisture_zone_5;

bool automatic_mode;
bool manual_pump_0;
bool manual_pump_1;
bool manual_pump_2;
bool manual_pump_3;
bool manual_pump_4;
bool manual_pump_5;
bool manual_water_pump;

bool pump_0_status;
bool pump_1_status;
bool pump_2_status;
bool pump_3_status;
bool pump_4_status;
bool pump_5_status;
bool water_pump_status;
bool system_ready;

// === Funktionsprototypen (Callbacks) ===
void onAutomaticModeChange();
void onManualPump0Change();
void onManualPump1Change();
void onManualPump2Change();
void onManualPump3Change();
void onManualPump4Change();
void onManualPump5Change();
void onManualWaterPumpChange();

void initProperties() {
  ArduinoCloud.addProperty(water_level, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(water_level_percent, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_0, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_1, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_2, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_3, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_4, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(moisture_zone_5, READ, ON_CHANGE, NULL);

  ArduinoCloud.addProperty(automatic_mode, READWRITE, ON_CHANGE, onAutomaticModeChange);
  ArduinoCloud.addProperty(manual_pump_0, READWRITE, ON_CHANGE, onManualPump0Change);
  ArduinoCloud.addProperty(manual_pump_1, READWRITE, ON_CHANGE, onManualPump1Change);
  ArduinoCloud.addProperty(manual_pump_2, READWRITE, ON_CHANGE, onManualPump2Change);
  ArduinoCloud.addProperty(manual_pump_3, READWRITE, ON_CHANGE, onManualPump3Change);
  ArduinoCloud.addProperty(manual_pump_4, READWRITE, ON_CHANGE, onManualPump4Change);
  ArduinoCloud.addProperty(manual_pump_5, READWRITE, ON_CHANGE, onManualPump5Change);
  ArduinoCloud.addProperty(manual_water_pump, READWRITE, ON_CHANGE, onManualWaterPumpChange);

  ArduinoCloud.addProperty(pump_0_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_1_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_2_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_3_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_4_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(pump_5_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(water_pump_status, READ, ON_CHANGE, NULL);
  ArduinoCloud.addProperty(system_ready, READ, ON_CHANGE, NULL);
}
