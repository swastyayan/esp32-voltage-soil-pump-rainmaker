/***************************************************
 * ESP RainMaker – 3-Phase Voltage + Soil Moisture + Pump Control
 * Using ZMPT101B + ESP32 + ESP RainMaker
 * Boards manager URLs: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 ***************************************************/

#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <ZMPT101B.h>
#include <SimpleTimer.h>
#include <wifi_provisioning/manager.h>

// ---------- GPIO Pins ----------
#define PIN_R       32    // R Phase
#define PIN_Y       33    // Y Phase
#define PIN_B       34    // B Phase
#define SOIL_PIN    35    // Soil Moisture sensor
#define RELAY_PIN   14    // Pump Relay
#define RESET_PIN   0     // For WiFi/Factory reset

// ---------- ZMPT Config ----------
#define MAINS_FREQ  50.0f
#define SENSITIVITY 500.0f
#define CALIBRATION_FACTOR 1.49f

// ---------- Safety Limits ----------
#define MIN_SAFE_VOLTAGE 150.0f
#define SOIL_DRY_LIMIT   40   // % threshold

// ---------- BLE Provisioning ----------
const char *service_name = "PROV_PUMP";
const char *pop = "1234567";

// ---------- Global Variables ----------
float voltageR, voltageY, voltageB;
int soilMoisture;
bool pumpState = false;     // Manual toggle from RainMaker
bool wifi_connected = false;
bool actualRelayState = false;

// ---------- Create Sensor Objects ----------
ZMPT101B sensorR(PIN_R, MAINS_FREQ);
ZMPT101B sensorY(PIN_Y, MAINS_FREQ);
ZMPT101B sensorB(PIN_B, MAINS_FREQ);

// ---------- SimpleTimer ----------
SimpleTimer timer;

// ---------- RainMaker Devices ----------
static Device *voltR_dev = nullptr;
static Device *voltY_dev = nullptr;
static Device *voltB_dev = nullptr;
static Device *soil_dev  = nullptr;
static Switch pump("Pump");

// ---------- Provisioning Events ----------
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#endif
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("\nConnected to Wi-Fi!");
      wifi_connected = true;
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("Provisioning Success!");
      break;
  }
}

// ---------- Callback for Pump Control ----------
void write_callback(Device *device, Param *param, const param_val_t val,
                    void *priv_data, write_ctx_t *ctx) {
  if (strcmp(device->getDeviceName(), "Pump") == 0) {
    if (strcmp(param->getParamName(), "Power") == 0) {
      pumpState = val.val.b;
      Serial.printf("Manual Pump Toggle: %s\n", pumpState ? "ON" : "OFF");
      param->updateAndReport(val);
    }
  }
}

// ---------- Custom Devices (use SLIDER as gauge substitute) ----------
void createCustomDevices() {
  // Voltage_R slider (acts as gauge-like display in app)
  voltR_dev = new Device("Voltage_R");
  Param voltR_param("Voltage", "custom.param.voltage", value(0.0f),
                    PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
  voltR_param.addUIType(ESP_RMAKER_UI_SLIDER);            // use SLIDER (no GAUGE in API)
  voltR_param.addBounds(value(0), value(300), value(1));  // 0..300 V
  voltR_dev->addParam(voltR_param);
  voltR_dev->assignPrimaryParam(voltR_dev->getParamByName("Voltage"));

  // Voltage_Y slider
  voltY_dev = new Device("Voltage_Y");
  Param voltY_param("Voltage", "custom.param.voltage", value(0.0f),
                    PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
  voltY_param.addUIType(ESP_RMAKER_UI_SLIDER);
  voltY_param.addBounds(value(0), value(300), value(1));
  voltY_dev->addParam(voltY_param);
  voltY_dev->assignPrimaryParam(voltY_dev->getParamByName("Voltage"));

  // Voltage_B slider
  voltB_dev = new Device("Voltage_B");
  Param voltB_param("Voltage", "custom.param.voltage", value(0.0f),
                    PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
  voltB_param.addUIType(ESP_RMAKER_UI_SLIDER);
  voltB_param.addBounds(value(0), value(300), value(1));
  voltB_dev->addParam(voltB_param);
  voltB_dev->assignPrimaryParam(voltB_dev->getParamByName("Voltage"));

  // Soil Moisture slider (0..100 %)
  soil_dev = new Device("SoilMoisture");
  Param soil_param("Moisture", "custom.param.percent", value(0),
                   PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
  soil_param.addUIType(ESP_RMAKER_UI_SLIDER);
  soil_param.addBounds(value(0), value(100), value(1));
  soil_dev->addParam(soil_param);
  soil_dev->assignPrimaryParam(soil_dev->getParamByName("Moisture"));
}

// ---------- Read Voltages ----------
void readVoltages() {
  voltageR = sensorR.getRmsVoltage() * CALIBRATION_FACTOR;
  voltageY = sensorY.getRmsVoltage() * CALIBRATION_FACTOR;
  voltageB = sensorB.getRmsVoltage() * CALIBRATION_FACTOR;

  Serial.printf("R: %.2f V | Y: %.2f V | B: %.2f V\n", voltageR, voltageY, voltageB);

  if (voltR_dev) voltR_dev->updateAndReportParam("Voltage", voltageR);
  if (voltY_dev) voltY_dev->updateAndReportParam("Voltage", voltageY);
  if (voltB_dev) voltB_dev->updateAndReportParam("Voltage", voltageB);
}

// ---------- Read Soil Moisture ----------
void readSoil() {
  int raw = analogRead(SOIL_PIN);
  soilMoisture = map(raw, 0, 4095, 100, 0); // 100=wet, 0=dry

  Serial.printf("Soil Moisture: %d %% (raw=%d)\n", soilMoisture, raw);
  if (soil_dev) soil_dev->updateAndReportParam("Moisture", soilMoisture);
}

// ---------- Motor Control ----------
void controlMotor() {
  bool safeVoltages = (voltageR > MIN_SAFE_VOLTAGE &&
                       voltageY > MIN_SAFE_VOLTAGE &&
                       voltageB > MIN_SAFE_VOLTAGE);

  bool soilDry = (soilMoisture < SOIL_DRY_LIMIT);

  bool shouldRun = (safeVoltages && soilDry && pumpState);

  if (shouldRun) {
    digitalWrite(RELAY_PIN, HIGH); // Pump ON
    actualRelayState = true;
    Serial.println("Pump Status: ON");
  } else {
    digitalWrite(RELAY_PIN, LOW);  // Pump OFF
    actualRelayState = false;
    Serial.println("Pump Status: OFF");
  }

  // Report actual relay state
  pump.updateAndReportParam("Power", actualRelayState);
}

// ---------- Send Data Periodically ----------
void sendData() {
  if (!wifi_connected) return;
  readVoltages();
  readSoil();
  controlMotor();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Pump OFF initially
  pinMode(RESET_PIN, INPUT);

  sensorR.setSensitivity(SENSITIVITY);
  sensorY.setSensitivity(SENSITIVITY);
  sensorB.setSensitivity(SENSITIVITY);

  // --- RainMaker Node ---
  Node my_node = RMaker.initNode("PumpController");
  pump.addCb(write_callback);

  // create custom devices
  createCustomDevices();

  // add devices
  my_node.addDevice(*voltR_dev);
  my_node.addDevice(*voltY_dev);
  my_node.addDevice(*voltB_dev);
  my_node.addDevice(*soil_dev);
  my_node.addDevice(pump);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  RMaker.start();

  // Timer every 5 sec
  timer.setInterval(5000);

  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE,
                          WIFI_PROV_SECURITY_1, pop, service_name);
#endif
}

// ---------- Loop ----------
void loop() {
  if (timer.isReady()) {
    sendData();
    timer.reset();
  }

  // --- Reset Button Logic ---
  if (digitalRead(RESET_PIN) == LOW) {
    delay(100);
    int startTime = millis();
    while (digitalRead(RESET_PIN) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000) {
      Serial.println("Factory Reset!");
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.println("Wi-Fi Reset!");
      RMakerWiFiReset(2);
    }
  }
}

