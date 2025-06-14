#include <Arduino.h>
#include "esp_log.h"
#include "Wire.h"        // Pro I2C komunikaci
#include <driver/ledc.h> // ESP‑IDF LEDC driver (ledc_timer_config, ledc_channel_config, ledc_set_duty)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define TAG "MAIN"

#include <M5AtomS3.h>  // Specifická knihovna pro M5AtomS3
#include "M5GFX.h"     // Grafická knihovna pro displej
#include "M5Unified.h" // Sjednocená knihovna M5Stack
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_MODEM_SIM800
// Check if PSRAM is available and initialize canvas with PSRAM
#include "credentials.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>
// #include <StreamDebugger.h>
// StreamDebugger debugger(Serial2, Serial);

M5Canvas canvas = M5Canvas(&M5.Display);
;

#define BTN1 41 // Definice pinu pro tlačítko (GPIO 41)

#include <Preferences.h> // Pro ukládání dat do NVS (Non-Volatile Storage)

// #include <WiFi.h>                     // Pro WiFi připojení
// #include <AsyncTCP.h>                 // Asynchronní TCP knihovna
// #include <ESPAsyncWebServer.h>        // Asynchronní webový server
// #include <ESPAsyncHTTPUpdateServer.h> // Pro OTA aktualizace firmwaru
// ESPAsyncHTTPUpdateServer updateServer;
// AsyncWebServer server(80); // Webový server na portu 80

#include <Adafruit_INA228.h>
const uint8_t bat_addr = 0x40;   // Adresa INA228 pro baterii
const uint8_t solar_addr = 0x41; // Adresa INA228 pro solární panel
Adafruit_INA228 ina228_bat = Adafruit_INA228();
Adafruit_INA228 ina228_solar = Adafruit_INA228();
bool ina_inicialized = false;

float bat_shuntVoltage = 0.0;
float bat_busVoltage = 0.0;
float bat_current = 0.0;
float bat_power = 0.0;

float solar_shuntVoltage = 0.0;
float solar_busVoltage = 0.0;
float solar_current = 0.0;
float solar_power = 0.0;

#include "NimBLEDevice.h"
#include "NimBLEClient.h"
float BLE_temperature = 0.0f; // Teplota z BLE senzoru
float BLE_humidity = 0.0f;    // Vlhkost z BLE senzoru
float BLE_voltage = 0.0f;     // Napětí z BLE senzoru (pokud je potřeba)
static constexpr uint32_t scanTimeMs = 15 * 1000;
static bool doConnect = false;
static bool BLEConnected = false;
static uint8_t BLERetries = 0;
static const NimBLEAdvertisedDevice *advDevice;
#define BLE_DEVICE_NAME "LYWSD03MMC"
#define BLE_SCAN_MAX_RESULTS 80
// The remote service we wish to connect to.
#define SERVICE_UUID "ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6"
// The characteristic of the remote service we are interested in.
#define CHAR_UUID "ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6"
void ble_setup();

uint32_t lastReconnectAttempt = 0;
void gsmLoop();
void gsmSetup();
void gsmTask(void *pvParameters);
bool mqttConnected = false;
// Your GPRS credentials, if any

const char *topicStatus = "solarfan/status";
const char *topicControl = "solarfan/control";

#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// MQTT details

void measure();
// Funkce pro obsluhu nenalezených stránek webového serveru

// Deklarace funkcí
void setupWiFiClient();
void buttonLoop();
void shortPressed();
void longPressed();
void drawGUI(); // Deklarace funkce pro kreslení GUI
void setupPWM();

// PWM definice
#define PWM_GPIO GPIO_NUM_6                 // GPIO pro PWM výstup
#define PWM_CHANNEL 1                       // PWM kanál
#define PWM_FREQUENCY 25000                 // Frekvence PWM (25 kHz)
#define PWM_RESOLUTION 10                   // Rozlišení PWM (10 bit, 0‑1023)
#define PWM_MAX ((1 << PWM_RESOLUTION) - 1) // Maximální hodnota duty‑cycle (1023 pro 10‑bit)
#define PWM_MIN 100                         // Minimální hodnota duty‑cycle (0)
uint16_t pwm = 0;

Preferences preferences; // Instance pro NVS Preferences

// Globální proměnné
static unsigned long buttonPressStartTime = 0;
static bool buttonPressed = false;
static bool longButtonPressed = false;
static bool updateStarted = false;
static float rotationAngle = 0.0f; // Úhel rotace pro displej
// HardwareSerial Serial1(2);         // UART2
TinyGsm modem(Serial2);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
void publishMeasurements();

/** Define a class to handle the callbacks for client connection events */
class ClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient *pClient) override
  {
    BLERetries = 0;
    ESP_LOGI(TAG, "Connected");
  }

  void onDisconnect(NimBLEClient *pClient, int reason) override
  {
    ESP_LOGI(TAG, "%s Disconnected, reason = %d - Starting scan", pClient->getPeerAddress().toString().c_str(), reason);
    NimBLEDevice::getScan()->start(scanTimeMs, true, true);
    BLEConnected = false;
  }

} clientCallbacks;

/** Define a class to handle the callbacks when advertisements are received */
class ScanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
  {
    if (advertisedDevice->getRSSI() < -82 || !advertisedDevice->haveName())
    {
      NimBLEDevice::getScan()->erase(advertisedDevice);
      return;
    }

    std::string deviceName = advertisedDevice->getName();

    // Hledáme zařízení s přesným názvem BLEName
    if (deviceName == BLE_DEVICE_NAME)
    {
      ESP_LOGI(TAG, "Found exact matching device: %s", BLE_DEVICE_NAME);
      NimBLEDevice::getScan()->stop();
      advDevice = advertisedDevice;
      doConnect = true;
    }

    ESP_LOGI(TAG, "Advertised Device found: %s, heap: %d", deviceName.c_str(), ESP.getFreeHeap());
  }

  /** Callback to process the results of the completed scan or restart it */
  void onScanEnd(const NimBLEScanResults &results, int reason) override
  {
    ESP_LOGI(TAG, "Scan Ended, reason: %d, device count: %d; Restarting scan", reason, results.getCount());

    // if (BLERetries < BLEMaxRetries)
    // {

    if (NimBLEDevice::getConnectedClients().empty())
    {
      // NimBLEClient *pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());

      // after end of scan, we look for the table in the current list
      ESP_LOGI(TAG, "Scan End results.count %d", results.getCount());

      for (int i = 0; i < results.getCount(); i++)
      {
        const NimBLEAdvertisedDevice *device = results.getDevice(i);
        String deviceName = device->getName().c_str();
        ESP_LOGI(TAG, "Device name: %s", deviceName.c_str());
        if (deviceName == BLE_DEVICE_NAME)
        {
          ESP_LOGI(TAG, "Found exact matching device: %s, rssi: %d", BLE_DEVICE_NAME, device->getRSSI());
          NimBLEDevice::getScan()->stop();

          advDevice = results.getDevice(i);
          doConnect = true;
        }
      }

      NimBLEScan *pScan = NimBLEDevice::getScan();

      // NimBLEDevice::getScan()->stop();
      pScan->start(scanTimeMs, false, true);
      pScan->setDuplicateFilter(false); // if is false then onResult will report all new results
    }
    else
    {
      ESP_LOGI(TAG, "getConnectedClients not empty");
      if (BLEConnected)
      {
        std::vector<NimBLEClient *> connectedClients = NimBLEDevice::getConnectedClients();
        for (auto *client : connectedClients)
        {
          if (client && client->isConnected())
          {
            client->disconnect();
            ESP_LOGI("BLE", "Disconnected client: %s", client->getPeerAddress().toString().c_str());
          }
        }
      }
    }
    // }
  }
} scanCallbacks;

bool connectToBTServer()
{
  ESP_LOGI(TAG, "heap %d", ESP.getFreeHeap());

  NimBLEClient *pClient = nullptr;

  /** Check if we have a client we should reuse first **/
  if (NimBLEDevice::getCreatedClientCount())
  {
    /**
     *  Special case when we already know this device, we send false as the
     *  second argument in connect() to prevent refreshing the service database.
     *  This saves considerable time and power.
     */
    pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
    if (pClient)
    {
      if (!pClient->connect(advDevice, false))
      {
        ESP_LOGI(TAG, "Reconnect failed");
        return false;
      }
      ESP_LOGI(TAG, "Reconnected client");
    }
    else
    {
      /**
       *  We don't already have a client that knows this device,
       *  check for a client that is disconnected that we can use.
       */
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  /** No client to reuse? Create a new one. */
  if (!pClient)
  {
    if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS)
    {
      ESP_LOGI(TAG, "Max clients reached - no more connections available");
      return false;
    }

    pClient = NimBLEDevice::createClient();

    ESP_LOGI(TAG, "New client created");

    pClient->setClientCallbacks(&clientCallbacks, false);
    /**
     *  Set initial connection parameters:
     *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
     *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
     *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 150 * 10ms = 1500ms timeout
     */
    pClient->setConnectionParams(24, 40, 0, 200, 16, 16);

    /** Set how long we are willing to wait for the connection to complete (milliseconds), default is 30000. */
    pClient->setConnectTimeout(5 * 1000);

    if (!pClient->connect(advDevice))
    {
      /** Created a client but failed to connect, don't need to keep it as it has no data */
      NimBLEDevice::deleteClient(pClient);
      ESP_LOGI(TAG, "Failed to connect, deleted client");
      return false;
    }
  }

  if (!pClient->isConnected())
  {
    if (!pClient->connect(advDevice))
    {
      ESP_LOGI(TAG, "Failed to connect");
      return false;
    }
  }

  ESP_LOGI(TAG, "Connected to: %s RSSI: %d", pClient->getPeerAddress().toString().c_str(), pClient->getRssi());

  /** Now we can read/write/subscribe the characteristics of the services we are interested in */
  NimBLERemoteService *pSvc = nullptr;
  NimBLERemoteCharacteristic *pChrTable = nullptr;

  // NimBLERemoteDescriptor *pDsc = nullptr;

  pSvc = pClient->getService(SERVICE_UUID);
  if (pSvc)
  {
    pChrTable = pSvc->getCharacteristic(CHAR_UUID);
    if (pChrTable)
    {
      // if (pChrTable->canRead())
      // {
      //   std::string value = pChrTable->readValue();
      //   if (value.length() >= sizeof(uint16_t))
      //   {
      //     uint16_t intValue = *(uint16_t *)value.data();
      //     tableController->set_height(intValue / 10.0f);

      //     ESP_LOGI(TAG, "Characteristic value: %u", intValue);
      //   }
      //   else
      //   {
      //     ESP_LOGI(TAG, "Characteristic value is smaller than expected (less than 2 bytes)!");
      //   }
      // }
      // else
      // {
      //   ESP_LOGI(TAG, "Characteristic does not support reading.");
      // }

      if (pChrTable->canNotify())
      {
        pChrTable->subscribe(true, [](NimBLERemoteCharacteristic *pCharacteristic, uint8_t *pData, size_t length, bool isNotify)
                             {
          ESP_LOGI(TAG, "Notify callback for characteristic %s, length=%d",
                   pCharacteristic->getUUID().toString().c_str(), length);

          if (length >= 3) {
            // Temperature: signed int16 little‑endian, value is °C × 100
            int16_t rawTemp = (int16_t)(pData[0] | (pData[1] << 8));
            BLE_temperature = rawTemp / 100.0f;

            // Humidity: single byte, % RH
            BLE_humidity = pData[2];

            // Optional battery voltage: unsigned int16 little‑endian, value is mV
            if (length >= 5) {
              uint16_t rawVolt = (uint16_t)(pData[3] | (pData[4] << 8));
              BLE_voltage = rawVolt / 1000.0f;
            }

            ESP_LOGI(TAG,
                     "temp = %.2f °C ; humidity = %.1f %% ; voltage = %.3f V",
                     BLE_temperature, BLE_humidity, BLE_voltage);
          } else {
            ESP_LOGI(TAG, "Notification data size unexpected: %d bytes", length);
          } });

        ESP_LOGI(TAG, "Subscribed to notifications!");
      }
      else
      {
        ESP_LOGI(TAG, "Characteristic does not support notifications.");
      }
    }
    else
    {
      ESP_LOGI(TAG, "Failed to find characteristic for table height.");
    }
  }

  // BLESaveToWhiteList();
  // Počkej krátkou dobu na dokončení párování
  ESP_LOGI(TAG, "Connection completed (no secureConnection check needed)");
  ESP_LOGI(TAG, "Done with this device!");

  return true;
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
  ESP_LOGI(TAG, "Message arrived [%s]: %.*s", topic, len, payload);

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicControl)
  {
    // ledStatus = !ledStatus;
    // Convert payload to string and then to integer
    String msg = String((char *)payload, len);
    pwm = msg.toInt();
    // Ensure pwm value is within valid range (0‑PWM_MAX for 10‑bit resolution)
    pwm = constrain(pwm, 0, PWM_MAX); // Udrž hodnotu v rozsahu rozlišení
    // Set PWM value
    if (pwm < PWM_MIN)
    {
      pwm = 0; // Ensure minimum duty cycle
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));
    ESP_LOGI(TAG, "Setting PWM to %d", pwm);
    // digitalWrite(LED_PIN, ledStatus);
    // mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  }
}

// Initialize PWM
void setupPWM()
{

  // --- Configure LEDC timer ---
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = PWM_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&timer_conf);

  // --- Configure LEDC channel ---
  ledc_channel_config_t channel_conf = {
      .gpio_num = PWM_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = static_cast<ledc_channel_t>(PWM_CHANNEL),
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0};
  ledc_channel_config(&channel_conf);

  // Ensure initial duty is applied
  ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));

  ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, 0);
}

boolean mqttConnect()
{
  ESP_LOGI(TAG, "Connecting to MQTT broker %s:%d", broker, mqttPort);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("SolarFan", MQTT_USER, MQTT_PASS);

  // Or, if you want to authenticate MQTT:
  // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (!status)
  {
    ESP_LOGI(TAG, "fail (state = %d)", mqtt.state());
    return false;
  }
  ESP_LOGI(TAG, "success");
  // mqtt.publish(topicInit, "GsmClientTest started");
  mqtt.subscribe(topicControl);
  return mqtt.connected();
}

void publishMeasurements()
{
  // Make sure MQTT is connected before trying to publish
  if (!mqtt.connected())
  {
    return;
  }

  // Build a JSON string with all fields
  //   tmp  = BLE_temperature (°C)
  //   hum  = BLE_humidity (%)
  //   bv   = BLE_voltage (V)
  //   batv = bat_busVoltage (V)
  //   bp   = bat_power (mW)
  //   sv   = solar_busVoltage (V)
  //   sp   = solar_power (mW)
  //   pwm  = pwm (0–1023)
  String payload = "{";
  payload += "\"tmp\":" + String(BLE_temperature, 2);
  payload += ",\"hum\":" + String(BLE_humidity, 1);
  payload += ",\"bv\":" + String(BLE_voltage, 3);
  payload += ",\"batv\":" + String(bat_busVoltage, 2);
  payload += ",\"bp\":" + String(bat_current * bat_busVoltage, 0);
  payload += ",\"sv\":" + String(solar_busVoltage, 2);
  payload += ",\"sp\":" + String(solar_power, 0);
  payload += ",\"pwm\":" + String(pwm);
  payload += "}";

  // Publish to your chosen MQTT topic
  mqtt.publish(topicStatus, payload.c_str());
}

void setup()
{
  setupPWM();
  // Inicializace NVS pro ukládání nastavení displeje
  preferences.begin("display", false);
  rotationAngle = preferences.getFloat("rotation", 0.0f); // Načtení úhlu rotace

  // Inicializace M5Stack AtomS3 a displeje
  M5.begin();
  M5.Display.begin(); // Explicitní inicializace displeje pro jistotu

  M5.Display.setBrightness(35); // Nastavení jasu displeje

  Serial.begin(115200);                  // Inicializace sériové komunikace
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Zpoždění pro připojení sériového monitoru

  // Vytvoření canvasu o velikosti displeje
  canvas.createSprite(M5.Display.width(), M5.Display.height());
  canvas.setTextColor(WHITE); // Nastavení barvy textu na bílou

  pinMode(BTN1, INPUT_PULLUP); // Nastavení pinu tlačítka s interním pull-up rezistorem

  // Create FreeRTOS timer for publishing measurements every 5 minutes
  static TimerHandle_t publishTimer = NULL;
  if (publishTimer == NULL)
  {
    publishTimer = xTimerCreate(
        "PublishTimer",
        pdMS_TO_TICKS(300000), // 5 minutes = 300000 ms
        pdTRUE,                // Auto reload
        nullptr,
        [](TimerHandle_t xTimer)
        {
          publishMeasurements();
        });
    xTimerStart(publishTimer, 0);
  }

  Wire.begin();

  if (ina228_bat.begin(bat_addr) && ina228_solar.begin(solar_addr))
  {
    ina_inicialized = true;
    ESP_LOGI(TAG, "INA228 devices initialized successfully");

    ina228_bat.setShunt(0.015, 10.0);
    ina228_bat.setAveragingCount(INA228_COUNT_64);
    ina228_bat.setVoltageConversionTime(INA228_TIME_540_us);
    ina228_bat.setCurrentConversionTime(INA228_TIME_280_us);

    ina228_solar.setShunt(0.015, 10.0);
    ina228_solar.setAveragingCount(INA228_COUNT_64);
    ina228_solar.setVoltageConversionTime(INA228_TIME_540_us);
    ina228_solar.setCurrentConversionTime(INA228_TIME_280_us);
  }
  else
  {
    ESP_LOGE(TAG, "Failed to initialize INA228 devices");
  }

  ble_setup();
  xTaskCreatePinnedToCore(gsmTask, "GSM", 8192, NULL, 1, NULL, 1);

  publishMeasurements(); // Initial publish
}

void ble_setup()
{
  ESP_LOGI(TAG, "Starting NimBLE Client");

  /** Initialize NimBLE and set the device name */
#if !CONFIG_BT_BLUEDROID_ENABLED // Arduino vždy vypíná Bluedroid
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif
  ESP_LOGI(TAG, "Free DRAM after NimBLE: %u", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  NimBLEDevice::init("SolarFan");
  NimBLEDevice::setPower(8);

  // NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND);   // Jen bonding, bez MITM
  // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // Just Works režim

  // NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  // NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

  /** Set the callbacks to call when scan events occur, no duplicates */
  NimBLEScan *pScan = NimBLEDevice::getScan();

  /** Set the callbacks to call when scan events occur, no duplicates */
  pScan->setScanCallbacks(&scanCallbacks, false);

  /**
   * Active scan will gather scan response data from advertisers
   *  but will use more energy from both devices
   */
  pScan->setActiveScan(true);
  pScan->setMaxResults(BLE_SCAN_MAX_RESULTS);

  /** Start scanning for advertisers */
  pScan->start(scanTimeMs, true, true);
  ESP_LOGI(TAG, "Scanning for peripherals");
}

void loop()
{
  delay(20); // Krátké zpoždění pro stabilitu

  // Volání buttonLoop() pro zpracování stisku tlačítka
  buttonLoop();

  // BLE scanning and connection to sensors named "LYWSD03MMC"
  // (Non-blocking scan and connection handled by MyAdvertisedDeviceCallbacks)

  static unsigned long lastDrawTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDrawTime >= 1000)
  {
    measure();
    drawGUI();
    lastDrawTime = currentTime;
  }

  if (doConnect)
  {
    doConnect = false;
    /** Found a device we want to connect to, do it now */
    if (connectToBTServer())
    {
      BLEConnected = true;

      ESP_LOGI(TAG, "Success! we should now be getting notifications!");
    }
    else
    {
      BLEConnected = false;

      ESP_LOGI(TAG, "Failed to connect, starting scan");
      NimBLEDevice::getScan()->start(scanTimeMs, true, true);
    }
  }
}

void measure()
{
  // Battery measurements
  if (!ina_inicialized)
  {
    // ESP_LOGE(TAG, "INA228 devices not initialized");
    return; // Pokud INA228 není inicializován, ukončíme měření
  }

  bat_shuntVoltage = ina228_bat.readShuntVoltage();
  bat_busVoltage = ina228_bat.readBusVoltage() / 1000000.0;
  bat_current = ina228_bat.readCurrent();
  bat_power = ina228_bat.readPower();

  solar_shuntVoltage = ina228_solar.readShuntVoltage();
  solar_busVoltage = ina228_solar.readBusVoltage() / 1000000.0;
  solar_current = ina228_solar.readCurrent();
  solar_power = ina228_solar.readPower();

  // Table header
  ESP_LOGV(TAG, "Battery & Solar Measurements Table");
  ESP_LOGV(TAG, "+---------------+-------------+-------------+");
  ESP_LOGV(TAG, "| Parameter     | Battery     | Solar       |");
  ESP_LOGV(TAG, "+---------------+-------------+-------------+");
  // ESP_LOGV(TAG, "| Shunt Voltage | %8.2f mV | %8.2f mV |", bat_shuntVoltage, solar_shuntVoltage);
  ESP_LOGV(TAG, "| Bus Voltage   | %8.2f V  | %8.2f V  |", bat_busVoltage, solar_busVoltage);
  // ESP_LOGV(TAG, "| Current       | %8.2f mA | %8.2f mA |", bat_current, solar_current);
  ESP_LOGV(TAG, "| Power         | %8.1f mW | %8.1f mW |", bat_power, solar_power);
  ESP_LOGV(TAG, "+---------------+-------------+-------------+");
}

// Funkce pro zpracování stisku tlačítka
void buttonLoop()
{
  if (digitalRead(BTN1) == LOW)
  { // Tlačítko stisknuto (aktivní LOW)
    if (!buttonPressed)
    {
      buttonPressed = true;
      buttonPressStartTime = millis();
    }
    // Pokud držíte tlačítko déle než 2 sekundy a ještě nebyl detekován dlouhý stisk
    else if (!longButtonPressed && (millis() - buttonPressStartTime > 2000))
    {
      longButtonPressed = true;
      longPressed(); // Zavoláme funkci pro dlouhý stisk
    }
  }
  else
  { // Tlačítko uvolněno
    if (buttonPressed)
    {
      if (!longButtonPressed)
      {
        shortPressed(); // Zavoláme funkci pro krátký stisk
      }
      // Resetujeme stav tlačítka
      buttonPressed = false;
      longButtonPressed = false;
    }
  }
}

// Funkce pro krátký stisk tlačítka
void shortPressed()
{
  ESP_LOGI(TAG, "Short button press");

  // Změna úhlu rotace displeje
  rotationAngle = fmod(rotationAngle + 15, 360.0f);
  preferences.putFloat("rotation", rotationAngle); // Uložení nového úhlu
}

// Funkce pro dlouhý stisk tlačítka
void longPressed()
{
  ESP_LOGI(TAG, "Long button press");

  if (updateStarted)
  {
    ESP.restart(); // Restart zařízení, pokud je aktualizace spuštěna
    return;
  }
  preferences.end(); // Uzavření NVS Preferences

  setupWiFiClient(); // Spuštění WiFi klienta a OTA serveru

  updateStarted = true; // Nastavení příznaku, že aktualizace začala
}

// Funkce pro kreslení grafického uživatelského rozhraní
void drawGUI()
{
  canvas.fillSprite(BLACK); // Vyplnění canvasu černou barvou
  // Vykreslení kruhu pro ukazatel rychlosti
  int centerX = canvas.width() / 2;
  int centerY = canvas.height() / 2;

  if (updateStarted)
  {
    // Update mode display
    canvas.setTextSize(0.6);
    canvas.setTextDatum(middle_center);
    canvas.drawString("Updating", centerX, centerY - 11);

    // if (WiFi.status() == WL_CONNECTED)
    // {
    //   IPAddress IP = WiFi.localIP();
    //   canvas.setTextSize(0.4);
    //   canvas.drawString(IP.toString().c_str(), centerX, centerY + 11);
    // }
  }
  else
  {
    // Nastavení pro canvas (jak bylo poskytnuto)
    // --- Nastavení pro canvas ---
    canvas.setFont(&fonts::Font4); // Nastavení písma (Font4 je dobrá volba pro čitelnost)
    canvas.setTextSize(0.6);       // Nastavení velikosti textu pro všechny výpisy
    // canvas.setTextDatum(middle_center); // Zarovnání textu na střed (horizontálně i vertikálně)

    int x = 10;           // Středová osa X displeje
    int y = 10;           // Počáteční pozice Y pro střed prvního řádku textu
    int lineSpacing = 17; // Mezera mezi středy řádků textu (upraveno pro velikost písma 0.6)

    // --- Vykreslení informací na displej ---
    // 2. Sekce Solárního panelu - zkrácené popisky
    canvas.drawString("Solar", x, y); // Zkrácený název
    if (mqttConnected)
    {
      canvas.drawString("Mqtt 1", x + 60, y);
    }
    else
    {
      canvas.drawString("Mqtt 0", x + 60, y);
    }
    y += lineSpacing; // Mezera před další sekcí

    canvas.drawString(String(solar_busVoltage, 2) + "  V", x, y);
    canvas.drawString(String(solar_power, 0) + "  mW", x + 50, y);
    y += lineSpacing; // Mezera před další sekcí

    // 3. Sekce Baterie - zkrácené popisky
    canvas.drawString("Bat", x, y); // Zkrácený název
    y += lineSpacing;               // Mezera před další sekcí

    canvas.drawString(String(bat_busVoltage, 2) + "  V", x, y);
    canvas.drawString(String(bat_current * bat_busVoltage, 0) + "  mW", x + 50, y);

    y += lineSpacing; // Mezera před další sekcí
    // 4. Sekce BLE Senzor - zkrácené popisky
    canvas.drawString("BLE", x, y); // Zkrácený název
    y += lineSpacing;               // Mezera před další sekcí
    canvas.drawString(String(BLE_temperature, 2) + "  °C", x, y);
    canvas.drawString(String(BLE_humidity, 1) + "  %", x + 60, y);
    y += lineSpacing; // Mezera před další sekcí
    canvas.drawString(String(BLE_voltage, 3) + "  V", x, y);
  }

  // Otočení a zobrazení canvasu na displeji
  canvas.pushRotated(rotationAngle);
}

// void notFound(AsyncWebServerRequest *request)
// {
//   request->send(404, "text/plain", "Not found");
// }

// Funkce pro nastavení WiFi klienta a OTA serveru
void setupWiFiClient()
{
  // Připojení k WiFi síti
  // WiFi.mode(WIFI_STA);
  // // Nahraď SSID a heslo správnými údaji!
  // WiFi.begin("Vivien", "Bionicman123");

  // ESP_LOGI(TAG, "Connecting to WiFi...");

  // // Čekání na připojení
  // int timeout = 20; // Maximální čas připojení (10 sekund)
  // while (WiFi.status() != WL_CONNECTED && timeout > 0)
  // {
  //   delay(500);
  //   ESP_LOGI(TAG, ".");
  //   timeout--;
  // }

  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   ESP_LOGI(TAG, "Connected to WiFi!");
  //   IPAddress IP = WiFi.localIP();
  //   ESP_LOGI(TAG, "Client IP address: %s", IP.toString().c_str());

  //   // Nastavení webového serveru
  //   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  //             {
  //     String html = "<html><body>";
  //     html += "<h1>FanSpeed</h1>";
  //     html += "<p><a href='/update'>Update firmware</a> (login: admin, password: admin)</p>";
  //     html += "</body></html>";
  //     request->send(200, "text/html", html); });

  //   server.onNotFound(notFound);

  //   // Nastavení OTA aktualizačního serveru s přihlašovacími údaji
  //   updateServer.setup(&server, "admin", "admin");
  //   server.begin(); // Spuštění webového serveru
  // }
  // else
  // {
  //   ESP_LOGI(TAG, "Failed to connect to WiFi!");
  // }
}

void gsmSetup()
{
  ESP_LOGI(TAG, "Starting GSM modem Serial...");
  // Serial1.begin(9600, SERIAL_8N1, 39, 38); // Inicializace sériového portu pro modem
  Serial2.end();
  Serial2.setPins(38, 39, -1); // Nastavení pinů pro RX, TX, RST (pokud není potřeba, použijte -1)
  delay(100);

  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!
  // Set GSM module baud rate
  TinyGsmAutoBaud(Serial2, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  vTaskDelay(6000 / portTICK_PERIOD_MS);

  // Quick sanity check – does the modem answer "AT"?
  ESP_LOGI(TAG, "Testing basic AT command...");
  if (modem.testAT())
  {
    ESP_LOGI(TAG, "Modem replied OK to AT");
  }
  else
  {
    ESP_LOGE(TAG, "No response to AT command – check wiring or power");
    return; // Abort further setup
  }

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  ESP_LOGI(TAG, "Initializing modem (no full restart)...");
  modem.init(); // soft‑init without reboot; faster if RST pin is not wired

  String modemInfo = modem.getModemInfo();
  ESP_LOGI(TAG, "Modem Info: %s", modemInfo.c_str());

  ESP_LOGI(TAG, "Waiting for network...");
  bool networkOk = false;
  for (int attempt = 1; attempt <= 3 && !networkOk; ++attempt)
  {
    networkOk = modem.waitForNetwork(60000L);
    if (!networkOk)
    {
      ESP_LOGI(TAG, "Network not found (attempt %d/3)", attempt);
      delay(5000);
    }
  }
  if (!networkOk)
  {
    ESP_LOGE(TAG, "Unable to register to network");
    return;
  }
  ESP_LOGI(TAG, "success");

  if (modem.isNetworkConnected())
  {
    ESP_LOGI(TAG, "Network connected");
  }

  // GPRS connection parameters are usually set after network registration
  ESP_LOGI(TAG, "Connecting to %s", apn);
  bool gprsOk = false;
  for (int attempt = 1; attempt <= 3 && !gprsOk; ++attempt)
  {
    gprsOk = modem.gprsConnect(apn, gprsUser, gprsPass);
    if (!gprsOk)
    {
      ESP_LOGI(TAG, "GPRS attach failed (attempt %d/3)", attempt);
      delay(5000);
    }
  }
  if (!gprsOk)
  {
    ESP_LOGE(TAG, "Unable to attach to GPRS");
    return;
  }
  ESP_LOGI(TAG, "success");

  if (modem.isGprsConnected())
  {
    ESP_LOGI(TAG, "GPRS connected");
  }

  // MQTT Broker setup
  mqtt.setServer(broker, mqttPort);
  mqtt.setCallback(mqttCallback);

  mqttConnect();
}

void gsmLoop()
{
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected())
  {
    ESP_LOGI(TAG, "Network disconnected");
    if (!modem.waitForNetwork(180000L, true))
    {
      ESP_LOGI(TAG, " fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected())
    {
      ESP_LOGI(TAG, "Network re-connected");
    }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected())
    {
      ESP_LOGI(TAG, "GPRS disconnected!");
      ESP_LOGI(TAG, "Connecting to %s", apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass))
      {
        ESP_LOGI(TAG, " fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected())
      {
        ESP_LOGI(TAG, "GPRS reconnected");
      }
    }
  }

  if (!mqtt.connected())
  {
    ESP_LOGI(TAG, "=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    mqttConnected = false;

    if (t - lastReconnectAttempt > 10000L)
    {
      lastReconnectAttempt = t;
      if (mqttConnect())
      {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  else
  {
    mqttConnected = true;
  }

  mqtt.loop();
}

void gsmTask(void *pvParameters)
{
  gsmSetup(); // one‑time init
  for (;;)
  {
    gsmLoop();                           // keep GSM & MQTT alive
    vTaskDelay(10 / portTICK_PERIOD_MS); // yield to other tasks
  }
}
