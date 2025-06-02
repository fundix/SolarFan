#include <Arduino.h>
#include "esp_log.h"
#include "Wire.h" // Pro I2C komunikaci
#define TAG "MAIN"

#include <M5AtomS3.h>  // Specifická knihovna pro M5AtomS3
#include "M5GFX.h"     // Grafická knihovna pro displej
#include "M5Unified.h" // Sjednocená knihovna M5Stack

// Check if PSRAM is available and initialize canvas with PSRAM
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

bool connectToServer()
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
          }
        });

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

void measure();
// Funkce pro obsluhu nenalezených stránek webového serveru

// Deklarace funkcí
void setupWiFiClient();
void buttonLoop();
void shortPressed();
void longPressed();

// PWM definice
#define PWM_GPIO 38         // GPIO pro PWM výstup
#define PWM_CHANNEL 0       // PWM kanál
#define PWM_FREQUENCY 10000 // Frekvence PWM (10 kHz)
#define PWM_RESOLUTION 10   // Rozlišení PWM (10bit, 0-1023)

Preferences preferences; // Instance pro NVS Preferences

// Globální proměnné
static unsigned long buttonPressStartTime = 0;
static bool buttonPressed = false;
static bool longButtonPressed = false;
static bool updateStarted = false;
static float rotationAngle = 0.0f; // Úhel rotace pro displej

void drawGUI(); // Deklarace funkce pro kreslení GUI

void setup()
{
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

  // Vytvoření FreeRTOS timeru pro pravidelné volání drawGUI()
  // static TimerHandle_t guiTimer = NULL;
  // if (guiTimer == NULL)
  // {
  //   guiTimer = xTimerCreate(
  //       "GUITimer",
  //       pdMS_TO_TICKS(250), // Volat každých 250 ms
  //       pdTRUE,             // Auto reload (opakovaně)
  //       nullptr,
  //       [](TimerHandle_t xTimer) // Lambda funkce volaná timerem
  //       {
  //         drawGUI(); // Kreslení GUI
  //       });
  //   xTimerStart(guiTimer, 0); // Spuštění timeru
  // }

  // Inicializace I2C sběrnice (Wire) na pinech 2 a 1.
  // POZNÁMKA: M5AtomS3 má obvykle interní I2C na GPIO 38/39 pro IMU.
  // Wire.begin(2, 1) inicializuje Wire (I2C0) na těchto pinech.
  // Pokud používáš externí I2C zařízení na jiných pinech, zkontroluj dokumentaci.
  Wire.begin();

  // I2C scan
  // Serial.println("I2C scan...");
  // for (int address = 1; address < 127; address++)
  // {
  //   Wire.beginTransmission(address);
  //   int error = Wire.endTransmission();
  //   if (error == 0)
  //   {
  //     Serial.printf("I2C device found at 0x%02X\n", address);
  //   }
  // }
  // Serial.println("I2C scan done");

  if (ina228_bat.begin(bat_addr) && ina228_solar.begin(solar_addr))
  {
    ina_inicialized = true; // Pokud se podařilo inicializovat oba INA228, nastavíme příznak
    ESP_LOGI(TAG, "INA228 devices initialized successfully");

    ina228_bat.setShunt(0.015, 10.0);
    ina228_bat.setAveragingCount(INA228_COUNT_64);
    ina228_bat.setVoltageConversionTime(INA228_TIME_540_us);
    ina228_bat.setCurrentConversionTime(INA228_TIME_280_us);
  }
  else
  {
    ESP_LOGE(TAG, "Failed to initialize INA228 devices");
  }

  ble_setup();
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
  delay(10); // Krátké zpoždění pro stabilitu

  // Zde se objevoval konflikt s M5GFX knihovnou.
  // I2C sken by neměl běžet neustále v loop() s aktivním GUI timerem.
  // Prozatím je zakomentován. Pokud ho potřebuješ, zvaž spouštění na událost
  // (např. stisk tlačítka) nebo v samostatném FreeRTOS tasku se synchronizací.

  /*
  int address;
  int error;
  M5.Display.printf("\nscanning Address [HEX]\n"); // Používáme M5.Display pro konzistenci
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      M5.Display.print(address, HEX);
      M5.Display.print(" ");
    }
    else
      M5.Display.print(".");

    delay(10);
  }
  delay(1000);
  M5.Display.setCursor(1, 12);
  M5.Display.fillRect(1, 15, 128, 128, BLACK);
  */

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
    if (connectToServer())
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
  ESP_LOGI(TAG, "Battery & Solar Measurements Table");
  ESP_LOGI(TAG, "+---------------+-------------+-------------+");
  ESP_LOGI(TAG, "| Parameter     | Battery     | Solar       |");
  ESP_LOGI(TAG, "+---------------+-------------+-------------+");
  // ESP_LOGI(TAG, "| Shunt Voltage | %8.2f mV | %8.2f mV |", bat_shuntVoltage, solar_shuntVoltage);
  ESP_LOGI(TAG, "| Bus Voltage   | %8.2f V  | %8.2f V  |", bat_busVoltage, solar_busVoltage);
  // ESP_LOGI(TAG, "| Current       | %8.2f mA | %8.2f mA |", bat_current, solar_current);
  ESP_LOGI(TAG, "| Power         | %8.2f mW | %8.2f mW |", bat_power, solar_power);
  ESP_LOGI(TAG, "+---------------+-------------+-------------+");
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
    y += lineSpacing;                 // Mezera před další sekcí

    canvas.drawString(String(solar_busVoltage, 2) + "  V", x, y);
    canvas.drawString(String(solar_power, 0) + "  mW", x + 60, y);
    y += lineSpacing; // Mezera před další sekcí

    // 3. Sekce Baterie - zkrácené popisky
    canvas.drawString("Bat", x, y); // Zkrácený název
    y += lineSpacing;               // Mezera před další sekcí

    canvas.drawString(String(bat_busVoltage, 2) + "  V", x, y);
    canvas.drawString(String(bat_power, 0) + "  mW", x + 60, y);
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