#include <Arduino.h>
#include "esp_log.h"
#include "Wire.h" // Pro I2C komunikaci
#define TAG "MAIN"

#include <M5AtomS3.h>  // Specifická knihovna pro M5AtomS3
#include "M5GFX.h"     // Grafická knihovna pro displej
#include "M5Unified.h" // Sjednocená knihovna M5Stack

M5Canvas canvas(&M5.Display); // Vytvoření canvasu pro kreslení na displej

#define BTN1 41 // Definice pinu pro tlačítko (GPIO 41)

#include <Preferences.h> // Pro ukládání dat do NVS (Non-Volatile Storage)

#include <WiFi.h>                     // Pro WiFi připojení
#include <AsyncTCP.h>                 // Asynchronní TCP knihovna
#include <ESPAsyncWebServer.h>        // Asynchronní webový server
#include <ESPAsyncHTTPUpdateServer.h> // Pro OTA aktualizace firmwaru
ESPAsyncHTTPUpdateServer updateServer;
AsyncWebServer server(80); // Webový server na portu 80

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

#include <BLEDevice.h>
BLEClient *pClient;
BLEScan *pBLEScan;
#define SCAN_TIME 0              // seconds
volatile bool connected = false; // Updated to volatile for use across callbacks
#undef CONFIG_BTC_TASK_STACK_SIZE
#define CONFIG_BTC_TASK_STACK_SIZE 37888
float BLE_temperature = 0.0f; // Teplota z BLE senzoru
float BLE_humidity = 0.0f;    // Vlhkost z BLE senzoru
float BLE_voltage = 0.0f;     // Napětí z BLE senzoru (pokud je potřeba)
void connectSensor(BLEAddress htSensorAddress);
void registerNotification();

// std::string addresses[10];
// int addressCount = 0;

// The remote service we wish to connect to.
static BLEUUID serviceUUID("ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6");

void measure();
// Funkce pro obsluhu nenalezených stránek webového serveru
void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

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

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
    connected = true;
    ESP_LOGI(TAG, " * Connected %s", pclient->getPeerAddress().toString().c_str());
    registerNotification(); // Register BLE notifications now that the connection callback confirms link
  }

  void onDisconnect(BLEClient *pclient)
  {
    connected = false;
    pBLEScan->start(SCAN_TIME, false); // Restart scan after disconnection
    ESP_LOGI(TAG, " * Disconnected %s", pclient->getPeerAddress().toString().c_str());
  }
};

// Callback for handling discovered BLE devices in a non-blocking scan
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice) override
  {
    if (advertisedDevice.haveName())
    {
      ESP_LOGI(TAG, "Advertised Device: %s", advertisedDevice.toString().c_str());
    }
    if (advertisedDevice.haveName() && advertisedDevice.getName() == "LYWSD03MMC")
    {
      BLEAddress sensorAddress = advertisedDevice.getAddress();
      ESP_LOGI(TAG, "+ Connecting to sensor: %s", sensorAddress.toString().c_str());
      connectSensor(sensorAddress);
      // After connecting and registering for notifications,
      // the notification callback will handle incoming data.
    }
  }
};

static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{
  // float temp;
  // float humi;
  // float voltage;
  ESP_LOGI(TAG, " + Notify callback for characteristic %s", pBLERemoteCharacteristic->getUUID().toString().c_str());
  BLE_temperature = (pData[0] | (pData[1] << 8)) * 0.01; // little endian
  BLE_humidity = pData[2];
  BLE_voltage = (pData[3] | (pData[4] << 8)) * 0.001; // little endian
  ESP_LOGI(TAG, "temp = %.1f C ; humidity = %.1f %% ; voltage = %.3f V", BLE_temperature, BLE_humidity, BLE_voltage);
  // pClient->disconnect();
}

void registerNotification()
{
  if (!connected)
  {
    ESP_LOGI(TAG, " - Premature disconnection");
    return;
  }
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    ESP_LOGI(TAG, " - Failed to find our service UUID: ");
    ESP_LOGI(TAG, " - Failed to find our service UUID: %s", serviceUUID.toString().c_str());
    pClient->disconnect();
  }
  ESP_LOGI(TAG, " + Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  BLERemoteCharacteristic *pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    ESP_LOGI(TAG, " - Failed to find our characteristic UUID: ");
    ESP_LOGI(TAG, " - Failed to find our characteristic UUID: %s", charUUID.toString().c_str());
    pClient->disconnect();
  }
  ESP_LOGI(TAG, " + Found our characteristic");
  pRemoteCharacteristic->registerForNotify(notifyCallback);
}

void createBleClientWithCallbacks()
{
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
}

void connectSensor(BLEAddress htSensorAddress)
{
  ESP_LOGI(TAG, "Connecting (blocking) to %s", htSensorAddress.toString().c_str());
  pBLEScan = BLEDevice::getScan();
  pBLEScan->stop(); // Stop ongoing scan to free the controller before connecting
  vTaskDelay(50 / portTICK_PERIOD_MS); // krátká pauza

  // Always create a fresh BLEClient to avoid stale state
  if (pClient != nullptr && pClient->isConnected())
  {
    pClient->disconnect();
  }
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  // First try with RANDOM address because LYWSD03MMC uses a random static MAC
  bool success = pClient->connect(htSensorAddress, BLE_ADDR_TYPE_RANDOM, true); // 'true' = wait until complete
  if (!success)
  {
    ESP_LOGW(TAG, " - RANDOM address connect failed, retrying default");
    success = pClient->connect(htSensorAddress, BLE_ADDR_TYPE_PUBLIC, true);
  }

  if (!success)
  {
    ESP_LOGE(TAG, " - Connection failed in blocking mode");
    pBLEScan->start(SCAN_TIME, false); // Resume scanning
    return;
  }

  ESP_LOGI(TAG, " + Connected (blocking), registering notifications");
  connected = true;       // Make sure flag is set
  registerNotification(); // Now set up notifications immediately
}

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

  BLEDevice::init("ESP32");
  createBleClientWithCallbacks();

  pBLEScan = BLEDevice::getScan(); // create new scan
  // Use non-blocking scan with a callback
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), false);
  pBLEScan->setActiveScan(true); // active scan uses more power, but get results faster
  // pBLEScan->setInterval(0x50);
  // pBLEScan->setWindow(0x30);
  pBLEScan->start(SCAN_TIME, false); // Start non-blocking scan indefinitely
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

    if (WiFi.status() == WL_CONNECTED)
    {
      IPAddress IP = WiFi.localIP();
      canvas.setTextSize(0.4);
      canvas.drawString(IP.toString().c_str(), centerX, centerY + 11);
    }
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

// Funkce pro nastavení WiFi klienta a OTA serveru
void setupWiFiClient()
{
  // Připojení k WiFi síti
  WiFi.mode(WIFI_STA);
  // Nahraď SSID a heslo správnými údaji!
  WiFi.begin("Vivien", "Bionicman123");

  ESP_LOGI(TAG, "Connecting to WiFi...");

  // Čekání na připojení
  int timeout = 20; // Maximální čas připojení (10 sekund)
  while (WiFi.status() != WL_CONNECTED && timeout > 0)
  {
    delay(500);
    ESP_LOGI(TAG, ".");
    timeout--;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    ESP_LOGI(TAG, "Connected to WiFi!");
    IPAddress IP = WiFi.localIP();
    ESP_LOGI(TAG, "Client IP address: %s", IP.toString().c_str());

    // Nastavení webového serveru
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      String html = "<html><body>";
      html += "<h1>FanSpeed</h1>";
      html += "<p><a href='/update'>Update firmware</a> (login: admin, password: admin)</p>";
      html += "</body></html>";
      request->send(200, "text/html", html); });

    server.onNotFound(notFound);

    // Nastavení OTA aktualizačního serveru s přihlašovacími údaji
    updateServer.setup(&server, "admin", "admin");
    server.begin(); // Spuštění webového serveru
  }
  else
  {
    ESP_LOGI(TAG, "Failed to connect to WiFi!");
  }
}