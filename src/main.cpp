#ifndef ATOMS3
#define ATOMS3
#endif

#include <Arduino.h>
#include "esp_log.h"
#define TAG "MAIN"

#include "M5GFX.h"
#include "M5Unified.h"

M5Canvas canvas(&M5.Display);

#define BTN1 41

#include <Preferences.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncHTTPUpdateServer.h>
ESPAsyncHTTPUpdateServer updateServer;
AsyncWebServer server(80);

void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

// Target device name
// #define TARGET_DEVICE_NAME "SIGMA SPEED 17197"
#define TARGET_DEVICE_NAME "SIGMA SPEED"

// UUID of CSC service and measurement characteristic
#define CSC_SERVICE_UUID "1816"
#define CSC_CHAR_UUID "2A5B"

// Scan duration (0 = continuous)
#define SCAN_TIME_MS 0

const float WHEEL_CIRCUMFERENCE = 2.146; // [m]

// Global variables for storing previous measurement (for speed calculation)
uint32_t previousCumulativeRevs = 0;
uint16_t previousLastEventTime = 0;
bool firstMeasurement = true;

void setupWiFiClient();
void buttonLoop();
void shortPressed();
void longPressed();

// PWM definitions
#define PWM_GPIO 38
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 10000 // 10 kHz
#define PWM_RESOLUTION 10   // 10bit (0-1023)

#ifndef ATOMS3
// WS2812 LED definitions
#define LED_PIN 35
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
#endif

Preferences preferences;

// Speed limits
#define MIN_SPEED 3.0f  // km/h, below this value PWM = 0%
#define MAX_SPEED 35.0f // km/h, at this value PWM = 100%
// Global variables
static bool doConnect = false;

#ifdef ATOMS3
static unsigned long buttonPressStartTime = 0;
static bool buttonPressed = false;
static bool longButtonPressed = false;
static bool updateStarted = false;
static float rotationAngle = 0.0f;
#endif

void drawGUI();

void setup()
{
  preferences.begin("display", false);
  rotationAngle = preferences.getFloat("rotation", 0.0f);

  M5.begin();
  M5.Display.begin();

  M5.Display.setBrightness(35);

  Serial.begin(115200);
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for serial monitor connection

  canvas.createSprite(M5.Display.width(), M5.Display.height());
  canvas.setTextColor(WHITE);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_GPIO, PWM_CHANNEL);

  pinMode(BTN1, INPUT_PULLUP);

  static TimerHandle_t guiTimer = NULL;
  if (guiTimer == NULL)
  {
    guiTimer = xTimerCreate(
        "GUITimer",
        pdMS_TO_TICKS(250),
        pdTRUE, // Auto reload
        nullptr,
        [](TimerHandle_t xTimer)
        {
          drawGUI();
        });
    xTimerStart(guiTimer, 0);
  }
}

void loop()
{
  delay(10);

  buttonLoop();
}

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

void shortPressed()
{
  ESP_LOGI(TAG, "Short button press");

  rotationAngle = fmod(rotationAngle + 15, 360.0f);

  preferences.putFloat("rotation", rotationAngle);
}

void longPressed()
{
  ESP_LOGI(TAG, "Long button press");

  if (updateStarted)
  {
    ESP.restart();
    return;
  }
  preferences.end();

  setupWiFiClient();

  updateStarted = true;
}

void drawGUI()
{
  canvas.fillSprite(BLACK);
  // Draw speed gauge circle
  int centerX = canvas.width() / 2;
  int centerY = canvas.height() / 2;
  int radius = min(centerX, centerY) - 5;

  if (updateStarted)
  {
    canvas.fillArc(centerX, centerY, 59, 90, 0, 360, GREEN);
    canvas.setTextSize(0.6);
    canvas.setTextDatum(middle_center);
    canvas.drawString("Updating", centerX, centerY - 11);

    if (WiFi.status() == WL_CONNECTED)
    {
      IPAddress IP = WiFi.localIP();

      // Vypsání IP na displej (M5Canvas)
#ifdef ATOMS3
      canvas.setTextSize(0.4);
      canvas.setTextDatum(middle_center);
      canvas.drawString(IP.toString().c_str(), centerX, centerY + 11);
#endif
    }
  }
  else
  {
    // Draw background circle
    // canvas.drawCircle(centerX, centerY, radius, WHITE);

    float comp_speed = 0;

    // Calculate angle based on current speed (0-270 degrees)
    float angle = (comp_speed / MAX_SPEED) * 270.0f;
    // angle = 270;
    // Draw filled arc from -45 to current angle
    canvas.fillArc(centerX, centerY, radius - 17, radius + 5,
                   135,         // Start at -45 degrees (225 in fillArc coordinates)
                   135 + angle, // End at calculated angle
                   RED);

    canvas.setFont(&fonts::Font7);
    canvas.setTextSize(1);
    canvas.setTextDatum(middle_center);
    canvas.drawString(String(12, 0), centerX, centerY);
    // canvas.drawString("45", centerX, centerY);

    canvas.setFont(&fonts::FreeSans18pt7b);
    canvas.setTextSize(0.5);
    canvas.drawString(String(12, 0), centerX, 107);
    // canvas.drawString("100", centerX - 1, 107);
  }

  // canvas.pushSprite(0, 0);
  canvas.pushRotated(rotationAngle);
  // canvas.pushSprite(0, 0);
}

void setupWiFiClient()
{
  // Připojení k WiFi síti
  WiFi.mode(WIFI_STA);
  WiFi.begin("Vivien", "Bionicman123"); // Nahraď SSID a heslo správnými údaji

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

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      String html = "<html><body>";
      html += "<h1>FanSpeed</h1>";
      html += "<p><a href='/update'>Update firmware</a> (login: admin, password: admin)</p>";
      html += "</body></html>";
      request->send(200, "text/html", html); });

    server.onNotFound(notFound);

    // setup the updateServer with credentials
    updateServer.setup(&server, "admin", "admin");
    server.begin();
  }
  else
  {
    ESP_LOGI(TAG, "Failed to connect to WiFi!");
  }
}