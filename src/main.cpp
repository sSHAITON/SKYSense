#include <Arduino.h>
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "time.h"
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200; // GMT+7 (dalam detik) untuk Indonesia
const int daylightOffset_sec = 0;
char timeStringBuff[50];

static const char *TAG = "SENSOR_APP";

// Tambahkan di bagian global
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule()
{
  ets_printf("Watchdog trigger reset\n");
  esp_restart();
}

// Konfigurasi Firebase
#define API_KEY "AIzaSyDMupFZksIeuDGcQr1K5YI56Y3Cn1fcvT0"
#define DATABASE_URL "https://projeksister-default-rtdb.asia-southeast1.firebasedatabase.app"

// Konfigurasi WiFi
#define WIFI_SSID "Hannya"
#define WIFI_PASSWORD "qwertyuiop"

// Konfigurasi OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Konfigurasi DHT
#define DHT_PIN 5
#define DHT_TYPE DHT22

// Konfigurasi Rain Sensor
#define RAIN_PIN 4
#define MILLIMETERS_PER_TIP 0.70

// Variabel untuk anemometer
volatile byte rpmcount = 0;
float rpm, rps;
float velocity_kmh, velocity_ms;
int GPIO_pulse = 23;           // Pin anemometer
const float timemeasure = 2.0; // Interval pengukuran dalam detik
unsigned long timeold = 0;
unsigned long last_micros = 0;

// Variabel untuk data sensor
float temperature = 0;
float humidity = 0;
float curah_hujan = 0.00;
float uvIndex = 0.0;
float pressure_hPa = 0;
float altitude = 0;

// Variabel untuk sensor hujan
volatile boolean rainFlag = false;
long int rainTipCount = 0;
float rainfall = 0.00;
const unsigned long RAIN_RESET_INTERVAL = 3600000; // 1 hour in milliseconds
unsigned long lastRainTime = 0;

// Objek Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;
bool signupOK = false;

// Objek lainnya
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP280 bmp;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int currentSlide = 0;

// Task Handles untuk FreeRTOS
TaskHandle_t TaskReadSensorsHandle;
TaskHandle_t TaskSendFirebaseHandle;
TaskHandle_t TaskUpdateDisplayHandle;
TaskHandle_t TaskReadBMPHandle;

// Semaphore dan Mutex
SemaphoreHandle_t xAnemometerSemaphore;
SemaphoreHandle_t xRainSemaphore;
SemaphoreHandle_t xMutex;

// Deklarasi fungsi
void IRAM_ATTR rpm_anemometer();
void IRAM_ATTR rainInterrupt();
void readTempHumidity();
void measureWindSpeed();
void processRainfall();
String fuzzyLogic(float temp, float hum, String &tempCategory, String &humCategory);
void sendToRealtimeDB(float temp, float hum, float windSpeed, float rainfall, float pressure_hPa, float uvIndex);
void updateDisplay(int slide);

// FreeRTOS Task
void TaskReadSensors(void *pvParameters);
void TaskSendFirebase(void *pvParameters);
void TaskUpdateDisplay(void *pvParameters);

const unsigned long DEBOUNCE_DELAY = 500; // 100ms debounce time
volatile unsigned long lastRainInterruptTime = 0;

// Deklarasi fungsi tambahan untuk BMP280
void readBMPSensor();
void TaskReadBMP(void *pvParameters);

// Add user id and device id
#define USER_ID "2"
#define DEVICE_ID "2A"

// Add near top with other defines
#define FIREBASE_UPDATE_INTERVAL 60000  // 1 minute in milliseconds
#define FIREBASE_HISTORY_INTERVAL 60000 // 1 minute in milliseconds
#define FIREBASE_LATEST_INTERVAL 5000   // 5 seconds in milliseconds

// Add this near the top with other defines
#define DEBUG_INTERVAL 10000 // Increase debug interval to 10 seconds

// Add near the top with other defines
#define DEVICE_STATUS_INTERVAL 30000 // Update device status every 30 seconds

// Add these with other global variables
unsigned long lastStatusUpdate = 0;
bool deviceConnected = false;

// Add these with other global variables
unsigned long lastFuzzyDebugTime = 0;
const unsigned long FUZZY_DEBUG_INTERVAL = 5000; // Print every 5 seconds

void setup()
{
  Serial.begin(115200);

  pinMode(32, INPUT); // Set GPIO 36 sebagai input untuk sensor GUVA-S12SD

  // Konfigurasi Watchdog Timer
  timer = timerBegin(0, 80, true); // Timer 0, preskaler 80
  timerAttachInterrupt(timer, &resetModule, true);
  timerAlarmWrite(timer, 8000000, false); // 8 detik
  timerAlarmEnable(timer);

  // Inisialisasi semaphore dan mutex
  xAnemometerSemaphore = xSemaphoreCreateCounting(100, 0);
  xRainSemaphore = xSemaphoreCreateCounting(100, 0);
  xMutex = xSemaphoreCreateMutex();

  if (xMutex == NULL || xAnemometerSemaphore == NULL || xRainSemaphore == NULL)
  {
    ESP_LOGE(TAG, "Failed to create semaphore or mutex!");
  }

  // Konfigurasi WiFi
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  // Inisialisasi BMP280
  if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor!"));
    while (1)
      ;
  }

  // Konfigurasi BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // Inisialisasi OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED tidak terdeteksi");
    while (true)
      ;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();

  // Inisialisasi DHT
  dht.begin();

  // Konfigurasi Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", ""))
  {
    Serial.println("Firebase signup successful");
    signupOK = true;
  }
  else
  {
    Serial.printf("Firebase signup failed: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Konfigurasi anemometer
  pinMode(GPIO_pulse, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING);

  // Konfigurasi sensor hujan
  pinMode(RAIN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainInterrupt, FALLING);

  // Task FreeRTOS
  xTaskCreatePinnedToCore(TaskReadSensors, "TaskReadSensors", 10000, NULL, 1, &TaskReadSensorsHandle, 0);
  xTaskCreatePinnedToCore(TaskSendFirebase, "TaskSendFirebase", 10000, NULL, 1, &TaskSendFirebaseHandle, 1);
  xTaskCreatePinnedToCore(TaskUpdateDisplay, "TaskUpdateDisplay", 10000, NULL, 1, &TaskUpdateDisplayHandle, 1);
  xTaskCreatePinnedToCore(TaskReadBMP, "TaskReadBMP", 10000, NULL, 1, &TaskReadBMPHandle, 0);

  // Inisialisasi NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Tunggu sampai waktu tersinkronisasi
  while (!time(nullptr))
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nTime synchronized with NTP!");

  deviceStartTime = millis(); // Record start time
}

// Fungsi untuk mendapatkan timestamp yang diformat
String getFormattedTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return String("Time not available");
  }

  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeStringBuff);
}

void readUVSensor()
{
  if (xSemaphoreTake(xMutex, portMAX_DELAY))
  {
    int sensorValue = analogRead(32);                   // Baca input analog dari GPIO 36
    float sensorVoltage = sensorValue * (3.3 / 4095.0); // Konversi nilai ke voltase
    uvIndex = sensorVoltage / 0.1;                      // Konversi voltase ke UV index (0.1V per UV index)

    // Tambahkan log untuk nilai sensorVoltage
    ESP_LOGE(TAG, "UV Sensor Voltage: %.2f V", sensorVoltage);
    ESP_LOGE(TAG, "UV Index: %.2f", uvIndex);

    xSemaphoreGive(xMutex);
  }
}

void loop()
{
  timerWrite(timer, 0);
}

void TaskReadBMP(void *pvParameters)
{
  while (true)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
      pressure_hPa = bmp.readPressure() / 100.0; // Convert Pa to hPa
      altitude = bmp.readAltitude(1013.25);      // Sesuaikan dengan tekanan permukaan laut lokal

      ESP_LOGI(TAG, "Pressure: %.2f hPa", pressure_hPa);
      ESP_LOGI(TAG, "Altitude: %.2f m", altitude);

      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void updateDisplay(int slide)
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  switch (slide)
  {
  case 0:
    display.setCursor(2, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Temperature (C):");
    display.setTextSize(2);
    display.println(temperature, 1);
    break;
  case 1:
    display.setCursor(2, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Humidity (%):");
    display.setTextSize(2);
    display.println(humidity, 1);
    break;
  case 2:
    display.setCursor(2, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Wind Speed (km/h):");
    display.setTextSize(2);
    display.println(velocity_kmh, 1);
    break;
  case 3:
    display.setCursor(2, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Rainfall (mm):");
    display.setTextSize(2);
    display.println(rainfall, 1);
    break;
  case 4:
    display.setCursor(0, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("UV Index:");
    display.setTextSize(2);
    display.println(uvIndex, 1);
    break;
  case 5:
    display.setCursor(0, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Pressure (hPa):");
    display.setTextSize(2);
    display.println(pressure_hPa, 0);
    break;
  case 6:
    display.setCursor(0, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Altitude (m):");
    display.setTextSize(2);
    display.println(altitude, 1);
    break;
  case 7:
    display.setCursor(0, 20);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Time:");
    display.setTextSize(1);
    display.println(getFormattedTime());
    break;
  }

  display.display();
}

float calculateMembership(float value, float start, float peak, float end)
{
  if (value <= start || value >= end)
    return 0;
  if (value == peak)
    return 1;
  return (value < peak) ? (value - start) / (peak - start) : (end - value) / (end - peak);
}

float calculateTriangularMembership(float value, float a, float b, float c)
{
  if (value <= a || value >= c)
    return 0;
  if (value == b)
    return 1;
  if (value < b)
    return (value - a) / (b - a);
  return (c - value) / (c - b);
}

// Fungsi untuk menghitung derajat keanggotaan trapesium
float calculateTrapezoidalMembership(float value, float a, float b, float c, float d)
{
  if (value <= a || value >= d)
    return 0;
  if (value >= b && value <= c)
    return 1;
  if (value < b)
    return (value - a) / (b - a);
  return (d - value) / (d - c);
}

struct WeatherCondition
{
  String condition;
  float certainty;
};

WeatherCondition fuzzyLogic(float temp, float hum, float windSpeed, float rainfall, float pressure, float uvIndex)
{
  // Temperature membership functions (°C)
  float tempDingin = (temp <= 20) ? 1 : (temp <= 25) ? (25 - temp) / 5
                                                     : 0;
  float tempNormal = (temp <= 20) ? 0 : (temp <= 25) ? (temp - 20) / 5
                                    : (temp <= 30)   ? (30 - temp) / 5
                                                     : 0;
  float tempPanas = (temp <= 25) ? 0 : (temp <= 30) ? (temp - 25) / 5
                                   : (temp <= 35)   ? 1
                                   : (temp <= 40)   ? (40 - temp) / 5
                                                    : 0;

  // Humidity membership functions (%)
  float humKering = (hum <= 30) ? 1 : (hum <= 50) ? (50 - hum) / 20
                                                  : 0;
  float humNormal = (hum <= 30) ? 0 : (hum <= 50) ? (hum - 30) / 20
                                  : (hum <= 70)   ? (70 - hum) / 20
                                                  : 0;
  float humLembab = (hum <= 50) ? 0 : (hum <= 70) ? (hum - 50) / 20
                                                  : 1;

  // Wind Speed membership functions (km/h) - Made more lenient
  float windTenang = (windSpeed <= 5) ? 1 : (windSpeed <= 15) ? (15 - windSpeed) / 10
                                                              : 0;
  float windSedang = (windSpeed <= 5) ? 0 : (windSpeed <= 15) ? (windSpeed - 5) / 10
                                        : (windSpeed <= 25)   ? (25 - windSpeed) / 10
                                                              : 0;
  float windKencang = (windSpeed <= 15) ? 0 : (windSpeed <= 25) ? (windSpeed - 15) / 10
                                                                : 1;

  // Rules evaluation with default fallback
  std::vector<std::pair<float, String>> conditions;

  // Basic condition rules with more lenient thresholds
  conditions.push_back({min({tempNormal, humNormal}), "Clear"});
  conditions.push_back({min({tempNormal, humLembab}), "Cloudy"});
  conditions.push_back({min({humLembab, windTenang}), "Cloudy"});
  conditions.push_back({min({tempPanas, humKering}), "Hot"});
  conditions.push_back({min({humLembab, windKencang}), "Windy"});

  // Rain condition rules
  if (rainfall > 0)
  {
    conditions.push_back({min({humLembab, rainfall > 0 ? 1.0f : 0.0f}), "Light Rain"});
    conditions.push_back({min({humLembab, rainfall > 2 ? 1.0f : 0.0f}), "Moderate Rain"});
    conditions.push_back({min({humLembab, rainfall > 5 ? 1.0f : 0.0f}), "Heavy Rain"});
  }

  // Default condition with low certainty if no other conditions are met
  conditions.push_back({0.1, "Clear"});

  float maxCertainty = 0;
  String finalCondition = "Clear";

  for (const auto &condition : conditions)
  {
    if (condition.first > maxCertainty)
    {
      maxCertainty = condition.first;
      finalCondition = condition.second;
    }
  }

  // If certainty is too low but we have clear indicators, use them
  if (maxCertainty < 0.1)
  {
    if (temp > 30)
    {
      finalCondition = "Hot";
      maxCertainty = 0.3;
    }
    else if (hum > 70)
    {
      finalCondition = "Cloudy";
      maxCertainty = 0.3;
    }
  }

  return {finalCondition, maxCertainty};
}

float min(std::initializer_list<float> values)
{
  float minVal = *values.begin();
  for (float val : values)
  {
    if (val < minVal)
      minVal = val;
  }
  return minVal;
}

struct RainPrediction
{
  String intensity;
  float certainty;
};

RainPrediction predictRain(float temp, float humidity, float pressure)
{
  static unsigned long lastDebugTime = 0;
  static uint32_t callCount = 0;
  unsigned long currentMillis = millis();
  callCount++;

  // Calculate memberships
  float tempCold = (temp <= 11) ? 1 : (temp <= 21) ? (21 - temp) / 10
                                                   : 0;
  float tempWarm1 = (temp <= 20) ? 0 : (temp <= 21) ? (temp - 20)
                                   : (temp <= 26)   ? 1
                                   : (temp <= 27)   ? (27 - temp)
                                                    : 0;
  float tempWarm2 = calculateTriangularMembership(temp, 26, 27, 28);
  float tempWarm3 = calculateTriangularMembership(temp, 27, 29, 31);
  float tempHot = (temp <= 31) ? 0 : (temp <= 32) ? (temp - 31)
                                 : (temp <= 40)   ? 1
                                                  : 0;

  float humDry = (humidity <= 30) ? 1 : (humidity <= 50) ? (50 - humidity) / 20
                                                         : 0;
  float humHumid1 = calculateTriangularMembership(humidity, 40, 55, 70);
  float humHumid2 = calculateTriangularMembership(humidity, 70, 75, 80);
  float humHumid3 = calculateTriangularMembership(humidity, 79, 84, 89);
  float humWet = calculateTriangularMembership(humidity, 88, 94, 100);

  float pressLow = (pressure <= 980) ? 1 : (pressure <= 1007) ? (1007 - pressure) / 27
                                                              : 0;
  float pressMedium = calculateTriangularMembership(pressure, 1006, 1007, 1008);
  float pressHigh = (pressure <= 1008) ? 0 : (pressure <= 1010) ? (pressure - 1008) / 2
                                         : (pressure <= 1014)   ? 1
                                                                : 0;

  std::vector<std::pair<float, String>> predictions;

  // All 75 rules implementation
  // Cold temperature (1-15)
  predictions.push_back({min({tempCold, humDry, pressLow}), "No Rain"});
  predictions.push_back({min({tempCold, humDry, pressMedium}), "No Rain"});
  predictions.push_back({min({tempCold, humDry, pressHigh}), "No Rain"});
  predictions.push_back({min({tempCold, humHumid1, pressLow}), "Light Rain"});
  predictions.push_back({min({tempCold, humHumid1, pressMedium}), "No Rain"});
  predictions.push_back({min({tempCold, humHumid1, pressHigh}), "No Rain"});
  predictions.push_back({min({tempCold, humHumid2, pressLow}), "Light Rain"});
  predictions.push_back({min({tempCold, humHumid2, pressMedium}), "Light Rain"});
  predictions.push_back({min({tempCold, humHumid2, pressHigh}), "Light Rain"});
  predictions.push_back({min({tempCold, humHumid3, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempCold, humHumid3, pressMedium}), "Moderate Rain"});
  predictions.push_back({min({tempCold, humHumid3, pressHigh}), "Light Rain"});
  predictions.push_back({min({tempCold, humWet, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempCold, humWet, pressMedium}), "Moderate Rain"});
  predictions.push_back({min({tempCold, humWet, pressHigh}), "Moderate Rain"});

  // Warm1 temperature (16-30)
  predictions.push_back({min({tempWarm1, humDry, pressLow}), "No Rain"});
  predictions.push_back({min({tempWarm1, humDry, pressMedium}), "No Rain"});
  predictions.push_back({min({tempWarm1, humDry, pressHigh}), "No Rain"});
  predictions.push_back({min({tempWarm1, humHumid1, pressLow}), "Light Rain"});
  predictions.push_back({min({tempWarm1, humHumid1, pressMedium}), "Light Rain"});
  predictions.push_back({min({tempWarm1, humHumid1, pressHigh}), "No Rain"});
  predictions.push_back({min({tempWarm1, humHumid2, pressLow}), "Moderate Rain"});
  predictions.push_back({min({tempWarm1, humHumid2, pressMedium}), "Light Rain"});
  predictions.push_back({min({tempWarm1, humHumid2, pressHigh}), "Light Rain"});
  predictions.push_back({min({tempWarm1, humHumid3, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempWarm1, humHumid3, pressMedium}), "Moderate Rain"});
  predictions.push_back({min({tempWarm1, humHumid3, pressHigh}), "Moderate Rain"});
  predictions.push_back({min({tempWarm1, humWet, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempWarm1, humWet, pressMedium}), "Heavy Rain"});
  predictions.push_back({min({tempWarm1, humWet, pressHigh}), "Moderate Rain"});

  // Warm2 temperature (31-45)
  predictions.push_back({min({tempWarm2, humDry, pressLow}), "No Rain"});
  predictions.push_back({min({tempWarm2, humDry, pressMedium}), "No Rain"});
  predictions.push_back({min({tempWarm2, humDry, pressHigh}), "No Rain"});
  predictions.push_back({min({tempWarm2, humHumid1, pressLow}), "Light Rain"});
  predictions.push_back({min({tempWarm2, humHumid1, pressMedium}), "Light Rain"});
  predictions.push_back({min({tempWarm2, humHumid1, pressHigh}), "No Rain"});
  predictions.push_back({min({tempWarm2, humHumid2, pressLow}), "Moderate Rain"});
  predictions.push_back({min({tempWarm2, humHumid2, pressMedium}), "Moderate Rain"});
  predictions.push_back({min({tempWarm2, humHumid2, pressHigh}), "Light Rain"});
  predictions.push_back({min({tempWarm2, humHumid3, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempWarm2, humHumid3, pressMedium}), "Heavy Rain"});
  predictions.push_back({min({tempWarm2, humHumid3, pressHigh}), "Moderate Rain"});
  predictions.push_back({min({tempWarm2, humWet, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempWarm2, humWet, pressMedium}), "Heavy Rain"});
  predictions.push_back({min({tempWarm2, humWet, pressHigh}), "Moderate Rain"});

  // Warm3 temperature (46-60)
  predictions.push_back({min({tempWarm3, humDry, pressLow}), "No Rain"});
  predictions.push_back({min({tempWarm3, humDry, pressMedium}), "No Rain"});
  predictions.push_back({min({tempWarm3, humDry, pressHigh}), "No Rain"});
  predictions.push_back({min({tempWarm3, humHumid1, pressLow}), "Light Rain"});
  predictions.push_back({min({tempWarm3, humHumid1, pressMedium}), "No Rain"});
  predictions.push_back({min({tempWarm3, humHumid1, pressHigh}), "No Rain"});
  predictions.push_back({min({tempWarm3, humHumid2, pressLow}), "Moderate Rain"});
  predictions.push_back({min({tempWarm3, humHumid2, pressMedium}), "Light Rain"});
  predictions.push_back({min({tempWarm3, humHumid2, pressHigh}), "Light Rain"});
  predictions.push_back({min({tempWarm3, humHumid3, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempWarm3, humHumid3, pressMedium}), "Moderate Rain"});
  predictions.push_back({min({tempWarm3, humHumid3, pressHigh}), "Moderate Rain"});
  predictions.push_back({min({tempWarm3, humWet, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempWarm3, humWet, pressMedium}), "Heavy Rain"});
  predictions.push_back({min({tempWarm3, humWet, pressHigh}), "Moderate Rain"});

  // Hot temperature (61-75)
  predictions.push_back({min({tempHot, humDry, pressLow}), "No Rain"});
  predictions.push_back({min({tempHot, humDry, pressMedium}), "No Rain"});
  predictions.push_back({min({tempHot, humDry, pressHigh}), "No Rain"});
  predictions.push_back({min({tempHot, humHumid1, pressLow}), "Light Rain"});
  predictions.push_back({min({tempHot, humHumid1, pressMedium}), "No Rain"});
  predictions.push_back({min({tempHot, humHumid1, pressHigh}), "No Rain"});
  predictions.push_back({min({tempHot, humHumid2, pressLow}), "Moderate Rain"});
  predictions.push_back({min({tempHot, humHumid2, pressMedium}), "Light Rain"});
  predictions.push_back({min({tempHot, humHumid2, pressHigh}), "Light Rain"});
  predictions.push_back({min({tempHot, humHumid3, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempHot, humHumid3, pressMedium}), "Moderate Rain"});
  predictions.push_back({min({tempHot, humHumid3, pressHigh}), "Moderate Rain"});
  predictions.push_back({min({tempHot, humWet, pressLow}), "Heavy Rain"});
  predictions.push_back({min({tempHot, humWet, pressMedium}), "Heavy Rain"});
  predictions.push_back({min({tempHot, humWet, pressHigh}), "Moderate Rain"});

  float maxCertainty = 0;
  String finalPrediction = "Unknown";

  for (const auto &pred : predictions)
  {
    if (pred.first > maxCertainty)
    {
      maxCertainty = pred.first;
      finalPrediction = pred.second;
    }
  }

  // Only print debug every 10 seconds
  if (currentMillis - lastDebugTime >= DEBUG_INTERVAL)
  {
    Serial.printf("\n=== Rain Prediction (#%lu) ===\n", callCount);
    Serial.printf("Result: %s (Certainty: %.2f)\n",
                  finalPrediction.c_str(), maxCertainty);
    lastDebugTime = currentMillis;
  }

  return {finalPrediction, maxCertainty};
}

void IRAM_ATTR rpm_anemometer()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xAnemometerSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR rainInterrupt()
{
  unsigned long interruptTime = millis();
  // Check if interrupt was triggered within debounce period
  if (interruptTime - lastRainInterruptTime > DEBOUNCE_DELAY)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xRainSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR();
    }
    lastRainInterruptTime = interruptTime;
  }
}

void processRainfall()
{
  int tipCount = 0;
  while (xSemaphoreTake(xRainSemaphore, 0) == pdTRUE)
  {
    tipCount++;
  }

  if (tipCount > 0)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
      rainTipCount += tipCount;
      rainfall = rainTipCount * MILLIMETERS_PER_TIP;
      lastRainTime = millis(); // Update last rain time
      xSemaphoreGive(xMutex);
    }
  }
  else if (millis() - lastRainTime > RAIN_RESET_INTERVAL)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY))
    {
      rainTipCount = 0;
      rainfall = 0.00;
      xSemaphoreGive(xMutex);
    }
  }
}

void readTempHumidity()
{
  if (xSemaphoreTake(xMutex, portMAX_DELAY))
  {
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    xSemaphoreGive(xMutex);
  }
}

void measureWindSpeed()
{
  if ((millis() - timeold) >= timemeasure * 1000)
  {
    int pulseSum = 0;
    while (xSemaphoreTake(xAnemometerSemaphore, 0) == pdTRUE)
    {
      pulseSum++;
    }

    rps = float(pulseSum) / timemeasure;
    velocity_ms = rps;
    velocity_kmh = velocity_ms * 3.6;

    timeold = millis();
  }
}

void sendToRealtimeDB(float temp, float hum, float windSpeed, float rainfall, float pressure_hPa, float uvIndex)
{
  if (!Firebase.ready())
  {
    Serial.println("Firebase not ready!");
    return;
  }

  if (!signupOK)
  {
    Serial.println("Firebase signup failed!");
    return;
  }

  // Get current time components
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }

  // Create time strings
  char dateStr[11];
  char timeStr[9];
  char fullTimestamp[20];

  strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &timeinfo);
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
  strftime(fullTimestamp, sizeof(fullTimestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);

  // Get weather condition using fuzzy logic
  WeatherCondition weather = fuzzyLogic(temp, hum, windSpeed, rainfall, pressure_hPa, uvIndex);

  // Get rain prediction
  RainPrediction rainPred = predictRain(temp, hum, pressure_hPa);

  // Modify the Firebase paths to include user and device hierarchy
  String basePath = String("uid=") + USER_ID + "/deviceid=" + DEVICE_ID;
  String latestPath = basePath + "/latest_reading";

  json.clear();
  // Add sensor data
  json.add("temperature", temp);
  json.add("humidity", hum);
  json.add("windSpeed_kmph", windSpeed);
  json.add("rainfall_mm", rainfall);
  json.add("pressure_hPa", pressure_hPa);
  json.add("uvIndex", uvIndex);
  json.add("altitude", altitude); // Add altitude to JSON
  json.add("timestamp", fullTimestamp);
  json.add("weatherCondition", weather.condition);
  json.add("weatherCertainty", weather.certainty);

  // Add rain prediction data
  json.add("rainPrediction", rainPred.intensity);
  json.add("rainPredictionCertainty", rainPred.certainty);

  // Send to latest_reading with new path
  if (Firebase.RTDB.setJSON(&fbdo, latestPath.c_str(), &json))
  {
    Serial.println("Latest reading update successful");
  }
  else
  {
    Serial.println("Latest reading update failed");
    Serial.println(fbdo.errorReason());
  }

  // Send to history with new path
  String formattedTime = String(fullTimestamp);
  formattedTime.replace(" ", "_");
  formattedTime.replace(":", "-");
  String historyPath = basePath + "/history/" + formattedTime;

  if (Firebase.RTDB.setJSON(&fbdo, historyPath.c_str(), &json))
  {
    Serial.println("History data saved successfully");
  }
  else
  {
    Serial.println("History data save failed");
    Serial.println(fbdo.errorReason());
  }
}

void TaskReadSensors(void *pvParameters)
{
  while (true)
  {
    readTempHumidity();             // Baca suhu dan kelembaban
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay singkat

    measureWindSpeed();             // Ukur kecepatan angin
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay singkat

    processRainfall();              // Proses curah hujan
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay singkat

    readUVSensor(); // Baca sensor UV

    ESP_LOGI(TAG, "Temperature (DHT): %.2f °C", temperature);
    ESP_LOGI(TAG, "Humidity (DHT): %.2f %%", humidity);
    ESP_LOGV(TAG, "Wind Speed: %.2f km/h", velocity_kmh);
    ESP_LOGV(TAG, "Wind Speed: %.2f m/s", velocity_ms);
    ESP_LOGW(TAG, "Rainfall: %.2f mm", rainfall);
    ESP_LOGI(TAG, "Pressure (BMP): %.2f hPa", pressure_hPa); // Log pressure in hPa
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void updateDeviceStatus()
{
  if (!Firebase.ready() || !signupOK)
    return;

  String basePath = String("uid=") + USER_ID + "/deviceid=" + DEVICE_ID + "/device_status";
  FirebaseJson statusJson;

  // Update connection status
  deviceConnected = (WiFi.status() == WL_CONNECTED && Firebase.ready());

  // Prepare status data
  statusJson.clear();
  statusJson.add("connected", deviceConnected);
  statusJson.add("last_seen", getFormattedTime());

  // Send to Firebase
  if (Firebase.RTDB.setJSON(&fbdo, basePath.c_str(), &statusJson))
  {
    Serial.println("Device status updated successfully");
  }
  else
  {
    Serial.println("Device status update failed");
    Serial.println(fbdo.errorReason());
  }
}

void TaskSendFirebase(void *pvParameters)
{
  unsigned long lastHistoryUpdate = 0;
  unsigned long lastLatestUpdate = 0;

  while (true)
  {
    unsigned long currentMillis = millis();

    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      // Create JSON with current sensor data
      json.clear();
    }
    json.add("temperature", temperature);
    json.add("humidity", humidity);
    json.add("windSpeed_kmph", velocity_kmh);
    json.add("rainfall_mm", rainfall);
    json.add("pressure_hPa", pressure_hPa);
    json.add("uvIndex", uvIndex);
    json.add("altitude", altitude);

    // Get current timestamp
    char fullTimestamp[20];
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
      strftime(fullTimestamp, sizeof(fullTimestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
    }

    json.add("timestamp", fullTimestamp);

    // Get weather condition and rain prediction
    WeatherCondition weather = fuzzyLogic(temperature, humidity, velocity_kmh, rainfall, pressure_hPa, uvIndex);
    RainPrediction rainPred = predictRain(temperature, humidity, pressure_hPa);

    json.add("weatherCondition", weather.condition);
    json.add("weatherCertainty", weather.certainty);
    json.add("rainPrediction", rainPred.intensity);
    json.add("rainPredictionCertainty", rainPred.certainty);

    // Update latest reading more frequently
    if (currentMillis - lastLatestUpdate >= FIREBASE_LATEST_INTERVAL)
    {
      String basePath = String("uid=") + USER_ID + "/deviceid=" + DEVICE_ID;
      String latestPath = basePath + "/latest_reading";

      if (Firebase.RTDB.setJSON(&fbdo, latestPath.c_str(), &json))
      {
        Serial.println("Latest reading update successful");
      }
      else
      {
        Serial.println("Latest reading update failed");
        Serial.println(fbdo.errorReason());
      }

      lastLatestUpdate = currentMillis;
    }

    // Update history every minute
    if (currentMillis - lastHistoryUpdate >= FIREBASE_HISTORY_INTERVAL)
    {
      String basePath = String("uid=") + USER_ID + "/deviceid=" + DEVICE_ID;
      String formattedTime = String(fullTimestamp);
      formattedTime.replace(" ", "_");
      formattedTime.replace(":", "-");
      String historyPath = basePath + "/history/" + formattedTime;

      if (Firebase.RTDB.setJSON(&fbdo, historyPath.c_str(), &json))
      {
        Serial.println("History data saved successfully");
      }
      else
      {
        Serial.println("History data save failed");
        Serial.println(fbdo.errorReason());
      }

      lastHistoryUpdate = currentMillis;
    }

    // Update device status
    if (currentMillis - lastStatusUpdate >= DEVICE_STATUS_INTERVAL)
    {
      updateDeviceStatus();
      lastStatusUpdate = currentMillis;
    }

    xSemaphoreGive(xMutex);
  }

  vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
}

void TaskUpdateDisplay(void *pvParameters)
{
  while (true)
  {
    currentSlide = (currentSlide + 1) % 8;
    updateDisplay(currentSlide);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}