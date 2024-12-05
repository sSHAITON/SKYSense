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
  // Temperature membership
  float tempDingin = calculateTriangularMembership(temp, 15, 20, 25);
  float tempNormal = calculateTriangularMembership(temp, 20, 25, 30);
  float tempPanas = calculateTrapezoidalMembership(temp, 25, 30, 35, 40);

  // Humidity membership
  float humKering = calculateTriangularMembership(hum, 0, 30, 50);
  float humNormal = calculateTriangularMembership(hum, 30, 50, 70);
  float humLembab = calculateTrapezoidalMembership(hum, 50, 70, 80, 100);

  // Wind Speed membership (in km/h)
  float windTenang = calculateTriangularMembership(windSpeed, 0, 5, 15);
  float windSedang = calculateTriangularMembership(windSpeed, 10, 20, 30);
  float windKencang = calculateTrapezoidalMembership(windSpeed, 25, 35, 50, 60);

  // Rainfall membership (in mm)
  float rainTidak = calculateTriangularMembership(rainfall, 0, 0, 0.5);
  float rainRingan = calculateTriangularMembership(rainfall, 0.3, 2, 5);
  float rainSedang = calculateTriangularMembership(rainfall, 4, 10, 20);
  float rainLebat = calculateTrapezoidalMembership(rainfall, 15, 25, 50, 100);

  // Pressure membership (in hPa, converted from Pa)
  float pressureHPa = pressure / 100.0; // Convert Pa to hPa
  float pressureLow = calculateTriangularMembership(pressureHPa, 980, 1000, 1013);
  float pressureNormal = calculateTriangularMembership(pressureHPa, 1010, 1013, 1016);
  float pressureHigh = calculateTrapezoidalMembership(pressureHPa, 1015, 1020, 1030, 1040);

  // UV Index membership
  float uvRendah = calculateTriangularMembership(uvIndex, 0, 2, 5);
  float uvSedang = calculateTriangularMembership(uvIndex, 3, 6, 8);
  float uvTinggi = calculateTrapezoidalMembership(uvIndex, 6, 8, 10, 12);

  // Rules definition with certainty values
  struct Rule
  {
    float strength;
    String condition;
  };

  std::vector<Rule> rules;

  // Add rules with their conditions
  rules.push_back({min({tempNormal, humNormal, windTenang, rainTidak, pressureNormal, uvRendah}),
                   "Cerah"});

  rules.push_back({min({tempNormal, humNormal, windTenang, rainTidak, pressureLow, uvRendah}),
                   "Berawan"});

  rules.push_back({min({tempDingin, humLembab, windSedang, rainRingan, pressureLow}),
                   "Hujan Ringan"});

  rules.push_back({min({tempDingin, humLembab, windSedang, rainSedang, pressureLow}),
                   "Hujan Sedang"});

  rules.push_back({min({tempDingin, humLembab, windKencang, rainLebat, pressureLow}),
                   "Hujan Lebat"});

  rules.push_back({min({tempPanas, humKering, windTenang, rainTidak, pressureNormal, uvTinggi}),
                   "Panas"});

  rules.push_back({min({tempNormal, humNormal, windKencang, rainTidak, pressureLow}),
                   "Berangin"});

  rules.push_back({min({tempDingin, humLembab, windKencang, rainLebat, pressureLow}),
                   "Badai"});

  // Find rule with maximum strength
  float maxStrength = 0;
  String finalCondition = "Tidak Diketahui";

  for (const Rule &rule : rules)
  {
    if (rule.strength > maxStrength)
    {
      maxStrength = rule.strength;
      finalCondition = rule.condition;
    }
  }

  // Debug print
  Serial.println("\n--- Fuzzy Logic Debug ---");
  Serial.printf("Temperature (%.1f°C): Dingin=%.2f, Normal=%.2f, Panas=%.2f\n",
                temp, tempDingin, tempNormal, tempPanas);
  Serial.printf("Humidity (%.1f%%): Kering=%.2f, Normal=%.2f, Lembab=%.2f\n",
                hum, humKering, humNormal, humLembab);
  Serial.printf("Wind Speed (%.1f km/h): Tenang=%.2f, Sedang=%.2f, Kencang=%.2f\n",
                windSpeed, windTenang, windSedang, windKencang);
  Serial.printf("Rainfall (%.1f mm): Tidak=%.2f, Ringan=%.2f, Sedang=%.2f, Lebat=%.2f\n",
                rainfall, rainTidak, rainRingan, rainSedang, rainLebat);
  Serial.printf("Pressure (%.1f hPa): Low=%.2f, Normal=%.2f, High=%.2f\n",
                pressureHPa, pressureLow, pressureNormal, pressureHigh);
  Serial.printf("UV Index (%.1f): Rendah=%.2f, Sedang=%.2f, Tinggi=%.2f\n",
                uvIndex, uvRendah, uvSedang, uvTinggi);

  return {finalCondition, maxStrength};
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

      // Get weather condition
      WeatherCondition weather = fuzzyLogic(temperature, humidity, velocity_kmh, rainfall, pressure_hPa, uvIndex);
      json.add("weatherCondition", weather.condition);
      json.add("weatherCertainty", weather.certainty);

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

      xSemaphoreGive(xMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
  }
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
