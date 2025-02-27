#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include "secrets.h"

// ======== CONFIGURATION OPTIONS ========
// Uncomment ONE of these to select power mode
//#define POWER_MODE_ACTIVE    // For AC-powered version with light sleep
#define POWER_MODE_DEEPSLEEP // For battery-powered version with deep sleep

// ======== COMMON CONFIGURATION ========
#define DEBUG_LOGGING

#define COMMUNICATION_INTERVAL_MINUTES 5
#define WIFI_TIMEOUT_MS 10000 // WiFi connection timeout
#define MAX_WIFI_POWER 78     // Maximum WiFi power (78/4 = 19.5 dBm)
#define MAX_WIFI_RETRIES 5    // Maximum WiFi connection retries

// ======== MODE-SPECIFIC CONFIGURATION ========
#ifdef POWER_MODE_ACTIVE
  #define WATCHDOG_TIMEOUT 30   // Longer timeout for active mode (seconds)
  #define LIGHT_SLEEP_TIME_US 1000000 // 1 second light sleep
#endif

#ifdef POWER_MODE_DEEPSLEEP
  #define WATCHDOG_TIMEOUT 15   // Shorter timeout for deep sleep mode (seconds)
  #define DEEP_SLEEP_TIME_US (COMMUNICATION_INTERVAL_MINUTES * 60 * 1000000)
  // RTC memory to store persistent data across deep sleep cycles
  RTC_DATA_ATTR int bootCount = 0;
#endif

// ======== LOGGING MACROS ========
#ifdef DEBUG_LOGGING
  #define LOG_INIT() Serial.begin(115200)
  #define LOG_PRINT(x) Serial.print(x)
  #define LOG_PRINTLN(x) Serial.println(x)
#else
  #define LOG_INIT()
  #define LOG_PRINT(x)
  #define LOG_PRINTLN(x)
#endif

// ======== GLOBAL VARIABLES ========
char serialNumber[18];
bool hasBME280 = true;
hw_timer_t *watchdogTimer = NULL;
const char* googleScriptUrl = "https://script.google.com/macros/s/" GOOGLE_SCRIPT_KEY "/exec";

#ifdef POWER_MODE_ACTIVE
  const unsigned long interval = COMMUNICATION_INTERVAL_MINUTES * 60 * 1000;
  unsigned long previousMillis = -interval;
#endif

#define I2C_SDA 6
#define I2C_SCL 7
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;

// ======== FUNCTION DECLARATIONS ========
void setupWatchdog();
void feedWatchdog();
bool connectToWiFi();
bool reinitializeBME280();
bool sendSensorData();
bool sendSensorDataWithValues(float temp, float humidity, float pressure);

// ======== WATCHDOG FUNCTIONS ========
// Watchdog timer interrupt handler
void IRAM_ATTR resetModule() {
    esp_restart();
}

// Function to initialize watchdog timer
void setupWatchdog() {
    watchdogTimer = timerBegin(0, 80, true);                // Timer 0, divider 80
    timerAttachInterrupt(watchdogTimer, &resetModule, true); // Attach callback
    timerAlarmWrite(watchdogTimer, WATCHDOG_TIMEOUT * 1000000UL, false); // Set time in microseconds
    timerAlarmEnable(watchdogTimer);                        // Enable interrupt
}

// Function to feed the watchdog
void feedWatchdog() {
    if (watchdogTimer) {
        timerWrite(watchdogTimer, 0);
    }
}

// Function to disable watchdog (for deep sleep mode)
void disableWatchdog() {
    if (watchdogTimer) {
        timerAlarmDisable(watchdogTimer);
        timerDetachInterrupt(watchdogTimer);
        timerEnd(watchdogTimer);
        watchdogTimer = NULL;
    }
}

// ======== WIFI FUNCTIONS ========
bool connectToWiFi() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    
    // Set maximum WiFi power before connecting
    esp_wifi_set_max_tx_power(MAX_WIFI_POWER);
    
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    // Verify power setting was applied
    int8_t power;
    esp_wifi_get_max_tx_power(&power);
    LOG_PRINTLN("WiFi TX power set to: " + String(power/4) + " dBm");

    unsigned long startAttemptTime = millis();
    int attempts = 0;

    while (WiFi.status() != WL_CONNECTED && 
           millis() - startAttemptTime < WIFI_TIMEOUT_MS && 
           attempts < MAX_WIFI_RETRIES) {
        delay(500);
        LOG_PRINT(".");
        attempts++;
        feedWatchdog();
    }

    if (WiFi.status() == WL_CONNECTED) {
        LOG_PRINTLN("\nConnected to WiFi");
        LOG_PRINTLN("IP address: " + WiFi.localIP().toString());
        LOG_PRINTLN("Signal strength (RSSI): " + String(WiFi.RSSI()) + " dBm");
        
        // Ensure power setting is maintained after connection
        esp_wifi_set_max_tx_power(MAX_WIFI_POWER);
        return true;
    } else {
        LOG_PRINTLN("\nFailed to connect to WiFi");
        return false;
    }
}

// ======== SENSOR FUNCTIONS ========
bool reinitializeBME280() {
    LOG_PRINTLN("Reinitializing BME280...");
    I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
    
    int bmeRetries = 3;
    while (bmeRetries > 0) {
        if (bme.begin(0x76, &I2CBME)) {
            LOG_PRINTLN("BME280 reinitialized successfully");
            return true;
        }
        LOG_PRINTLN("BME280 reinit attempt failed, retrying...");
        delay(1000);
        bmeRetries--;
        feedWatchdog();
    }
    
    LOG_PRINTLN("Failed to reinitialize BME280");
    return false;
}

// ======== DATA TRANSMISSION FUNCTIONS ========
bool sendSensorData() {
    feedWatchdog();

    if (WiFi.status() != WL_CONNECTED) {
        if (!connectToWiFi()) {
            return false;
        }
    }

    HTTPClient http;
    bool success = false;

    try {
        String payload = "{";
        if (hasBME280) {
            float temp = bme.readTemperature();
            float humidity = bme.readHumidity();
            float pressure = bme.readPressure() / 100.0F;
            
            // Check if readings are valid
            if (isnan(temp) || isnan(humidity) || isnan(pressure)) {
                LOG_PRINTLN("Invalid sensor readings, attempting sensor reinit...");
                if (reinitializeBME280()) {
                    // Try reading again after reinitialization
                    temp = bme.readTemperature();
                    humidity = bme.readHumidity();
                    pressure = bme.readPressure() / 100.0F;
                    
                    if (isnan(temp) || isnan(humidity) || isnan(pressure)) {
                        LOG_PRINTLN("Still getting invalid readings after reinit");
                        hasBME280 = false;
                        return false;
                    }
                } else {
                    hasBME280 = false;
                    return false;
                }
            }
            
            payload += "\"temperature\":" + String(temp) + ",";
            payload += "\"humidity\":" + String(humidity) + ",";
            payload += "\"pressure\":" + String(pressure) + ",";
        }
        payload += "\"comment\":\"MAC: " + String(serialNumber) +"\"}";
        
        LOG_PRINTLN("Payload: " + payload);

        http.begin(googleScriptUrl);
        http.addHeader("Content-Type", "application/json");
        http.setTimeout(10000);
        
        int httpResponseCode = http.POST(payload);
        
        if (httpResponseCode > 0) {
            LOG_PRINTLN("HTTP Response Code: " + String(httpResponseCode));
            LOG_PRINTLN("Response: " + http.getString());
            if (httpResponseCode >= 200 && httpResponseCode < 500) {
                // since google script accepts POST even without following
                // the redirection, we accept all success, redirect and 
                // server error responses as successful communication 
                success = true;
            }
        } else {
            LOG_PRINTLN("Error on HTTP request: " + http.errorToString(httpResponseCode));
        }
        
        http.end();
    } catch (const std::exception& e) {
        LOG_PRINTLN("Exception in HTTP request: " + String(e.what()));
    }
    
    return success;
}

// ======== SETUP & LOOP ========
void setup() {
    LOG_INIT();
    delay(1000);
    
    // Get MAC address for device identification
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(serialNumber, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    LOG_PRINTLN("\n\n=== ESP32 Weather Station ===");
    
    #ifdef POWER_MODE_ACTIVE
        LOG_PRINTLN("Mode: ACTIVE with light sleep");
    #endif
    
    #ifdef POWER_MODE_DEEPSLEEP
        LOG_PRINTLN("Mode: DEEP SLEEP");
        bootCount++;
        LOG_PRINTLN("Boot count: " + String(bootCount));
    #endif
    
    LOG_PRINTLN("Device MAC: " + String(serialNumber));
    
    // Initialize I2C and BME280
    I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
    
    if (!bme.begin(0x76, &I2CBME)) {
        LOG_PRINTLN("Could not find BME280 sensor!");
        hasBME280 = false;
    } else {
        LOG_PRINTLN("BME280 sensor found");
        // Weather station settings
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X1,  // temperature
                        Adafruit_BME280::SAMPLING_X1,  // pressure
                        Adafruit_BME280::SAMPLING_X1,  // humidity
                        Adafruit_BME280::FILTER_OFF,
                        Adafruit_BME280::STANDBY_MS_1000);
    }
    
    // Setup watchdog
    setupWatchdog();
    
    #ifdef POWER_MODE_DEEPSLEEP
        // For deep sleep mode, try to send data immediately
        if (sendSensorData()) {
            LOG_PRINTLN("Data sent successfully, going to deep sleep");
        } else {
            LOG_PRINTLN("Failed to send data, going to deep sleep anyway");
        }
        
        // Disable watchdog before deep sleep
        disableWatchdog();
        
        // Configure deep sleep
        esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME_US);
        LOG_PRINTLN("Going to deep sleep for " + String(COMMUNICATION_INTERVAL_MINUTES) + " minutes");
        delay(100);
        esp_deep_sleep_start();
    #endif
}

void loop() {
    #ifdef POWER_MODE_ACTIVE
        feedWatchdog();
        
        unsigned long currentMillis = millis();
        
        // Check if it's time to send data
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            
            LOG_PRINTLN("\n=== Periodic data transmission ===");
            
            if (sendSensorData()) {
                LOG_PRINTLN("Data sent successfully");
            } else {
                LOG_PRINTLN("Failed to send data");
            }
        }
        
        // Use light sleep between checks to save power
        esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_TIME_US);
        esp_light_sleep_start();
    #endif
    
    // This part is only reached in active mode
    // Deep sleep mode never reaches this point as it restarts from setup()
}

