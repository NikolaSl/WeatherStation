#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include "secrets.h"

// ======== CONFIGURATION OPTIONS ========
// Uncomment ONE of these to select power mode
#define POWER_MODE_ACTIVE    // For AC-powered version with light sleep
//#define POWER_MODE_DEEPSLEEP // For battery-powered version with deep sleep

// ======== COMMON CONFIGURATION ========
#define DEBUG_LOGGING

#define WIFI_TIMEOUT_MS 10000 // WiFi connection timeout
#define MAX_WIFI_POWER 78     // Maximum WiFi power (78/4 = 19.5 dBm)
#define MAX_WIFI_RETRIES 5    // Maximum WiFi connection retries

// APRS-IS server details
const char* aprsServer = "rotate.aprs.net";  // APRS-IS server
const int aprsPort = 14580;                   // Standard APRS-IS port

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

#define I2C_SDA 3
#define I2C_SCL 4
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;

// ======== FUNCTION DECLARATIONS ========
void setupWatchdog();
void feedWatchdog();
bool connectToWiFi();
bool reinitializeBME280();
String convertToAPRSFormat(float coordinate, bool isLatitude);
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

bool sendToAPRS(float temp, float humidity, float pressure) {
    if (WiFi.status() != WL_CONNECTED) {
        if (!connectToWiFi()) {
            return false;
        }
    }

    WiFiClient client;
    bool success = false;
    
    LOG_PRINTLN("Connecting to APRS-IS server...");
    
    if (client.connect(aprsServer, aprsPort)) {
        LOG_PRINTLN("Connected to APRS-IS server");
        
        // Format temperature in Fahrenheit for APRS
        float tempF = (temp * 9.0 / 5.0) + 32.0;
        
        // Login to APRS-IS
        String loginString = "user " + String(APRS_CALLSIGN) + " pass " + String(APRS_PASSCODE) + 
                                    " vers " + String(APRS_COMMENT) + "\r\n";
        client.print(loginString);
        LOG_PRINTLN("Sent: " + loginString);
        
        // Wait for server response
        unsigned long timeout = millis();
        while (client.available() == 0) {
            if (millis() - timeout > 5000) {
                LOG_PRINTLN("APRS-IS server timeout!");
                client.stop();
                return false;
            }
            feedWatchdog();
        }
        
        // Read server response
        String response = client.readStringUntil('\n');
        LOG_PRINTLN("Server response: " + response);
        
        if (response.indexOf("failed") == -1) {
            // Convert latitude and longitude to APRS format
            // APRS format: DDMM.hhN/DDDMM.hhW (degrees, minutes, hundredths of minutes)
            String aprsLat = convertToAPRSFormat(APRS_LATITUDE, true);
            String aprsLon = convertToAPRSFormat(APRS_LONGITUDE, false);
            
            // Format weather data according to APRS specification
            // Format: !DDMM.hhN/DDDMM.hhW_wind/temp/rain/etc...
            String weatherPacket = String(APRS_CALLSIGN) + ">APRS,TCPIP*:";
            weatherPacket += "!" + aprsLat + "/" + aprsLon + "_"; // Position and symbol (underscore for WX station)
            
            // Wind direction and speed (000/000 if not available)
            // Format: dddd/s (degrees, speed in knots)
            // weatherPacket += "000/000";
            
            // Temperature in Fahrenheit (format: t-40 to t999, three digits)
            int tempInt = (int)tempF;
            weatherPacket += "t";
            if (tempInt < 0) {
                weatherPacket += "-" + String(abs(tempInt)).substring(0, 2);
            } else if (tempInt < 100) {
                weatherPacket += "0" + String(tempInt);
            } else {
                weatherPacket += String(tempInt).substring(0, 3);
            }
            
            // Humidity (format: h00 to h99, two digits, 00=100%)
            int humInt = (int)humidity;
            if (humInt == 100) humInt = 0;  // 00 means 100% in APRS
            weatherPacket += "h";
            if (humInt < 10) {
                weatherPacket += "0" + String(humInt);
            } else {
                weatherPacket += String(humInt);
            }
            
            // Barometric pressure (format: b00000 to b99999, in tenths of millibars/hPa)
            int pressureInt = (int)(pressure * 10);
            weatherPacket += "b";
            if (pressureInt < 10000) {
                weatherPacket += "0" + String(pressureInt);
            } else {
                weatherPacket += String(pressureInt);
            }
            
            // Add comment
            weatherPacket += " - " + String(APRS_COMMENT);
            
            // Send the packet
            weatherPacket += "\r\n";
            client.print(weatherPacket);
            LOG_PRINTLN("Sent APRS packet: " + weatherPacket);
            
            // Wait briefly to ensure packet is sent
            delay(500);
            success = true;
        } else {
            LOG_PRINTLN("APRS-IS authentication failed");
        }
        
        client.stop();
        LOG_PRINTLN("Disconnected from APRS-IS server");
    } else {
        LOG_PRINTLN("Connection to APRS-IS server failed");
    }
    
    return success;
}

// Helper function to convert decimal degrees to APRS format (DDMM.hhX)
String convertToAPRSFormat(float coordinate, bool isLatitude) {
    bool isPositive = coordinate >= 0;
    coordinate = abs(coordinate);

    int degrees = (int)coordinate;
    float minutes = (coordinate - degrees) * 60.0;

    String result = "";

    // Format degrees
    if (isLatitude) {
        // Latitude: 2 digits for degrees (e.g., 37° -> 37)
        if (degrees < 10) result += "0";
        result += String(degrees);
    } else {
        // Longitude: 3 digits for degrees (e.g., 122° -> 122)
        if (degrees < 100) result += "0";
        if (degrees < 10) result += "0";
        result += String(degrees);
    }

    // Format minutes (MM.hh)
    int minutesInt = (int)minutes;
    int hundredths = (int)round((minutes - minutesInt) * 100); // Use round()

    if (minutesInt < 10) result += "0";
    result += String(minutesInt) + ".";

    if (hundredths < 10) result += "0";
    result += String(hundredths);

    // Add N/S or E/W indicator
    if (isLatitude) {
        result += (isPositive ? "N" : "S");
    } else {
        result += (isPositive ? "E" : "W");
    }

    // Debugging output
    LOG_PRINTLN("convertToAPRSFormat Debug:");
    LOG_PRINTLN("  Coordinate: " + String(coordinate));
    LOG_PRINTLN("  Degrees: " + String(degrees));
    LOG_PRINTLN("  Minutes: " + String(minutes));
    LOG_PRINTLN("  MinutesInt: " + String(minutesInt));
    LOG_PRINTLN("  Hundredths: " + String(hundredths));
    LOG_PRINTLN("  Result: " + result);

    return result;
}


bool sendSensorData() {
    feedWatchdog();

    if (WiFi.status() != WL_CONNECTED) {
        if (!connectToWiFi()) {
            return false;
        }
    }

    bool googleSuccess = false;
    bool aprsSuccess = false;
    bool googleConfigured = strlen(GOOGLE_SCRIPT_KEY) > 10; // Simple check if key is configured
    bool aprsConfigured = strlen(APRS_CALLSIGN) > 2 && strcmp(APRS_CALLSIGN, "NOCALL") != 0;

    try {
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
            
            // Send to Google Script if configured
            if (googleConfigured) {
                LOG_PRINTLN("Google Script is configured, sending data...");
                String payload = "{";
                payload += "\"temperature\":" + String(temp) + ",";
                payload += "\"humidity\":" + String(humidity) + ",";
                payload += "\"pressure\":" + String(pressure) + ",";
                payload += "\"comment\":\"MAC: " + String(serialNumber) +"\"}";
                
                LOG_PRINTLN("Google Script Payload: " + payload);

                HTTPClient http;
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
                        googleSuccess = true;
                        LOG_PRINTLN("Successfully sent data to Google");
                    }
                } else {
                    LOG_PRINTLN("Error on HTTP request: " + http.errorToString(httpResponseCode));
                }
                
                http.end();
            } else {
                LOG_PRINTLN("Google Script not configured, skipping");
            }
            
            // Send to APRS.fi if configured
            if (aprsConfigured) {
                LOG_PRINTLN("APRS is configured, sending data...");
                aprsSuccess = sendToAPRS(temp, humidity, pressure);
            } else {
                LOG_PRINTLN("APRS not configured, skipping");
            }
            
            // Return true if at least one configured service received the data
            bool success = false;
            if (googleConfigured || aprsConfigured) {
                success = googleSuccess || aprsSuccess;
            } else {
                // If nothing is configured, consider it a success to avoid retries
                LOG_PRINTLN("No data services configured");
                success = true;
            }
            
            LOG_PRINTLN("Data transmission summary:");
            if (googleConfigured) LOG_PRINTLN("- Google: " + String(googleSuccess ? "SUCCESS" : "FAILED"));
            if (aprsConfigured) LOG_PRINTLN("- APRS: " + String(aprsSuccess ? "SUCCESS" : "FAILED"));
            
            return success;
        }
    } catch (const std::exception& e) {
        LOG_PRINTLN("Exception in HTTP request: " + String(e.what()));
    }
    
    return false;
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

