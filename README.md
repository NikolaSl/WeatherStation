# ESP32 Weather Station with BME280 Sensor (PlatformIO)

This project implements a weather station using an ESP32 microcontroller and a BME280 sensor. It measures temperature, humidity, and atmospheric pressure and can transmit this data to either Google Sheets using a Google Apps Script or to the APRS-IS network for display on APRS maps (like APRS.fi).  The project supports both AC-powered (active mode with light sleep) and battery-powered (deep sleep) operation.  This project is designed to be built and uploaded using PlatformIO in Visual Studio Code.

## Features

*   **BME280 Sensor Support:** Reads temperature, humidity, and pressure data from the BME280 sensor.
*   **WiFi Connectivity:** Connects to a WiFi network to transmit data.
*   **Google Sheets Integration (Optional):** Sends sensor data to a Google Sheet using a Google Apps Script.
*   **APRS-IS Integration (Optional):** Transmits sensor data to the APRS-IS network for display on APRS maps.
*   **Power Management:**
    *   **Active Mode (AC-Powered):** Uses light sleep to reduce power consumption while maintaining WiFi connection.
    *   **Deep Sleep Mode (Battery-Powered):** Enters deep sleep between readings to minimize power consumption.
*   **Watchdog Timer:** Implements a watchdog timer to prevent the ESP32 from freezing.
*   **Debug Logging:** Provides detailed logging information via the serial port (can be disabled).
*   **PlatformIO Support:**  Designed for easy building and uploading using PlatformIO in Visual Studio Code.

## Hardware Requirements

*   ESP32 Development Board (e.g., ESP32-C3, ESP32-WROOM-32)
*   BME280 Barometric Pressure, Temperature, and Humidity Sensor
*   Connecting Wires
*   (Optional) Power Supply (USB or AC adapter for active mode, battery for deep sleep mode)

## Software Requirements

*   Visual Studio Code (VS Code)
*   PlatformIO IDE Extension for VS Code

## Installation

1.  **Install Visual Studio Code:** Download and install Visual Studio Code from the official website: [https://code.visualstudio.com/](https://code.visualstudio.com/)

2.  **Install PlatformIO IDE Extension:**
    *   Open Visual Studio Code.
    *   Go to the Extensions view (Ctrl+Shift+X or Cmd+Shift+X).
    *   Search for `PlatformIO IDE` and install the extension.
    *   Restart VS Code after installation.

## Configuration

1.  **Clone the Repository:** Clone this GitHub repository to your local machine.

2.  **Open the Project in VS Code:** Open the cloned repository folder in Visual Studio Code using `File` -> `Open Folder...`.

3.  **Create `include/secrets.h`:** Configuration is stored in `include/secrets.h`.  This file will store your sensitive information (WiFi credentials, API keys, etc.), so be sure to keep it out of version control.  Populate `include/secrets.h` with the following, replacing the placeholder values with your actual credentials:

    ```c++
    #define WIFI_SSID "YourWiFiSSID"
    #define WIFI_PASS "YourWiFiPassword"
    #define GOOGLE_SCRIPT_KEY "YourGoogleAppsScriptKey" // Leave empty if not using Google Sheets
    #define APRS_CALLSIGN "YOURCALLSIGN" // Replace with your amateur radio callsign, e.g., "N0CALL-7"
    #define APRS_PASSCODE "YourAPRSISPasscode" // Generate from your callsign, see APRS-IS documentation
    #define APRS_COMMENT "ESP32 BME280 WX" // A short description of your station
    #define APRS_LATITUDE 37.7749 // Your latitude in decimal degrees
    #define APRS_LONGITUDE -122.4194 // Your longitude in decimal degrees
    #define COMMUNICATION_INTERVAL_MINUTES 5 // How often to send data (minutes)
    ```

    **Important:**  Do *not* commit your `include/secrets.h` file to a public repository!  Add it to your `.gitignore` file or use `git update-index --assume-unchanged include/secrets.h` to prevent it from being tracked by Git.

4.  **Configure Power Mode:** In `src/WeatherStation.cpp` file choose the desired power mode by uncommenting either `#define POWER_MODE_ACTIVE` or `#define POWER_MODE_DEEPSLEEP`.

    *   **`POWER_MODE_ACTIVE`:**  For AC-powered operation.  The ESP32 will stay connected to WiFi and use light sleep to reduce power consumption.
    *   **`POWER_MODE_DEEPSLEEP`:** For battery-powered operation. The ESP32 will disconnect from WiFi and enter deep sleep between readings to conserve power.

5.  **Configure APRS Settings (Optional):** If you want to send data to the APRS-IS network, configure the following settings in `include/secrets.h`:

    *   `APRS_CALLSIGN`: Your amateur radio callsign.  This is required for APRS-IS.
    *   `APRS_PASSCODE`: Your APRS-IS passcode.  You can generate this passcode from your callsign using an online APRS passcode calculator (search for "APRS passcode generator").
    *   `APRS_LATITUDE`: The latitude of your weather station in decimal degrees.
    *   `APRS_LONGITUDE`: The longitude of your weather station in decimal degrees.
    *   `APRS_COMMENT`: A short comment describing your weather station.  Keep this short to avoid exceeding APRS packet limits.

6.  **Configure Google Sheets Integration (Optional):** If you want to send data to a Google Sheet, you need to create a Google Apps Script and obtain the script's deployment ID.

    *   **Create a Google Sheet:** Create a new Google Sheet in your Google Drive.
    *   **Add header row to the sheet with the following columns: `Timestamp`, `Temperature`, `Humidity`, `Pressure`, `Comment`
    *   **Create a Google Apps Script:**
        *   In the Google Sheet, go to `Tools` -> `Script editor`.
        *   Paste the following code into the script editor:

            ```javascript
            function doPost(e) {
                const sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();
                const data = JSON.parse(e.postData.contents);
                sheet.appendRow([new Date(), data.temperature, data.humidity, data.pressure, data.comment]);
                var rowCount = sheet.getLastRow();
                if (rowCount > 10081) {
                    var rowsToDelete = rowCount - 10081;
                    sheet.deleteRows(2, rowsToDelete); // Start from row 2 to keep the header intact
                }
                return ContentService.createTextOutput("Success");
            }
            ```

        *   Modify the script to match the column order you want in your spreadsheet.
        *   Save the script with a name, e.g., `WeatherUpdate.gs`.
    *   **Deploy the Script:**
        *   Go to `Deploy` -> `New deployment`.
        *   Select `Web app` as the type.
        *   Set `Who has access` to `Anyone with the link`.
        *   Click `Deploy`.
        *   Copy the "Web app URL" (this is your Google Apps Script key).
    *   **Enter the Google Apps Script Key:**  Paste the "Web app URL" into the `GOOGLE_SCRIPT_KEY` definition in your `include/secrets.h` file.  *Only* the part after `/s/` and before `/exec` is the key.

7.  **Adjust `COMMUNICATION_INTERVAL_MINUTES`:**  In `include/secrets.h`, set the `COMMUNICATION_INTERVAL_MINUTES` to the desired interval (in minutes) between data transmissions.  This setting affects both Google Sheets and APRS-IS transmissions.  For battery-powered operation, a longer interval will significantly extend battery life.

8.  **I2C Pins:** The code uses SDA pin 3 and SCL pin 4.  If you need to change these, modify the `#define I2C_SDA` and `#define I2C_SCL` definitions in `src/WeatherStation.cpp` file.

## Building and Uploading (PlatformIO)

1.  **Connect the Hardware:** Connect the BME280 sensor to the ESP32 according to the following wiring:

    *   BME280 SDA  -> ESP32 SDA (GPIO 3 by default)
    *   BME280 SCL  -> ESP32 SCL (GPIO 4 by default)
    *   BME280 VCC  -> ESP32 3.3V
    *   BME280 GND  -> ESP32 GND

2.  **Build the Project:** In VS Code, open the PlatformIO sidebar (click the PlatformIO icon in the Activity Bar).  Under `Project Tasks`, select your environment (usually `env:esp32dev` or similar) and click `Build`.  This will compile the code.

3.  **Upload the Code:** After the build is successful, in the PlatformIO sidebar, select your environment and click `Upload`.  This will upload the compiled code to your ESP32.

4.  **Monitor Serial Output:** In VS Code, open the PlatformIO Serial Monitor (click the "Plug" icon in the bottom toolbar or use the command `PlatformIO: Serial Monitor`).  This will display the serial output from the ESP32.  Set the baud rate to 115200.
