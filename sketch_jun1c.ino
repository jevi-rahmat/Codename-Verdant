/*
 * ============================================================
 * Automatic Irrigation Controller - ESP32
 * 3 Soil Moisture Sensors + 3 Solenoid Valves (via Relay+Transistor)
 * ============================================================
 *
 * WIRING SUMMARY:
 * ---------------------------------------------------------------
 * Sensor 1 VCC  -> GPIO_SENSOR1_POWER (3.3V switched)
 * Sensor 1 AO   -> GPIO 34 (ADC1_CH6)
 *
 * Sensor 2 VCC  -> GPIO_SENSOR2_POWER
 * Sensor 2 AO   -> GPIO 35 (ADC1_CH7)
 *
 * Sensor 3 VCC  -> GPIO_SENSOR3_POWER
 * Sensor 3 AO   -> GPIO 32 (ADC1_CH4)
 *
 * Transistor 1 Base -> GPIO 25  (controls Relay 1 -> Solenoid 1)
 * Transistor 2 Base -> GPIO 26  (controls Relay 2 -> Solenoid 2)
 * Transistor 3 Base -> GPIO 27  (controls Relay 3 -> Solenoid 3)
 *
 * Relay coil side powered by Step-Up converter via transistor.
 * Solenoid side powered by Step-Up converter through relay NO contact.
 *
 * Communication: Serial (USB) for config & monitoring
 *                Optional: BLE / WiFi (stub included)
 * ============================================================
 *
 * BEHAVIOR:
 *  - Setiap siklus, sensor dihidupkan satu per satu secara bergantian
 *  - Jika tanah kering (ADC < dry_threshold), solenoid dibuka
 *  - Lama buka solenoid dihitung berdasarkan seberapa kering tanah
 *    (interpolasi linear antara min_water_ms dan max_water_ms)
 *  - Setelah solenoid menutup, sensor dihidupkan lagi untuk verifikasi
 *  - Semua threshold & durasi dapat diatur via Serial command
 *
 * SERIAL COMMANDS (baud 115200):
 *  SET_DRY <ch> <value>      -> set dry threshold ch 1-3 (0-4095)
 *  SET_WET <ch> <value>      -> set wet threshold ch 1-3 (0-4095)
 *  SET_MIN_WATER <ch> <ms>   -> set minimum watering duration (ms)
 *  SET_MAX_WATER <ch> <ms>   -> set maximum watering duration (ms)
 *  SET_CYCLE <ms>            -> set scan cycle interval (ms)
 *  STATUS                    -> print current config & last readings
 *  HELP                      -> print command list
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include "time.h"

// ---------------------------------------------------------------
// MILLIS
// ---------------------------------------------------------------
bool solenoidIsOpen[3] = {false, false, false};
unsigned long solenoidOpenTime[3] = {0, 0, 0};
long solenoidTargetDuration[3] = {0, 0, 0};

// ---------------------------------------------------------------
// HTTP CLIENT
// ---------------------------------------------------------------
HTTPClient http;

// ---------------------------------------------------------------
// PERSISTENT DATA
// ---------------------------------------------------------------
Preferences preferences;

// ---------------------------------------------------------------
// NTP SERVER
// ---------------------------------------------------------------
const char* ntpServer = "id.pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

// ---------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------
// Sensor power pins (digital output, HIGH = sensor ON)
static const int PIN_SENSOR_PWR[3]  = {33, 14, 12};

// Sensor ADC pins
static const int PIN_SENSOR_ADC[3]  = {34, 35, 32};

// Transistor base pins (HIGH = transistor ON = relay energized = solenoid OPEN)
static const int PIN_SOLENOID[3]    = {25, 26, 27};

// Wi-Fi LED pins
static const int PIN_WIFI_LED[2]    = {4, 13}; // {Red, Green}

// ---------------------------------------------------------------
// DEFAULT CONFIG
// ---------------------------------------------------------------
// ADC nilai TINGGI = tanah KERING (kapasitif sensor, nilai naik kalau kering)
// Sesuaikan jika sensor Anda bersifat terbalik (resistif)
struct ChannelConfig {
    int  dry_threshold;   // ADC >= nilai ini -> KERING
    int  wet_threshold;   // ADC <= nilai ini -> BASAH
    long min_water_ms;    // durasi siram minimum (ms)
    long max_water_ms;    // durasi siram maksimum (ms)
};

static ChannelConfig cfg[3] = {
    {2800, 1800, 2000, 15000},  // Channel 1
    {2800, 1800, 2000, 15000},  // Channel 2
    {2800, 1800, 2000, 15000},  // Channel 3
};

static long cycle_interval_ms = 5000; // jeda antar satu siklus scan penuh

// ---------------------------------------------------------------
// RUNTIME STATE
// ---------------------------------------------------------------
struct ChannelState {
    int  last_adc_before;  // bacaan sebelum siram
    int  last_adc_after;   // bacaan setelah siram
    bool last_watered;     // apakah disiram pada siklus terakhir
    long last_water_duration_ms;
    bool solenoid_ok;      // true jika ADC after < ADC before (solenoid bekerja)
};

static ChannelState state[3] = {};

// ---------------------------------------------------------------
// BUFFER
// ---------------------------------------------------------------

String potData[] = {"", "", ""};
int potSensorValue[] = {0, 0, 0};
String potMoisturePercent[] = {"", "", ""};
String potAction[] = {"", "", ""};
String potPumpDuration[] = {"", "", ""};
String potSoilCondition[] = {"", "", ""};
String potTimestampSensor[] = {"", "", ""};

// ---------------------------------------------------------------
// HELPER: SENSOR
// ---------------------------------------------------------------

/**
 * Hidupkan sensor ch, baca ADC beberapa kali (oversampling),
 * kemudian matikan sensor lagi.
 * Return: rata-rata nilai ADC (0-4095)
 */
int readSensor(int ch) {
    // Hidupkan sensor
    digitalWrite(PIN_SENSOR_PWR[ch], HIGH);
    delay(150); // beri waktu sensor stabil

    // Oversampling 8x
    long sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += analogRead(PIN_SENSOR_ADC[ch]);
        delay(10);
    }

    // Matikan sensor
    digitalWrite(PIN_SENSOR_PWR[ch], LOW);

    potSensorValue[ch] = (int)(sum / 8);
    return (int)(sum / 8);
}

// ---------------------------------------------------------------
// WIFI CONNECTIVITY
// ---------------------------------------------------------------

String email = "";
String ssid = ""; 
String password = ""; 
const char* AP_SSID = "Toga-ESP";
const char* AP_PASS = "";

WebServer server(80);

void handleSaveConfig() {
    preferences.begin("wifi-config", false);

    ssid = server.arg("wifi_ssid");
    password = server.arg("wifi_pass");
    email = server.arg("email");

    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("email", email);

    preferences.end();

    Serial.println("{SSID: " + ssid + ", Password: " + password + ", Email: " + email + "}");
    server.send(200, "text/html", "<h1>Config Saved!</h1>");

    delay(2000);
    ESP.restart();
}

void handleRoot() {
    String html = "<!DOCTYPE html><html>";
    html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<link rel=\"icon\" href=\"data:,\">";
    html += "<style>html {font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}";
    html += ".form_section {text-align: left;}</style></head>";
    html += "<body><h1>Toga Configuration</h1>";
    html += "<form action=\"/save-config\" method=\"POST\" class=\"form_section\"><table><tr><td><label for=\"\">SSID</label></td><td><input type=\"text\" name=\"wifi_ssid\"></td></tr>";
    html += "<tr><td><label for=\"\">Password</label></td><td><input type=\"password\" name=\"wifi_pass\"></td></tr>";
    html += "<tr><td><label for=\"\">Email</label></td><td><input type=\"email\" name=\"email\"></td></tr></table>";
    html += "<tr><td><input type=\"submit\" value=\"Save\"></td></tr></form></body></head></html>";

    server.send(200, "text/html", html);
}

void connectToWiFi() {
  const int MAX_RETRIES = 3;
  int retryCount = 0;

  preferences.begin("wifi-config", true);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  email = preferences.getString("email", "");
  preferences.end();

  if (ssid.length() == 0) {
    
    digitalWrite(PIN_WIFI_LED[1], HIGH);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    Serial.print("Access point initialized");
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save-config", HTTP_POST, handleSaveConfig);
    server.begin();

  } else {

    digitalWrite(PIN_WIFI_LED[0], HIGH);
    WiFi.mode(WIFI_STA);

    while (retryCount <= MAX_RETRIES && WiFi.status() != WL_CONNECTED) {
      Serial.printf("\n Percobaan koneksi ke-%d dari %d\n", retryCount + 1, MAX_RETRIES);
      Serial.print("SSID: ");
      Serial.println(ssid);
      
      WiFi.begin(ssid, password);
      
      // tunggu 20 detik
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
        
        // Tampilkan status setiap 5 detik
        if (attempts % 10 == 0) {
          Serial.print(" [");
          Serial.print(WiFi.status());
          Serial.print("]");
        }
      }
      
      Serial.println();
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" WiFi Terhubung!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

        return; 
      }
      
      // Gagal, analisa
      Serial.printf(" Gagal koneksi (Status: %d)\n", WiFi.status());
      retryCount++;
      
      if (retryCount < MAX_RETRIES) {
        Serial.println(" Tunggu 2 detik sebelum retry...");
        delay(2000);
      } else {
        Serial.println("[SYSTEM] Can't connect to Wi-Fi. Resetting");
        preferences.begin("wifi-config", false);
        preferences.clear();
        preferences.end();

        Serial.println("[NOTIFICATION] Wi-Fi settings cleared. Restarting...");
        delay(2000);
        ESP.restart();
      }
    }

    digitalWrite(PIN_WIFI_LED[0], LOW);
  }
}

// ---------------------------------------------------------------
// HELPER: SOLENOID
// ---------------------------------------------------------------

void openSolenoid(int ch) {
    digitalWrite(PIN_SOLENOID[ch], HIGH);
}

void closeSolenoid(int ch) {
    digitalWrite(PIN_SOLENOID[ch], LOW);
}

// ---------------------------------------------------------------
// HELPER: WATERING DURATION CALCULATION
// ---------------------------------------------------------------
/**
 * Hitung durasi siram berdasarkan seberapa kering tanah.
 * Semakin kering (ADC makin jauh dari wet_threshold),
 * semakin lama disiram, capped di max_water_ms.
 */
long calcWaterDuration(int ch, int adc_val) {
    int dry  = cfg[ch].dry_threshold;
    int wet  = cfg[ch].wet_threshold;
    long mn  = cfg[ch].min_water_ms;
    long mx  = cfg[ch].max_water_ms;

    if (adc_val <= wet) return mn;
    if (adc_val >= dry) return mx;

    // Interpolasi linear
    float ratio = (float)(adc_val - wet) / (float)(dry - wet);
    return mn + (long)(ratio * (mx - mn));
}

// ---------------------------------------------------------------
// SERIAL CONFIG PARSER
// ---------------------------------------------------------------
void printStatus() {
    Serial.println("\n===== STATUS =====");
    Serial.printf("Cycle interval : %ld ms\n", cycle_interval_ms);
    for (int i = 0; i < 3; i++) {
        Serial.printf("\n[Channel %d]\n", i + 1);
        Serial.printf("  Dry threshold  : %d\n",  cfg[i].dry_threshold);
        Serial.printf("  Wet threshold  : %d\n",  cfg[i].wet_threshold);
        Serial.printf("  Min water      : %ld ms\n", cfg[i].min_water_ms);
        Serial.printf("  Max water      : %ld ms\n", cfg[i].max_water_ms);
        Serial.printf("  Last ADC before: %d\n",  state[i].last_adc_before);
        Serial.printf("  Last ADC after : %d\n",  state[i].last_adc_after);
        Serial.printf("  Last watered   : %s\n",  state[i].last_watered ? "YES" : "NO");
        if (state[i].last_watered) {
            Serial.printf("  Water duration : %ld ms\n", state[i].last_water_duration_ms);
            Serial.printf("  Solenoid OK    : %s\n",
                state[i].solenoid_ok ? "YES (moisture increased)" : "NO (no change detected)");
        }
    }
    Serial.println("==================\n");
}

void printHelp() {
    Serial.println("\n===== COMMANDS =====");
    Serial.println("SET_DRY <ch 1-3> <adc 0-4095>");
    Serial.println("SET_WET <ch 1-3> <adc 0-4095>");
    Serial.println("SET_MIN_WATER <ch 1-3> <ms>");
    Serial.println("SET_MAX_WATER <ch 1-3> <ms>");
    Serial.println("SET_CYCLE <ms>");
    Serial.println("STATUS");
    Serial.println("HELP");
    Serial.println("====================\n");
}

void handleSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    Serial.printf("> Received: %s\n", line.c_str());

    if (line.startsWith("SET_DRY ")) {
        int ch, val;
        if (sscanf(line.c_str(), "SET_DRY %d %d", &ch, &val) == 2 && ch >= 1 && ch <= 3) {
            cfg[ch - 1].dry_threshold = constrain(val, 0, 4095);
            Serial.printf("OK: Channel %d dry_threshold = %d\n", ch, cfg[ch-1].dry_threshold);
        } else { Serial.println("ERROR: invalid args"); }

    } else if (line.startsWith("SET_WET ")) {
        int ch, val;
        if (sscanf(line.c_str(), "SET_WET %d %d", &ch, &val) == 2 && ch >= 1 && ch <= 3) {
            cfg[ch - 1].wet_threshold = constrain(val, 0, 4095);
            Serial.printf("OK: Channel %d wet_threshold = %d\n", ch, cfg[ch-1].wet_threshold);
        } else { Serial.println("ERROR: invalid args"); }

    } else if (line.startsWith("SET_MIN_WATER ")) {
        int ch; long ms;
        if (sscanf(line.c_str(), "SET_MIN_WATER %d %ld", &ch, &ms) == 2 && ch >= 1 && ch <= 3) {
            cfg[ch - 1].min_water_ms = ms;
            Serial.printf("OK: Channel %d min_water_ms = %ld\n", ch, ms);
        } else { Serial.println("ERROR: invalid args"); }

    } else if (line.startsWith("SET_MAX_WATER ")) {
        int ch; long ms;
        if (sscanf(line.c_str(), "SET_MAX_WATER %d %ld", &ch, &ms) == 2 && ch >= 1 && ch <= 3) {
            cfg[ch - 1].max_water_ms = ms;
            Serial.printf("OK: Channel %d max_water_ms = %ld\n", ch, ms);
        } else { Serial.println("ERROR: invalid args"); }

    } else if (line.startsWith("SET_CYCLE ")) {
        long ms;
        if (sscanf(line.c_str(), "SET_CYCLE %ld", &ms) == 1) {
            cycle_interval_ms = ms;
            Serial.printf("OK: cycle_interval_ms = %ld\n", ms);
        } else { Serial.println("ERROR: invalid args"); }

    } else if (line == "STATUS") {
        printStatus();

    } else if (line == "HELP") {
        printHelp();

    } else if (line == "RESET_WIFI") {
        preferences.begin("wifi-config", false);
        preferences.clear();
        preferences.end();

        Serial.println("[NOTIFICATION] Wi-Fi settings cleared. Restarting...");
        delay(2000);
        ESP.restart();
    }
    
    else {
        Serial.println("ERROR: unknown command. Type HELP.");
    }
}

String printLocalTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to fetch time");
        return "null";
    }

    char timeStringBuff[20] = "";
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}

// ---------------------------------------------------------------
// MAIN IRRIGATION CYCLE LOGIC
// ---------------------------------------------------------------

/**
 * Proses satu channel:
 * 1. Baca sensor (sensor ON -> baca -> sensor OFF)
 * 2. Jika kering: buka solenoid selama durasi terhitung
 * 3. Baca sensor lagi untuk verifikasi
 * 4. Log hasil
 *
 * Prinsip: tidak ada dua perangkat hidup bersamaan.
 * Fungsi ini blocking selama solenoid terbuka.
 */
void processChannel(int ch) {
    Serial.printf("\n--- Channel %d ---\n", ch + 1);

    // STEP 1: Baca sensor sebelum
    Serial.printf("[CH%d] Membaca sensor...\n", ch + 1);
    int adc_before = readSensor(ch);
    state[ch].last_adc_before = adc_before;
    Serial.printf("[CH%d] ADC = %d  (dry>=%d, wet<=%d)\n",
        ch + 1, adc_before, cfg[ch].dry_threshold, cfg[ch].wet_threshold);

    int clampedAdc = constrain(adc_before, 1800, 2400);
    int percentage = map(clampedAdc, 1800, 2400, 100, 0);
    String strPercent = (String) percentage;
    potMoisturePercent[ch] = strPercent + "%";

    // Cek kondisi tanah
    if (adc_before < cfg[ch].dry_threshold) {
        // Tanah BASAH atau SEDANG - tidak perlu disiram
        Serial.printf("[CH%d] Status: CUKUP BASAH. Skip siram.\n", ch + 1);
        state[ch].last_watered = false;
        state[ch].last_adc_after = adc_before;
        potAction[ch] = "Pump OFF";
        potPumpDuration[ch] = "0 seconds";
        potSoilCondition[ch] = "Normal";
        potTimestampSensor[ch] = printLocalTime();
        return;
    }

    // STEP 2: Tanah KERING -> siram
    long dur = calcWaterDuration(ch, adc_before);
    state[ch].last_water_duration_ms = dur;
    state[ch].last_watered = true;

    Serial.printf("[CH%d] Status: KERING. Membuka solenoid selama %ld ms...\n", ch + 1, dur);
    potAction[ch] = "Pump ON";
    potSoilCondition[ch] = "Dry";
    potPumpDuration[ch] = String(dur / 1000.0, 1) + " seconds";
    potTimestampSensor[ch] = printLocalTime();

    openSolenoid(ch);
    delay(dur);
    closeSolenoid(ch);
    Serial.printf("[CH%d] Solenoid ditutup.\n", ch + 1);

    // Beri jeda sejenak sebelum baca sensor verifikasi
    delay(500);

    // STEP 3: Baca sensor setelah siram (verifikasi solenoid bekerja)
    Serial.printf("[CH%d] Membaca sensor verifikasi...\n", ch + 1);
    int adc_after = readSensor(ch);
    state[ch].last_adc_after = adc_after;

    // Jika ADC turun (tanah jadi lebih basah), solenoid bekerja
    state[ch].solenoid_ok = (adc_after < adc_before);
    Serial.printf("[CH%d] ADC setelah siram = %d  -> Solenoid %s\n",
        ch + 1, adc_after,
        state[ch].solenoid_ok ? "BEKERJA NORMAL" : "KEMUNGKINAN TIDAK BERFUNGSI!");
}

void sendDataToServer() {
    String data = "{\"address\": \"" + WiFi.macAddress() + "\",";
    data += "\"email\": \"" + email + "\",";
    data += "\"pots\": [";

    for (int i = 0; i < 3; i++) {
        data += potData[i];
    }

    data += "]}";
    int httpResponseCode = http.POST(data);
    Serial.println("Server Status: " + String(httpResponseCode));

    http.end();
}

void buildData(int order) {
    String index = (String) order;

    String potContent = "{\"potIndex\": " + index + ",";
    potContent += "\"sensorValue\": \"" + (String) potSensorValue[order] + "\",";
    potContent += "\"moisturePercent\": \"" + potMoisturePercent[order] + "\",";
    potContent += "\"soilCondition\": \"" + potSoilCondition[order] + "\",";
    potContent += "\"action\": \"" + potAction[order] + "\",";
    potContent += "\"pumpDuration\": \"" + potPumpDuration[order] + "\",";
    potContent += "\"timestampSensor\": \"" + potTimestampSensor[order] + "\"}";

    if (order != 2) {
        potContent += ",";
    };

    potData[order] = potContent;
}

// ---------------------------------------------------------------
// SETUP & LOOP
// ---------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n===================================");
    Serial.println("  Irrigation Controller - ESP32");
    Serial.println("===================================");

    // Inisialisasi pin sensor power
    for (int i = 0; i < 3; i++) {
        pinMode(PIN_SENSOR_PWR[i], OUTPUT);
        digitalWrite(PIN_SENSOR_PWR[i], LOW); // sensor OFF by default
    }

    // Inisialisasi pin solenoid (transistor base)
    for (int i = 0; i < 3; i++) {
        pinMode(PIN_SOLENOID[i], OUTPUT);
        digitalWrite(PIN_SOLENOID[i], LOW); // solenoid TUTUP by default
    }

    // Wi-Fi LED
    for (int i = 0; i < 2; i++) {
      pinMode(PIN_WIFI_LED[i], OUTPUT);
      digitalWrite(PIN_WIFI_LED[i], LOW);
    }

    // ADC setup
    analogReadResolution(12); // 12-bit ADC (0-4095)
    analogSetAttenuation(ADC_11db); // range ~0-3.3V

    connectToWiFi();
    http.begin("http://192.168.1.1:8081/api/sensor-readings");
    http.addHeader("Content-Type", "application/json");

    Serial.println("Inisialisasi selesai.");
    printHelp();
}

void loop() {
    // Cek dan proses perintah serial kapan saja
    server.handleClient();
    handleSerial();
    // handleSaveConfig();

    // Jalankan satu siklus scan semua channel secara bergantian
    Serial.println("\n====== MULAI SIKLUS SCAN ======");
    for (int ch = 0; ch < 3; ch++) {
        processChannel(ch);
        buildData(ch);
        // Jeda kecil antar channel untuk memastikan tidak ada overlap
        delay(300);
        // Proses serial di sela-sela channel
        handleSerial();
    }

    sendDataToServer();
    Serial.println("====== SIKLUS SELESAI ======");

    // Tunggu sebelum siklus berikutnya (sambil tetap merespons serial)
    long elapsed = 0;
    long start = millis();
    while (elapsed < cycle_interval_ms) {
        handleSerial();
        delay(50);
        elapsed = millis() - start;
    }
}