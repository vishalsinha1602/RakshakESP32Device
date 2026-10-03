#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#include "secrets.h"   // Wi-Fi and GeoGuard device credentials

// Add these two values to secrets.h using the ESP32 credentials
// shown once when creating an ESP32 device in GeoGuard.
#ifndef GEOGUARD_DEVICE_PUBLIC_ID
#define GEOGUARD_DEVICE_PUBLIC_ID ""
#endif
#ifndef GEOGUARD_DEVICE_KEY
#define GEOGUARD_DEVICE_KEY ""
#endif
#ifndef GEOGUARD_USE_LOCAL_HTTP
#define GEOGUARD_USE_LOCAL_HTTP 0
#endif
#ifndef GEOGUARD_LOCAL_HTTP_URL
// Replace 192.168.1.100 with the LAN IPv4 address of your development PC.
#define GEOGUARD_LOCAL_HTTP_URL "http://192.168.1.100:8080/api/v1/devices/locations/iot"
#endif

#if GEOGUARD_USE_LOCAL_HTTP
const char* GEOGUARD_LOCATION_URL = GEOGUARD_LOCAL_HTTP_URL;
#else
const char* GEOGUARD_LOCATION_URL =
    "https://geoguard-backend.onrender.com/api/v1/devices/locations/iot";
#endif
const unsigned long LOCATION_UPDATE_INTERVAL_MS = 2000;
unsigned long lastLocationUpdate = 0;
TaskHandle_t geoGuardTaskHandle = NULL;
volatile bool geoGuardSendRequested = false;
volatile bool geoGuardSendInFlight = false;
double geoGuardLatitude = 0;
double geoGuardLongitude = 0;
double geoGuardSpeed = 0;


// =====================================================
//                         OLED
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA 21
#define OLED_SCL 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// OLED status. The device continues to run if the OLED is disconnected.
bool oledAvailable = false;
uint8_t oledI2CAddress = 0x3C;

// =====================================================
//                 GPS ACCURACY FILTER
// =====================================================

constexpr uint8_t GPS_FILTER_WINDOW = 7;
constexpr uint8_t MIN_SATELLITES = 5;
constexpr double MAX_HDOP = 4.0;
constexpr unsigned long MAX_FIX_AGE_MS = 3000;
constexpr double STATIONARY_SPEED_KMPH = 2.5;
constexpr double STATIONARY_RADIUS_METERS = 18.0;
constexpr double MAX_GPS_JUMP_METERS = 120.0;
constexpr uint8_t MOVEMENT_CONFIRMATIONS_REQUIRED = 3;

struct GPSFilteredPoint {
  double latitude = 0.0;
  double longitude = 0.0;
  double speedKmph = 0.0;
  bool valid = false;
};

GPSFilteredPoint filteredLocation;
double latitudeSamples[GPS_FILTER_WINDOW];
double longitudeSamples[GPS_FILTER_WINDOW];
uint8_t gpsSampleCount = 0;
uint8_t gpsSampleIndex = 0;

bool stationaryLock = false;
uint8_t movementConfirmations = 0;
double lastCandidateLatitude = 0.0;
double lastCandidateLongitude = 0.0;
unsigned long lastFilteredFixTime = 0;

// =====================================================
//                 OLED HELPERS
// =====================================================

void oledShowText(const char* title, const char* line1 = nullptr, const char* line2 = nullptr) {
  if (!oledAvailable) {
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 5);
  display.println(title);

  if (line1 != nullptr) {
    display.setCursor(0, 25);
    display.println(line1);
  }

  if (line2 != nullptr) {
    display.setCursor(0, 45);
    display.println(line2);
  }

  display.display();
}

bool i2cDeviceExists(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool initializeOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);
  delay(100);

  Serial.println("Scanning I2C for OLED...");

  if (i2cDeviceExists(0x3C)) {
    oledI2CAddress = 0x3C;
  } else if (i2cDeviceExists(0x3D)) {
    oledI2CAddress = 0x3D;
  } else {
    Serial.println("OLED I2C device not found at 0x3C or 0x3D");
    oledAvailable = false;
    return false;
  }

  Serial.print("OLED found at 0x");
  Serial.println(oledI2CAddress, HEX);

  oledAvailable = display.begin(SSD1306_SWITCHCAPVCC, oledI2CAddress);

  if (!oledAvailable) {
    Serial.println("SSD1306 initialization failed");
    return false;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("RAKSHAK");
  display.setTextSize(1);
  display.setCursor(0, 32);
  display.println("OLED OK");
  display.display();
  delay(800);

  return true;
}


// =====================================================
//                          GPS
// =====================================================

#define GPS_RX 16
#define GPS_TX 17

TinyGPSPlus gps;
HardwareSerial GPS(2);


// =====================================================
//                 BUTTON / LED / MOTOR
// =====================================================

#define BUTTON_PIN 25
#define LED_PIN 32
#define MOTOR_PIN 26
#define BATTERY_ADC_PIN 34

// Measure the 18650 cell before the DC-DC converter using a 100k/100k divider.
// The divider halves the maximum 4.2 V cell voltage at the ESP32 ADC input.
const float BATTERY_DIVIDER_RATIO = 2.0f;

const unsigned long DEBOUNCE_DELAY_MS = 40;


// =====================================================
//                  SYSTEM STATE MACHINE
// =====================================================

enum SystemState {

  STATE_GPS_SEARCHING,
  STATE_GPS_LOCKED,

  // Emergency button is currently active
  STATE_EMERGENCY

};

SystemState state = STATE_GPS_SEARCHING;


// =====================================================
//                 BUTTON PRESS LATCH
// =====================================================

// Prevents another alert while the same physical
// button press is still being held.

bool emergencyPressHandled = false;


// =====================================================
//              LED BLINK TASK
// =====================================================

// Emergency LED
volatile bool emergencyBlinkTaskEnabled = false;

// GPS searching LED
volatile bool gpsBlinkTaskEnabled = false;

TaskHandle_t emergencyLedTaskHandle = NULL;


// Emergency location delivery uses the GeoGuard sender task.
volatile bool geoGuardSendForEmergency = false;
volatile bool emergencyLocationPending = false;
volatile bool emergencyLocationInFlight = false;
volatile bool emergencyLocationSucceeded = false;
volatile bool emergencyLocationFailed = false;


// =====================================================
//                         BUTTON
// =====================================================

int lastRawButtonState = HIGH;
int debouncedButtonState = HIGH;

unsigned long lastButtonEdgeTime = 0;


// =====================================================
//                    OLED ANIMATION
// =====================================================

int loadingFrame = 0;

unsigned long lastAnimationTime = 0;

const unsigned long ANIMATION_TIME = 250;


// =====================================================
//                     SPEED FILTER
// =====================================================

#define SPEED_SAMPLES 5

double speedSamples[SPEED_SAMPLES];

int speedIndex = 0;

bool speedBufferFull = false;


unsigned long lastEmergencyLocationAttempt = 0;
const unsigned long EMERGENCY_LOCATION_RETRY_MS = 2000;


// =====================================================
//              LED BLINK TASK
// =====================================================
//
// GPS SEARCHING and EMERGENCY use this SAME task.
//
// Both blink:
// ON  -> 200 ms
// OFF -> 200 ms
//
// Therefore the physical LED behavior is identical.
// =====================================================

void emergencyLedTask(void *parameter) {

  bool ledState = false;

  while (true) {

    // =================================================
    // GPS SEARCHING OR EMERGENCY
    // =================================================

    if (
      emergencyBlinkTaskEnabled ||
      gpsBlinkTaskEnabled
    ) {

      ledState = !ledState;

      digitalWrite(
        LED_PIN,
        ledState
      );

      // EXACT SAME BLINK TIMING
      vTaskDelay(
        pdMS_TO_TICKS(200)
      );
    }

    // =================================================
    // LED OFF
    // =================================================

    else {

      ledState = false;

      digitalWrite(
        LED_PIN,
        LOW
      );

      vTaskDelay(
        pdMS_TO_TICKS(20)
      );
    }
  }
}



// =====================================================
//                 GPS FILTER UTILITIES
// =====================================================

double gpsDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double EARTH_RADIUS_M = 6371000.0;
  const double degToRad = 0.017453292519943295;

  const double p1 = lat1 * degToRad;
  const double p2 = lat2 * degToRad;
  const double dLat = (lat2 - lat1) * degToRad;
  const double dLon = (lon2 - lon1) * degToRad;

  const double a =
      sin(dLat / 2.0) * sin(dLat / 2.0) +
      cos(p1) * cos(p2) *
      sin(dLon / 2.0) * sin(dLon / 2.0);

  return 2.0 * EARTH_RADIUS_M * asin(sqrt(a));
}

double medianOf(double* values, uint8_t count) {
  double sorted[GPS_FILTER_WINDOW];

  for (uint8_t i = 0; i < count; ++i) {
    sorted[i] = values[i];
  }

  for (uint8_t i = 1; i < count; ++i) {
    double key = sorted[i];
    int8_t j = static_cast<int8_t>(i) - 1;

    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      --j;
    }

    sorted[j + 1] = key;
  }

  if (count % 2 == 1) {
    return sorted[count / 2];
  }

  return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0;
}

void resetGPSFilter() {
  gpsSampleCount = 0;
  gpsSampleIndex = 0;
  stationaryLock = false;
  movementConfirmations = 0;
  lastCandidateLatitude = 0.0;
  lastCandidateLongitude = 0.0;
  lastFilteredFixTime = 0;

  filteredLocation.latitude = 0.0;
  filteredLocation.longitude = 0.0;
  filteredLocation.speedKmph = 0.0;
  filteredLocation.valid = false;
}

bool gpsQualityIsGood() {
  if (!gps.location.isValid()) {
    return false;
  }

  if (gps.location.age() > MAX_FIX_AGE_MS) {
    return false;
  }

  if (!gps.satellites.isValid() || gps.satellites.value() < MIN_SATELLITES) {
    return false;
  }

  if (gps.hdop.isValid() && gps.hdop.hdop() > MAX_HDOP) {
    return false;
  }

  return true;
}

bool updateFilteredGPS() {
  if (!gpsQualityIsGood()) {
    return false;
  }

  const double rawLatitude = gps.location.lat();
  const double rawLongitude = gps.location.lng();
  double rawSpeed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

  if (rawSpeed < 0.8) {
    rawSpeed = 0.0;
  }

  latitudeSamples[gpsSampleIndex] = rawLatitude;
  longitudeSamples[gpsSampleIndex] = rawLongitude;

  if (gpsSampleCount < GPS_FILTER_WINDOW) {
    gpsSampleCount++;
  }

  gpsSampleIndex = (gpsSampleIndex + 1) % GPS_FILTER_WINDOW;

  const double candidateLatitude = medianOf(latitudeSamples, gpsSampleCount);
  const double candidateLongitude = medianOf(longitudeSamples, gpsSampleCount);

  if (!filteredLocation.valid) {
    filteredLocation.latitude = candidateLatitude;
    filteredLocation.longitude = candidateLongitude;
    filteredLocation.speedKmph = rawSpeed;
    filteredLocation.valid = true;
    stationaryLock = rawSpeed < STATIONARY_SPEED_KMPH;
    lastFilteredFixTime = millis();
    movementConfirmations = 0;
    return true;
  }

  const double distance = gpsDistanceMeters(
      filteredLocation.latitude,
      filteredLocation.longitude,
      candidateLatitude,
      candidateLongitude);

  unsigned long now = millis();
  double elapsedSeconds =
      lastFilteredFixTime == 0
          ? 2.0
          : (now - lastFilteredFixTime) / 1000.0;

  if (elapsedSeconds < 0.5) {
    elapsedSeconds = 0.5;
  }

  const double speedMps = rawSpeed / 3.6;
  const double expectedDistance = speedMps * elapsedSeconds * 3.0 + 20.0;
  const double allowedJump = min(MAX_GPS_JUMP_METERS, max(30.0, expectedDistance));

  // Hard rejection of an obvious GPS spike.
  if (distance > allowedJump && rawSpeed < 80.0) {
    Serial.print("GPS OUTLIER REJECTED: ");
    Serial.print(distance, 1);
    Serial.println(" m");
    return false;
  }

  // When stationary, hold a stable anchor against normal GPS wander.
  if (stationaryLock && rawSpeed < STATIONARY_SPEED_KMPH) {
    if (distance <= STATIONARY_RADIUS_METERS) {
      filteredLocation.speedKmph = 0.0;
      lastFilteredFixTime = now;
      movementConfirmations = 0;
      return true;
    }

    // Require several consistent fixes before leaving stationary lock.
    if (movementConfirmations == 0) {
      lastCandidateLatitude = candidateLatitude;
      lastCandidateLongitude = candidateLongitude;
      movementConfirmations = 1;
      return true;
    }

    const double candidateProgress = gpsDistanceMeters(
        lastCandidateLatitude,
        lastCandidateLongitude,
        candidateLatitude,
        candidateLongitude);

    if (candidateProgress >= 5.0) {
      lastCandidateLatitude = candidateLatitude;
      lastCandidateLongitude = candidateLongitude;
      movementConfirmations++;
    } else {
      movementConfirmations = 1;
      lastCandidateLatitude = candidateLatitude;
      lastCandidateLongitude = candidateLongitude;
    }

    if (movementConfirmations < MOVEMENT_CONFIRMATIONS_REQUIRED) {
      return true;
    }

    stationaryLock = false;
    movementConfirmations = 0;
  }

  filteredLocation.latitude = candidateLatitude;
  filteredLocation.longitude = candidateLongitude;
  filteredLocation.speedKmph = rawSpeed;
  lastFilteredFixTime = now;

  if (rawSpeed < STATIONARY_SPEED_KMPH && distance <= STATIONARY_RADIUS_METERS) {
    stationaryLock = true;
    filteredLocation.speedKmph = 0.0;
  }

  return true;
}

// =====================================================
//                         SETUP
// =====================================================

void setup() {

  Serial.begin(115200);
  delay(200);

  // ===================================================
  // BUTTON
  // ===================================================

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastRawButtonState = digitalRead(BUTTON_PIN);
  debouncedButtonState = lastRawButtonState;

  // ===================================================
  // LED
  // ===================================================

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ===================================================
  // VIBRATION MOTOR
  // ===================================================

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // ===================================================
  // BATTERY ADC
  // ===================================================

  pinMode(BATTERY_ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  // ===================================================
  // OLED FIRST
  // ===================================================

  initializeOLED();

  if (!oledAvailable) {
    Serial.println("Continuing without OLED.");
  }

  // ===================================================
  // GPS
  // ===================================================

  GPS.begin(
      9600,
      SERIAL_8N1,
      GPS_RX,
      GPS_TX
  );

  resetGPSFilter();

  // ===================================================
  // WIFI
  // ===================================================

  connectWiFi();

  // ===================================================
  // START GPS SEARCH
  // ===================================================

  gpsBlinkTaskEnabled = true;
  showGPSSearchingScreen();

  // ===================================================
  // BACKGROUND TASKS LAST
  // ===================================================

  xTaskCreatePinnedToCore(
      emergencyLedTask,
      "EmergencyLED",
      2048,
      NULL,
      1,
      &emergencyLedTaskHandle,
      0
  );

  xTaskCreatePinnedToCore(
      geoGuardTask,
      "GeoGuardSend",
      8192,
      NULL,
      1,
      &geoGuardTaskHandle,
      0
  );

  Serial.println();
  Serial.println("==============================");
  Serial.println("       RAKSHAK READY");
  Serial.println("==============================");
  Serial.println("Button : GPIO 25");
  Serial.println("LED    : GPIO 32");
  Serial.println("Motor  : GPIO 26");
  Serial.println("GPS RX : GPIO 16");
  Serial.println("GPS TX : GPIO 17");
  Serial.println("OLED SDA: GPIO 21");
  Serial.println("OLED SCL: GPIO 22");
}


// =====================================================
//                          LOOP
// =====================================================

void loop() {

  // ===================================================
  // READ GPS
  // ===================================================

  while (GPS.available()) {

    gps.encode(
      GPS.read()
    );
  }


  static unsigned long lastGpsDiagnostic = 0;
  if (millis() - lastGpsDiagnostic >= 2000) {
    lastGpsDiagnostic = millis();

    Serial.print("RAW GPS | valid: ");
    Serial.print(gps.location.isValid() ? "YES" : "NO");
    Serial.print(" | age: ");
    Serial.print(gps.location.age());
    Serial.print(" ms | SAT: ");
    Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
    Serial.print(" | HDOP: ");
    if (gps.hdop.isValid()) {
      Serial.print(gps.hdop.hdop(), 2);
    } else {
      Serial.print("N/A");
    }

    if (gps.location.isValid()) {
      Serial.print(" | RAW: ");
      Serial.print(gps.location.lat(), 7);
      Serial.print(",");
      Serial.print(gps.location.lng(), 7);
    }

    if (filteredLocation.valid) {
      Serial.print(" | FILTERED: ");
      Serial.print(filteredLocation.latitude, 7);
      Serial.print(",");
      Serial.print(filteredLocation.longitude, 7);
    }

    Serial.println();
  }

  // Send only a filtered, quality-checked GPS position to GeoGuard.
  if (state != STATE_EMERGENCY &&
      !geoGuardSendRequested &&
      !geoGuardSendInFlight &&
      (lastLocationUpdate == 0 ||
       millis() - lastLocationUpdate >= LOCATION_UPDATE_INTERVAL_MS)) {

    const bool filterUpdated = updateFilteredGPS();

    if (filterUpdated && filteredLocation.valid) {
      lastLocationUpdate = millis();
      geoGuardLatitude = filteredLocation.latitude;
      geoGuardLongitude = filteredLocation.longitude;
      geoGuardSpeed = filteredLocation.speedKmph;
      geoGuardSendRequested = true;

      Serial.print("GPS -> FILTERED: ");
      Serial.print(geoGuardLatitude, 7);
      Serial.print(", ");
      Serial.print(geoGuardLongitude, 7);
      Serial.print(" | SAT: ");
      Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
      Serial.print(" | HDOP: ");
      if (gps.hdop.isValid()) {
        Serial.println(gps.hdop.hdop(), 2);
      } else {
        Serial.println("N/A");
      }
    }
  }


  // ===================================================
  // BUTTON
  // ===================================================

  bool pressedEdge = false;

  bool releasedEdge = false;


  updateButton(
    pressedEdge,
    releasedEdge
  );


  // ===================================================
  // BUTTON PRESSED
  // ===================================================

  if (
    pressedEdge &&
    !emergencyPressHandled
  ) {

    emergencyPressHandled = true;


    Serial.println();
    Serial.println("==============================");
    Serial.println(
      " EMERGENCY BUTTON PRESSED"
    );
    Serial.println("==============================");


    state =
        STATE_EMERGENCY;


    // Turn GPS blink off
    // because emergency blink will take over.
    gpsBlinkTaskEnabled = false;


    // Start emergency LED
    emergencyBlinkTaskEnabled = true;


    // Start vibration
    digitalWrite(
      MOTOR_PIN,
      HIGH
    );


    // Reset OLED animation
    loadingFrame = 0;

    lastAnimationTime =
        millis();


    showEmergencyScreen();
  }


  // ===================================================
  // BUTTON RELEASED
  // ===================================================

  if (releasedEdge) {

    Serial.println();
    Serial.println(
      "EMERGENCY BUTTON RELEASED"
    );


    // Stop vibration
    digitalWrite(
      MOTOR_PIN,
      LOW
    );


    // Stop emergency LED
    emergencyBlinkTaskEnabled = false;

    digitalWrite(
      LED_PIN,
      LOW
    );


    // Allow next press
    emergencyPressHandled = false;


    // If we are still in emergency state,
    // return to the appropriate GPS state.

    if (
      state == STATE_EMERGENCY
    ) {

      if (
        filteredLocation.valid
      ) {

        // GPS is already locked,
        // so LED must remain OFF.

        gpsBlinkTaskEnabled = false;

        digitalWrite(
          LED_PIN,
          LOW
        );


        state =
            STATE_GPS_LOCKED;


        showGPSData();
      }

      else {

        // GPS still searching,
        // enable SAME emergency-style blink.

        state =
            STATE_GPS_SEARCHING;


        resetSearchAnimation();


        showGPSSearchingScreen();
      }
    }
  }


  // ===================================================
  // EMERGENCY MODE
  // ===================================================

  if (
    state == STATE_EMERGENCY
  ) {

    handleEmergencyPending();

    delay(10);

    return;
  }


  // ===================================================
  // NORMAL GPS SEARCHING
  // ===================================================

  if (
    state == STATE_GPS_SEARCHING
  ) {

    if (
      filteredLocation.valid
    ) {

      // GPS LOCKED
      // Stop searching LED.

      gpsBlinkTaskEnabled = false;

      digitalWrite(
        LED_PIN,
        LOW
      );


      state =
          STATE_GPS_LOCKED;


      showGPSData();
    }

    else {

      handleGPSSearching();
    }
  }


  // ===================================================
  // NORMAL GPS LOCKED
  // ===================================================

  else if (
    state == STATE_GPS_LOCKED
  ) {

    if (
      !filteredLocation.valid
    ) {

      state =
          STATE_GPS_SEARCHING;
      resetGPSFilter();


      // Start SAME blink as emergency
      resetSearchAnimation();
    }

    else {

      // GPS locked = LED OFF

      gpsBlinkTaskEnabled = false;

      emergencyBlinkTaskEnabled = false;

      digitalWrite(
        LED_PIN,
        LOW
      );


      showGPSData();
    }
  }


  delay(20);
}


// =====================================================
//                    BUTTON DEBOUNCE
// =====================================================

void updateButton(
  bool &pressedEdge,
  bool &releasedEdge
) {

  int raw =
      digitalRead(BUTTON_PIN);


  if (
    raw != lastRawButtonState
  ) {

    lastButtonEdgeTime =
        millis();

    lastRawButtonState =
        raw;
  }


  if (
    millis() - lastButtonEdgeTime >
    DEBOUNCE_DELAY_MS
  ) {

    if (
      raw != debouncedButtonState
    ) {

      debouncedButtonState =
          raw;


      if (
        debouncedButtonState == LOW
      ) {

        pressedEdge = true;
      }

      else {

        releasedEdge = true;
      }
    }
  }
}


// =====================================================
//             HANDLE EMERGENCY BUTTON
// =====================================================

void handleEmergencyPending() {
  if (!emergencyLocationInFlight && emergencyLocationSucceeded) {
    emergencyLocationSucceeded = false;
    emergencyLocationPending = false;
    Serial.println("EMERGENCY LOCATION ACCEPTED BY GEOGUARD");

    emergencyBlinkTaskEnabled = false;
    digitalWrite(LED_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);

    if (gps.location.isValid()) {
      gpsBlinkTaskEnabled = false;
      state = STATE_GPS_LOCKED;
      showGPSData();
    } else {
      state = STATE_GPS_SEARCHING;
      resetSearchAnimation();
      showGPSSearchingScreen();
    }
    return;
  }

  if (!emergencyLocationInFlight && emergencyLocationFailed) {
    emergencyLocationFailed = false;
    emergencyLocationPending = false;
    Serial.println("GeoGuard send failed; emergency location will retry");
  }

  showEmergencyScreen();

  if (filteredLocation.valid &&
      !emergencyLocationPending &&
      !geoGuardSendRequested &&
      !geoGuardSendInFlight &&
      (lastEmergencyLocationAttempt == 0 ||
       millis() - lastEmergencyLocationAttempt >= EMERGENCY_LOCATION_RETRY_MS)) {
    lastEmergencyLocationAttempt = millis();
    geoGuardLatitude = filteredLocation.latitude;
    geoGuardLongitude = filteredLocation.longitude;
    geoGuardSpeed = filteredLocation.speedKmph;

    emergencyLocationPending = true;
    emergencyLocationInFlight = false;
    emergencyLocationSucceeded = false;
    emergencyLocationFailed = false;
    geoGuardSendForEmergency = true;
    geoGuardSendRequested = true;

    Serial.println("GPS LOCATION READY - SENDING SOS TO GEOGUARD");
    Serial.print("LAT: ");
    Serial.println(geoGuardLatitude, 6);
    Serial.print("LON: ");
    Serial.println(geoGuardLongitude, 6);
  }

  if (!filteredLocation.valid) {
    Serial.println("GPS QUALITY NOT READY - WAITING");
  }
}

// =====================================================
//                    CONNECT WIFI
// =====================================================

bool connectWiFi() {

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.println();
  Serial.println("==============================");
  Serial.println("      WIFI CONNECTION");
  Serial.println("==============================");

  WiFi.mode(WIFI_STA);

  const char* jnuNetworks[] = {
    "JNU Academic",
    "JNU Campus"
  };

  for (int i = 0; i < 2; i++) {

    Serial.println();
    Serial.print("Trying JNU WiFi: ");
    Serial.println(jnuNetworks[i]);

    if (oledAvailable) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 5);
      display.println("CONNECTING WIFI");
      display.setCursor(0, 25);
      display.println(jnuNetworks[i]);
      display.setCursor(0, 45);
      display.println("PEAP / MSCHAPV2");
      display.display();
    }

    WiFi.disconnect(true);
    delay(500);

    WiFi.begin(
      jnuNetworks[i],
      WPA2_AUTH_PEAP,
      JNU_IDENTITY,
      JNU_USERNAME,
      JNU_PASSWORD
    );

    unsigned long start = millis();

    while (
      WiFi.status() != WL_CONNECTED &&
      millis() - start < 10000
    ) {
      while (GPS.available()) {
        gps.encode(GPS.read());
      }

      delay(300);
      Serial.print(".");
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {

      Serial.println();
      Serial.println("==============================");
      Serial.println("JNU WIFI CONNECTED!");
      Serial.println("==============================");
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());

      if (oledAvailable) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 5);
        display.println("WIFI CONNECTED");
        display.setCursor(0, 20);
        display.println(WiFi.SSID() == "JNU Academic" ? "JNU ACADEMIC" : "JNU CAMPUS");
        display.setCursor(0, 35);
        display.print("IP: ");
        display.println(WiFi.localIP());
        display.setCursor(0, 50);
        display.println("Internet Ready");
        display.display();
      }

      delay(1500);
      return true;
    }

    Serial.print("Failed: ");
    Serial.println(jnuNetworks[i]);
  }

  Serial.println();
  Serial.println("Both JNU WiFi networks unavailable.");
  Serial.println();
  Serial.println("Trying phone hotspot...");

  if (oledAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 5);
    display.println("CONNECTING WIFI");
    display.setCursor(0, 25);
    display.println("PHONE HOTSPOT");
    display.setCursor(0, 45);
    display.println(WIFI1_SSID);
    display.display();
  }

  WiFi.disconnect(true);
  delay(500);

  WiFi.begin(
    WIFI1_SSID,
    WIFI1_PASSWORD
  );

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 10000
  ) {
    while (GPS.available()) {
      gps.encode(GPS.read());
    }

    delay(300);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println();
    Serial.println("==============================");
    Serial.println("HOTSPOT WIFI CONNECTED!");
    Serial.println("==============================");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    if (oledAvailable) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 5);
      display.println("WIFI CONNECTED");
      display.setCursor(0, 20);
      display.println("PHONE HOTSPOT");
      display.setCursor(0, 35);
      display.print("SSID: ");
      display.println(WiFi.SSID());
      display.setCursor(0, 50);
      display.println("Internet Ready");
      display.display();
    }

    delay(1500);
    return true;
  }

  Serial.println();
  Serial.println("==============================");
  Serial.println("ALL WIFI CONNECTIONS FAILED");
  Serial.println("==============================");

  if (oledAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 5);
    display.println("WIFI FAILED");
    display.setCursor(0, 25);
    display.println("JNU: NOT AVAILABLE");
    display.setCursor(0, 40);
    display.println("HOTSPOT: NOT AVAILABLE");
    display.setCursor(0, 55);
    display.println("Retry required");
    display.display();
  }

  return false;
}


// =====================================================
//                 GEOGUARD LOCATION
// =====================================================

int readBatteryPercentage() {
  const int sampleCount = 16;
  uint32_t millivoltTotal = 0;

  for (int i = 0; i < sampleCount; i++) {
    millivoltTotal += analogReadMilliVolts(BATTERY_ADC_PIN);
    delay(4);
  }

  const float adcVoltage =
      (millivoltTotal / static_cast<float>(sampleCount)) / 1000.0f;
  const float cellVoltage = adcVoltage * BATTERY_DIVIDER_RATIO;

  // A disconnected divider or implausible reading is reported as unknown.
  if (cellVoltage < 2.5f || cellVoltage > 4.35f) {
    Serial.println("Battery ADC reading invalid; check divider wiring");
    return -1;
  }

  // Approximate 18650 state of charge from its Li-ion voltage curve.
  const float voltageTable[] = {
      4.20f, 4.15f, 4.11f, 4.08f, 4.02f, 3.98f, 3.95f,
      3.91f, 3.87f, 3.85f, 3.84f, 3.82f, 3.80f, 3.79f,
      3.77f, 3.75f, 3.73f, 3.71f, 3.69f, 3.61f, 3.30f
  };
  const int percentageTable[] = {
      100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50,
      45, 40, 35, 30, 25, 20, 15, 10, 5, 0
  };
  const int tableSize = sizeof(percentageTable) / sizeof(percentageTable[0]);

  if (cellVoltage >= voltageTable[0]) return 100;
  if (cellVoltage <= voltageTable[tableSize - 1]) return 0;

  for (int i = 0; i < tableSize - 1; i++) {
    if (cellVoltage <= voltageTable[i] && cellVoltage >= voltageTable[i + 1]) {
      const float fraction =
          (cellVoltage - voltageTable[i + 1]) /
          (voltageTable[i] - voltageTable[i + 1]);
      const float percentage = percentageTable[i + 1] +
          fraction * (percentageTable[i] - percentageTable[i + 1]);
      return constrain(static_cast<int>(roundf(percentage)), 0, 100);
    }
  }

  return -1;
}

bool postGeoGuardLocation(
  double latitude,
  double longitude,
  double speedKmph,
  bool sos
) {
  if (String(GEOGUARD_DEVICE_PUBLIC_ID).length() == 0 ||
      String(GEOGUARD_DEVICE_KEY).length() == 0) {
    Serial.println("GeoGuard credentials missing in secrets.h");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("GeoGuard update skipped: Wi-Fi disconnected");
    return false;
  }

#if GEOGUARD_USE_LOCAL_HTTP
  WiFiClient client;
#else
  WiFiClientSecure client;
  client.setInsecure();
#endif
  HTTPClient https;
  https.setTimeout(8000);

  if (!https.begin(client, GEOGUARD_LOCATION_URL)) {
    Serial.println("GeoGuard HTTPS begin failed");
    return false;
  }

  https.addHeader("Content-Type", "application/json");
  https.addHeader("X-Device-Key", GEOGUARD_DEVICE_KEY);

  String body = "{\"devicePublicId\":\"";
  body += GEOGUARD_DEVICE_PUBLIC_ID;
  body += "\",\"latitude\":";
  body += String(latitude, 7);
  body += ",\"longitude\":";
  body += String(longitude, 7);
  body += ",\"speed\":";
  body += String(speedKmph, 2);
  const int batteryPercentage = readBatteryPercentage();
  body += ",\"batteryLevel\":";
  body += batteryPercentage >= 0 ? String(batteryPercentage) : "null";
  body += ",\"sos\":";
  body += sos ? "true}" : "false}";

  int httpCode = https.POST(body);
  Serial.print("GeoGuard HTTP: ");
  Serial.println(httpCode);

  if (httpCode > 0) {
    Serial.println(https.getString());
  }
  https.end();
  return httpCode >= 200 && httpCode < 300;
}

void geoGuardTask(void *parameter) {
  for (;;) {
    if (!geoGuardSendRequested) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    const bool emergencySend = geoGuardSendForEmergency;
    geoGuardSendInFlight = true;
    geoGuardSendRequested = false;
    geoGuardSendForEmergency = false;
    if (emergencySend) {
      emergencyLocationInFlight = true;
    }

    bool sent = postGeoGuardLocation(
      geoGuardLatitude,
      geoGuardLongitude,
      geoGuardSpeed,
      emergencySend
    );

    if (emergencySend) {
      emergencyLocationSucceeded = sent;
      emergencyLocationFailed = !sent;
      emergencyLocationInFlight = false;
    }

    Serial.println(sent ? "GeoGuard location accepted" : "GeoGuard location send failed");
    geoGuardSendInFlight = false;
  }
}


// =====================================================
//                  GPS SEARCHING
// =====================================================

void handleGPSSearching() {

  // ===================================================
  // USE SAME LED TASK AS EMERGENCY
  // ===================================================

  gpsBlinkTaskEnabled = true;


  // OLED searching animation
  showGPSSearchingScreen();
}


// =====================================================
//              GPS SEARCHING SCREEN
// =====================================================

void showGPSSearchingScreen() {

  if (!oledAvailable) {
    return;
  }


  if (
    millis() - lastAnimationTime <
    ANIMATION_TIME
  ) {

    return;
  }


  lastAnimationTime =
      millis();


  loadingFrame++;


  if (
    loadingFrame >= 4
  ) {

    loadingFrame = 0;
  }


  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );


  display.setTextSize(2);

  display.setCursor(
    0,
    0
  );

  display.println(
    "RAKSHAK"
  );


  display.setTextSize(1);

  display.setCursor(
    0,
    25
  );

  display.println(
    "Searching satellites"
  );


  display.setCursor(
    0,
    40
  );

  display.print(
    "SAT: "
  );

  display.println(
    gps.satellites.value()
  );


  display.setCursor(
    70,
    40
  );


  for (
    int i = 0;
    i < 4;
    i++
  ) {

    display.print(
      i == loadingFrame
        ? "*"
        : "."
    );

    display.print(" ");
  }


  display.display();
}


// =====================================================
//                  EMERGENCY SCREEN
// =====================================================

void showEmergencyScreen() {

  if (!oledAvailable) {
    return;
  }


  static unsigned long lastScreen = 0;


  // OLED update every 100 ms
  if (
    millis() - lastScreen < 100
  ) {

    return;
  }


  lastScreen =
      millis();


  // ===================================================
  // UPDATE ANIMATION
  // ===================================================

  if (
    millis() - lastAnimationTime >=
    ANIMATION_TIME
  ) {

    lastAnimationTime =
        millis();


    loadingFrame++;


    if (
      loadingFrame >= 4
    ) {

      loadingFrame = 0;
    }
  }


  // ===================================================
  // OLED
  // ===================================================

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );


  // ===================================================
  // TITLE
  // ===================================================

  display.setTextSize(2);

  display.setCursor(
    0,
    0
  );

  display.println(
    "EMERGENCY"
  );


  // ===================================================
  // STATUS
  // ===================================================

  display.setTextSize(1);

  display.setCursor(
    0,
    23
  );


  if (
    !gps.location.isValid()
  ) {

    // GPS SEARCHING

    display.println(
      "GPS: SEARCHING"
    );

    display.setCursor(
      0,
      40
    );

    display.print(
      "Waiting for GPS"
    );


    for (
      int i = 0;
      i < loadingFrame;
      i++
    ) {

      display.print(".");
    }
  }


  else if (
    emergencyLocationInFlight ||
    (emergencyLocationPending &&
     (geoGuardSendRequested || geoGuardSendInFlight))
  ) {

    // SENDING ALERT

    display.println(
      "GPS: READY"
    );

    display.setCursor(
      0,
      40
    );

    display.print(
      "Sending to GeoGuard"
    );


    for (
      int i = 0;
      i <= loadingFrame;
      i++
    ) {

      display.print(".");
    }
  }


  else {

    // GPS READY, WAITING TO SEND

    display.println(
      "GPS: READY"
    );

    display.setCursor(
      0,
      40
    );

    display.print(
      "Preparing alert"
    );


    for (
      int i = 0;
      i <= loadingFrame;
      i++
    ) {

      display.print(".");
    }
  }


  // ===================================================
  // DISPLAY
  // ===================================================

  display.display();
}


// =====================================================
//                    SPEED FILTER
// =====================================================

double getFilteredSpeed() {

  double currentSpeed =
      filteredLocation.valid ? filteredLocation.speedKmph : 0.0;


  // Ignore tiny GPS movement

  if (
    currentSpeed < 0.5
  ) {

    currentSpeed = 0.0;
  }


  speedSamples[speedIndex] =
      currentSpeed;


  speedIndex++;


  if (
    speedIndex >= SPEED_SAMPLES
  ) {

    speedIndex = 0;

    speedBufferFull = true;
  }


  int count =
      speedBufferFull
        ? SPEED_SAMPLES
        : max(
            speedIndex,
            1
          );


  double total = 0.0;


  for (
    int i = 0;
    i < count;
    i++
  ) {

    total +=
        speedSamples[i];
  }


  double average =
      total / count;


  if (
    average < 0.5
  ) {

    average = 0.0;
  }


  return average;
}


// =====================================================
//                     GPS DATA
// =====================================================

void showGPSData() {

  if (!oledAvailable) {
    return;
  }


  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);


  // ===================================================
  // HEADER
  // ===================================================

  display.setCursor(
    0,
    0
  );

  display.print(
    "GPS: RAKSHAK"
  );


  display.setCursor(
    78,
    0
  );

  display.print(
    "SAT:"
  );

  display.print(
    gps.satellites.value()
  );


  // ===================================================
  // LATITUDE
  // ===================================================

  display.setCursor(
    0,
    15
  );

  display.print(
    "LAT:"
  );

  display.println(
    filteredLocation.latitude,
    6
  );


  // ===================================================
  // LONGITUDE
  // ===================================================

  display.setCursor(
    0,
    29
  );

  display.print(
    "LON:"
  );

  display.println(
    filteredLocation.longitude,
    6
  );


  // ===================================================
  // SPEED
  // ===================================================

  double speed =
      getFilteredSpeed();


  display.setCursor(
    0,
    45
  );

  display.print(
    "SPD:"
  );

  display.print(
    speed,
    1
  );

  display.println(
    " km/h"
  );


  display.display();


  // ===================================================
  // SERIAL
  // ===================================================

  Serial.print(
    "SAT: "
  );

  Serial.print(
    gps.satellites.value()
  );


  Serial.print(
    " | LAT: "
  );

  Serial.print(
    gps.location.lat(),
    6
  );


  Serial.print(
    " | LON: "
  );

  Serial.print(
    gps.location.lng(),
    6
  );


  Serial.print(
    " | SPEED: "
  );

  Serial.print(
    speed,
    2
  );


  Serial.println(
    " km/h"
  );
}


// =====================================================
//                 RESET SEARCH ANIMATION
// =====================================================

void resetSearchAnimation() {

  loadingFrame = 0;

  lastAnimationTime =
      millis();


  // ===================================================
  // START SAME LED BLINK AS EMERGENCY
  // ===================================================

  gpsBlinkTaskEnabled = true;


  digitalWrite(
    LED_PIN,
    LOW
  );
}
