/*
  ESP32 Sensor/Controller (ESP B):
  - Ultrasonic: TRIG 4, ECHO 36
  - Servo for ultrasonic scan: GPIO 19
  - Soil moisture sensor (ADC): GPIO 27
  - Servo for soil probe: GPIO 26 (ONLY moves when MAIN is STOPPED)
  - DHT11: GPIO 18 (ONLY reads when MAIN is STOPPED)
  - I2C LCD: SDA 21, SCL 22 (16x2)

  Firebase RTDB REST:
  - /robot.json               (sensor values)
  - /control/cmd.json         ("STOP"/"GO")   written by this ESP
  - /control/window.json      ("RUN"/"PAUSE") read by this ESP, written by MAIN; this ESP sets RUN after upload
  - /control/status.json      ("RUNNING"/"STOPPED") read by this ESP (written by MAIN)
*/

// Include Wire library for I2C communication (used by LCD)
#include <Wire.h>
// Include LCD library for I2C-based 16x2 character display
#include <LiquidCrystal_I2C.h>
// Include DHT sensor library for temperature and humidity readings
#include <DHT.h>
// Include ESP32 servo library for controlling servo motors
#include <ESP32Servo.h>

// Include WiFi library for network connectivity
#include <WiFi.h>
// Include HTTP client library for REST API calls to Firebase
#include <HTTPClient.h>
// Include secure WiFi client for HTTPS connections
#include <WiFiClientSecure.h>

// ============ WIFI ============
// WiFi network name (SSID) to connect to
const char* WIFI_SSID = "Isra";
// WiFi password for authentication
const char* WIFI_PASS = "12345678";

// ============ FIREBASE RTDB ============
// Base URL for Firebase Realtime Database REST API
const char* FIREBASE_BASE =
  "https://agropatrol-3-default-rtdb.asia-southeast1.firebasedatabase.app";

// Firebase path for robot sensor data (distance, soil, humidity, temp)
const char* ROBOT_NODE  = "/robot.json";

// Control nodes
// Firebase path for obstacle command (STOP/GO) written by this ESP
const char* CMD_NODE     = "/control/cmd.json";
// Firebase path for operation window (RUN/PAUSE) - controls when to read sensors
const char* WINDOW_NODE  = "/control/window.json";
// Firebase path for main robot status (RUNNING/STOPPED)
const char* STATUS_NODE  = "/control/status.json";
// Firebase path for stop reason description
const char* REASON_NODE  = "/control/reason.json";
// Firebase path for obstacle distance in centimeters
const char* OBST_NODE    = "/control/obstacleCm.json";

// Firebase authentication token (empty if database rules allow public access)
const char* FIREBASE_AUTH = ""; // keep empty if rules are open

// Upload intervals
// Interval for optional heartbeat sensor data upload (30 seconds)
const unsigned long FIREBASE_INTERVAL_MS = 30000;  // optional heartbeat upload
// Interval for checking and updating obstacle control commands (800ms)
const unsigned long CONTROL_INTERVAL_MS  = 800;

// ============ PINS ============
// GPIO pin for ultrasonic sensor trigger signal
#define TRIG_PIN  4
// GPIO pin for ultrasonic sensor echo signal
#define ECHO_PIN  36

// GPIO pin for servo that rotates ultrasonic sensor for scanning
#define SERVO_ULTRA_PIN 19
// GPIO pin for servo that moves soil moisture probe up/down
#define SERVO_SOIL_PIN  26

// GPIO pin for soil moisture sensor analog input
#define SOIL_PIN 27
// GPIO pin for DHT11 temperature/humidity sensor
#define DHT_PIN  18

// ============ DHT ============
// Define DHT sensor type as DHT11
#define DHTTYPE DHT11
// Create DHT sensor object with pin and type
DHT dht(DHT_PIN, DHTTYPE);

// ============ LCD ============
// Try 0x3F if your LCD doesn't show anything
// Create LCD object with I2C address 0x27, 16 columns, 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============ SERVOS ============
// Create servo object for ultrasonic sensor scanning
Servo ultraServo;
// Create servo object for soil probe positioning
Servo soilServo;

// ============ SETTINGS ============
// Distance threshold in cm below which robot should STOP
const int OBSTACLE_CM = 20;
// Distance threshold in cm above which robot can GO (creates hysteresis)
const int OBSTACLE_CLEAR_CM = 28;

// Ultrasonic scan servo angles (change if needed)
// Center position angle for ultrasonic servo (facing forward)
const int SERVO_CENTER = 90;
// Left scan position angle for ultrasonic servo
const int SERVO_LEFT   = 150;
// Right scan position angle for ultrasonic servo
const int SERVO_RIGHT  = 30;

// Soil servo angles (adjust for your mechanism)
// Angle when soil probe is raised up (not measuring)
const int SOIL_UP_ANGLE   = 8;
// Angle when soil probe is lowered down (measuring soil)
const int SOIL_DOWN_ANGLE = 90;

// Soil timing (only used during STOPPED read)
// Time in milliseconds to wait after lowering probe for reading to stabilize
const unsigned long SOIL_SETTLE_MS   = 600;

// Soil calibration (must calibrate)
// ADC value when soil is completely dry (maximum resistance)
int SOIL_DRY = 3200;
// ADC value when soil is completely wet (minimum resistance)
int SOIL_WET = 1300;

// ============ STATE ============
// Timestamp of last ultrasonic distance scan
unsigned long lastScan = 0;
// Timestamp of last LCD display update
unsigned long lastLCD = 0;
// Timestamp of last Firebase sensor data upload
unsigned long lastFB = 0;
// Timestamp of last control command update
unsigned long lastControl = 0;
// Timestamp of last pause/window status check
unsigned long lastPauseCheck = 0;

// Timestamp until which obstacle warning should be displayed on LCD
unsigned long obstacleScreenUntil = 0;
// Text showing last turn direction decision (LEFT/RIGHT)
String lastTurnText = "----";

// Current measured distance in centimeters (-1 means no reading)
int distanceCm = -1;
// Current soil moisture percentage (0-100%, -1 means not yet read)
int soilPercent = -1;
// Current humidity percentage (NAN means not yet read)
float humidity = NAN;
// Current temperature in Celsius (NAN means not yet read)
float temperature = NAN;

// Secure WiFi client object for HTTPS connections to Firebase
WiFiClientSecure secureClient;
// Last command sent to Firebase to avoid redundant writes
String lastCmdSent = "";

// ============ HELPERS ============
// Build complete Firebase URL by combining base URL with node path and optional auth
String makeUrl(const char* node){
  // Start with base Firebase URL
  String url = String(FIREBASE_BASE) + String(node);
  // If authentication token is provided, append it as query parameter
  if (strlen(FIREBASE_AUTH) > 0) {
    url += "?auth=";
    url += FIREBASE_AUTH;
  }
  // Return complete URL
  return url;
}

// Remove surrounding quotes from Firebase JSON string responses
String stripQuotes(String s){
  // Remove leading/trailing whitespace
  s.trim();
  // If string starts and ends with quotes and is at least 2 chars long
  if (s.startsWith("\"") && s.endsWith("\"") && s.length() >= 2) {
    // Extract substring without first and last character (the quotes)
    s = s.substring(1, s.length()-1);
  }
  // Return cleaned string
  return s;
}

// ============ WIFI RECONNECT ============
// Ensure WiFi connection is active, reconnect if disconnected
void ensureWiFi() {
  // If WiFi is already connected, nothing to do
  if (WiFi.status() == WL_CONNECTED) return;

  // Log that WiFi was lost
  Serial.println("WiFi lost -> reconnecting...");
  // Disconnect completely to reset state
  WiFi.disconnect(true);
  // Begin new connection attempt with credentials
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Record start time for timeout
  unsigned long start = millis();
  // Wait for connection up to 15 seconds
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    // Wait 300ms between checks
    delay(300);
    // Print progress indicator
    Serial.print(".");
  }
  // Print newline after dots
  Serial.println();

  // Check if reconnection succeeded
  if (WiFi.status() == WL_CONNECTED) {
    // Log successful reconnection
    Serial.print("WiFi back. IP: ");
    // Print assigned IP address
    Serial.println(WiFi.localIP());
  } else {
    // Log failed reconnection
    Serial.println("WiFi reconnect FAILED");
  }
}

// ============ FIREBASE REST (DEBUG) ============
// Perform HTTP GET request to Firebase and return string value
String firebaseGET_String(const char* node){
  // If WiFi is not connected, return empty string
  if (WiFi.status() != WL_CONNECTED) return "";
  // Build complete URL for this node
  String url = makeUrl(node);

  // Create HTTP client object
  HTTPClient http;
  // Set request timeout to 8 seconds
  http.setTimeout(8000);

  // Initialize HTTP connection with secure client and URL
  if (!http.begin(secureClient, url)) {
    // Log error if connection initialization fails
    Serial.println("GET begin failed");
    // Return empty string
    return "";
  }

  // Perform GET request and get response code
  int code = http.GET();
  // Get response body as string
  String body = http.getString();
  // Close connection and free resources
  http.end();

  // Remove quotes from JSON string value and return
  return stripQuotes(body);
}

// Perform HTTP PUT request to Firebase with JSON payload
bool firebasePUT_JSON(const char* node, const String& jsonPayload){
  // If WiFi is not connected, fail immediately
  if (WiFi.status() != WL_CONNECTED) return false;
  // Build complete URL for this node
  String url = makeUrl(node);

  // Create HTTP client object
  HTTPClient http;
  // Set request timeout to 8 seconds
  http.setTimeout(8000);

  // Initialize HTTP connection with secure client and URL
  if (!http.begin(secureClient, url)) {
    // Log error if connection initialization fails
    Serial.println("PUT begin failed");
    // Return failure
    return false;
  }

  // Set content type header to JSON
  http.addHeader("Content-Type", "application/json");

  // Perform PUT request with JSON payload and get response code
  int code = http.PUT(jsonPayload);
  // Get response body (for error logging)
  String resp = http.getString();
  // Close connection and free resources
  http.end();

  // Log the PUT operation details
  Serial.print("PUT ");
  Serial.print(node);
  Serial.print(" code=");
  Serial.println(code);

  // If response code indicates error (not 2xx)
  if (code < 200 || code >= 300) {
    // Log error response
    Serial.print("PUT resp: ");
    Serial.println(resp);
  }

  // Return true if status code is 2xx (success)
  return (code >= 200 && code < 300);
}

// Perform HTTP PUT request to Firebase with string value (wraps in JSON quotes)
bool firebasePUT_String(const char* node, const String& value){
  // Wrap string in quotes to make valid JSON and call JSON PUT
  return firebasePUT_JSON(node, "\"" + value + "\"");
}

// Perform HTTP PUT request to Firebase with integer value
bool firebasePUT_Int(const char* node, int value){
  // Convert int to string (JSON number) and call JSON PUT
  return firebasePUT_JSON(node, String(value));
}

// ============ SENSORS ============
// Read distance from ultrasonic sensor in centimeters
int readUltrasonicCM() {
  // Ensure trigger pin is low initially
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  // Send 10 microsecond pulse on trigger pin
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  // Return trigger pin to low
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo pulse duration (timeout after 30ms)
  unsigned long d = pulseIn(ECHO_PIN, HIGH, 30000UL);
  // If no echo received (timeout), return -1 for invalid reading
  if (d == 0) return -1;
  // Convert duration to distance: duration * speed of sound / 2
  return (int)(d * 0.0343 / 2.0);
}

// Convert soil moisture ADC value to percentage (0% dry, 100% wet)
int soilToPercent(int adc) {
  // Constrain ADC reading to calibrated range
  adc = constrain(adc, SOIL_WET, SOIL_DRY);
  // Map ADC range to 0-100% (DRY=0%, WET=100%)
  return constrain(map(adc, SOIL_DRY, SOIL_WET, 0, 100), 0, 100);
}

// Scan left and right with ultrasonic to determine best turn direction
int scanForBestDirection() {
  // Move servo to left position
  ultraServo.write(SERVO_LEFT);
  // Wait for servo to reach position
  delay(350);
  // Read distance on left side
  int left = readUltrasonicCM();

  // Move servo to right position
  ultraServo.write(SERVO_RIGHT);
  // Wait for servo to reach position
  delay(350);
  // Read distance on right side
  int right = readUltrasonicCM();

  // Return servo to center position
  ultraServo.write(SERVO_CENTER);
  // Wait for servo to settle
  delay(200);

  // If left reading invalid, assume very far (999cm)
  if (left < 0) left = 999;
  // If right reading invalid, assume very far (999cm)
  if (right < 0) right = 999;

  // Return 1 if left is clearer (>=), 2 if right is clearer
  return (left >= right) ? 1 : 2;
}

// obstacle command update (STOP/GO)
// Update Firebase command based on obstacle distance with hysteresis
void updateCmdForMain(){
  // If distance reading is invalid, do nothing
  if (distanceCm <= 0) return;

  // Variable to hold command to send
  String cmdToSend;
  // If obstacle is closer than threshold, send STOP
  if (distanceCm <= OBSTACLE_CM) cmdToSend = "STOP";
  // If obstacle is farther than clear threshold, send GO
  else if (distanceCm >= OBSTACLE_CLEAR_CM) cmdToSend = "GO";
  // If in hysteresis zone (between thresholds), don't change command
  else return;

  // If command hasn't changed from last sent, avoid redundant write
  if (cmdToSend == lastCmdSent) return;

  // Send command to Firebase
  bool ok = firebasePUT_String(CMD_NODE, cmdToSend);
  // Also update obstacle distance value
  firebasePUT_Int(OBST_NODE, distanceCm);
  // Update reason for stop/go
  firebasePUT_String(REASON_NODE, (cmdToSend == "STOP") ? "Obstacle" : "Clear");

  // If upload succeeded, update last sent command
  if (ok) lastCmdSent = cmdToSend;
}

// Force soil read now (ONLY called when MAIN STOPPED)
// Perform soil moisture reading by lowering probe, reading, and raising
void readSoilNow(){
  // Move servo to lower probe into soil
  soilServo.write(SOIL_DOWN_ANGLE);
  // Wait for probe to settle in soil
  delay(SOIL_SETTLE_MS);

  // Read analog value from soil moisture sensor
  int soilADC = analogRead(SOIL_PIN);
  // Convert ADC value to percentage
  soilPercent = soilToPercent(soilADC);

  // Raise probe back up
  soilServo.write(SOIL_UP_ANGLE);

  // Log raw ADC value
  Serial.print("Soil ADC: ");
  Serial.print(soilADC);
  Serial.print("  Soil%: ");
  // Log calculated percentage
  Serial.println(soilPercent);
}

// Upload sensor JSON
// Build and upload complete sensor data JSON object to Firebase
void uploadRobotJson(){
  // Start building JSON object
  String json = "{";
  // Add distance field
  json += "\"distanceCm\":" + String(distanceCm) + ",";
  // Add soil moisture percentage field
  json += "\"soilPercent\":" + String(soilPercent) + ",";
  // Add humidity field (null if not available)
  json += "\"humidity\":" + (isnan(humidity) ? "null" : String(humidity, 1)) + ",";
  // Add temperature field (null if not available)
  json += "\"temperature\":" + (isnan(temperature) ? "null" : String(temperature, 1)) + ",";
  // Add timestamp in milliseconds since boot
  json += "\"updatedAtMs\":" + String(millis());
  // Close JSON object
  json += "}";

  // Upload JSON to Firebase
  bool ok = firebasePUT_JSON(ROBOT_NODE, json);
  // Log result
  Serial.println(ok ? "robot.json OK" : "robot.json FAIL");
}

// ONLY when MAIN is STOPPED + PAUSE
// Perform sensor readings that require robot to be stopped
void doPausedSensorWork(){
  // Log that we're doing stopped sensor work
  Serial.println("MAIN STOPPED -> reading soil + DHT now...");

  // Read soil moisture (requires probe movement)
  readSoilNow();

  // Read humidity from DHT sensor
  humidity = dht.readHumidity();
  // Read temperature from DHT sensor
  temperature = dht.readTemperature();

  // Upload all sensor data to Firebase
  uploadRobotJson();

  // restart MAIN
  // Set window to RUN to allow main robot to continue
  firebasePUT_String(WINDOW_NODE, "RUN");
  // Log that we're resuming main robot
  Serial.println("STOPPED work done -> WINDOW=RUN");
}

// ============ SETUP ============
// Initialize hardware, WiFi, and Firebase on startup
void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);

  // Configure trigger pin as output
  pinMode(TRIG_PIN, OUTPUT);
  // Configure echo pin as input
  pinMode(ECHO_PIN, INPUT);

  // Initialize I2C with SDA=21, SCL=22
  Wire.begin(21, 22);
  // Initialize LCD display
  lcd.init();
  // Turn on LCD backlight
  lcd.backlight();
  // Clear LCD screen
  lcd.clear();
  // Display startup message
  lcd.print("Starting...");

  // Initialize DHT sensor
  dht.begin();

  // Set servo PWM frequency to 50Hz
  ultraServo.setPeriodHertz(50);
  // Attach ultrasonic servo with pulse width range 500-2400us
  ultraServo.attach(SERVO_ULTRA_PIN, 500, 2400);
  // Move ultrasonic servo to center position
  ultraServo.write(SERVO_CENTER);

  // Set servo PWM frequency to 50Hz
  soilServo.setPeriodHertz(50);
  // Attach soil servo with pulse width range 500-2400us
  soilServo.attach(SERVO_SOIL_PIN, 500, 2400);
  // Move soil probe to up position
  soilServo.write(SOIL_UP_ANGLE);

  // Set ADC resolution to 12 bits (0-4095 range)
  analogReadResolution(12);

  // Set WiFi mode to station (client)
  WiFi.mode(WIFI_STA);
  // Begin WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Record start time for connection timeout
  unsigned long start = millis();
  // Wait up to 20 seconds for WiFi connection
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    // Wait 300ms between checks
    delay(300);
  }

  // Disable SSL certificate validation (insecure but simpler for Firebase)
  secureClient.setInsecure();

  // init defaults
  // Initialize Firebase command to GO (robot can start)
  firebasePUT_String(CMD_NODE, "GO");
  // Remember last sent command
  lastCmdSent = "GO";

  // Initialize timing variables to current time
  lastFB = millis();
  lastControl = millis();
  lastPauseCheck = millis();

  // Clear LCD after initialization
  lcd.clear();
}

// ============ LOOP ============
// Main program loop - runs continuously
void loop() {
  // Ensure WiFi connection is active
  ensureWiFi();
  // Get current time in milliseconds
  unsigned long now = millis();

  // Ultrasonic distance read
  // If 250ms have passed since last scan
  if (now - lastScan >= 250) {
    // Update last scan timestamp
    lastScan = now;
    // Read current distance from ultrasonic sensor
    distanceCm = readUltrasonicCM();

    // If valid reading and obstacle detected
    if (distanceCm > 0 && distanceCm < OBSTACLE_CM) {
      // Scan left and right to determine best turn direction
      int dir = scanForBestDirection();
      // Update turn text based on direction (1=LEFT, 2=RIGHT)
      lastTurnText = (dir == 1) ? "LEFT " : "RIGHT";
      // Show obstacle screen for next 1.2 seconds
      obstacleScreenUntil = now + 1200;
    }
  }

  // obstacle cmd update (STOP/GO)
  // If control interval has elapsed
  if (now - lastControl >= CONTROL_INTERVAL_MS) {
    // Update timestamp
    lastControl = now;
    // Update Firebase command based on current distance
    updateCmdForMain();
  }

  // OPTIONAL: periodic upload (distance only if you want)
  // If periodic upload interval has elapsed
  if (now - lastFB >= FIREBASE_INTERVAL_MS) {
    // Update timestamp
    lastFB = now;
    // only upload current distance (soil/DHT unchanged until STOPPED)
    // Upload current sensor data (mainly distance)
    uploadRobotJson();
  }

  // pause check (this is where soil + DHT read happens)
  // If 1 second has passed since last pause check
  if (now - lastPauseCheck >= 1000) {
    // Update timestamp
    lastPauseCheck = now;

    // Read current window state from Firebase
    String window = firebaseGET_String(WINDOW_NODE);
    // Read current robot status from Firebase
    String status = firebaseGET_String(STATUS_NODE);

    // If window is PAUSE and main robot is STOPPED
    if (window == "PAUSE" && status == "STOPPED") {
      // Perform soil and DHT sensor readings
      doPausedSensorWork();
      // Wait 1.2 seconds before continuing
      delay(1200);
    }
  }

  // LCD update
  // If 700ms have passed since last LCD update
  if (now - lastLCD >= 700) {
    // Update timestamp
    lastLCD = now;

    // If we should be showing obstacle warning and have valid distance
    if (now < obstacleScreenUntil && distanceCm > 0) {
      // Clear LCD display
      lcd.clear();
      // Move cursor to start of first line
      lcd.setCursor(0, 0);
      // Print obstacle label
      lcd.print("Obs ");
      // Print distance value
      lcd.print(distanceCm);
      // Print unit
      lcd.print("cm");

      // Move cursor to start of second line
      lcd.setCursor(0, 1);
      // Print turn direction label
      lcd.print("Turn: ");
      // Print suggested turn direction
      lcd.print(lastTurnText);
      // Exit early to keep obstacle screen displayed
      return;
    }

    // Normal display mode - show all sensor readings
    // Move cursor to start of first line
    lcd.setCursor(0, 0);
    // Print soil label
    lcd.print("Soil:");
    // Print soil percentage if available, otherwise "---"
    if (soilPercent >= 0) lcd.print(soilPercent); else lcd.print("---");
    // Print humidity label
    lcd.print("% H:");
    // Print humidity value if available, otherwise "---"
    if (!isnan(humidity)) lcd.print((int)humidity); else lcd.print("---");
    // Print percentage and space
    lcd.print("% ");

    // Move cursor to start of second line
    lcd.setCursor(0, 1);
    // Print temperature label
    lcd.print("T:");
    // Print temperature value if available, otherwise "--"
    if (!isnan(temperature)) lcd.print((int)temperature); else lcd.print("--");
    // Print distance label
    lcd.print("C D:");
    // Print distance value if available, otherwise "---"
    if (distanceCm > 0) lcd.print(distanceCm); else lcd.print("---");
    // Print unit and space
    lcd.print("cm ");
  }
}
