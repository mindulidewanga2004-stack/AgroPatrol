#include <Arduino.h>              // Core Arduino functions
#include <WiFi.h>                 // ESP32 WiFi
#include <HTTPClient.h>           // HTTP REST requests
#include <WiFiClientSecure.h>     // HTTPS client (Firebase uses HTTPS)

// =================== WIFI ===================
const char* WIFI_SSID = "Isra";        // WiFi SSID
const char* WIFI_PASS = "12345678";    // WiFi password

// =================== FIREBASE ===================
const char* FIREBASE_BASE =
  "https://agropatrol-3-default-rtdb.asia-southeast1.firebasedatabase.app"; // Firebase RTDB base URL

// Firebase nodes used by MAIN ESP
const char* CMD_NODE    = "/control/cmd.json";      // STOP / GO (written by ESP-B)
const char* WINDOW_NODE = "/control/window.json";   // RUN / PAUSE (written by MAIN, read by both)
const char* STATUS_NODE = "/control/status.json";   // RUNNING / STOPPED (written by MAIN, read by ESP-B)
const char* MAIN_LAST   = "/control/mainLastMs.json"; // heartbeat timestamp (optional)

const char* FIREBASE_AUTH = ""; // keep empty if rules open (no auth token)

WiFiClientSecure secureClient; // HTTPS client

// =================== L298N PINS ===================
// ENA/ENB are PWM enable pins (speed). IN1..IN4 set direction.
#define ENA 14
#define IN1 27
#define IN2 26

#define ENB 32
#define IN3 25
#define IN4 33

// =================== IR (ANALOG) ===================
// Analog line sensors connected to ADC pins
#define IR_LEFT    35
#define IR_CENTER  39
#define IR_RIGHT   34

// =================== PWM ===================
#define PWM_FREQ 20000     // PWM frequency
#define PWM_RES  8         // resolution bits: 0..255
#define CH_A     0         // LEDC channel A
#define CH_B     1         // LEDC channel B

// =================== TUNING ===================
int BASE_SPEED   = 140;    // forward speed normally
int TURN_SPEED   = 170;    // stronger speed when turning
int SEARCH_SPEED = 150;    // speed while searching for line
int THRESH = 2000;         // threshold separating black/white
bool BLACK_IS_LOW_ANALOG = true; // true if black gives LOWER ADC (common)

// =================== RUN WINDOW ===================
// Robot runs only within a timed "window"
const unsigned long RUN_WINDOW_MS = 60000; // 60 seconds run window

bool windowRunning = false;        // track if window is active
unsigned long windowStartMs = 0;   // when run window started

// obstacle pause fix (so obstacle stop doesn't consume window time)
bool wasObstacleStop = false;            // are we currently stopped due to obstacle?
unsigned long obstacleStopStartMs = 0;   // when obstacle stop began

// =================== MOTOR HELPERS ===================

// Clamp speed into 0..255 (because PWM is 8-bit)
int clamp255(int v){ 
  return v < 0 ? 0 : (v > 255 ? 255 : v); 
}

// Write PWM speeds to both motor channels
void setSpeed(int a,int b){
  ledcWrite(CH_A, clamp255(a)); // motor A speed
  ledcWrite(CH_B, clamp255(b)); // motor B speed
}

// Move forward: both motors forward
void forwardMotor(int a,int b){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); // left motor forward
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); // right motor forward
  setSpeed(a,b);                                  // set PWM speeds
}

// Turn left: left motor reverse, right motor forward (spin turn)
void leftTurn(int spd){
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); // left motor reverse
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  // right motor forward
  setSpeed(spd, spd);                              // same speed both sides
}

// Turn right: left motor forward, right motor reverse (spin turn)
void rightTurn(int spd){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  // left motor forward
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); // right motor reverse
  setSpeed(spd, spd);                              // same speed both sides
}

// Stop both motors
void stopMotor(){ 
  setSpeed(0,0); 
}

// =================== IR (LINE SENSOR) ===================

// Read analog IR sensor value
int readIR(int pin){ 
  return analogRead(pin); 
}

// Decide if sensor is detecting "black line"
bool onBlack(int pin){
  int v = readIR(pin); // raw ADC reading
  return BLACK_IS_LOW_ANALOG ? (v < THRESH) : (v > THRESH); // compare with threshold
}

// =================== SEARCH LOGIC ===================
// When line is lost, rotate a bit in last known direction until found.
int lastDir = 1; // +1 = last turn right, -1 = last turn left

void searchLine(){
  unsigned long t0 = millis();           // start search time
  while(millis() - t0 < 300){            // search for up to 300ms
    if(lastDir < 0) leftTurn(SEARCH_SPEED); // rotate left
    else            rightTurn(SEARCH_SPEED);// rotate right

    // If any sensor sees black again, stop searching and return to line logic
    if(onBlack(IR_LEFT) || onBlack(IR_CENTER) || onBlack(IR_RIGHT)) return;
  }
  stopMotor(); // if not found after 300ms, stop briefly
}

// =================== FIREBASE HELPERS ===================

// Build URL for Firebase node
String makeUrl(const char* node){
  String url = String(FIREBASE_BASE) + node; // base + path
  if(strlen(FIREBASE_AUTH) > 0){             // if auth token is used
    url += "?auth=";
    url += FIREBASE_AUTH;
  }
  return url;
}

// Remove quotes from Firebase string responses
String stripQuotes(String s){
  s.trim();
  if(s.startsWith("\"") && s.endsWith("\"")) {
    s = s.substring(1, s.length()-1);
  }
  return s;
}

// GET a node value (returns string without quotes)
String firebaseGET(const char* node){
  if(WiFi.status() != WL_CONNECTED) return ""; // if offline, return empty
  HTTPClient http;
  http.begin(secureClient, makeUrl(node));     // open HTTPS connection
  int code = http.GET();                       // GET (code not used)
  String body = http.getString();              // response body
  http.end();
  return stripQuotes(body);                    // cleaned string
}

// PUT a string value ("RUN", "PAUSE", etc.)
void firebasePUT(const char* node, const String& val){
  if(WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(secureClient, makeUrl(node));
  http.addHeader("Content-Type","application/json");
  http.PUT("\"" + val + "\""); // JSON string must be quoted
  http.end();
}

// PUT an integer value (heartbeat millis, etc.)
void firebasePUT_Int(const char* node, unsigned long v){
  if(WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(secureClient, makeUrl(node));
  http.addHeader("Content-Type","application/json");
  http.PUT(String(v)); // JSON number
  http.end();
}

// =================== WIFI ===================

// Reconnect WiFi if disconnected (simple version)
void ensureWiFi(){
  if(WiFi.status() == WL_CONNECTED) return;
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// =================== WINDOW CONTROL ===================
// RUN window start: tell ESP-B we are running, and allow motors.
void startWindow(){
  windowRunning = true;          // mark window active
  windowStartMs = millis();      // start time reference
  firebasePUT(WINDOW_NODE,"RUN");        // window = RUN
  firebasePUT(STATUS_NODE,"RUNNING");    // status = RUNNING
  Serial.println("== RUN WINDOW STARTED ==");
}

// Stop the run window: pause system so ESP-B can do soil+DHT work
void stopWindow(){
  windowRunning = false;         // mark window inactive
  stopMotor();                   // stop motors physically
  firebasePUT(WINDOW_NODE,"PAUSE");      // signal ESP-B: ok to read sensors
  firebasePUT(STATUS_NODE,"STOPPED");    // status stopped
  Serial.println("== WINDOW PAUSED ==");
}

// =================== SETUP ===================
void setup(){
  Serial.begin(115200); // start serial monitor

  // Motor direction pins
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  // Setup PWM channels
  ledcSetup(CH_A, PWM_FREQ, PWM_RES); // channel A PWM
  ledcSetup(CH_B, PWM_FREQ, PWM_RES); // channel B PWM
  ledcAttachPin(ENA, CH_A);           // ENA uses channel A
  ledcAttachPin(ENB, CH_B);           // ENB uses channel B

  analogReadResolution(12); // ADC 0..4095
  stopMotor();              // safety stop at boot

  // WiFi connect
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status() != WL_CONNECTED) delay(300); // wait until connected
  secureClient.setInsecure(); // skip certificate validation

  startWindow(); // start running immediately
}

// =================== LOOP ===================
void loop(){
  ensureWiFi(); // maintain WiFi connection

  // ---------- heartbeat ----------
  // Update mainLastMs every 2 seconds so others know MAIN is alive
  static unsigned long lastHB = 0;
  if(millis() - lastHB > 2000){
    lastHB = millis();
    firebasePUT_Int(MAIN_LAST, millis());
  }

  // ---------- poll commands ----------
  static unsigned long lastPoll = 0;
  static String cmd = "GO";     // default: GO
  static String window = "RUN"; // default: RUN

  if(millis() - lastPoll > 500){
    lastPoll = millis();
    String c = firebaseGET(CMD_NODE);      // STOP/GO from ESP-B
    String w = firebaseGET(WINDOW_NODE);   // RUN/PAUSE from MAIN (or updated)
    if(c.length()) cmd = c;                // update if valid
    if(w.length()) window = w;             // update if valid
  }

  // ---------- PAUSE window ----------
  // If window is PAUSE, MAIN must stop and do nothing.
  if(window == "PAUSE"){
    stopMotor();          // stop motors
    windowRunning = false;// mark not running
    return;               // exit loop early
  }

  // ---------- obstacle STOP ----------
  // If ESP-B detected obstacle, it writes STOP into cmd.
  if(cmd == "STOP"){
    // record start of obstacle stop (only once)
    if(!wasObstacleStop){
      wasObstacleStop = true;
      obstacleStopStartMs = millis();
    }
    stopMotor();                       // immediate stop
    firebasePUT(STATUS_NODE,"STOPPED");// tell ESP-B MAIN is stopped
    return;                            // exit loop early
  } else {
    // command is GO now. If we were previously stopped by obstacle,
    // shift windowStartMs forward so the obstacle pause doesn't consume run time.
    if(wasObstacleStop){
      windowStartMs += millis() - obstacleStopStartMs; // compensate
      wasObstacleStop = false;
    }
  }

  // ---------- ensure RUN window ----------
  if(!windowRunning){
    startWindow(); // if window was not active, start again
  }

  // ================= LINE FOLLOWER =================
  bool L = onBlack(IR_LEFT);    // left sensor on black?
  bool C = onBlack(IR_CENTER);  // center sensor on black?
  bool R = onBlack(IR_RIGHT);   // right sensor on black?

  // Interpret sensor pattern and drive motors
  if(L && C && R) forwardMotor(BASE_SPEED, BASE_SPEED);                 // line wide / intersection
  else if(C && !L && !R) forwardMotor(BASE_SPEED, BASE_SPEED);          // centered on line
  else if(L && C && !R){ lastDir = -1; forwardMotor(BASE_SPEED-30, BASE_SPEED+30); } // drift left
  else if(R && C && !L){ lastDir = 1;  forwardMotor(BASE_SPEED+30, BASE_SPEED-30); } // drift right
  else if(L && !C && !R){ lastDir = -1; forwardMotor(BASE_SPEED-70, TURN_SPEED); }   // hard left
  else if(R && !C && !L){ lastDir = 1;  forwardMotor(TURN_SPEED, BASE_SPEED-70); }   // hard right
  else searchLine(); // lost line completely => search routine

  // ---------- run window timeout ----------
  // After 60 seconds of actual running (excluding obstacle pauses), pause the system.
  if(millis() - windowStartMs >= RUN_WINDOW_MS){
    stopWindow();
  }
}
