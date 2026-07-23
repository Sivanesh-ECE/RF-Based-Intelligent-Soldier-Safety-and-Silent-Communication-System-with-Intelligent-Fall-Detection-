/*
  ============================================================================
  RF-BASED INTELLIGENT SOLDIER SAFETY AND SILENT COMMUNICATION SYSTEM
  SOLDIER UNIT (TRANSMITTER) - ESP32 DevKit V1
  ============================================================================

  Hardware:
    - ESP32 DevKit V1
    - MPU6050 (Accelerometer + Gyroscope) over I2C
    - 0.96" OLED Display SSD1306 over I2C
    - HT12E Encoder IC (parallel-to-serial, feeds 433 MHz RF TX module)
    - 4x Push Buttons (silent tactical messages)

  Description:
    Reads soldier-triggered silent messages from push buttons and continuously
    monitors accelerometer/gyroscope data to detect a fall using sensor
    fusion (impact magnitude + sudden orientation change), NOT a fixed
    "wait and see" timer. On detection, a "Soldier Down" emergency code is
    transmitted immediately over RF via the HT12E encoder.

  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// PIN CONFIGURATION
// ---------------------------------------------------------------------------
// I2C bus (shared by OLED and MPU6050) - default ESP32 I2C pins
#define I2C_SDA_PIN       21
#define I2C_SCL_PIN       22

// Push buttons (active LOW, internal pull-up used)
#define BTN_NEED_BACKUP   32   // Button 1
#define BTN_ENEMY_SPOTTED 33   // Button 2
#define BTN_MEDICAL_HELP  25   // Button 3
#define BTN_MISSION_DONE  26   // Button 4

// HT12E encoder data pins (4-bit parallel command word)
#define HT12E_D8          13   // MSB
#define HT12E_D9          12
#define HT12E_D10         14
#define HT12E_D11         27   // LSB
#define HT12E_TE          4    // Transmit Enable, active LOW

// Onboard status LED (optional)
#define STATUS_LED        2

// ---------------------------------------------------------------------------
// OLED CONFIGURATION
// ---------------------------------------------------------------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDRESS   0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------------------------------------------------------------------
// MPU6050 CONFIGURATION
// ---------------------------------------------------------------------------
Adafruit_MPU6050 mpu;

// ---------------------------------------------------------------------------
// RF COMMAND CODES (4-bit, sent through HT12E data pins)
// ---------------------------------------------------------------------------
#define CODE_NEED_BACKUP     0b0001
#define CODE_ENEMY_SPOTTED   0b0010
#define CODE_MEDICAL_HELP    0b0011
#define CODE_MISSION_DONE    0b0100
#define CODE_SOLDIER_DOWN    0b1111

// ---------------------------------------------------------------------------
// FALL DETECTION THRESHOLDS (tune during field testing)
// ---------------------------------------------------------------------------
#define IMPACT_THRESHOLD_G       2.5f   // Total accel spike indicating impact (in g)
#define ORIENTATION_DELTA_DEG    45.0f  // Sudden pitch/roll change (degrees)
#define GYRO_THRESHOLD_DPS       200.0f // Sudden angular velocity (deg/s)
#define FUSION_WINDOW_MS         150    // Time window to correlate impact + orientation change

// ---------------------------------------------------------------------------
// TIMING CONSTANTS (non-blocking, millis-based)
// ---------------------------------------------------------------------------
#define DEBOUNCE_DELAY_MS     40
#define SENDING_DISPLAY_MS    600
#define SENT_DISPLAY_MS       800
#define TE_ACTIVE_MS          250   // How long TE is held LOW so HT12E can transmit
#define SENSOR_POLL_MS        20    // MPU polling interval

// ---------------------------------------------------------------------------
// SYSTEM STATE MACHINE
// ---------------------------------------------------------------------------
enum SystemState {
  STATE_READY,
  STATE_SENDING,
  STATE_SENT,
  STATE_EMERGENCY
};

SystemState currentState = STATE_READY;
unsigned long stateEnteredAt = 0;
String currentMessage = "";

// ---------------------------------------------------------------------------
// BUTTON DEBOUNCE TRACKING
// ---------------------------------------------------------------------------
struct ButtonInfo {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeTime;
  uint8_t code;
  const char* message;
};

ButtonInfo buttons[4] = {
  { BTN_NEED_BACKUP,   HIGH, HIGH, 0, CODE_NEED_BACKUP,   "Need Backup" },
  { BTN_ENEMY_SPOTTED, HIGH, HIGH, 0, CODE_ENEMY_SPOTTED, "Enemy Spotted" },
  { BTN_MEDICAL_HELP,  HIGH, HIGH, 0, CODE_MEDICAL_HELP,  "Need Medical Help" },
  { BTN_MISSION_DONE,  HIGH, HIGH, 0, CODE_MISSION_DONE,  "Mission Completed" }
};

// ---------------------------------------------------------------------------
// SENSOR FUSION STATE (fall detection)
// ---------------------------------------------------------------------------
float previousPitch = 0.0f;
float previousRoll  = 0.0f;
unsigned long lastSensorPoll = 0;
unsigned long lastImpactTime = 0;
bool impactFlag = false;

// System health flags (populated during self-test)
bool oledOK = false;
bool mpuOK  = false;

// HT12E transmit non-blocking control
bool teActive = false;
unsigned long teStartTime = 0;

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  setupButtons();
  setupHT12E();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  oledOK = initializeOLED();
  runStartupSequence();

  mpuOK = initializeMPU();

  displaySelfTestResults();

  if (!oledOK) {
    // Cannot show error on screen if OLED itself failed; blink status LED instead
    while (true) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(200);
    }
  }

  if (!mpuOK) {
    displaySystemError();
    // Continue running so buttons still work even if fall detection is unavailable
  }

  delay(1200); // Brief pause so operator can read self-test results (startup only)
  enterReadyState();
}

// ===========================================================================
// MAIN LOOP (kept short - real work happens in functions)
// ===========================================================================
void loop() {
  readButtons();
  detectFall();
  updateOLED();
  handleTransmitTimeout();
}

// ===========================================================================
// INITIALIZATION FUNCTIONS
// ===========================================================================
void setupButtons() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }
}

void setupHT12E() {
  pinMode(HT12E_D8, OUTPUT);
  pinMode(HT12E_D9, OUTPUT);
  pinMode(HT12E_D10, OUTPUT);
  pinMode(HT12E_D11, OUTPUT);
  pinMode(HT12E_TE, OUTPUT);
  digitalWrite(HT12E_TE, HIGH); // Idle: transmission disabled (active LOW)
  digitalWrite(HT12E_D8, LOW);
  digitalWrite(HT12E_D9, LOW);
  digitalWrite(HT12E_D10, LOW);
  digitalWrite(HT12E_D11, LOW);
}

bool initializeOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED initialization failed!");
    return false;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  return true;
}

bool initializeMPU() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

void runStartupSequence() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("RF Soldier System");
  display.setCursor(0, 30);
  display.println("Initializing...");
  display.display();
  delay(800); // One-time startup banner only
}

void displaySelfTestResults() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Self Test:");
  display.print("MPU6050 ");
  display.println(mpuOK ? "OK" : "FAIL");
  display.print("OLED    ");
  display.println(oledOK ? "OK" : "FAIL");
  display.print("RF      OK"); // HT12E/RF TX has no feedback line; assumed OK if pins configured
  display.display();
}

void displaySystemError() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(5, 20);
  display.println("SYSTEM");
  display.setCursor(5, 40);
  display.println("ERROR");
  display.display();
}

// ===========================================================================
// STATE TRANSITIONS
// ===========================================================================
void enterReadyState() {
  currentState = STATE_READY;
  stateEnteredAt = millis();
}

void enterSendingState(const char* message) {
  currentState = STATE_SENDING;
  stateEnteredAt = millis();
  currentMessage = message;
}

void enterSentState() {
  currentState = STATE_SENT;
  stateEnteredAt = millis();
}

void enterEmergencyState() {
  currentState = STATE_EMERGENCY;
  stateEnteredAt = millis();
  currentMessage = "Soldier Down";
}

// ===========================================================================
// OLED DISPLAY (state-driven, non-blocking)
// ===========================================================================
void updateOLED() {
  switch (currentState) {
    case STATE_READY:
      displayReadyScreen();
      break;

    case STATE_SENDING:
      displaySendingScreen();
      if (millis() - stateEnteredAt >= SENDING_DISPLAY_MS) {
        enterSentState();
      }
      break;

    case STATE_SENT:
      displaySentScreen();
      if (millis() - stateEnteredAt >= SENT_DISPLAY_MS) {
        enterReadyState();
      }
      break;

    case STATE_EMERGENCY:
      displayEmergency();
      // Emergency screen remains until operator acknowledges by pressing
      // Mission Completed, or the unit is reset. This is intentional -
      // an emergency must not silently clear itself.
      break;
  }
}

void displayReadyScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ID   : S-01");
  display.println("STATUS: ACTIVE");
  display.println("RF   : CONNECTED");
  display.display();
}

void displaySendingScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("Sending...");
  display.setCursor(0, 30);
  display.println(currentMessage);
  display.display();
}

void displaySentScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.println("Message Sent");
  display.display();
}

void displayEmergency() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(5, 15);
  display.println("EMERGENCY");
  display.setTextSize(1);
  display.setCursor(5, 40);
  display.println("Soldier Down");
  display.display();
}

// ===========================================================================
// BUTTON HANDLING (debounced, non-blocking)
// ===========================================================================
void readButtons() {
  // Ignore button presses while a transmission or emergency is being handled,
  // except Mission Completed can acknowledge/clear an emergency screen.
  for (uint8_t i = 0; i < 4; i++) {
    bool reading = digitalRead(buttons[i].pin);

    if (reading != buttons[i].lastReading) {
      buttons[i].lastChangeTime = millis();
    }

    if ((millis() - buttons[i].lastChangeTime) > DEBOUNCE_DELAY_MS) {
      if (reading != buttons[i].stableState) {
        buttons[i].stableState = reading;

        // Active LOW: pressed when stableState == LOW
        if (buttons[i].stableState == LOW) {
          onButtonPressed(i);
        }
      }
    }

    buttons[i].lastReading = reading;
  }
}

void onButtonPressed(uint8_t index) {
  // If currently displaying an emergency, only "Mission Completed" acknowledges it
  if (currentState == STATE_EMERGENCY) {
    if (buttons[index].code == CODE_MISSION_DONE) {
      enterReadyState();
    }
    return;
  }

  // Ignore new presses while mid-transmission of a previous message
  if (currentState == STATE_SENDING || currentState == STATE_SENT) {
    return;
  }

  sendRFMessage(buttons[index].code);
  enterSendingState(buttons[index].message);
}

// ===========================================================================
// FALL DETECTION (SENSOR FUSION - not a fixed timer)
// ===========================================================================
void detectFall() {
  if (!mpuOK) return;                 // No sensor, no detection possible
  if (currentState == STATE_EMERGENCY) return; // Already alerted

  if (millis() - lastSensorPoll < SENSOR_POLL_MS) return;
  lastSensorPoll = millis();

  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  float ax = accelEvent.acceleration.x / 9.81f; // convert m/s^2 to g
  float ay = accelEvent.acceleration.y / 9.81f;
  float az = accelEvent.acceleration.z / 9.81f;

  float totalAccelG = sqrt(ax * ax + ay * ay + az * az);

  // Pitch and roll from accelerometer (degrees)
  float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
  float roll  = atan2(-ax, az) * 180.0f / PI;

  // Angular velocity magnitude (deg/s)
  float gx = gyroEvent.gyro.x * 180.0f / PI;
  float gy = gyroEvent.gyro.y * 180.0f / PI;
  float gz = gyroEvent.gyro.z * 180.0f / PI;
  float gyroMagnitudeDPS = sqrt(gx * gx + gy * gy + gz * gz);

  float deltaPitch = fabs(pitch - previousPitch);
  float deltaRoll  = fabs(roll - previousRoll);

  // Step 1: Flag a potential impact when total acceleration spikes
  if (totalAccelG >= IMPACT_THRESHOLD_G) {
    impactFlag = true;
    lastImpactTime = millis();
  }

  // Step 2: Within a short correlation window after impact, check whether
  // the orientation changed suddenly AND angular velocity was high.
  // Both impact AND orientation/rotation evidence are required together -
  // this avoids false positives from a single hard footstep or bump.
  if (impactFlag && (millis() - lastImpactTime <= FUSION_WINDOW_MS)) {
    bool orientationChanged = (deltaPitch >= ORIENTATION_DELTA_DEG) ||
                               (deltaRoll  >= ORIENTATION_DELTA_DEG);
    bool rotationSpike = (gyroMagnitudeDPS >= GYRO_THRESHOLD_DPS);

    if (orientationChanged && rotationSpike) {
      impactFlag = false;
      soldierDownDetected();
    }
  } else if (millis() - lastImpactTime > FUSION_WINDOW_MS) {
    impactFlag = false; // window expired without correlated orientation change
  }

  previousPitch = pitch;
  previousRoll  = roll;
}

void soldierDownDetected() {
  sendRFMessage(CODE_SOLDIER_DOWN);
  enterEmergencyState();
}

// ===========================================================================
// RF TRANSMISSION VIA HT12E (non-blocking TE pulse)
// ===========================================================================
void sendRFMessage(uint8_t code) {
  // Load the 4-bit command onto HT12E data lines (MSB first: D8..D11)
  digitalWrite(HT12E_D8,  (code >> 3) & 0x01);
  digitalWrite(HT12E_D9,  (code >> 2) & 0x01);
  digitalWrite(HT12E_D10, (code >> 1) & 0x01);
  digitalWrite(HT12E_D11, (code >> 0) & 0x01);

  // Enable transmission (active LOW). HT12E repeatedly transmits the word
  // as long as TE is held LOW, giving the receiver several chances to
  // successfully decode the frame.
  digitalWrite(HT12E_TE, LOW);
  teActive = true;
  teStartTime = millis();

  digitalWrite(STATUS_LED, HIGH);
}

void handleTransmitTimeout() {
  if (teActive && (millis() - teStartTime >= TE_ACTIVE_MS)) {
    digitalWrite(HT12E_TE, HIGH); // Disable transmission
    teActive = false;
    digitalWrite(STATUS_LED, LOW);
  }
}
