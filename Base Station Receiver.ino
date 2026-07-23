/*
  ============================================================================
  RF-BASED INTELLIGENT SOLDIER SAFETY AND SILENT COMMUNICATION SYSTEM
  BASE STATION (RECEIVER) - Arduino UNO R3
  ============================================================================

  Hardware:
    - Arduino UNO R3
    - HT12D Decoder IC (fed by 433 MHz RF RX module)
    - 0.96" OLED Display SSD1306 over I2C
    - Active Buzzer Module
    - Green LED, Red LED

  Description:
    Continuously monitors the HT12D decoder outputs. When a valid 4-bit
    command word arrives (VT pin goes HIGH), the command is decoded and
    displayed on the OLED, an LED is driven according to severity, and an
    active buzzer sounds a pulsed alarm pattern for emergencies.

  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// PIN CONFIGURATION
// ---------------------------------------------------------------------------
#define HT12D_VT     2   // Valid Transmission flag from HT12D (active HIGH)
#define HT12D_D8     3   // Decoded data bit (MSB)
#define HT12D_D9     4
#define HT12D_D10    5
#define HT12D_D11    6   // Decoded data bit (LSB)

#define LED_GREEN    7
#define LED_RED      8
#define ALARM_PIN    9   // Active buzzer module IN pin (digital HIGH = ON).
                          // Active buzzers have their own internal oscillator,
                          // so no PWM/tone() is needed - just switch it on/off.
                          // Most 5V active buzzer modules can be driven directly
                          // from a digital pin (they include their own driver
                          // transistor); no external interface components needed.

// OLED uses hardware I2C: SDA -> A4, SCL -> A5 (UNO default, no #define needed)

// ---------------------------------------------------------------------------
// OLED CONFIGURATION
// ---------------------------------------------------------------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDRESS   0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------------------------------------------------------------------
// RF COMMAND CODES (must match transmitter mapping)
// ---------------------------------------------------------------------------
#define CODE_NEED_BACKUP     0b0001
#define CODE_ENEMY_SPOTTED   0b0010
#define CODE_MEDICAL_HELP    0b0011
#define CODE_MISSION_DONE    0b0100
#define CODE_SOLDIER_DOWN    0b1111

// ---------------------------------------------------------------------------
// TIMING CONSTANTS
// ---------------------------------------------------------------------------
#define MESSAGE_DISPLAY_MS   3000   // How long a received message stays on screen
#define ALARM_TONE_MS        4000   // Total alarm duration
#define SIREN_TOGGLE_MS      200    // On/off pulse interval for the buzzer
#define VT_DEBOUNCE_MS       30     // Debounce for VT (valid transmission) pulses

// ---------------------------------------------------------------------------
// STATE MACHINE
// ---------------------------------------------------------------------------
enum ReceiverState {
  STATE_WAITING,
  STATE_MESSAGE,
  STATE_EMERGENCY
};

ReceiverState currentState = STATE_WAITING;
unsigned long stateEnteredAt = 0;
String currentMessageText = "";

// Alarm (buzzer) non-blocking control
bool alarmActive = false;
unsigned long alarmStartTime = 0;
unsigned long lastToneToggle = 0;
bool buzzerOn = true;

// VT debounce tracking
bool lastVTReading = LOW;
unsigned long lastVTChangeTime = 0;

// ===========================================================================
// SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(HT12D_VT, INPUT);
  pinMode(HT12D_D8, INPUT);
  pinMode(HT12D_D9, INPUT);
  pinMode(HT12D_D10, INPUT);
  pinMode(HT12D_D11, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  initializeOLED();
  displayWaitingScreen();
}

// ===========================================================================
// MAIN LOOP (kept short - real work happens in functions)
// ===========================================================================
void loop() {
  decodeMessage();
  updateOLED();
  activateAlarm();
}

// ===========================================================================
// INITIALIZATION
// ===========================================================================
bool initializeOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED initialization failed!");
    // Fall back to serial-only operation; LEDs/alarm still function.
    return false;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  return true;
}

// ===========================================================================
// RF DECODING (HT12D)
// ===========================================================================
void decodeMessage() {
  bool vtReading = digitalRead(HT12D_VT);

  // Debounce the VT line - RF noise can cause spurious brief pulses
  if (vtReading != lastVTReading) {
    lastVTChangeTime = millis();
  }

  if ((millis() - lastVTChangeTime) > VT_DEBOUNCE_MS) {
    if (vtReading == HIGH && lastVTReading != HIGH) {
      // Rising edge confirmed valid: read the 4-bit data word
      uint8_t code = 0;
      code |= (digitalRead(HT12D_D8)  << 3);
      code |= (digitalRead(HT12D_D9)  << 2);
      code |= (digitalRead(HT12D_D10) << 1);
      code |= (digitalRead(HT12D_D11) << 0);

      handleReceivedCode(code);
    }
  }

  lastVTReading = vtReading;
}

void handleReceivedCode(uint8_t code) {
  switch (code) {
    case CODE_NEED_BACKUP:
      showMessage("Need Backup", false);
      break;
    case CODE_ENEMY_SPOTTED:
      showMessage("Enemy Spotted", false);
      break;
    case CODE_MEDICAL_HELP:
      showMessage("Need Medical Help", true);
      break;
    case CODE_MISSION_DONE:
      showMessage("Mission Completed", false);
      break;
    case CODE_SOLDIER_DOWN:
      triggerEmergency();
      break;
    default:
      // Unrecognized / corrupted code - ignore
      break;
  }
}

// ===========================================================================
// STATE TRANSITIONS
// ===========================================================================
void showMessage(const char* text, bool isCritical) {
  currentState = STATE_MESSAGE;
  currentMessageText = text;
  stateEnteredAt = millis();

  driveLEDs(isCritical);
  if (isCritical) {
    startAlarm();
  }
}

void triggerEmergency() {
  currentState = STATE_EMERGENCY;
  currentMessageText = "Soldier Down";
  stateEnteredAt = millis();

  driveLEDs(true);
  startAlarm();
}

// ===========================================================================
// OLED DISPLAY (state-driven, non-blocking)
// ===========================================================================
void updateOLED() {
  switch (currentState) {
    case STATE_WAITING:
      displayWaitingScreen();
      break;

    case STATE_MESSAGE:
      displayMessage();
      if (millis() - stateEnteredAt >= MESSAGE_DISPLAY_MS) {
        currentState = STATE_WAITING;
      }
      break;

    case STATE_EMERGENCY:
      displayEmergency();
      // Emergency screen persists until a new non-emergency message
      // (e.g. Mission Completed) is received from the soldier unit.
      break;
  }
}

void displayWaitingScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("COMMAND CENTER");
  display.setCursor(0, 30);
  display.println("Waiting...");
  display.display();
}

void displayMessage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ID : S-01");
  display.setCursor(0, 25);
  display.println(currentMessageText);
  display.display();
}

void displayEmergency() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(5, 15);
  display.println("EMERGENCY");
  display.setTextSize(1);
  display.setCursor(5, 40);
  display.println("SOLDIER DOWN");
  display.display();
}

// ===========================================================================
// LED CONTROL
// ===========================================================================
void driveLEDs(bool isCritical) {
  if (isCritical) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
  }
}

// ===========================================================================
// ALARM CONTROL (active buzzer, non-blocking pulsed pattern)
// ===========================================================================
void startAlarm() {
  alarmActive = true;
  alarmStartTime = millis();
  lastToneToggle = millis();
  buzzerOn = true;
  digitalWrite(ALARM_PIN, HIGH); // Active buzzer: HIGH = sound ON
}

void activateAlarm() {
  if (!alarmActive) return;

  if (millis() - alarmStartTime >= ALARM_TONE_MS) {
    digitalWrite(ALARM_PIN, LOW); // Turn buzzer off
    alarmActive = false;
    return;
  }

  // Pulse the buzzer on/off to create an attention-grabbing alarm pattern
  // (active buzzers only support on/off - they have their own fixed-pitch
  // internal oscillator, so pitch alternation like a siren isn't possible).
  if (millis() - lastToneToggle >= SIREN_TOGGLE_MS) {
    lastToneToggle = millis();
    buzzerOn = !buzzerOn;
    digitalWrite(ALARM_PIN, buzzerOn ? HIGH : LOW);
  }
}
