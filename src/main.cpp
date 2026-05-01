#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

// --- Pin Definitions for Feather ESP32-S3 ---
#define TFT_CS    10
#define TFT_DC    9
#define TFT_RST   6
#define TFT_MOSI  35  // Software SPI "MO" pin
#define TFT_SCLK  36  // Software SPI "SCK" pin

// --- LED Status Pins ---
#define LED_GREEN  11 // Pressurized
#define LED_RED  12 // Unpressurized / Low Pressure
#define LED_YELLOW    13 // Warning / Halt

// Default I2C address for the PCA9546A multiplexer
#define PCA9546A_ADDR 0x70

// Fan pins
#define FAN_TACH_PIN A4  // Tachometer Pin
#define FAN_PWM_PIN 5

// Initialize the ST7789 object using software SPI
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Define our sensor objects
Adafruit_BMP3XX bmp1; // Mux Channel 0 (Outside)
Adafruit_BMP3XX bmp2; // Mux Channel 1 (Inside)
Adafruit_BME280 bme;  // Mux Channel 2 (Inside Ambient)

// --- Control Variables ---
float targetDelta_Pa = 15.0; // Target pressure difference in purely Pascals
// Tuned PID constants for 0-255 PWM output
float Kp = 12.0; 
float Ki = 0.5;
float Kd = 2.0;
float error = 0, integral = 0, previous_error = 0;

int currentFanSpeed = 0;   
bool manualMode = false;   
int manualPWM = 0;        

// --- Safety & Watchdog Variables ---
unsigned long systemStartTime = 0;
const unsigned long DAILY_RESET_MS = 86400000UL; // 24 hours

bool isFullSpeed = false;
unsigned long fullSpeedStartTime = 0;
bool systemHalted = false;
bool haltScreenDrawn = false; // <--- NEW: Tracks if we already painted the screen
const unsigned long WARN_TIME_MS = 300000UL; // 5 minutes
const unsigned long HALT_TIME_MS = 900000UL; // 15 minutes

// --- Tachometer Variables ---
volatile unsigned long tachPulses = 0; 
volatile unsigned long lastIsrTime = 0;
unsigned long lastTachTime = 0;
int currentRPM = 0;
int systemState = 0; // State variable for MATLAB

// --- Interrupt Service Routine (ISR) ---
void IRAM_ATTR tachISR() {
  unsigned long isrTime = micros();
  // DEBOUNCE: Lowered to 500us (0.5ms) to catch the lightning-fast 8-pulse tachometer!
  if (isrTime - lastIsrTime > 500) {
    tachPulses++;
    lastIsrTime = isrTime;
  }
}

// Helper function to switch PCA9546A channels (0 to 3)
void switchMuxChannel(uint8_t channel) {
  if (channel > 3) return; 
  Wire.beginTransmission(PCA9546A_ADDR);
  Wire.write(1 << channel); 
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  
  // Setup Fan PWM
  pinMode(FAN_PWM_PIN, OUTPUT);
  analogWrite(FAN_PWM_PIN, 0); 
  analogWriteFrequency(25000); // Set PWM frequency to 25 kHz for quieter operation

  // Setup LEDs
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);

  // Setup Fan Tachometer 
  pinMode(FAN_TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), tachISR, FALLING);
  lastTachTime = millis();
  systemStartTime = millis();

  tft.init(240, 320);
  tft.setRotation(1); 
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(true);

  Wire.begin();

  // --- Initialize Sensors ---
  switchMuxChannel(0);
  if (bmp1.begin_I2C()) { 
    bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp1.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp1.setOutputDataRate(BMP3_ODR_12_5_HZ);
  }

  switchMuxChannel(1);
  if (bmp2.begin_I2C()) {
    bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp2.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp2.setOutputDataRate(BMP3_ODR_12_5_HZ);
  }

  switchMuxChannel(2);
  bme.begin();
}

void loop() {
  // --- 0.1. Watchdog: 24-Hour Automatic Reset ---
  if (millis() - systemStartTime >= DAILY_RESET_MS) {
    ESP.restart(); 
  }

// --- 0.5. HALT STATE LOCKOUT ---
  if (systemHalted) {
    analogWrite(FAN_PWM_PIN, 0); // Keep fan off
    
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_GREEN, LOW);
    
    // Only erase and draw the screen ONCE
    if (!haltScreenDrawn) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 0);
      tft.setTextSize(3);
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      tft.println(" SYSTEM HALT!  ");
      tft.println("               ");
      tft.println(" FAN TIMEOUT   ");
      tft.println(" (15 MIN AT MAX SPEED)  ");
      tft.println("               ");
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.println(" PRESS BUTTON  ");
      tft.println(" TO RESET      ");
      
      haltScreenDrawn = true; // Lock the flag so this block never runs again
    }
    
// Send 7-value string with State 3 (Halt) to MATLAB
    // Format: delta_PSI, outTemp, inTemp, Hum, PWM, RPM, State
    Serial.println("0.000,0.0,0.0,0.0,0,0,3");
    delay(500);
    return; // Exits loop() until reset
  }
  
// --- 1. Calculate RPM ---
  noInterrupts();
  unsigned long pulses = tachPulses;
  tachPulses = 0; 
  interrupts();

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTachTime;

if (pulses > 0 && elapsedTime > 0) {
    // 1. Calculate raw granular math (Updated for 8-pulse industrial tachometer)
    float rawRPM = (pulses / 8.0) * (60000.0 / elapsedTime);
    
    // 2. EMA Filter: 30% New Data, 70% Old Data (Smooths the needle!)
    currentRPM = (0.3 * rawRPM) + (0.7 * currentRPM);
    
    // 3. UI Safety Cap: Lock it so it never exceeds 5500
    currentRPM = constrain(currentRPM, 0, 5500);
    
    lastTachTime = currentTime;
  } else if (currentTime - lastTachTime > 1000) {
    // If 1 full second passes with no pulses, the fan has stopped!
    currentRPM = 0;
  }

  // --- 2. Check for incoming MATLAB Commands ---
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); 
    
    if (command.startsWith("A")) {
      manualMode = false; 
    } 
    else if (command.startsWith("M")) {
      manualMode = true; 
      int commaIndex = command.indexOf(',');
      if (commaIndex > 0) {
        manualPWM = command.substring(commaIndex + 1).toInt();
        manualPWM = constrain(manualPWM, 0, 255);
      }
    }
else if (command.startsWith("T")) {
      int commaIndex = command.indexOf(',');
      if (commaIndex > 0) {
        // Read directly in Pascals! No math required.
        targetDelta_Pa = command.substring(commaIndex + 1).toFloat();
      }
    }
  }

  // --- 3. Read Sensors ---
  float outPress_Pa = 0.0;
  float inPress_Pa = 0.0;
  float outTemp_C = 0.0;
  float inTemp_C = 0.0;

  switchMuxChannel(0);
  if (bmp1.performReading()) {
    outPress_Pa = bmp1.pressure; // Native reading is already in Pascals!
    outTemp_C = bmp1.temperature;
  }
  
  switchMuxChannel(1);
  if (bmp2.performReading()) {
    inPress_Pa = bmp2.pressure;
    inTemp_C = bmp2.temperature;
  }

  // Pure Pascal math
  float currentDelta_Pa = outPress_Pa - inPress_Pa; 

  // --- 4. Fan Control Logic ---
  if (manualMode) {
    currentFanSpeed = manualPWM;
  } else {
    // PID Loop now running entirely in Pascals
    error = targetDelta_Pa - currentDelta_Pa;
    integral += error; 
    integral = constrain(integral, -500, 500); // Increased to allow fan to hit max speed
    float derivative = error - previous_error;
    
    float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
    currentFanSpeed = constrain(pidOutput, 0, 255);
    previous_error = error;
  }

  analogWrite(FAN_PWM_PIN, currentFanSpeed);

// --- 5. Redundancy / Timeout Math ---
  // Using 250 instead of 255 as a safe threshold for "Maximum Speed"
  if (currentFanSpeed >= 250) { 
    if (!isFullSpeed) {
      isFullSpeed = true;
      fullSpeedStartTime = millis(); 
    }
  } else {
    // The moment the fan drops below max (either by Auto PID or Manual Slider), reset everything!
    isFullSpeed = false; 
    fullSpeedStartTime = 0; 
  }

  unsigned long fullSpeedDuration = 0;
  if (isFullSpeed) {
    fullSpeedDuration = millis() - fullSpeedStartTime;
  }

  if (fullSpeedDuration >= HALT_TIME_MS) {
    systemHalted = true; 
  }

  // --- 5.5. LED Status Light Logic ---
  bool isPressurized = currentDelta_Pa >= (targetDelta_Pa * 0.90);

  // 1. Evaluate Pressure Independently (Red vs Green)
  if (isPressurized) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
  }

  // 2. Evaluate Warning Independently (Yellow)
  if (fullSpeedDuration >= WARN_TIME_MS) {
    digitalWrite(LED_YELLOW, HIGH);
    systemState = isPressurized ? 4 : 2; 
  } else {
    digitalWrite(LED_YELLOW, LOW);
    systemState = isPressurized ? 1 : 0; 
  }

  // --- 6. Conversions for Display & MATLAB ---
  // Temperature still converted to F for US readability
  float outTemp_F = (outTemp_C * 1.8) + 32.0;
  
  switchMuxChannel(2);
  float ambTemp_F = (bme.readTemperature() * 1.8) + 32.0;
  float ambHum    = bme.readHumidity();

  // --- 7. Send Telemetry to MATLAB ---
  // Format: delta_Pa, outTemp_F, ambTemp_F, ambHum, pwm, rpm, state
  Serial.print(currentDelta_Pa, 1); Serial.print(",");
  Serial.print(outTemp_F, 2); Serial.print(","); 
  Serial.print(ambTemp_F, 2); Serial.print(",");
  Serial.print(ambHum, 2);    Serial.print(",");
  Serial.print(currentFanSpeed); Serial.print(",");
  Serial.print(currentRPM);   Serial.print(",");
  Serial.println(systemState);

  // --- 8. Update TFT Display ---
  tft.setCursor(0, 0);
  tft.setTextSize(2); 

  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK); 
  tft.println("Outside Chamber:");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print("T: "); tft.print(outTemp_F, 1); tft.println(" F        "); 
  // Print whole Pascals (0 decimal places for absolute pressure)
  tft.print("P: "); tft.print(outPress_Pa, 0); tft.println(" Pa    "); 
  tft.println(); 

  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.println("Inside Chamber:");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print("P: "); tft.print(inPress_Pa, 0); tft.println(" Pa    ");
  tft.print("T: "); tft.print(ambTemp_F, 1); tft.println(" F        ");
  tft.print("H: "); tft.print(ambHum, 0); tft.println(" %  ");
  tft.println();

  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  tft.println("Pressure Delta:");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  // Print delta with 1 decimal place for fine tuning
  tft.print("Diff: "); tft.print(currentDelta_Pa, 1); tft.println(" Pa      "); 
  tft.println();

  tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  if (manualMode) tft.print("MANUAL: ");
  else tft.print("AUTO:   ");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print(currentFanSpeed); tft.print("/255 "); 
  tft.print(currentRPM); tft.println(" RPM         ");

// --- 5 MINUTE WARNING DISPLAY LOGIC ---
  if (fullSpeedDuration >= WARN_TIME_MS) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    // Split into two explicit lines (under 26 chars each)
    tft.println("WARNING! MAX FAN FOR "); 
    tft.println("5+ MINS - CHECK SYSTEM "); 
  } else {
    // Exactly 26 blank spaces per line to guarantee a clean wipe!
    tft.setTextColor(ST77XX_BLACK, ST77XX_BLACK);
    tft.println("                          ");
    tft.println("                          ");
  }

  delay(150);
}