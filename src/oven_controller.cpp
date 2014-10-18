/*******************************************************************************
*
* Uses the RocketScream reflow shield for various purposes
*
* Required Libraries
* ==================
* - Arduino PID Library:
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above):
*   >> https://github.com/rocketscream/MAX31855
*
*******************************************************************************/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <MAX31855.h>

#include "reflow.h"

typedef enum SWITCH {
    SWITCH_NONE,
    SWITCH_1,
    SWITCH_2
}   switch_t;

typedef enum DEBOUNCE_STATE {
    DEBOUNCE_STATE_IDLE,
    DEBOUNCE_STATE_CHECK,
    DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define SENSOR_SAMPLING_TIME 1000
#define DEBOUNCE_PERIOD_MIN 50

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
    140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENT *****
int ledRedPin = 4;
int ssrPin = 5;
int buzzerPin = 6;
int switchPin = A0;

// Thermocouple reading
double input;

// Should we check that an action is needed
unsigned long nextCheck;

// When we should read temperature next
unsigned long nextRead;

// Switch debounce timer
long lastDebounceTime;

// Switch press status
switch_t switchStatus;

// Switch debounce state machine state variable
debounceState_t debounceState;

// Display temp in C or F, unit_t comes from MAX31855 library
unit_t tempMode;

// Specify LCD interface
int lcdRsPin = 7;
int lcdEPin = 8;
int lcdD4Pin = 9;
int lcdD5Pin = 10;
int lcdD6Pin = 11;
int lcdD7Pin = 12;
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

// Specify MAX31855 thermocouple interface
int thermocoupleSOPin = A3;
int thermocoupleCSPin = A2;
int thermocoupleCLKPin = A1;
MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin, thermocoupleCLKPin);

LeadedSettings reflow_settings = LeadedSettings();
Reflow reflow = Reflow(ledRedPin, ssrPin, buzzerPin, &reflow_settings);

void setup()
{
    // SSR pin initialization to ensure switch oven is off
    digitalWrite(ssrPin, LOW);
    pinMode(ssrPin, OUTPUT);

    // Buzzer pin initialization to ensure annoying buzzer is off
    digitalWrite(buzzerPin, LOW);
    pinMode(buzzerPin, OUTPUT);

    // LED pins initialization and turn on upon start-up (active low)
    digitalWrite(ledRedPin, LOW);
    pinMode(ledRedPin, OUTPUT);

    // Start-up splash
    digitalWrite(buzzerPin, HIGH);
    lcd.begin(8, 2);
    lcd.createChar(0, degree);
    lcd.clear();

    lcd.print("Temp");
    lcd.setCursor(0, 1);
    lcd.print("Control");

    digitalWrite(buzzerPin, LOW);
    delay(2500);
    lcd.clear();
    digitalWrite(buzzerPin, HIGH);

    // Serial communication at 57600 bps
    Serial.begin(57600);

    // Turn off LED (active low)
    digitalWrite(ledRedPin, HIGH);

    // Start showing temp in celsius
    tempMode = CELSIUS;

    // Initialize time keeping variables
    nextCheck = millis();
    nextRead = millis();

    // Output reflow settings for debugging
    reflow_settings.print();
}

void loop()
{
    // Time to read thermocouple?
    if (millis() > nextRead) {
        // Read thermocouple next sampling period
        nextRead += SENSOR_SAMPLING_TIME;
        // Read current temperature
        input = thermocouple.readThermocouple(CELSIUS);
    }

    if (millis() > nextCheck) {
        // Check input in the next seconds
        nextCheck += 1000;

        // Clear LCD
        lcd.clear();

        // Print current system state
        reflow.write_lcd_message(lcd);

        // Move the cursor to the 2 line
        lcd.setCursor(0, 1);

        // Indicate which error condition is occuring
        if (input == FAULT_OPEN) {
            lcd.print("Open Err");
            Serial.println("Temperature fault: open error");
        } else if (input == FAULT_SHORT_GND) {
            lcd.print("GND Err");
            Serial.println("Temperature fault: ground short error");
        } else if (input == FAULT_SHORT_VCC) {
            lcd.print("VCC Err");
            Serial.println("Temperature fault: vcc short error");
        } else {
            // Print current temperature
            switch (tempMode) {
            case CELSIUS:
                lcd.print(input);
                break;
            case FAHRENHEIT:
                lcd.print(input * 9/5 + 32);
                break;
            }

#if ARDUINO >= 100
            // Print degree Celsius symbol
            lcd.write((uint8_t)0);
#else
            // Print degree Celsius symbol
            lcd.print(0, BYTE);
#endif
            switch (tempMode) {
            case CELSIUS:
                lcd.print("C ");
                break;
            case FAHRENHEIT:
                lcd.print("F ");
                break;
            }
        }
    }

    // If switch 1 is pressed
    if (switchStatus == SWITCH_1) {
        // If a process is on going cancel
        if (reflow.on()) {
            reflow.end();
        } else {
            reflow.begin();
        }
    }

    // Simple switch debounce state machine (for switch #1)
    // switch supported))
    switch (debounceState) {
    case DEBOUNCE_STATE_IDLE:
        // No valid switch press
        switchStatus = SWITCH_NONE;
        // If switch #1 is pressed
        if (analogRead(switchPin) == 0)
        {
            // Intialize debounce counter
            lastDebounceTime = millis();
            // Proceed to check validity of button press
            debounceState = DEBOUNCE_STATE_CHECK;
        }

        break;

    case DEBOUNCE_STATE_CHECK:
        if (analogRead(switchPin) == 0)
        {
            // If minimum debounce period is completed
            if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN) {
                // Proceed to wait for button release
                debounceState = DEBOUNCE_STATE_RELEASE;
            }
        }
        // False trigger
        else {
            // Reinitialize button debounce state machine
            debounceState = DEBOUNCE_STATE_IDLE;
        }

        break;

    case DEBOUNCE_STATE_RELEASE:
        if (analogRead(switchPin) > 0)
        {
            // Valid switch 1 press
            switchStatus = SWITCH_1;
            // Reinitialize button debounce state machine
            debounceState = DEBOUNCE_STATE_IDLE;
        }

        break;
    }

}
