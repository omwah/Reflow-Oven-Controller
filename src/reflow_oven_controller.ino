/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.20
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
*
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller.
* The reflow curve used in this firmware is meant for lead-free profile
* (it's even easier for leaded process!). You'll need to use the MAX31855
* library for Arduino if you are having a shield of v1.60 & above which can be
* downloaded from our GitHub repository. Please check our wiki
* (www.rocketscream.com/wiki) for more information on using this piece of code
* together with the reflow oven controller shield.
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          |
*     |         x     |                   |                          |
*     |       x       |                   |                          |
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
*                                                                Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID
* implementation, he gave a lot of help in making this reflow oven controller
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of
* any information or materials on this reflow oven controller is entirely at
* your own risk, for which we shall not be liable.
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/
* You are free to take this piece of code, use it and modify it.
* All we ask is attribution including the supporting libraries used in this
* firmware.
*
* Required Libraries
* ==================
* - Arduino PID Library:
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above):
*   >> https://github.com/rocketscream/MAX31855
*
* Revision  Description
* ========  ===========
* 1.20          Adds supports for v1.60 (and above) of Reflow Oven Controller
*           Shield:
*                     - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*             to be used for user application).
*                     - Uses analog based switch (allowing D2 & D3 to be used for user
*                         application).
*                       Adds waiting state when temperature too hot to start reflow process.
*                       Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10      Arduino IDE 1.0 compatible.
* 1.00      Initial public release.
*******************************************************************************/

// ***** INCLUDES *****
#include <LiquidCrystal.h>

// Newer board version starts from v1.60 using MAX31855KASA+ chip
#include <MAX31855.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE {
    REFLOW_STATE_IDLE,
    REFLOW_STATE_PREHEAT,
    REFLOW_STATE_SOAK,
    REFLOW_STATE_REFLOW,
    REFLOW_STATE_COOL,
    REFLOW_STATE_COMPLETE,
    REFLOW_STATE_TOO_HOT,
    REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS {
    REFLOW_STATUS_OFF,
    REFLOW_STATUS_ON
} reflowStatus_t;

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
#define TEMPERATURE_ROOM 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define DEBOUNCE_PERIOD_MIN 50

#define SOAK_PERIOD 90000
#define REFLOW_PERIOD 60000
#define TEMPERATURE_COOL_MIN 100

// Temperature constants for Pb-Free Operation
//#define TEMPERATURE_SOAK_MIN 150
//#define TEMPERATURE_SOAK_MAX 200
//#define TEMPERATURE_REFLOW_MAX 250

// Temperature constants for Sn63 Pb37 - 183 + 20
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 150
#define TEMPERATURE_REFLOW_MAX 215

#define TEMPERATURE_SOAK_DIFF TEMPERATURE_SOAK_MAX - TEMPERATURE_SOAK_MIN
#define SOAK_MICRO_PERIOD TEMPERATURE_SOAK_DIFF > 0 ? SOAK_PERIOD / (TEMPERATURE_SOAK_DIFF / SOAK_TEMPERATURE_STEP) : 0

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// Maximum number of reading errors before going back to idle
#define MAX_NUM_ERRORS 5

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
    "Ready",
    "Pre-heat",
    "Soak",
    "Reflow",
    "Cool",
    "Complete",
    "Wait,hot",
    "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
    140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENT *****
int ssrPin = 5;
int thermocoupleSOPin = A3;
int thermocoupleCSPin = A2;
int thermocoupleCLKPin = A1;
int lcdRsPin = 7;
int lcdEPin = 8;
int lcdD4Pin = 9;
int lcdD5Pin = 10;
int lcdD6Pin = 11;
int lcdD7Pin = 12;
int ledRedPin = 4;
int buzzerPin = 6;
int switchPin = A0;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long timerPeriod;
unsigned long reflowPeriod;
unsigned long buzzerPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Specify LCD interface
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

// Specify MAX31855 thermocouple interface
MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin,
                      thermocoupleCLKPin);

void setup()
{
    // SSR pin initialization to ensure reflow oven is off
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
    lcd.print("Reflow");
    lcd.setCursor(0, 1);
    lcd.print("Oven 1.2");
    digitalWrite(buzzerPin, LOW);
    delay(2500);
    lcd.clear();

    // Serial communication at 57600 bps
    Serial.begin(57600);

    // Turn off LED (active low)
    digitalWrite(ledRedPin, HIGH);

    // Set window size
    windowSize = 2000;
    // Initialize time keeping variable
    nextCheck = millis();
    // Initialize thermocouple reading variable
    nextRead = millis();
}

void loop()
{
    // Current time
    unsigned long now;

    // Time to read thermocouple?
    if (millis() > nextRead) {
        // Read thermocouple next sampling period
        nextRead += SENSOR_SAMPLING_TIME;
        // Read current temperature
        input = thermocouple.readThermocouple(CELSIUS);

        // If thermocouple problem detected
        if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) ||
                (input == FAULT_SHORT_VCC))
        {
            // Illegal operation
            reflowState = REFLOW_STATE_ERROR;
            reflowStatus = REFLOW_STATUS_OFF;
        }
    }

    if (millis() > nextCheck) {
        // Check input in the next seconds
        nextCheck += 1000;

        // If reflow process is on going
        if (reflowStatus == REFLOW_STATUS_ON) {
            // Toggle red LED as system heart beat
            digitalWrite(ledRedPin, !(digitalRead(ledRedPin)));
            // Increase seconds timer for reflow curve analysis
            timerSeconds++;
            // Send temperature and time stamp to serial
            Serial.print(timerSeconds);
            Serial.print(",");
            Serial.print(setpoint);
            Serial.print(",");
            Serial.print(input);
            Serial.print(",");
            Serial.println(output);
        } else {
            // Turn off red LED
            digitalWrite(ledRedPin, HIGH);
        }

        // Clear LCD
        lcd.clear();
        // Print current system state
        lcd.print(lcdMessagesReflowStatus[reflowState]);
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
            lcd.print(input);

#if ARDUINO >= 100
            // Print degree Celsius symbol
            lcd.write((uint8_t)0);
#else
            // Print degree Celsius symbol
            lcd.print(0, BYTE);
#endif
            lcd.print("C ");
        }
    }

    // Reflow oven controller state machine
    switch (reflowState) {
    case REFLOW_STATE_IDLE:

        // If oven temperature is still above room temperature
        if (input >= TEMPERATURE_ROOM) {
            reflowState = REFLOW_STATE_TOO_HOT;
        } else {
            // If switch is pressed to start reflow process
            if (switchStatus == SWITCH_1) {
                // Send header for CSV file
                Serial.println("Time,Setpoint,Input,Output");
                // Intialize seconds timer for serial debug information
                timerSeconds = 0;
                // Initialize PID control window starting time
                windowStartTime = millis();
                // Ramp up to minimum soaking temperature
                setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
                // Tell the PID to range between 0 and the full window size
                reflowOvenPID.SetOutputLimits(0, windowSize);
                reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
                // Turn the PID on
                reflowOvenPID.SetMode(AUTOMATIC);
                // Proceed to preheat stage
                reflowState = REFLOW_STATE_PREHEAT;
            }
        }

        break;

    case REFLOW_STATE_PREHEAT:
        reflowStatus = REFLOW_STATUS_ON;

        // If minimum soak temperature is achieve
        if (input >= TEMPERATURE_SOAK_MIN) {
            // Chop soaking period into smaller sub-period, if there is a min and max
            // for the soak period
            timerSoak = millis() + SOAK_MICRO_PERIOD;
            // Set maximum period for soak step
            timerPeriod = millis() + SOAK_PERIOD;
            // Set less agressive PID parameters for soaking ramp
            reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
            // Ramp up to first section of soaking temperature
            setpoint = TEMPERATURE_SOAK_MIN;
            // Proceed to soaking state
            reflowState = REFLOW_STATE_SOAK;

            Serial.println("Begin soak");
        }

        break;

    case REFLOW_STATE_SOAK:

        // If micro soak temperature is achieved
        if (SOAK_MICRO_PERIOD > 0 && millis() > timerSoak) {
            timerSoak = millis() + SOAK_MICRO_PERIOD;

            // Increment micro setpoint
            setpoint += SOAK_TEMPERATURE_STEP;
        }

        if (millis() > timerPeriod) {
            // Set agressive PID parameters for reflow ramp
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            // Ramp up to first section of soaking temperature
            setpoint = TEMPERATURE_REFLOW_MAX;
            // Proceed to reflowing state
            reflowState = REFLOW_STATE_REFLOW;

            Serial.println("Begin reflow");
        }

        break;

    case REFLOW_STATE_REFLOW:

        if(input < TEMPERATURE_REFLOW_MAX - 5) {
            // Set period for reflow state
            reflowPeriod = millis() + REFLOW_PERIOD;
        }
        
        // Start cool down when time has passed for reflow
        if (millis() > reflowPeriod) {
            // Set PID parameters for cooling ramp
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            // Ramp down to minimum cooling temperature
            setpoint = TEMPERATURE_COOL_MIN;
            // Proceed to cooling state
            reflowState = REFLOW_STATE_COOL;

            Serial.println("Begin cool");
        }

        break;

    case REFLOW_STATE_COOL:

        // If minimum cool temperature is achieve
        if (input <= TEMPERATURE_COOL_MIN) {
            // Retrieve current time for buzzer usage
            buzzerPeriod = millis() + 1000;
            // Turn on buzzer and green LED to indicate completion
            digitalWrite(buzzerPin, HIGH);
            // Turn off reflow process
            reflowStatus = REFLOW_STATUS_OFF;
            // Proceed to reflow Completion state
            reflowState = REFLOW_STATE_COMPLETE;

            Serial.println("Complete");
        }

        break;

    case REFLOW_STATE_COMPLETE:
        if (millis() > buzzerPeriod) {
            // Turn off buzzer and green LED
            digitalWrite(buzzerPin, LOW);

            // Reflow process ended
            reflowState = REFLOW_STATE_IDLE;
        }

        break;

    case REFLOW_STATE_TOO_HOT:

        // If oven temperature drops below room temperature
        if (input < TEMPERATURE_ROOM) {
            // Ready to reflow
            reflowState = REFLOW_STATE_IDLE;
        }

        break;

    case REFLOW_STATE_ERROR:

        // If thermocouple problem is still present
        if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) ||
                (input == FAULT_SHORT_VCC))
        {
            // Wait until thermocouple wire is connected
            reflowState = REFLOW_STATE_ERROR;
        } else {
            // Clear to perform reflow process
            reflowState = REFLOW_STATE_IDLE;
        }

        break;
    }

    // If switch 1 is pressed
    if (switchStatus == SWITCH_1) {
        // If currently reflow process is on going
        if (reflowStatus == REFLOW_STATUS_ON) {
            // Button press is for cancelling
            // Turn off reflow process
            reflowStatus = REFLOW_STATUS_OFF;
            // Reinitialize state machine
            reflowState = REFLOW_STATE_IDLE;
        }
    }

    // Simple switch debounce state machine (for switch #1 (both analog & digital
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

    // PID computation and SSR control
    if (reflowStatus == REFLOW_STATUS_ON) {
        now = millis();

        reflowOvenPID.Compute();

        if((now - windowStartTime) > windowSize) {
            // Time to shift the Relay Window
            windowStartTime += windowSize;
        }

        if(output > (now - windowStartTime)) {
            digitalWrite(ssrPin, HIGH);
        } else {
            digitalWrite(ssrPin, LOW);
        }
    }
    // Reflow oven process is off, ensure oven is off
    else {
        digitalWrite(ssrPin, LOW);
    }
}
