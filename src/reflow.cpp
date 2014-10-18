/*
 * Adapted from Reflow Oven Controller 
 * Version: 1.20
 * Date: 26-11-2012
 * Company: Rocket Scream Electronics
 * Author: Lim Phang Moh
 * Website: www.rocketscream.com
 *
 * Brief
 * =====
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
 */

#include <Arduino.h>
#include <MAX31855.h>

#include "reflow_statemachine.h"

// These should match index of REFLOW_STATE
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

Reflow::Reflow(int LedPin, int SSRPin, int BuzzerPin, ReflowSettings* Settings) : led_pin(LedPin), ssr_pin(SSRPin), buzzer_pin(BuzzerPin), settings(Settings)
{
    // Initial values for PID params
    kp = settings->pid_kp_preheat;
    ki = settings->pid_ki_preheat;
    kd = settings->pid_kd_preheat;

    // Specify PID control interface
    reflowOvenPID = new PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

    // Set window size
    windowSize = 2000;
}

Reflow::~Reflow() {
    delete reflowOvenPID;
}

void Reflow::check_state(double& new_input)
{
    input = new_input;

    // If thermocouple problem detected move directly
    // to error state
    if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) ||
       (input == FAULT_SHORT_VCC))
    {
        // Illegal operation
        reflowState = REFLOW_STATE_ERROR;
        reflowStatus = REFLOW_STATUS_OFF;
    }

    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON) {
        // Toggle red LED as system heart beat
        digitalWrite(led_pin, !(digitalRead(led_pin)));

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
        digitalWrite(led_pin, HIGH);

        // Reflow oven process is off, ensure oven is off
        digitalWrite(ssr_pin, LOW);
    }

    // Run a specific state's check function
    const StateStruct* p_state_map = state_map();
    (this->*p_state_map[reflowState].state_method)();

    // Perform PID computations and turn on or off SSR
    compute_pid();
}

void Reflow::write_lcd_message(LiquidCrystal& lcd) {
    lcd.print(lcdMessagesReflowStatus[reflowState]);
}

void Reflow::begin()
{
    // Send header for CSV file
    Serial.println("Time,Setpoint,Input,Output");

    // Intialize seconds timer for serial debug information
    timerSeconds = 0;

    // Initialize PID control window starting time
    windowStartTime = millis();

    // Ramp up to minimum soaking temperature
    setpoint = settings->temperature_soak_min + settings->soak_temperature_step;

    // Tell the PID to range between 0 and the full window size
    reflowOvenPID->SetOutputLimits(0, windowSize);
    reflowOvenPID->SetSampleTime(settings->pid_sample_time);

    // Turn the PID on
    reflowOvenPID->SetMode(AUTOMATIC);

    // Proceed to preheat stage
    reflowState = REFLOW_STATE_PREHEAT;
}

void Reflow::end()
{
    // Turn off reflow process
    reflowStatus = REFLOW_STATUS_OFF;
    // Reinitialize state machine
    reflowState = REFLOW_STATE_IDLE;
}

void Reflow::idle_state()
{
    // If oven temperature is still above room temperature
    if (input >= settings->temperature_room) {
        reflowState = REFLOW_STATE_TOO_HOT;
    }
}

void Reflow::preheat_state()
{
    reflowStatus = REFLOW_STATUS_ON;

    // If minimum soak temperature is achieve
    if (input >= settings->temperature_soak_min) {
        // Chop soaking period into smaller sub-period, if there is a min and max
        // for the soak period
        timerSoak = millis() + settings->soak_micro_period;

        // Set maximum period for soak step
        timerPeriod = millis() + settings->soak_period;

        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID->SetTunings(settings->pid_kp_soak,
                                  settings->pid_ki_soak, 
                                  settings->pid_kd_soak);

        // Ramp up to first section of soaking temperature
        setpoint = settings->temperature_soak_min;

        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;

        Serial.println("Begin soak");
    }
}

void Reflow::soak_state()
{
    // If micro soak temperature is achieved
    if (settings->soak_micro_period > 0 && millis() > timerSoak) {
        timerSoak = millis() + settings->soak_micro_period;

        // Increment micro setpoint
        setpoint += settings->soak_temperature_step;
    }

    if (millis() > timerPeriod) {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID->SetTunings(settings->pid_kp_reflow, 
                                  settings->pid_ki_reflow,
                                  settings->pid_kd_reflow);

        // Ramp up to first section of soaking temperature
        setpoint = settings->temperature_reflow_max;

        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW;

        Serial.println("Begin reflow");
    }
}


void Reflow::reflow_state()
{
    if(input < settings->temperature_reflow_max - 5) {
        // Set period for reflow state
        reflowPeriod = millis() + settings->reflow_period;
    }
    
    // Start cool down when time has passed for reflow
    if (millis() > reflowPeriod) {
        // Set PID parameters for cooling ramp
        reflowOvenPID->SetTunings(settings->pid_kp_reflow, 
                                  settings->pid_ki_reflow,
                                  settings->pid_kd_reflow);

        // Ramp down to minimum cooling temperature
        setpoint = settings->temperature_cool_min;

        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;

        Serial.println("Begin cool");
    }
}

void Reflow::cool_state()
{

    // If minimum cool temperature is achieve
    if (input <= settings->temperature_cool_min) {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;

        // Turn on buzzer and green LED to indicate completion
        digitalWrite(buzzer_pin, HIGH);

        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;

        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;

        Serial.println("Complete");
    }
}

void Reflow::complete_state()
{

    if (millis() > buzzerPeriod) {
        // Turn off buzzer and green LED
        digitalWrite(buzzer_pin, LOW);

        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
    }

}

void Reflow::too_hot_state()
{

    // If oven temperature drops below room temperature
    if (input < settings->temperature_room) {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
    }
}

void Reflow::error_state()
{

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

}

void Reflow::compute_pid()
{
    // PID computation and SSR control
    if (reflowStatus == REFLOW_STATUS_ON) {
        unsigned long now = millis();

        reflowOvenPID->Compute();

        if((now - windowStartTime) > windowSize) {
            // Time to shift the Relay Window
            windowStartTime += windowSize;
        }

        if(output > (now - windowStartTime)) {
            digitalWrite(ssr_pin, HIGH);
        } else {
            digitalWrite(ssr_pin, LOW);
        }
    }
}
