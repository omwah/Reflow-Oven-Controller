#ifndef REFLOW_STATEMACHINE_H
#define REFLOW_STATEMACHINE_H

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

#include "state_machine.h"

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

struct ReflowSettings {
    ReflowSettings() {
        init_defaults();
        init_settings();
        init_soak_micro();
    }

    void init_defaults() {
        pid_sample_time = 1000;

        temperature_room = 100;
        soak_temperature_step = 5;

        soak_period = 90000;
        reflow_period = 60000;
        temperature_cool_min = 100;

        // These need to be set for a particular implementation
        temperature_soak_min = 0;
        temperature_soak_max = 0;
        temperature_reflow_max = 0;

        pid_kp_preheat = 100;
        pid_ki_preheat = 0.025;
        pid_kd_preheat = 20;

        pid_kp_soak = 300;
        pid_ki_soak = 0.05;
        pid_kd_soak = 250;

        pid_kp_reflow = 300;
        pid_ki_reflow = 0.05;
        pid_kd_reflow = 350;
    }

    void init_soak_micro() {
        float temperature_soak_diff = temperature_soak_max - temperature_soak_min;
        if (temperature_soak_diff > 0) {
            soak_micro_period =  soak_period / (temperature_soak_diff / soak_temperature_step);
        } else {
            soak_micro_period = 0;
        }
    }

    // Implement this to overide settings
    virtual void init_settings() {};

    unsigned int pid_sample_time;

    float temperature_room;
    unsigned int soak_temperature_step;

    unsigned long soak_period;
    unsigned long reflow_period;
    float temperature_cool_min;

    float temperature_soak_min;
    float temperature_soak_max;
    float temperature_reflow_max;

    unsigned int soak_micro_period;

    // ***** PID PARAMETERS *****
    float pid_kp_preheat;
    float pid_ki_preheat;
    float pid_kd_preheat;

    float pid_kp_soak;
    float pid_ki_soak;
    float pid_kd_soak;

    float pid_kp_reflow;
    float pid_ki_reflow;
    float pid_kd_reflow;
};

struct LeadedSettings : public ReflowSettings {
    virtual void init_settings() {
        // Temperature constants for Sn63 Pb37 - 183 + 20
        temperature_soak_min = 150;
        temperature_soak_max = 150;
        temperature_reflow_max = 215;
    }

};

struct LeadedFreeSettings : public ReflowSettings {
    virtual void init_settings() {
        // Temperature constants for Pb-Free Operation
        temperature_soak_min = 150;
        temperature_soak_max = 200;
        temperature_reflow_max = 250;
    }
};

class Reflow : public StateMachine {
    public:
        Reflow(int LedPin, int SSRPin, int BuzzerPin, ReflowSettings* Settings);
        ~Reflow();
        void check_state(double& new_input);
        void write_lcd_message(LiquidCrystal &lcd);
        void begin();
        void end();
        bool on() { reflowStatus == REFLOW_STATUS_ON; }

    private:
        unsigned int led_pin;
        unsigned int ssr_pin;
        unsigned int buzzer_pin;

        // Settings for how to perform reflow
        ReflowSettings* settings;

        // These variables are used by the PID controller
        double setpoint;
        double input;
        double output;

        double kp;
        double ki;
        double kd;

        PID* reflowOvenPID;

        // Variables for keeping track of timing during states
        int windowSize;
        unsigned int timerSeconds;
        unsigned long windowStartTime;
        unsigned long timerSoak;
        unsigned long timerPeriod;
        unsigned long reflowPeriod;
        unsigned long buzzerPeriod;

        // Reflow oven controller state machine state variable
        reflowState_t reflowState;

        // Reflow oven controller status
        reflowStatus_t reflowStatus; 

        // State methods
        void idle_state();
        void preheat_state();
        void soak_state();
        void reflow_state();
        void cool_state();
        void complete_state();
        void too_hot_state();
        void error_state();

        // Does setting of SSR output
        void compute_pid();

        const StateStruct* state_map() {
            StateFunc state_map[10] = { NULL };
        }

//        BEGIN_STATE_MAP
//            STATE_MAP_ENTRY(idle_state)
//            STATE_MAP_ENTRY(preheat_state)
//            STATE_MAP_ENTRY(soak_state)
//            STATE_MAP_ENTRY(reflow_state)
//            STATE_MAP_ENTRY(cool_state)
//            STATE_MAP_ENTRY(complete_state)
//            STATE_MAP_ENTRY(too_hot_state)
//            STATE_MAP_ENTRY(error_state)
//        END_STATE_MAP
       
};

#endif
