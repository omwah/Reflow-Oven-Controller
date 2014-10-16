#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// Adapted from:
// http://www.drdobbs.com/cpp/state-machine-design-in-c/184401236

struct StateStruct;

// base class for state machines
class StateMachine {
protected:
    virtual const StateStruct* state_map() = 0;
};

typedef void (StateMachine::*StateFunc)();
struct StateStruct 
{
    StateFunc state_method;    
};

#define BEGIN_STATE_MAP \
public:\
const StateStruct* state_map() {\
    static const StateStruct StateMap[] = { 
 
#define STATE_MAP_ENTRY(entry)\
    { reinterpret_cast<StateFunc>(entry) },
 
#define END_STATE_MAP \
    { reinterpret_cast<StateFunc>(NULL) }\
    }; \
    return &StateMap[0]; }

#endif
