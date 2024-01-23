#ifndef GLOBAL_EEPROM_VARIABLES
#define GLOBAL_EEPROM_VARIABLES
#include "encoder_setup.h"
#include "low_pass_filter_setup.h"
#include "motor_bridge_control.h"
#include "simple_pid_control.h"


///////////////////////////////////////////////////
// store encoder pulsePerRev needed by encoder
int encA_ppr = 0;
int encB_ppr = 0;
unsigned long encA_stopFreq = 5000; // in us
unsigned long encB_stopFreq = 5000; // in us


int encA_clkPin = 2, encA_dirPin = 4; // encA_ppr parameter is decleared globally in the global_params_eeprom.h file.
int encB_clkPin = 3, encB_dirPin = 9; // encB_ppr parameter is decleared globally in the global_params_eeprom.h file.

QuadEncoder encA(encA_clkPin, encA_dirPin, encA_ppr);
QuadEncoder encB(encB_clkPin, encB_dirPin, encB_ppr);




/////////////////////////////
float rdirA = 1.00;
float rdirB = 1.00;
/////////////////////////////




// adaptive lowpass Filter
int orderA = 1;
float cutOffFreqA = 5.0;

int orderB = 1;
float cutOffFreqB = 5.0;

// Filter instance
AdaptiveLowPassFilter lpfA(orderA, cutOffFreqA);
AdaptiveLowPassFilter lpfB(orderB, cutOffFreqB);





// motor A H-Bridge Connection
int IN1 = 7, IN2 = 8, enA = 5;
MotorControl motorA(IN1, IN2, enA);

// motor B H-Bridge Connection
int IN3 = 11, IN4 = 12, enB = 6;
MotorControl motorB(IN3, IN4, enB);







///////////////////////////////////////////////
double outMin = -255.0, outMax = 255.0;

// motorA pid control global params needed by pid
double kpA = 0.0, kiA = 0.0, kdA = 0.0;
double targetA = 0.00, filteredAngVelA;
double outputA;

// motorB pid control global params needed by pid
double kpB = 0.0, kiB = 0.0, kdB = 0.0;
double targetB = 0.00, filteredAngVelB;
double outputB;


// motorA pid control
SimplePID pidMotorA(kpA, kiA, kdA, outMin, outMax);

// motorA pid control
SimplePID pidMotorB(kpB, kiB, kdB, outMin, outMax);





// check if in PID or PWM mode
bool pidMode = false; // true-PID MODE, false-SETUP MODE

// initial i2cAddress
int i2cAddress = 0;


#endif


