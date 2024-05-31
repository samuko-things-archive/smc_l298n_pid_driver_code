#ifndef GLOBAL_EEPROM_VARIABLES
#define GLOBAL_EEPROM_VARIABLES
#include "encoder_setup.h"
#include "adaptive_low_pass_filter.h"
#include "l298n_motor_control.h"
#include "simple_pid_control.h"


///////////////////////////////////////////////////
// store encoder pulsePerRev needed by encoder
float encA_ppr = 1000.0;
float encB_ppr = 1000.0;
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
L298NMotorControl motorA(IN1, IN2, enA);

// motor B H-Bridge Connection
int IN3 = 11, IN4 = 12, enB = 6;
L298NMotorControl motorB(IN3, IN4, enB);







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
int i2cAddress = 1;


// calcute allowable maximum angular velocity (for overall smooth operation of the whole system)
float freq_per_tick_allowable = 2000.0; //Hz
float wA_allowable = 10.00;
float wB_allowable = 10.00;

float calc_wA_allowable(){
  wA_allowable = (2*PI*freq_per_tick_allowable)/encA_ppr;
  return wA_allowable;
}

float calc_wB_allowable(){
  wB_allowable = (2*PI*freq_per_tick_allowable)/encB_ppr;
  return wB_allowable;
}


// maximum motor velocity that can be commanded
float maxVelA = calc_wA_allowable(); // in radians/sec
float maxVelB = calc_wB_allowable(); // in radians/sec



#endif


