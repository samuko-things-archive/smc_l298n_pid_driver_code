#include <Wire.h>
#include "global_eeprom_variables.h"
#include "serial_i2c_comm_api.h"

///////// my sepcial delay function ///////////////
void delayMs(int ms) {
  for (int i = 0; i < ms; i += 1) {
    delayMicroseconds(1000);
  }
}
//////////////////////////////////////////////////



// low pass filter (lpf) variables/parameters are decleared 
// globally in the global_eeprom_variables.h file.

void lpfInit(){
  lpfA.setOrder(orderA);
  lpfA.setCutOffFreq(cutOffFreqA);

  lpfB.setOrder(orderB);
  lpfB.setCutOffFreq(cutOffFreqB);
}




// encoder variables/parameters are decleared
// globally in the global_eeprom_variables.h file.

void encoderInit() {
  encA.setPulsePerRev(encA_ppr);
  encB.setPulsePerRev(encB_ppr);

  encA.setStopFreqInUs(encA_stopFreq);
  encB.setStopFreqInUs(encB_stopFreq);
  
  attachInterrupt(digitalPinToInterrupt(encA.clkPin), readEncoderA, FALLING);
  attachInterrupt(digitalPinToInterrupt(encB.clkPin), readEncoderB, FALLING);
}

void readEncoderA() {
  encA.freqPerTick = 1000000.00 / (float)(micros() - encA.oldFreqTime);
  encA.oldFreqTime = micros();
  encA.checkFreqTime = micros();

  if (digitalRead(encA.dirPin) == 1) {
    encA.tickCount += 1;
    encA.frequency = encA.freqPerTick / (float)encA.pulsePerRev;
  } else {
    encA.tickCount -= 1;
    encA.frequency = -1.00 * encA.freqPerTick / (float)encA.pulsePerRev;
  }

}

void readEncoderB() {
  encB.freqPerTick = 1000000.00 / (float)(micros() - encB.oldFreqTime);
  encB.oldFreqTime = micros();
  encB.checkFreqTime = micros();

  if (digitalRead(encB.dirPin) == 1) {
    encB.tickCount += 1;
    encB.frequency = encB.freqPerTick / (float)encB.pulsePerRev;
  } else {
    encB.tickCount -= 1;
    encB.frequency = -1.00 * encB.freqPerTick / (float)encB.pulsePerRev;
  }

}
////////////////////////////////////////////////////////////////




// motor_bridge_control varaiables/parameters are decleared
// globally in the global_eeprom_variables.h file.




// PID variables/parameters are decleared
// globally in the global_eeprom_variables.h file.

void pidInit() {
  pidMotorA.setParameters(kpA, kiA, kdA, outMin, outMax);
  pidMotorB.setParameters(kpB, kiB, kdB, outMin, outMax);
  pidMotorA.begin();
  pidMotorB.begin();
}
/////////////////////////////////////////////








////////////////////// MAIN CODE ////////////////////////////////////////

unsigned long serialCommTime, serialCommSampleTime = 10; //ms -> (1000/sampleTime) hz
unsigned long pidTime, pidSampleTime = 5; //ms -> (1000/sampleTime) hz
unsigned long pidStopTime, pidStopSampleTime = 1000; //ms -> (1000/sampleTime) hz

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);

  initLed0();
  initLed1();

  offLed0();
  offLed1();
  
  // update global params with eeprom contents
  updateGlobalParamsFromEERPOM();
  /////////////////////////////////////////////
  delay(500);
  
  Wire.begin(getI2CADDRESS());                
  Wire.onReceive(i2cSlaveReceiveData);
  Wire.onRequest(i2cSlaveSendData);

  onLed0();
  delay(1000);
  offLed0();
  delay(500);
  onLed1();
  delay(1000);
  offLed1();
  
  encoderInit();
  pidInit();
  lpfInit();
  /* motor needs no initialization as it used no global variable dependent on eeprom*/

  
  serialCommTime = millis();
  pidTime = millis();
  pidStopTime = millis();
}


void loop() {
  ///// do not touch ////////
  ///// useful for velocity reading to check when rotation has stopped
  encA.resetFrequency();
  encB.resetFrequency();
  ///////////////////////////////////////////////////////////////////



  /////////////////////////////////////////////////////////////
  if(abs(targetA)<0.01 && abs(targetB)<0.01){
    if(pidMode==true){
      if ((millis() - pidStopTime) >= pidStopSampleTime) {
        setPidModeFunc(0);
        pidStopTime = millis();
      }
    }
    else {
      pidStopTime = millis();
    }
  }
  else{
    if(pidMode==false){
      setPidModeFunc(1);
    }
    pidStopTime = millis();
  }
  ///////////////////////////////////////////////////////////////



  ////////////// PID OPERATION /////////////////////////////////
  filteredAngVelA = lpfA.filt(encA.getAngVel());
  filteredAngVelB = lpfB.filt(encB.getAngVel());

  if (pidMode){
    outputA = pidMotorA.compute(targetA, filteredAngVelA); // targetA is among the global params
    outputB = pidMotorB.compute(targetB, filteredAngVelB); // targetB is among the global params

    motorA.sendPWM((int)outputA);
    motorB.sendPWM((int)outputB); 
  }
  // if ((millis() - pidTime) >= pidSampleTime) {
  //   pidTime = millis();
  // }
  //////////////////////////////////////////////////////////////////////////



  ////////// using the serial communiaction API ////////////////////////
  if ((millis() - serialCommTime) >= serialCommSampleTime) {
    serialReceiveAndSendData();
    serialCommTime = millis();
  }
  /////////////////////////////////////////////////////////////////////

}
