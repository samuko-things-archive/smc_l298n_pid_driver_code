#ifndef SERIAL_I2C_COMM_API_H
#define SERIAL_I2C_COMM_API_H
#include<Arduino.h>
#include <Wire.h>
#include "eeprom_setup.h"



float maxFloat = 99999.888, minFloat = -99999.888;
long maxLong =  2147000000, minLong = -2147000000;



void initLed0()
{
  pinMode(A0, OUTPUT);
}
void onLed0()
{
  digitalWrite(A0, HIGH);
}
void offLed0()
{
  digitalWrite(A0, LOW);
}


void initLed1()
{
  pinMode(A1, OUTPUT);
}
void onLed1()
{
  digitalWrite(A1, HIGH);
}
void offLed1()
{
  digitalWrite(A1, LOW);
}


///////// DIFFERENT TASK FOR SERIAL AND I2C COMMUNICATION //////////

/////////////////////////////////////////////////////////////////////////////////////
String sendMotorsPos(){
  float angPosA = encA.getAngPos();
  float angPosB = encB.getAngPos();
  String data = String(constrain(rdirA*angPosA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB*angPosB, minFloat, maxFloat), 4);
  return data;
}
String sendMotorsPos3dp()
{
  float angPosA = encA.getAngPos();
  float angPosB = encB.getAngPos();
  String data = String(constrain(rdirA * angPosA, minFloat, maxFloat), 3);
  data += ",";
  data += String(constrain(rdirB * angPosB, minFloat, maxFloat), 3);
  return data;
}

String sendMotorsVel(){
  String data = String(constrain(rdirA*filteredAngVelA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB*filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
///////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////
String sendMotorAVel(){
  float rawAngVelA = encA.getAngVel();
  String data = String(constrain(rdirA*rawAngVelA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirA*filteredAngVelA, minFloat, maxFloat), 4);
  return data;
}

String sendMotorBVel(){
float rawAngVelB = encB.getAngVel();
  String data = String(constrain(rdirB*rawAngVelB, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB*filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
////////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////////////
String sendMotorAVelPID(){
  String data = String(constrain(rdirA*targetA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirA*filteredAngVelA, minFloat, maxFloat), 4);
  return data;
}

String sendMotorBVelPID(){
  String data = String(constrain(rdirB*targetB, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB*filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////
String sendMotorAData(){
  float angPosA = encA.getAngPos();
  String data = String(constrain(rdirA*angPosA, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirA*filteredAngVelA, minFloat, maxFloat), 4);
  return data;
}

String sendMotorBData(){
  float angPosB = encB.getAngPos();
  String data = String(constrain(rdirB*angPosB, minFloat, maxFloat), 4);
  data += ",";
  data += String(constrain(rdirB*filteredAngVelB, minFloat, maxFloat), 4);
  return data;
}
/////////////////////////////////////////////////////////////////////////////////////////





/////////////////////////////////////////////////////////////////////
String setMotorsPwm(int valA, int valB){
  if(!pidMode){
    motorA.sendPWM((int)rdirA*valA);
    motorB.sendPWM((int)rdirB*valB);
    return "1";
  }
  else return "0";
}


String setMotorsTarget(float valA, float valB){
  float tVelA = constrain(valA, -1.00*maxVelA, maxVelA);
  float tVelB = constrain(valB, -1.00*maxVelB, maxVelB); 

  // if (pidMode){
  //   targetA = rdirA*tVelA;
  //   targetB = rdirB*tVelB;
  //   return "1";
  // }
  // else return "0";

  targetA = rdirA*tVelA;
  targetB = rdirB*tVelB;
  return "1";
}
/////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////
String setPidModeFunc(int mode){
  if(mode == 0){
    pidMode = false;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    pidMotorA.begin();
    pidMotorB.begin();
  } else if (mode == 1) {
    pidMode = true;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    pidMotorA.begin();
    pidMotorB.begin();
  }
}
String setPidMode(int mode){
  if(mode == 0){
    pidMode = false;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    targetA = 0.00;
    targetB = 0.00;
    pidMotorA.begin();
    pidMotorB.begin();
    return "1";
  } else if (mode == 1) {
    pidMode = true;
    motorA.sendPWM(0);
    motorB.sendPWM(0);
    targetA = 0.00;
    targetB = 0.00;
    pidMotorA.begin();
    pidMotorB.begin();
    return "1";
  }
  else {
    return "0";
  }
}
String sendPidMode(){
  return String(pidMode);
}
////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////
String setEncAppr(float ppr){
  if(!pidMode){
    setPPR_A(ppr);
    encA_ppr = getPPR_A();
    return "1";
  }
  else return "0";
}
String sendEncAppr(){ 
  return String(encA_ppr);
}


String setEncBppr(float ppr){
  if(!pidMode){
    setPPR_B(ppr);
    encB_ppr = getPPR_B();
    return "1";
  }
  else return "0";
}
String sendEncBppr(){ 
  return String(encB_ppr);
}
//////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////
String setMotorAkp(float kp){
  if(!pidMode){
    setKP_A(kp);
    kpA = getKP_A();
    return "1";
  }
  else return "0";
}
String sendMotorAkp(){
  return String(kpA,4);
}



String setMotorBkp(float kp){
  if(!pidMode){
    setKP_B(kp);
    kpB = getKP_B();
    return "1";
  }
  else return "0";
}
String sendMotorBkp(){
  return String(kpB,4);
}
//////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
String setMotorAki(float ki){
  if(!pidMode){
    setKI_A(ki);
    kiA = getKI_A();
    return "1";
  }
  else return "0";
}
String sendMotorAki(){
  return String(kiA,4);
}


String setMotorBki(float ki){
  if(!pidMode){
    setKI_B(ki);
    kiB = getKI_B();
    return "1"; 
  }
  else return "0";
}
String sendMotorBki(){
  return String(kiB,4);
}
//////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
String setMotorAkd(float kd){
  if(!pidMode){
    setKD_A(kd);
    kdA = getKD_A();
    return "1"; 
  }
  else return "0";
}
String sendMotorAkd(){
  return String(kdA,4);
}


String setMotorBkd(float kd){
  if(!pidMode){
    setKD_B(kd);
    kdB = getKD_B();
    return "1";
  }
  else return "0";
}
String sendMotorBkd(){
  return String(kdB,4);
}
////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////
String setFilterOrderA(int order){
  if(!pidMode){
    if(order > 2) setFilterOrder_A(2);
    else if (order < 1) setFilterOrder_A(1);
    else setFilterOrder_A(order);
    orderA = getFilterOrder_A();
    return "1";
  }
  else return "0";
}
String sendFilterOrderA(){ 
  return String(orderA);
}


String setFilterOrderB(int order){
  if(!pidMode){
    if(order > 2) setFilterOrder_B(2);
    else if (order < 1) setFilterOrder_B(1);
    else setFilterOrder_B(order);
    orderB = getFilterOrder_B();
    return "1";
  }
  else return "0";
}
String sendFilterOrderB(){ 
  return String(orderB);
}
//////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////
String setCutOffFreqA(float f0){
  if(!pidMode){
    setFilterCutOffFreq_A(f0);
    cutOffFreqA = getFilterCutOffFreq_A();
    return "1";
  }
  else return "0";
}
String sendCutOffFreqA(){ 
  return String(cutOffFreqA);
}


String setCutOffFreqB(float f0){
  if(!pidMode){
    setFilterCutOffFreq_B(f0);
    cutOffFreqB = getFilterCutOffFreq_B();
    return "1";
  }
  else return "0";
}
String sendCutOffFreqB(){ 
  return String(cutOffFreqB);
}
//////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////
String setStopFreqA(float freq){
  if(!pidMode){
    setStopFreq_A(freq);
    encA_stopFreq = getStopFreq_A();
    return "1";
  }
  else return "0";
}
String sendStopFreqA(){ 
  return String(encA_stopFreq);
}


String setStopFreqB(float freq){
  if(!pidMode){
    setStopFreq_B(freq);
    encB_stopFreq = getStopFreq_B();
    return "1";
  }
  else return "0";
}
String sendStopFreqB(){ 
  return String(encB_stopFreq);
}
///////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////
String setRdirA(float rDirA){
  if(!pidMode){
    if(rDirA >= 0.0){
      setRDIR_A(1.00);
      rdirA = getRDIR_A();
    }
    else {
      setRDIR_A(-1.00);
      rdirA = getRDIR_A();
    }
    return "1";
  }
  else return "0";
}
String sendRdirA(){ 
  return String(rdirA);
}


String setRdirB(float rDirB){
  if(!pidMode){
    if(rDirB >= 0.0){
      setRDIR_B(1.00);
      rdirB = getRDIR_B();
    }
    else {
      setRDIR_B(-1.00);
      rdirB = getRDIR_B();
    }
    return "1";
  }
  else return "0";
}
String sendRdirB(){ 
  return String(rdirB);
}
//////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
String setI2Caddress(int address){
  if(!pidMode){
    setI2CADDRESS(address);
    i2cAddress = getI2CADDRESS();
    Wire.begin(i2cAddress);                
    return "1";
  }
  else return "0";
}
String sendI2Caddress(){
  return String(i2cAddress);
}


String resetEEPROM(){
  if(!pidMode){
    setFIRST_TIME(0);
    return "1";
  }
  else return "0";
}
////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////
String setMaxVelA(float vel){
  if(!pidMode){
    float max_vel = abs(vel);
    setMAXVEL_A(constrain(max_vel, -1.00*wA_allowable, wA_allowable));
    maxVelA = getMAXVEL_A();
    return "1";
  }
  else return "0";
}
String sendMaxVelA(){
  return String(maxVelA,0);
}

String setMaxVelB(float vel){
  if(!pidMode){
    float max_vel = abs(vel);
    setMAXVEL_B(constrain(max_vel, -1.00*wB_allowable, wB_allowable));
    maxVelB = getMAXVEL_B();
    return "1";
  }
  else return "0";
}
String sendMaxVelB(){
  return String(maxVelB,0);
}


String setAllowedFreq(float freq){
  if(!pidMode){
    setAllowableFreq(freq);
    freq_per_tick_allowable = getAllowableFreq();
    return "1";
  }
  else return "0";
}
String sendAllowedFreq(){
  return String(freq_per_tick_allowable,2);
}
////////////////////////////////////////////







///////////////// SERIAL COMMUNICATION //////////////////////
String ser_msg = "";

String serMsg = "", serMsgBuffer, serDataBuffer[3];

void serialReceiveAndSendData() {
  int indexPos = 0, i = 0;

  if (Serial.available() > 0) {
    while (Serial.available())
    {
      serMsg = Serial.readString();
    }
    serMsg.trim();
    if (serMsg != "") {
      do {
        indexPos = serMsg.indexOf(',');
        if (indexPos != -1) {
          serMsgBuffer = serMsg.substring(0, indexPos);
          serMsg = serMsg.substring(indexPos + 1, serMsg.length());
          serDataBuffer[i] = serMsgBuffer;
          serMsgBuffer = "";
        }
        else {
          if (serMsg.length() > 0)
            serDataBuffer[i] = serMsg;
        }
        i += 1;
      } while (indexPos >= 0);
    }


    if (serDataBuffer[0] != ""){

      onLed1();

      if(serDataBuffer[0] == "/pos"){
        ser_msg = sendMotorsPos();
        Serial.println(ser_msg);
        ser_msg = "";
      }
      
      else if (serDataBuffer[0] == "/vel") {
        ser_msg = sendMotorsVel();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/velA") {
        ser_msg = sendMotorAVel();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/velB") {
        ser_msg = sendMotorBVel();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pVelA") {
        ser_msg = sendMotorAVelPID();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pVelB") {
        ser_msg = sendMotorBVelPID();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/dataA") {
        ser_msg = sendMotorAData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/dataB") {
        ser_msg = sendMotorBData();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pwm") {
        int pwm_a = serDataBuffer[1].toInt();
        int pwm_b = serDataBuffer[2].toInt();
        ser_msg = setMotorsPwm(constrain(pwm_a, -255, 255), constrain(pwm_b, -255, 255));
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/tag") {
        ser_msg = setMotorsTarget(serDataBuffer[1].toFloat(), serDataBuffer[2].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/mode") {
        if (serDataBuffer[1]=="") ser_msg = sendPidMode();
        else ser_msg = setPidMode(serDataBuffer[1].toInt());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/pprA") {
        if (serDataBuffer[1]=="") ser_msg = sendEncAppr();
        else ser_msg = setEncAppr(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }
      
      else if (serDataBuffer[0] == "/pprB") {
        if (serDataBuffer[1]=="") ser_msg = sendEncBppr();
        else ser_msg = setEncBppr(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kpA") {
        if (serDataBuffer[1]=="") ser_msg = sendMotorAkp();
        else ser_msg = setMotorAkp(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kpB") {
        if (serDataBuffer[1]=="") ser_msg = sendMotorBkp();
        else ser_msg = setMotorBkp(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kiA") {
        if (serDataBuffer[1]=="") ser_msg = sendMotorAki();
        else ser_msg = setMotorAki(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kiB") {
        if (serDataBuffer[1]=="") ser_msg = sendMotorBki();
        else ser_msg = setMotorBki(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/kdA") {
        if (serDataBuffer[1]=="") ser_msg = sendMotorAkd();
        else ser_msg = setMotorAkd(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }
      
      else if (serDataBuffer[0] == "/kdB") {
        if (serDataBuffer[1]=="") ser_msg = sendMotorBkd();
        else ser_msg = setMotorBkd(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/ordA") {
        if (serDataBuffer[1]=="") ser_msg = sendFilterOrderA();
        else ser_msg = setFilterOrderA(serDataBuffer[1].toInt());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/ordB") {
        if (serDataBuffer[1]=="") ser_msg = sendFilterOrderB();
        else ser_msg = setFilterOrderB(serDataBuffer[1].toInt());
        Serial.println(ser_msg);
        ser_msg = "";
      }


      else if (serDataBuffer[0] == "/f0A") {
        if (serDataBuffer[1]=="") ser_msg = sendCutOffFreqA();
        else ser_msg = setCutOffFreqA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/f0B") {
        if (serDataBuffer[1]=="") ser_msg = sendCutOffFreqB();
        else ser_msg = setCutOffFreqB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/sfA") {
        if (serDataBuffer[1]=="") ser_msg = sendStopFreqA();
        else ser_msg = setStopFreqA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/sfB") {
        if (serDataBuffer[1]=="") ser_msg = sendStopFreqB();
        else ser_msg = setStopFreqB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/rdirA") {
        if (serDataBuffer[1]=="") ser_msg = sendRdirA();
        else ser_msg = setRdirA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }
      
      else if (serDataBuffer[0] == "/rdirB") {
        if (serDataBuffer[1]=="") ser_msg = sendRdirB();
        else ser_msg = setRdirB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/i2c") {
        if (serDataBuffer[1]=="") ser_msg = sendI2Caddress();
        else {
          int i2c_address = serDataBuffer[1].toInt();
          ser_msg = setI2Caddress(constrain(i2c_address, 0, 127));
        }
        Serial.println(ser_msg);
        ser_msg = "";
      }
      
      else if (serDataBuffer[0] == "/reset") {
        ser_msg = resetEEPROM();
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/freq") {
        if (serDataBuffer[1]=="") ser_msg = sendAllowedFreq();
        else ser_msg = setAllowedFreq(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/maxVelA") {
        if (serDataBuffer[1]=="") ser_msg = sendMaxVelA();
        else ser_msg = setMaxVelA(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      else if (serDataBuffer[0] == "/maxVelB") {
        if (serDataBuffer[1]=="") ser_msg = sendMaxVelB();
        else ser_msg = setMaxVelB(serDataBuffer[1].toFloat());
        Serial.println(ser_msg);
        ser_msg = "";
      }

      offLed1();

    } else {
      ser_msg = "0";
      Serial.println(ser_msg);
      ser_msg = "";
    }
  }
  
  serMsg = "";
  serMsgBuffer = "";
  serDataBuffer[0] = "";
  serDataBuffer[1] = "";
  serDataBuffer[2] = "";
}
//////////////////////////////////////////////////////////////////////////








///////////////// I2C COMMUNICATION //////////////////////

String i2c_msg = "";

String i2cMsg = "", i2cMsgBuffer, i2cDataBuffer[3];

//-----------------------------------------------------//
void saveData(String data_address)
{
  if (data_address == "/pwm")
  {
    int pwm_a = i2cDataBuffer[1].toInt();
    int pwm_b = i2cDataBuffer[2].toInt();
    setMotorsPwm(constrain(pwm_a, -255, 255), constrain(pwm_b, -255, 255));
  }

  else if (data_address == "/tag")
  {
    setMotorsTarget(i2cDataBuffer[1].toFloat(), i2cDataBuffer[2].toFloat());
  }

  // else if (data_address == "/mode") {
  //   setPidMode(i2cDataBuffer[1].toInt());
  // }
}

void getData(String data_address)
{
  if (data_address == "/pos")
  {
    i2c_msg = sendMotorsPos3dp();
  }

  else if (data_address == "/vel")
  {
    i2c_msg = sendMotorsVel();
  }

  else if (data_address == "/dataA")
  {
    i2c_msg = sendMotorAData();
  }

  else if (data_address == "/dataB")
  {
    i2c_msg = sendMotorBData();
  }

  else if (data_address == "/maxVelA")
  {
    i2c_msg = sendMaxVelA();
  }

  else if (data_address == "/maxVelB")
  {
    i2c_msg = sendMaxVelB();
  }

  // else if (data_address == "/mode") {
  //   i2c_msg = sendPidMode();
  // }
}

void i2cReceiveDataEvent(int dataSizeInBytes)
{
  onLed0();

  int indexPos = 0, i = 0;

  for (int j = 0; j < dataSizeInBytes; j += 1)
  {
    char c = Wire.read();
    i2cMsg += c;
  }

  i2cMsg.trim();

  if (i2cMsg != "")
  {
    do
    {
      indexPos = i2cMsg.indexOf(',');
      if (indexPos != -1)
      {
        i2cMsgBuffer = i2cMsg.substring(0, indexPos);
        i2cMsg = i2cMsg.substring(indexPos + 1, i2cMsg.length());
        i2cDataBuffer[i] = i2cMsgBuffer;
        i2cMsgBuffer = "";
      }
      else
      {
        if (i2cMsg.length() > 0)
          i2cDataBuffer[i] = i2cMsg;
      }
      i += 1;
    } while (indexPos >= 0);
  }

  if (i2cDataBuffer[1] != "")
  {
    saveData(i2cDataBuffer[0]);

    i2cMsg = "";
    i2cMsgBuffer = "";
    i2cDataBuffer[0] = "";
    i2cDataBuffer[1] = "";
    i2cDataBuffer[2] = "";
  }

  offLed0();
}

void i2cSendDataEvent()
{
  onLed0();
  getData(i2cDataBuffer[0]);

  // Send response back to Master
  char charArray[i2c_msg.length() + 1];
  i2c_msg.toCharArray(charArray, i2c_msg.length() + 1);

  Wire.write(charArray);

  i2c_msg = "";
  i2cMsg = "";
  i2cMsgBuffer = "";
  i2cDataBuffer[0] = "";
  i2cDataBuffer[1] = "";
  i2cDataBuffer[2] = "";
  offLed0();
}
//-----------------------------------------------------//

//////////////////////////////////////////////////////////

#endif
