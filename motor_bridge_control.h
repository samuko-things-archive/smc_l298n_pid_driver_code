#ifndef MOTOR_BRIDGE_CONTROL_H
#define MOTOR_BRIDGE_CONTROL_H
#include <Arduino.h>

class MotorControl {
  public:
    MotorControl(int IN1_pin, int IN2_pin, int en_pin);

    void sendPWM(int pwmVal);
    int getDirection();
    void test();

  private:
    int in1Pin, in2Pin, enPin;
    int dir = 1;

    void setForwardDirection();
    void setReverseDirection();
    void setHalt();
    void delayMs(int ms);

};



#endif