#include "Motor.h"

Motor::Motor(PinName encA, PinName encB, PinName In1, PinName In2, PinName EnA) :
encA(encA), encB(encB), In1(In1), In2(In2), EnA(EnA) {}

// Public Functions

int Motor::getEncoderCount() {
    return encoderCount;
}

void Motor::encoderCountControl(int desiredCount) {
    int mappedSpeed = map<float>(countToDegree(desiredCount), 0., 360.0, PWM_MIN, PWM_MAX);
    if (desiredCount > 0)   motorDrive(true,    mappedSpeed);
    else                    motorDrive(false,   mappedSpeed);
}

// Private Functions

void Motor::motorInit() {
    encA.rise(callback(this, &Motor::encA_HIGH_ISR));
    encA.fall(callback(this, &Motor::encA_LOW_ISR));
    EnA.period_us(DEF_PERIOD);
}

void Motor::motorDrive(bool Forward, int speed) {
    if (Forward)    {  In2.write(1); In1.write(0);  }
    else            {  In2.write(0); In1.write(1);  }
    EnA.pulsewidth_us(speed);
}

void Motor::motorStall(){
    In1.write(0); In2.write(0); EnA.pulsewidth_us(0);
}

float Motor::countToDegree(int count) {
    return (360.0 / float(COUNT_PER_REV)) * float(count);
}

template <class T>
T Motor::map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

void Motor::encA_HIGH_ISR() {
    int tmp2 = encB.read();
    if(tmp2)    {encoderCount--;}
    else        {encoderCount++;}
}

void Motor::encA_LOW_ISR() {
    int tmp2 = encB.read();
    if(tmp2)    {encoderCount++;}
    else        {encoderCount--;}
}



