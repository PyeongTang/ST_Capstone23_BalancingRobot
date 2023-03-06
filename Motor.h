#include "mbed.h"

#define SEC             1000000
#define DEF_PERIOD      50
#define PWM_MIN         32
#define PWM_MAX         50
#define COUNT_PER_REV   110

class Motor {
    public:
        Motor(PinName encA, PinName encB, PinName In1, PinName In2, PinName EnA);
        void motorInit();
        void encoderCountControl(int desiredCount = 0);
        void motorDrive(bool Forward = true, int speed = 0);
        void motorStall();
        int getEncoderCount();

    private:
        InterruptIn encA, encB;
        DigitalOut In1, In2;
        PwmOut EnA;
        int encoderCount = 0;
        void encA_HIGH_ISR();
        void encA_LOW_ISR();
        float countToDegree(int count);
        template <class T> T map(T x, T in_min, T in_max, T out_min, T out_max);
};