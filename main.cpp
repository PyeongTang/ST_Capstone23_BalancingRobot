#include "mbed.h"
#include "Motor.h"
#include "PID.h"
#include "PointDetector.h"

Serial  pc(USBTX, USBRX);

////////////  Count Set  ///////////
float refCount          =   330.0;
///////////////////////////////////
/*
    110 Count = 1 Revolution = 360 Degree = 2 Pi Radian

    Distance (Length of Arc) = r * theta = (Radius of Mecanum Wheel) * (360 degree / 110 count) * (Number of Count)
*/

int   positionX         =   0;
int   positionY         =   0;

float maxCount          =   110.0;
float minCount          =   0.0;

////////////  PID Tune  ///////////
float Kp                =   0.5;
float Ki                =   0.3;
float Kd                =   0.0;
///////////////////////////////////

float tol               =   3.0;

DigitalOut  shutdown(BUTTON1);

Motor   motor1              (D6, D7, D8, D9, D5);
// Motor   motor2  (Dx, Dx, Dx, Dx, Dx);
// Motor   motor3  (Dx, Dx, Dx, Dx, Dx);
// Motor   motor4  (Dx, Dx, Dx, Dx, Dx);

PID     plant1              (Kp, Ki, Kd, tol);
// PID     plant2  (Kp, Ki, Kd, tol);
// PID     plant3  (Kp, Ki, Kd, tol);
// PID     plant4  (Kp, Ki, Kd, tol);

PointDetector topPlate     (D1, D2, D3, D4);

void printValue             (Serial* pc_, Motor* motorObject, PID* plantObject, float refCount_, bool encoderCount_, bool error_, float second_);
void setUpMotorPlant        (Motor* motorObject, PID* plantObject, float ref_, float min_, float max_, float p_, float i_, float d_);
void printEndPoint          (Serial* pc_);
void feedBackMotor1         ();

int main() {
    Ticker tickFeedBackMotor1;
    tickFeedBackMotor1.attach_us(&feedBackMotor1, 1000);

    setUpMotorPlant(&motor1, &plant1, refCount, minCount, maxCount, Kp, Ki, Kd);

    // positionX = topPlate.getPositionX();
    // positionY = topPlate.getPositionY();

    while (plant1.Is_Finished() == 0) {
        printValue(&pc, &motor1, &plant1, refCount, true, true, 0.1);
        if (shutdown.read() == 0) break;
    }
    
    printEndPoint(&pc);
    motor1.motorStall();
    wait_us(1*SEC);

    return 0;
}

void setUpMotorPlant(Motor* motorObject, PID* plantObject, float ref_, float min_, float max_, float p_, float i_, float d_) {
    motorObject->motorInit        ();
    plantObject->Set_Reference    (ref_);
    plantObject->Set_Output_Limit (min_, max_);
    plantObject->Reset_Integral   ();
    plantObject->Set_PID          (p_, i_, d_);
}

void printValue(Serial* pc_, Motor* motorObject, PID* plantObject, float refCount_, bool encoderCount_, bool error_, float second_) {
                                 pc_->printf("Reference Count      : $%.2f; \t", refCount_                          );
    if      (encoderCount_)  {   pc_->printf("Motor Encoder Count  : $%d;   \t", motorObject->getEncoderCount()     );  }
    if      (error_)         {   pc_->printf("PID Controller Error : $%.2f; \t", plantObject->Get_Error()           );  }
                                 pc_->printf("\n");
    wait_us(second_*SEC);
}

void printEndPoint(Serial* pc_){
    pc_->printf("------------------------------------------------------------------------------\n");
    pc_->printf("--------------------- Motor has Reached To Reference Count -------------------\n");
    pc_->printf("------------------------------------------------------------------------------\n");
}

void moveTarget (Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4 , int x_pos, int y_pos) {
    
    // Forward          // Backward
    // F        F       // B        B
    // F        F       // B        B
    // Left             // Right
    // B        F       // F        B
    // F        B       // B        F
    // Forward Left     // Forward Right
    // N        F       // F        N
    // F        N       // N        F
    // Backward Left    // Backward Right
    // B        N       // N        B
    // N        B       // B        N
    
    // Top Right Plane
    if      (x_pos > 0 && y_pos > 0){
        motor1->encoderCountControl(     x_pos   +   y_pos   );      motor2->encoderCountControl(     -x_pos  +   y_pos   );
        
        motor3->encoderCountControl(     -x_pos  +   y_pos   );      motor4->encoderCountControl(     x_pos   +   y_pos   );
    }
    // Top Left Plane
    else if (x_pos < 0 && y_pos > 0){
        motor1->encoderCountControl(     -x_pos  +   y_pos   );      motor2->encoderCountControl(     x_pos   +   y_pos   );
        
        motor3->encoderCountControl(     x_pos   +   y_pos   );      motor4->encoderCountControl(     -x_pos  +   y_pos   );
    }
    // Bot Left Plane
    else if (x_pos < 0 && y_pos < 0){
        motor1->encoderCountControl(     -x_pos  -   y_pos   );      motor2->encoderCountControl(     x_pos   -   y_pos   );
        
        motor3->encoderCountControl(     x_pos   -   y_pos   );      motor4->encoderCountControl(     -x_pos  -   y_pos   );
    }
    // Bot Right Plane
    else if (x_pos > 0 && y_pos < 0){
        motor1->encoderCountControl(     x_pos   -   y_pos   );      motor2->encoderCountControl(     -x_pos  -   y_pos   );
        
        motor3->encoderCountControl(     -x_pos  -   y_pos   );      motor4->encoderCountControl(     x_pos   -   y_pos   );
    }
    else {}
}

void feedBackMotor1 (){    motor1.encoderCountControl(plant1.Calculate_Output(motor1.getEncoderCount()));  }
// void feedBackMotor2 (){    motor2.encoderCountControl(plant2.Calculate_Output(motor2.getEncoderCount()));  }
// void feedBackMotor3 (){    motor3.encoderCountControl(plant3.Calculate_Output(motor3.getEncoderCount()));  }
// void feedBackMotor_4 (){    motor4.encoderCountControl(plant4.Calculate_Output(motor4.getEncoderCount()));  }
