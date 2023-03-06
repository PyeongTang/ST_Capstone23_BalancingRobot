#include "mbed.h"

class PID {
    public :
        PID(float Kp, float Ki, float Kd, float tolerance);
        
        void Set_PID                    (float __Kp, float __Ki, float __Kd);
        void Set_Tolerance              (float __tolerance);
        void Set_Output_Limit           (float min, float max);
        void Set_Reference              (float target);

        float Get_Kp                    ();
        float Get_Ki                    ();
        float Get_Kd                    ();
        float Get_Error                 ();

        float Calculate_Integral        ();
        float Calculate_Derivative      ();
        float Calculate_Output          (float input);

        void Reset_Integral             ();
        bool Is_Finished                ();


    private :
        float   _Kp;
        float   _Ki;
        float   _Kd;
        float   _tolerance;

        float   _reference;
        float   _error;
        float   _previous_error;

        float   _integral;

        float   _dt;
        float   _min_output;
        float   _max_output;

        float   _output;
};