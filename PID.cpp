#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float tolerance) :
_Kp(Kp), _Ki(Ki), _Kd(Kd), _tolerance(tolerance) {}

void PID::Set_PID            (float __Kp, float __Ki, float __Kd){
    _Kp = __Kp;
    _Ki = __Ki;
    _Kd = __Kd;
}

void PID::Set_Tolerance      (float __tolerance){
    _tolerance = __tolerance;
}

void PID::Set_Output_Limit   (float min, float max){
    _min_output = min;
    _max_output = max;
}

void PID::Set_Reference      (float target){
    if(target == 0)     _reference = .001f;
    else                _reference = target;
}

float PID::Get_Kp()      { return _Kp;    }
float PID::Get_Ki()      { return _Ki;    }
float PID::Get_Kd()      { return _Kd;    }
float PID::Get_Error()   { return _error; }

float PID::Calculate_Integral(){
    _integral += (_error * _dt) / _reference;
    return _Ki * _integral;
}

float PID::Calculate_Derivative(){
    float derivative;
    derivative = ((_error - _previous_error) / _dt) / _reference;
    return _Kd * derivative;
}

float PID::Calculate_Output(float input){
    _dt = 0.001;
    _error = _reference - input;

    float derivate_term     = this->Calculate_Derivative();
    float integral_term     = this->Calculate_Integral();
    float propotional_term  = _Kp;

    _output                 = _error / _reference;
    _output                 = _output*propotional_term + derivate_term + integral_term;
    
    if(_output > _max_output)         _output = _max_output;
    if(_output < _min_output)         _output = _min_output;
        
    _previous_error = _error;
    
    return _output;
}

void PID::Reset_Integral(){
    _integral = 0;
}

bool PID::Is_Finished(){
    if (_output < _reference + _tolerance && _output > _reference - _tolerance) {
        return true;
    }
    else {
        return false;
    }
}