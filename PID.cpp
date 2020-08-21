#include "PID.h"
#include <Arduino.h>
 
PID::PID()
{
    integral    = 0;
    prev_error  = 0;
    current_time     = 0;
    prev_time   = 0;
    lateD      = 0;
}

float PID::control_P(float target, float current, float new_Kp)
{
    float error = target - current;
    float u = new_Kp*error;
    return u;
}
float PID::control_PI(float target, float current)
{
    Kp = 0.45 * Ku;
    Ti = 0.83 * Pu;
    Ki = (1 / Ti) * Kp;
    current_time = micros();
    float error = target - current;
    float dt = current_time - prev_time;
    integral += (error + prev_error) / 2 * dt;
    float u = Kp*error + Ki*integral;
    prev_error = error;
    prev_time = micros();
    return u;
}
float PID::control_PID(float target, float current)
{
    current_time = micros();
    float error = target - current;
    float dt = float(current_time - prev_time) / 1000000;
    integral += (error + prev_error) / 2 * dt;
    float differential = (error - prev_error) / dt;
    float u = Kp*error + Ki*integral + Kd*differential;
    prev_error = error;
    prev_time = current_time;
    return u;
}
void PID::setParameter(float new_Kp, float new_Ki, float new_Kd)
{  
  Kp = new_Kp;
  Ki = new_Ki;
  Kd = new_Kd;
}
void PID::setParameter(float new_Ku, float new_Pu)
{  
    Ku = new_Ku;
    Pu = new_Pu;
    
    Kp = 0.60 * Ku;
    Ti = 0.50 * Pu;
    Td = 0.125 * Pu;
    Ki = (1 / Ti) * Kp;
    Kd = Td * Kp;
}
void PID::reset(float target)
{
    integral    = 0;
    prev_error  = target;
    prev_time = micros();
}
