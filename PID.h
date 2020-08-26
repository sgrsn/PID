#ifndef PID_H
#define PID_H
 
class PID
{
    public:
        PID();
        float control_P(float target, float current, float new_Kp);
        float control_PI(float target, float current);
        float control_PID(float target, float current);
        float control_I_PD(float target, float current);
        void setParameter(float new_Kp, float new_Ki, float new_Kd);
        void setParameter(float new_Ku, float new_Pu);
        void reset(float target=0);
 
        float Ku;
        float Pu;
        float Kp;
        float Ti;
        float Td;
        float Ki;
        float Kd;
        
        private:
        float integral;
        float prev_error;
        unsigned long current_time;
        unsigned long prev_time;
        float lateD;
};
 
#endif
