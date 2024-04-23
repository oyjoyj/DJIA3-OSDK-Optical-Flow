typedef struct{
    float kp;
    float ki;
    float kd;
    float error[3];
    float dbuf[3];
    float out;
    float pout;
    float iout;
    float dout;
} pid_p;

extern void pid_init(pid_p *pid, float p, float i, float d);

extern float pid_calc(pid_p *pid, float set, float get);

extern float filter(float ori,float comp);
