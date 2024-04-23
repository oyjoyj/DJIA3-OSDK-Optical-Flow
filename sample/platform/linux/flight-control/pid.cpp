#include "pid.hpp"
#include <math.h>

void pid_init(pid_p *pid, float p, float i, float d)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;
    pid->dbuf[0] = 0;
    pid->dbuf[1] = 0;
    pid->dbuf[2] = 0;
    pid->out = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
}

float pid_calc(pid_p *pid, float set, float get)
{
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - get;
    pid->pout = pid->kp * pid->error[0];
    pid->iout += pid->ki *pid->error[0];
    pid->dbuf[2] = pid->dbuf[1];
    pid->dbuf[1] = pid->dbuf[0];
    pid->dbuf[0] = pid->error[0] - pid->error[1];
    pid->dout = pid->kd *pid->dbuf[0];
    pid->out = pid->pout + pid->iout + pid->dout;
    return pid->out;
}
#include <math.h>
float filter(float ori,float comp)
{
    if(abs(ori - comp) <= 2)
    {
        return ori;
    }
    else
    {
        return comp;
    }
}