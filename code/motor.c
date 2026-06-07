
#include "motor.h"
#include "control.h"
#include "findline.h"
#include "APP_Filter.h"

float Right_out = 0;   //右轮电机输出
float Left_out = 0;    //左轮电机输出
float motor_begin = 2600;
// === REMOVED for 4-wheel camera group: no suction fan ===
// float fan_speed = 7.5;  //6 20% 7 40% 8 60%

float iL, iR;          // 左右电机电流
float lf[4],rf[4];     // 左右电机电流平均滤波器
float iL_ys,iR_ys;     // 左右电机电流原始采集数据


void MotorInit()
{

    // 初始化电机输出
    pwm_init(PWM_L, 15000, 0);//ATOM 0模块的通道0 使用P02_4引脚输出PWM  PWM频率10kHZ  占空比百分之0/GTM_ATOM0_PWM_DUTY_MAX*100
    pwm_init(PWM_R, 15000, 0);

    // 初始化电机方向
    gpio_init(DIR_R,GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DIR_L,GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 初始化负压
    // === REMOVED for 4-wheel camera group: no suction fans ===
    // pwm_init(BDLC_L, 50, 0);
    // pwm_init(BDLC_R, 50, 0);

    //舵机
    pwm_init(SERVO_MOTOR_PWM, 50, 0);

    //电流检测初始化
    adc_init(ADC_L, ADC_10BIT); // 左
    adc_init(ADC_R, ADC_10BIT); // 右
}


int16 PWM_Limit(float  PWM,int16 max)
{
    if(PWM > -max && PWM < max)
        return (int16)PWM;     
    else if( PWM >=  max)
        PWM =max;    
    else
        PWM =-max;
    
    return (int16)PWM;
}


int preflag=1; 

void ALL_Out(void)
{
    if(CarInfo.Mode <= STOPM)
    {
        Right_out = 0;
        Left_out  = 0;
    }
    else if(CarInfo.Mode <= MOTORTEST)//电机测试
    {
        Right_out = -820;
        Left_out  = -820;
    }
    else if(CarInfo.Mode <= STAND)//标准模式
    {
          Left_out  =  L_SpeedControl;
          Right_out =  R_SpeedControl;
    }

//    Right_out = PWM_Limit((float)Right_out,5000);
//    Left_out = PWM_Limit((float)Left_out,5000);
//    pwm_set_duty(PWM_L_CH1, 1500);

    // === REMOVED for 4-wheel camera group: no suction fan control block ===
    // (Original drove BDLC_L/R duty + outer-side +1% on hard turns)
//    if(0){
//        pwm_set_duty(BDLC_L, 0);
//        pwm_set_duty(BDLC_R, 0);//6 20% 7 40%
//        if(Findline.err[0]>10)
//        {
//            pwm_set_duty(BDLC_R, (fan_speed +1 )* (PWM_DUTY_MAX / 100));
//        }
//        else if (lastError <-10)
//        {
//            pwm_set_duty(BDLC_L, (fan_speed +1 )* (PWM_DUTY_MAX / 100));
//        }
//    }
//    else{
//        pwm_set_duty(BDLC_L, 0 * (PWM_DUTY_MAX / 100));
//        pwm_set_duty(BDLC_R, 0 * (PWM_DUTY_MAX / 100));
//    }
if(system_ms>3000)
{
if(Right_out>=0)   // 设置右轮转动方向
    {
        gpio_set_level(DIR_R, 0);
        pwm_set_duty(PWM_R,(uint32)Right_out);    // 设置右转速
    }
    else
    {
        gpio_set_level(DIR_R, 1);
        pwm_set_duty(PWM_R,(uint32)-Right_out);    // 设置右转速
    }
    if(Left_out>=0)    // 设置左轮转动方向
    {
        gpio_set_level(DIR_L, 0);
        pwm_set_duty(PWM_L,(uint32)Left_out);    // 设置左转速
    }
    else
    {
        gpio_set_level(DIR_L, 1);
        pwm_set_duty(PWM_L,(uint32)-Left_out);    // 设置左转速
    }
}
}

float lf[4],rf[4];

void Get_Current(void){
    // 数字滤波参数 截止频率 1.5Hz  采样间隔 0.005s  输入 原始ADC采样数据  输出 滤波后的数据
    iL_ys = (float)(8.25*adc_mean_filter_convert(ADC_L, 5)/1024)-Zero_Error_L;
    iR_ys = (float)(8.25*adc_mean_filter_convert(ADC_R, 5)/1024)-Zero_Error_R;
    LPF_1_db(1,0.001,iL_ys,&iL);
    LPF_1_db(1,0.001,iR_ys,&iR);

    iL =  iL *0.15 + 0.85*(0.4*lf[0]+0.3*lf[1]+0.2*lf[2]+0.1*lf[3]);
    iR =  iR *0.15 + 0.85*(0.4*rf[0]+0.3*rf[1]+0.2*rf[2]+0.1*rf[3]);

    if(L_CarSpeed<0) iL = -iL;
    if(R_CarSpeed<0) iR = -iR;

    for(uint8 i = 3; i>0; i--)
    {
        lf[i] = lf[i-1];
        rf[i] = rf[i-1];
    }

    lf[0] = iL;
    rf[0] = iR;
}
