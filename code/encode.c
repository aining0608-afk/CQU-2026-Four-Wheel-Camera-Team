#include "encode.h"
#include "math.h"


int16 Left_count;
int16 Right_count;



float speed_avr;  //左右轮平均速度
float L_CarSpeed;   //左轮速度
float R_CarSpeed;   //右轮速度

float L_jishu;//滤波后计数值
float R_jishu;
float e_distance;//距离
void speedcount_init(void)
{
    encoder_dir_init(ENCODER1_GPT,count1_pin,dir1_pin);
    encoder_dir_init(ENCODER2_GPT,count2_pin,dir2_pin);
}

void GetSpeed(void)
{
    static int16 l_filter[4],r_filter[4];
    Left_count= encoder_get_count(ENCODER2_GPT);
    encoder_clear_count(ENCODER2_GPT);
    Right_count= -encoder_get_count(ENCODER1_GPT);
    encoder_clear_count(ENCODER1_GPT);



    for (int i = 3; i > 0; i--)
    {
        r_filter[i] = r_filter[i - 1];
        l_filter[i] = l_filter[i - 1];
    }
    r_filter[0]=Right_count;
    l_filter[0]=Left_count;

     //	//修改频率，编码器单位
    L_CarSpeed = (L_CarSpeed*0.15 + 0.85*(0.4*l_filter[0]+0.3*l_filter[1]+0.2*l_filter[2]+0.1*l_filter[3]) * SPEED_F / L_QD_UNIT);    //（计数值*计数频率/一米计数值）求出车速转换为M/S
    R_CarSpeed = (R_CarSpeed*0.15 + 0.85*(0.4*r_filter[0]+0.3*r_filter[1]+0.2*r_filter[2]+0.1*r_filter[3]) * SPEED_F / R_QD_UNIT);    //求出车速转换为M/S
    if (L_CarSpeed > 8)L_CarSpeed = 8;//限速
    if (R_CarSpeed > 8)R_CarSpeed = 8;



    speed_avr= (L_CarSpeed+R_CarSpeed)/2;

    /*
        for (int i = 3; i > 0; i--)
        {
            speed_avr_list[i] = speed_avr_list[i - 1];
        }
        speed_avr_list[0]=speed_avr;

        speed_avr_filt = 0.6*speed// Findline.err[0]=0.4*(float)(Findline.midline[qianzhan] - 80)+0.3*(float)(Findline.midline[qianzhan-1] - 80)+0.2*(float)(Findline.midline[qianzhan-2] - 80)+0.1*(float)(Findline.midline[qianzhan-3] - 80);_avr_list[0]+speed_avr_list[1]*0.2 + speed_avr_list[2]*0.2;
    */
        L_jishu=0.4*l_filter[0]+0.3*l_filter[1]+0.2*l_filter[2]+0.1*l_filter[3];
        R_jishu=0.4*r_filter[0]+0.3*r_filter[1]+0.2*r_filter[2]+0.1*r_filter[3];

        //计数值累加得距离，115为系数，根据实际调整
        // TODO[4WHEEL-CAL]: 125 is mini-car counts/cm coefficient. Re-derive for
        //                   4-wheel chassis (use L_QD_UNIT/100 if you want cm).
        e_distance += (L_jishu+R_jishu)/(2*125);

        //到一定距离停车，比赛时注释
        //if(e_distance>=4000){speed.Stan = 0; CarInfo.Mode = STAND;}

}
