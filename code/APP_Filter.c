#include "APP_Filter.h"

Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;
Butter_BufferData Butter_Buffer_ACC_X,Butter_Buffer_ACC_Y,Butter_Buffer_ACC_Z,Butter_Buffer_GyroX,Butter_Buffer_GyroY,Butter_Buffer_GyroZ,Butter_Buffer_Carangle;
Butter_BufferData Butter_Buffer_speed;
/****************************************
    一阶惯性低通滤波器
***************************************/
void LPF_1_db(float hz,float time,float in,float *out)
{
	*out += ( 1 / ( 1 + 1 / ( hz *6.28f *time ) ) ) *( in - *out );
}
/****************************************
    Butterworth低通滤波器参数初始化
***************************************/

void Butterworth_Parameter_Init(void)
{
  set_cutoff_frequency(200, 80,&Butter_80HZ_Parameter_Acce);
  set_cutoff_frequency(200, 60,&Butter_60HZ_Parameter_Acce);
  set_cutoff_frequency(200, 51,&Butter_51HZ_Parameter_Acce);
  set_cutoff_frequency(200, 30,&Butter_30HZ_Parameter_Acce);
  set_cutoff_frequency(200, 20,&Butter_20HZ_Parameter_Acce);
  set_cutoff_frequency(200, 15,&Butter_15HZ_Parameter_Acce);
  set_cutoff_frequency(200, 10,&Butter_10HZ_Parameter_Acce);
  set_cutoff_frequency(200, 5 ,&Butter_5HZ_Parameter_Acce);
  set_cutoff_frequency(200, 2 ,&Butter_2HZ_Parameter_Acce);//速度读取2ms
}


/*************************************************
函数名:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明:	加速度计低通滤波器
入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/

float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        static int LPF_Cnt=0;
        Buffer->Input_Butter[2]=curr_input;
        if(LPF_Cnt>=100)
        {
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
        }
        else
        {
          Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
          LPF_Cnt++;
        }
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return Buffer->Output_Butter[2];
}


void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
	float fr = sample_frequent / cutoff_frequent;
	float ohm = tanf(M_PI_F / fr);
	float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
	if (cutoff_frequent <= 0.0f) {
		// no filtering
		return;
	}
	LPF->b[0] = ohm * ohm / c;
	LPF->b[1] = 2.0f * LPF->b[0];
	LPF->b[2] = LPF->b[0];
        LPF->a[0]=1.0f;
	LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
	LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}
