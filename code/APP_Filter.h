#ifndef _Filter_H
#define _Filter_H
#include "zf_common_headfile.h"
#define M_PI_F 3.141592653589793f
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

extern Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;

extern Butter_BufferData Butter_Buffer_ACC_X,Butter_Buffer_ACC_Y,Butter_Buffer_ACC_Z,
Butter_Buffer_GyroX,Butter_Buffer_GyroY,Butter_Buffer_GyroZ,Butter_Buffer_Carangle;

extern Butter_BufferData Butter_Buffer_speed;
void LPF_1_db(float hz,float time,float in,float *out);
void Butterworth_Parameter_Init(void);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);


#endif
