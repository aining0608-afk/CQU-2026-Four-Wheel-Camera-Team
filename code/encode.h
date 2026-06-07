#ifndef __ENCODE_H
#define __ENCODE_H

/****Includes*************************************************/

#include "zf_common_headfile.h"

// **************************** 宏定义 ****************************
#define TASK_ENABLE 0

#define ENCODER2_GPT          TIM5_ENCODER
#define count2_pin            TIM5_ENCODER_CH1_P10_3
#define dir2_pin              TIM5_ENCODER_CH2_P10_1

#define ENCODER1_GPT          TIM6_ENCODER
#define count1_pin            TIM6_ENCODER_CH1_P20_3
#define dir1_pin              TIM6_ENCODER_CH2_P20_0
/****Definitions**********************************************/

#define SPEED_F 200.0f  //速度获取频率

// TODO[4WHEEL-CAL]: encoder counts per meter. Depends on wheel diameter, gear ratio,
//                   and encoder line count. Re-derive: push car along 1m straight,
//                   integrate count, set L/R_QD_UNIT to that value.
#define L_QD_UNIT 18250.0f//编码器一米的计数
#define R_QD_UNIT 18250.0f

extern int16 Left_count;
extern int16 Right_count;

extern float speed_avr;
extern float L_CarSpeed;
extern float R_CarSpeed;

extern float e_distance;

void speedcount_init(void);
void GetSpeed(void);

#endif
