#ifndef __MOTOR_H
#define __MOTOR_H

/****Includes*************************************************/

#include "zf_common_headfile.h"

/****Definitions**********************************************/
// === 4-WHEEL V2.5 motherboard motor connector ===
// V2.5 pin table: motor ch1 PWM=02.5/DIR=02.4, motor ch2 PWM=02.7/DIR=02.6
// TODO[4WHEEL-VERIFY]: confirm L/R orientation matches physical car after powering.
#define PWM_L              (ATOM0_CH7_P02_7)   // ch1 PWM (unchanged from mini)
#define DIR_L              (P02_6)             // ch1 DIR (was P02_7 on mini)
#define PWM_R              (ATOM0_CH5_P02_5)   // ch2 PWM (was P02_6 on mini)
#define DIR_R              (P02_4)             // ch2 DIR (was P02_4 on mini)
// (Original mini-board pins kept commented for reference)
// #define DIR_R              (P02_4)
// #define PWM_L              (ATOM0_CH5_P02_5)
// #define PWM_R              (ATOM0_CH6_P02_6)
// #define DIR_L              (P02_7)

//#define DIR_R              (P33_6)
//#define PWM_L              (ATOM2_CH4_P33_8)
//#define PWM_R              (ATOM2_CH3_P33_7)
//#define DIR_L              (P33_11)

// === REMOVED for 4-wheel camera group: no suction fans ===
// #define BDLC_L             (ATOM3_CH0_P33_4)
// #define BDLC_R             (ATOM3_CH1_P33_5)

#define SERVO_MOTOR_PWM     ATOM1_CH1_P33_9   // 定义主板上舵机对应引脚

// 定义电流采样引脚
#define ADC_L               ADC1_CH5_A21
#define ADC_R               ADC1_CH4_A20

// 定义电流采样零位误差
// TODO[4WHEEL-CAL]: zero-current ADC reading. Re-measure on the 4-wheel chassis
//                   with motors off and ADC quiescent; set both to that voltage.
#define Zero_Error_L        0.0605
#define Zero_Error_R        0.1415

void ALL_Out(void);
void MotorInit(void);
void Get_Current(void); // 电流检测函数

extern float Right_out ;
extern float Left_out;
// === REMOVED for 4-wheel camera group: no suction fans ===
// extern float fan_speed;

extern float iL;
extern float iR;

extern float iL_ys,iR_ys;


#endif
