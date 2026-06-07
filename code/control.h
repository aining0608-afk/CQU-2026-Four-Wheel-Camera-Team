#ifndef __CONTROL_H
#define __CONTROL_H
#include "zf_common_headfile.h"


#define ki_max  2.0  //积分量限幅值
#define OutPutMinLimit -4500
#define OutPutMaxLimit 4500
#define ErrorMaxLimit 2
#define ErrorMinLimit -2

#define Double_Loop 0

typedef enum
{
    STOPM = 1,
    MOTORTEST = 2,
    STAND = 3,
    CLOSELOOP = 4,
}CarMode;

typedef struct
{
    CarMode Mode;
    uint8 Condition;
}CarInfotypedef;

extern CarInfotypedef CarInfo;
extern uint8 stop_mode ;
extern float pout;
extern float pwmout;


typedef struct{
  float Error;                          //速度偏差
  float Stan;                           //标准速度
  float L_ControlIntegral;
  float R_ControlIntegral;
  float lowest;                    //速度设定最小值
  float highest;
  float L_Bigeest;
  float R_Bigeest;
}Speed_struct;

typedef struct
{
    float Expect;
    float KP;
    float KI;
    float KD;
//    float BaseKP;
//    float BaseKI;
//    float BaseKD;
    float Error;
    float ErrorFifo[5];
    float ErrorLast;
    float ErrorDtFifo[5];
    float ErrorTemp[4];
    float ErrorDtTemp[4];
    float InOut;
    float Integ;
    float OutPut;
//    float RampKP;
//    float RampKD;
}StandInfotypedef;


extern Speed_struct speed;

typedef struct
{
  void (*SpeedInit)();
  void (*MotorControl)(float* err);
  

} CONTROL_CLASS;
extern CONTROL_CLASS control;

typedef struct {
    float P;
    float I;
    float D;
} PID_CLASS;

extern PID_CLASS MotorPID;
extern PID_CLASS ServPID;
extern PID_CLASS ServPID1;

extern uint8 USE_GYRO;


extern StandInfotypedef L_SpeedLoop;
extern StandInfotypedef R_SpeedLoop;

extern StandInfotypedef L_ILoop;
extern StandInfotypedef R_ILoop;

void L_Speed_Loop(void);
void R_Speed_Loop(void);

void R_I_Loop(float Expect);
void L_I_Loop(float Expect);

extern float L_setSpeed;
extern float R_setSpeed;
extern float L_SpeedControl;
extern float R_SpeedControl;
extern float L_SpeedOutput;
extern float R_SpeedOutput;


void PID_init(void);
void stop(void);
//extern float speed_init ;
//extern float serv_error[2];
void ServControl(void);
void ServControl1(float err);
void fuzzy_ServControl(float Error);
void ServControl_mix(void);
extern float pout,dout,pout0;
extern float lastError;
extern float Kp_max,Kd_max;


extern float speed_diff;//后轮差速系数
#endif
