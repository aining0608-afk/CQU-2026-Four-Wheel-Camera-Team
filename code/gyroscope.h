#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_

#include "zf_common_headfile.h"
   
//#define PI                 (3.1415926535898)

typedef struct
{
  float ExYawratetemp;
  float ExStandUpAngle;
  float ExStandUpAngleFifo[4];
  float ExStandUpOldAgnle;
  float ExStandUpAgnleLimit;
  float ExGyroPitchrate;
  float ExYawrate;

  float Omega;
  float AccAngle;
  float MachineAngle1;
  float MachineAngle2;
  float Kangle;
  float Komega;
  float Pitch;
  float Pitchrate;
  float Yaw;
  float Yawrate;
  float Roll;
  float Rollrate;
  float TureYawrate;
  float Turn_Angle;
  float PitchNew;
  float yawNew;
  float rollNew;
  float Yaw_Avarage;
  float TureYawrate_Fifo[100];
}CarAgnleInfotypedef;


typedef struct
{
    float ACC_Angle;
    float Gravity_Angle;
    float Car_Angle;
    float CarY_Angle;
    float Angle_Speed;
    float Turn_Speed;
    float TurnAngle_Integral;
    float PitchAngle_Integral;
    uint8 speed_err_flag;

} GYRO_CLASS;


extern float CarY_Angle;

extern GYRO_CLASS gyro;
extern float Gravity_Angle;

















#define IMUConverter 0.000532f
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define TAN_MAP_SIZE    256
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define my_pow(a) ((a)*(a))

extern CarAgnleInfotypedef CarAngle;



void Kalman_Filter(float angle_m,float gyro_m) ;

void Get_Attitude(void);


#endif 
