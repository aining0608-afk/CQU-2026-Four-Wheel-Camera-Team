#include "findline.h"
#include "math.h"
#include "gyroscope.h"
#include "APP_Filter.h"
CarAgnleInfotypedef CarAngle;
GYRO_CLASS gyro={0};
float ACC_Angle;
float Gravity_Angle = 0;
float Car_Angle     = 0;
float CarY_Angle    = 0;
float Angle_Speed   = 0;
float Turn_Speed    = 0; 

float GRYZ;
float ACCX;
float ACCZ;
float ACCY;
void Get_Attitude(void)
{


    static float faiX,faiZ,faiY;
    static float temp_speed = 0;
    float ratio_gyro = 0.02;//将陀螺仪的值转化为 度每秒 的系数
    temp_speed = CarAngle.Pitch;
    imu660ra_get_gyro(); //得到陀螺仪数据
    imu660ra_get_acc(); //得到加速度数据


    LPF_1_db(35,1000,(float)imu660ra_gyro_z,&GRYZ);
    //GRYZ = LPButterworth(imu660ra_gyro_z,&Butter_Buffer_GyroZ,&Butter_2HZ_Parameter_Acce);
    ACCX = LPButterworth(imu660ra_acc_x,&Butter_Buffer_ACC_X,&Butter_10HZ_Parameter_Acce);
    ACCY = LPButterworth(imu660ra_acc_y,&Butter_Buffer_ACC_Y,&Butter_10HZ_Parameter_Acce);
    ACCZ = LPButterworth(imu660ra_acc_z,&Butter_Buffer_ACC_Z,&Butter_10HZ_Parameter_Acce);
//  LPF_1_db(50,4,(float)imu660ra_acc_x,&ACCX);
//  LPF_1_db(50,4,(float)imu660ra_acc_z,&ACCZ);
//  LPF_1_db(50,4,(float)imu660ra_acc_y,&ACCY);


    Turn_Speed = 0.8 * Turn_Speed + 0.2 *GRYZ* ratio_gyro;
  
    faiX = ACCX / 16384.0f;
    if (faiX > 1)  faiX = 1;
    if (faiX < -1) faiX = -1;
  

    ACC_Angle = asin(faiX)/0.01745329;
    faiY= ACCY/16384.0f;
    if (faiY > 1) faiY = 1;
    if (faiY < -1) faiY = -1;


    faiZ = ACCZ / 16384.0f;
    if (faiZ > 1)  faiZ = 1;
    if (faiZ < -1) faiZ = -1;

    // TODO[4WHEEL-CAL]: -84 is the IMU mounting offset on the mini-car. With car
    //                   sitting level, atan2(faiZ,faiX)*180/PI should read this
    //                   value. Replace -84 with the actual reading on 4-wheel chassis.
    Gravity_Angle =  180.0 * atan2(faiZ,faiX) / 3.14-84;    //车头抬起的角度，存在偏差
//  Gravity_Angle = 0.5 * Gravity_Angle + 0.5 * asin(faiY) / 0.01745329;

  
    Angle_Speed = -GRYZ * 0.03051757;
  
    //Kalman_Filter(ACC_Angle,Angle_Speed);            //调用卡尔曼滤波函数
    CarY_Angle = 0.9 * CarY_Angle + 0.1 * Gravity_Angle;

    CarAngle.Pitchrate = Angle_Speed;
    CarAngle.Yawrate   = -Turn_Speed;   //当前转动的角度（瞬时）
    if(abs(CarAngle.Yawrate)<0.5)
        CarAngle.Yawrate=0;

    CarAngle.TureYawrate = 2.25 * CarAngle.Yawrate / cos(CarAngle.Pitch * (PI / 180));
    gyro.PitchAngle_Integral +=  2.5 * CarAngle.Yawrate / 1900.0;    //累计转动的角度
}

static  float Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, dt = 0.015;
static float Pk[2][2] = { {1, 0}, {0, 1 }};
static float Pdot[4] = {0,0,0,0};
static float q_bias = 0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

//-----------------------------------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)			
{
  Car_Angle += (gyro_m - q_bias) * dt; //预测值
  Pdot[0] = Q_angle - Pk[0][1] - Pk[1][0];
  Pdot[1] = -Pk[1][1];
  Pdot[2] = -Pk[1][1];
  Pdot[3] = Q_gyro;
  
  Pk[0][0] += Pdot[0] * dt;
  Pk[0][1] += Pdot[1] * dt;
  Pk[1][0] += Pdot[2] * dt;
  Pk[1][1] += Pdot[3] * dt;
  
  angle_err = angle_m - Car_Angle;//测量值-预测值
  
  PCt_0 = Pk[0][0];
  PCt_1 = Pk[1][0];
  
  E = R_angle + PCt_0;
  
  K_0 = PCt_0 / E; //卡尔曼增益
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;
  t_1 = Pk[0][1];
  
  Pk[0][0] -= K_0 * t_0;
  Pk[0][1] -= K_0 * t_1;
  Pk[1][0] -= K_1 * t_0;
  Pk[1][1] -= K_1 * t_1;
  
  Car_Angle += K_0 * angle_err; //最优角度=预测值+卡尔曼增益*(测量值-预测值)
  q_bias	+= K_1 * angle_err;
  Angle_Speed = gyro_m - q_bias;
}

