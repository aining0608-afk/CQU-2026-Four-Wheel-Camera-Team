#include "control.h"
#include "gyroscope.h"
#include "findline.h"
#include "fuzzy.h"



StandInfotypedef L_SpeedLoop;
StandInfotypedef R_SpeedLoop;

StandInfotypedef L_ILoop;
StandInfotypedef R_ILoop;

float L_SpeedControl;
float R_SpeedControl;
float L_SpeedOutput;
float R_SpeedOutput;
float L_setSpeed;
float R_setSpeed;


Speed_struct speed;
CarInfotypedef CarInfo;
PID_CLASS ServPID1;
PID_CLASS ServPID;        //电机的参数
// TODO[4WHEEL-CAL]: re-tune for new chassis. Wider wheelbase amplifies diff effect.
//                   Original sign is negative; consider rewriting positive for clarity.
float speed_diff=-0.0015;//后轮差速系数0.003

// === 4-WHEEL curve slowdown (Method 1): linear deceleration vs |err| ===
// dynamic_stan = speed.Stan - curve_slow_K * |err|, clamped to curve_min_speed.
float curve_slow_K   = 0.05f;   // larger -> brake harder in curves
float curve_min_speed = 0.1f;    // floor speed, never below this
// TODO[4WHEEL-CAL]: pout0 is servo mechanical center; delt_pout_max is single-side
//                   travel. MUST recalibrate for 4-wheel servo geometry BEFORE first
//                   power-on, otherwise servo will hit the mechanical limit.
//                   Procedure: with motors disabled, sweep PWM to find left-stop,
//                   center, right-stop in raw duty units; pout0 = center;
//                   delt_pout_max = min(center-left_stop, right_stop-center) - 5.
float pout0 = 780;  //舵机中值， 首先需要调整此参数使得舵机打正
float delt_pout_max = 85; //舵机限幅 +-400
float pout,dout;

float Error;
float ErrorFifo[10];
float ErrorInteger;//积分值
float feedback[10];
float ErrorDtTemp[4];
float InOut;
float OutPut;
float OutTemp[4];

// TODO[4WHEEL-CAL]: fuzzy-PD output scale for servo. 4-wheel cars usually need a
//                   stronger gain (longer body). Try ~1.5x as starting point.
float Kp_max=5, Kd_max=2.5;



uint8 USE_GYRO=1;   // 是否使用串级PID

/***********************************************************************************************/
//void Speed_Loop();



void stop(void)
{
    speed.Stan = 0;
    CarInfo.Mode = STAND;//STOPM;

}
void PID_init(void)
{
    ServPID.P = 6.7;//3.8;3.7
    ServPID.D = 1.35;//1.4;//8 3 9.5 2.3 1.4

    ServPID1.P = 1.8;//1.3 0 1
    ServPID1.D = 0.2;
    ServPID1.I = 0;


    //分段式
    speed.Stan = speed_normal;

    L_SpeedLoop.KP = 1.0;//1.52
    L_SpeedLoop.KI = 0.01;//0.13
    L_SpeedLoop.KD = 0;   //0.0

    R_SpeedLoop.KP = 0.85;//1.4
    R_SpeedLoop.KI = 0.08;//0.12得减
    R_SpeedLoop.KD = 0;

    L_ILoop.KP = 6.5;
    L_ILoop.KI = 0.3;
    L_ILoop.KD = 0;

    R_ILoop.KP = 6.5;
    R_ILoop.KI = 0.3;
    R_ILoop.KD = 0;

}

void ServControl_mix(void)
{
    if(right_hdflag >=1 || left_hdflag>=1)
    {
        ServControl1(Findline.err[0]);
        ServControl();
    }
    else
    {
        fuzzy_ServControl(Findline.err[0]);
    }
}
void fuzzy_ServControl(float Error)
{
  pout = pout0-Fuzzy_P(Error,(Error - lastError))*Error - ServPID1.P*((Error-CarAngle.Yawrate)/10);//还是位置式PD；
  dout = Fuzzy_D(Error,(Error - lastError))*(Error - lastError);
  pout -= dout;
//  lastError=Findline.err[0];
  //舵机限幅
  if(pout >= (pout0 + delt_pout_max))
  {
      pout = pout0 + delt_pout_max;
  }
  else if(pout <= (pout0 - delt_pout_max))
  {
      pout = pout0 - delt_pout_max;
  }
  pwm_set_duty(SERVO_MOTOR_PWM,(uint32)pout);
}


void ServControl(void)
{
    if(USE_GYRO){
        pout =   pout0 - ServPID.P* OutPut;
        dout =  (OutPut + lastError)* ServPID.D;
        pout -= dout ;
    }//串级控制pid
    else {
        pout =   pout0 - ServPID.P* Findline.err[0];
        dout =  (Findline.err[0] - lastError)* ServPID.D;
        pout -= dout ;
    }//位置式pid
//环岛入环直接强制舵机打角
//    if(Findp.rightup[1]>25 && right_hdflag==3)
//    {
//        pout = pout0-70;
//    }
//    else
//    {
//        pout =   pout0 - ServPID.P* Findline.err[0];
//        dout =  (Findline.err[0] - lastError)* ServPID.D;
//        pout -= dout ;
//    }


    //舵机限幅
    if(pout >= (pout0 + delt_pout_max))
    {
        pout = pout0 + delt_pout_max;
    }
    else if(pout <= (pout0 - delt_pout_max))
    {
        pout = pout0 - delt_pout_max;
    }
    //pout    = 800;
    pwm_set_duty(SERVO_MOTOR_PWM,(uint32)pout);
}



void ServControl1(float err)            //舵机使用位置式pid，pwm_duty=(int)(P*error+D*(error-error_pre);
{
    Error=(err-CarAngle.Yawrate)/10;
    for(int i=9;i>0;i--)
       ErrorFifo[i] = ErrorFifo[i-1];
    ErrorFifo[0] =Error;

    ErrorInteger += ErrorFifo[0];
//    for(int i=9;i>0;i--)
//        feedback[i] = feedback[i-1];
//    feedback[0] = Error;


    for(int i=3;i>0;i--)
        ErrorDtTemp[i] = ErrorDtTemp[i-1];
    ErrorDtTemp[0] = ErrorFifo[0]-ErrorFifo[3];


    InOut =   ServPID1.P *ErrorFifo[0] +  ServPID1.I*ErrorInteger+
            ServPID1.D * (ErrorDtTemp[0] * 0.8 + ErrorDtTemp[1] * 0.1 + ErrorDtTemp[2] * 0.1);

    for(int i=3;i>0;i--)
        OutTemp[i] = OutTemp[i-1];
    OutTemp[0] = InOut;

    OutPut = OutTemp[0]*0.8 + OutTemp[1]*0.2;//滤波
    OutPut = OutPut*1.2;//改一下系数，用于匹配数量级

}


void L_Speed_Loop(void)
{
    // === curve slowdown injected ===
    float dynamic_stan = speed.Stan - curve_slow_K * fabs(Findline.err[0]);
    if (dynamic_stan < curve_min_speed) dynamic_stan = curve_min_speed;

    if(Findline.err[0]<5&& Findline.err[0]>-5&&trueshortflag==1)
    {
        L_setSpeed = dynamic_stan;
    }
    else{
    L_setSpeed = dynamic_stan*(1-speed_diff*(Findline.err[0]));
    }
    L_SpeedLoop.ErrorFifo[2] = L_SpeedLoop.ErrorFifo[1];
    L_SpeedLoop.ErrorFifo[1] = L_SpeedLoop.ErrorFifo[0];
    L_SpeedLoop.ErrorFifo[0] = L_SpeedLoop.Error;

    L_SpeedLoop.ErrorDtFifo[2] = L_SpeedLoop.ErrorDtFifo[1];
    L_SpeedLoop.ErrorDtFifo[1] = L_SpeedLoop.ErrorDtFifo[0];
    L_SpeedLoop.ErrorDtFifo[0] = L_SpeedLoop.ErrorFifo[0] - L_SpeedLoop.ErrorFifo[2];

    L_SpeedLoop.Error = (L_setSpeed - L_CarSpeed+((float)(pout-pout0)*0.014)) ;

    if(L_SpeedLoop.Error>=ErrorMaxLimit) L_SpeedLoop.Error = ErrorMaxLimit;
    if(L_SpeedLoop.Error<=ErrorMinLimit) L_SpeedLoop.Error = ErrorMinLimit;

    if(fabs(L_SpeedLoop.Error)<1)
        L_SpeedLoop.Integ += L_SpeedLoop.Error;

    if(L_SpeedLoop.Integ> ki_max) L_SpeedLoop.Integ = ki_max;   //对积分量进行限幅
    if(L_SpeedLoop.Integ<-ki_max) L_SpeedLoop.Integ = -ki_max;

    L_SpeedLoop.OutPut = L_SpeedLoop.KP * L_SpeedLoop.Error + L_SpeedLoop.KI * L_SpeedLoop.Integ + L_SpeedLoop.KD * (L_SpeedLoop.ErrorDtFifo[0] * 0.6 + L_SpeedLoop.ErrorDtFifo[1] * 0.4) ;



#if(Double_Loop)
    if(L_SpeedLoop.OutPut>=OutPutMaxLimit) L_SpeedLoop.OutPut = OutPutMaxLimit;
    if(L_SpeedLoop.OutPut<=OutPutMinLimit) L_SpeedLoop.OutPut = OutPutMinLimit;
    L_SpeedOutput = ( 0.2 * L_SpeedLoop.OutPut + 0.8 * L_SpeedOutput);

#else
    L_SpeedLoop.OutPut =  L_SpeedLoop.OutPut * 5000;
    if(L_SpeedLoop.OutPut>=OutPutMaxLimit) L_SpeedLoop.OutPut = OutPutMaxLimit;
    if(L_SpeedLoop.OutPut<=OutPutMinLimit) L_SpeedLoop.OutPut = OutPutMinLimit;
    L_SpeedControl = ( 0.2 * L_SpeedLoop.OutPut + 0.8 * L_SpeedControl);
#endif
}

void R_Speed_Loop(void)
{
    // === curve slowdown injected ===
    float dynamic_stan = speed.Stan - curve_slow_K * fabs(Findline.err[0]);
    if (dynamic_stan < curve_min_speed) dynamic_stan = curve_min_speed;

    if(Findline.err[0]<=5 && Findline.err[0]>=-5 && trueshortflag==1)
    {
        R_setSpeed = dynamic_stan;
    }
    else
    {
        R_setSpeed = dynamic_stan*(1+speed_diff*(Findline.err[0]));
    }


    R_SpeedLoop.ErrorFifo[2] = R_SpeedLoop.ErrorFifo[1];
    R_SpeedLoop.ErrorFifo[1] = R_SpeedLoop.ErrorFifo[0];
    R_SpeedLoop.ErrorFifo[0] = R_SpeedLoop.Error;

    R_SpeedLoop.ErrorDtFifo[2] = R_SpeedLoop.ErrorDtFifo[1];
    R_SpeedLoop.ErrorDtFifo[1] = R_SpeedLoop.ErrorDtFifo[0];
    R_SpeedLoop.ErrorDtFifo[0] = R_SpeedLoop.ErrorFifo[0] - R_SpeedLoop.ErrorFifo[2];

    R_SpeedLoop.Error = (R_setSpeed - R_CarSpeed+((float)(pout-pout0)*0.014)) ;

    if(R_SpeedLoop.Error>=ErrorMaxLimit) R_SpeedLoop.Error = ErrorMaxLimit;
    if(R_SpeedLoop.Error<=ErrorMinLimit) R_SpeedLoop.Error = ErrorMinLimit;

    if(fabs(R_SpeedLoop.Error)<1)
        R_SpeedLoop.Integ += R_SpeedLoop.Error;

    if(R_SpeedLoop.Integ> ki_max) R_SpeedLoop.Integ = ki_max;   //对积分量进行限幅
    if(R_SpeedLoop.Integ<-ki_max) R_SpeedLoop.Integ = -ki_max;

    R_SpeedLoop.OutPut = R_SpeedLoop.KP * R_SpeedLoop.Error + R_SpeedLoop.KI * R_SpeedLoop.Integ + R_SpeedLoop.KD * (R_SpeedLoop.ErrorDtFifo[0] * 0.6 + R_SpeedLoop.ErrorDtFifo[1] * 0.4) ;



#if(Double_Loop)
    if(R_SpeedLoop.OutPut>=OutPutMaxLimit) R_SpeedLoop.OutPut = OutPutMaxLimit;
    if(R_SpeedLoop.OutPut<=OutPutMinLimit) R_SpeedLoop.OutPut = OutPutMinLimit;
    R_SpeedOutput = ( 0.2 * R_SpeedLoop.OutPut + 0.8 * R_SpeedOutput);
#else
    R_SpeedLoop.OutPut =  R_SpeedLoop.OutPut * 5000;
    if(R_SpeedLoop.OutPut>=OutPutMaxLimit) R_SpeedLoop.OutPut = OutPutMaxLimit;
    if(R_SpeedLoop.OutPut<=OutPutMinLimit) R_SpeedLoop.OutPut = OutPutMinLimit;
    R_SpeedControl = ( 0.2 * R_SpeedLoop.OutPut + 0.8 * R_SpeedControl);
#endif
}


void L_I_Loop(float Expect)
{
    L_ILoop.ErrorFifo[2] = L_ILoop.ErrorFifo[1];
    L_ILoop.ErrorFifo[1] = L_ILoop.ErrorFifo[0];
    L_ILoop.ErrorFifo[0] = L_ILoop.Error;

    L_ILoop.ErrorDtFifo[2] = L_ILoop.ErrorDtFifo[1];
    L_ILoop.ErrorDtFifo[1] = L_ILoop.ErrorDtFifo[0];
    L_ILoop.ErrorDtFifo[0] = L_ILoop.ErrorFifo[0] - L_ILoop.ErrorFifo[2];

    L_ILoop.Error = Expect - iL;

    if(L_ILoop.Error>=ErrorMaxLimit) L_ILoop.Error = ErrorMaxLimit;
    if(L_ILoop.Error<=ErrorMinLimit) L_ILoop.Error = ErrorMinLimit;

    if(fabs(L_ILoop.Error)<1)
        L_ILoop.Integ += L_ILoop.Error;

    if(L_ILoop.Integ> ki_max) L_ILoop.Integ = ki_max;   //对积分量进行限幅
    if(L_ILoop.Integ<-ki_max) L_ILoop.Integ = -ki_max;

    L_ILoop.OutPut = L_ILoop.KP * L_ILoop.Error + L_ILoop.KI * L_ILoop.Integ + L_ILoop.KD * (L_ILoop.ErrorDtFifo[0] * 0.6 + L_ILoop.ErrorDtFifo[1] * 0.4) ;

    L_ILoop.OutPut =  L_ILoop.OutPut * 5000;

    if(L_ILoop.OutPut>=OutPutMaxLimit) L_ILoop.OutPut = OutPutMaxLimit;
    if(L_ILoop.OutPut<=OutPutMinLimit) L_ILoop.OutPut = OutPutMinLimit;

    L_SpeedControl = ( 0.2 * L_ILoop.OutPut + 0.8 * L_SpeedControl);
}


void R_I_Loop(float Expect)
{
    R_ILoop.ErrorFifo[2] = R_ILoop.ErrorFifo[1];
    R_ILoop.ErrorFifo[1] = R_ILoop.ErrorFifo[0];
    R_ILoop.ErrorFifo[0] = R_ILoop.Error;

    R_ILoop.ErrorDtFifo[2] = R_ILoop.ErrorDtFifo[1];
    R_ILoop.ErrorDtFifo[1] = R_ILoop.ErrorDtFifo[0];
    R_ILoop.ErrorDtFifo[0] = R_ILoop.ErrorFifo[0] - R_ILoop.ErrorFifo[2];

    R_ILoop.Error = Expect - iR ;

    if(R_ILoop.Error>=ErrorMaxLimit) R_ILoop.Error = ErrorMaxLimit;
    if(R_ILoop.Error<=ErrorMinLimit) R_ILoop.Error = ErrorMinLimit;

    if(fabs(R_ILoop.Error)<1)
        R_ILoop.Integ += R_ILoop.Error;

    if(R_ILoop.Integ> ki_max) R_ILoop.Integ = ki_max;   //对积分量进行限幅
    if(R_ILoop.Integ<-ki_max) R_ILoop.Integ = -ki_max;

    R_ILoop.OutPut = R_ILoop.KP * R_ILoop.Error + R_ILoop.KI * R_ILoop.Integ + R_ILoop.KD * (R_ILoop.ErrorDtFifo[0] * 0.6 + R_ILoop.ErrorDtFifo[1] * 0.4) ;

    R_ILoop.OutPut =  R_ILoop.OutPut * 5000;

    if(R_ILoop.OutPut>=OutPutMaxLimit) R_ILoop.OutPut = OutPutMaxLimit;
    if(R_ILoop.OutPut<=OutPutMinLimit) R_ILoop.OutPut = OutPutMinLimit;

    R_SpeedControl = ( 0.2 * R_ILoop.OutPut + 0.8 * R_SpeedControl);
}



  
