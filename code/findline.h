#ifndef _FINDLINE_H
#define _FINDLINE_H

#include "zf_common_headfile.h"
#include "headfile_code.h"




//宏定义
#define image_h 120//图像高度
#define image_w 160//图像宽度


#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//跳过的点数
#define border_max  image_w-2 //边界最大值
#define border_min  1   //边界最小值

extern float speed_huandao ;  // 环岛速度2.8
extern float speed_normal   ;// 常速3.0 f
extern float speed_zhidao   ;   //3.8

// === REMOVED for 4-wheel camera group: no suction fans ===
// #define fan_speed_huandao  9
// #define fan_speed_normal   9
// #define fan_speed_zhidao   9

//#define qianzhan           55
//#define qianzhan_init      55
extern int16 qianzhan , qianzhan_init ;

extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//  图像数组
extern uint8 image_thereshold;//图像分割阈值

extern uint8 stop_flag,banmaxian_biaozhiwei;
extern int16 left_hdflag;
extern int16 right_hdflag;
extern float ang_in;
extern int16 leftduan ;
extern int16 rightduan;
extern uint8 hd_state;
extern uint8 hd_right_state;
extern float parameterA,parameterB;//参数A为截距 参数B为斜率
extern uint8 shortflag;
extern uint8 shortwrong ;
extern uint8 trueshortflag ;//直道标志位

extern uint8 right_shortflag;
extern uint8 right_shortwrong ;
extern uint8 right_trueshortflag ;//右道标志位

extern uint8 left_shortflag;
extern uint8 left_shortwrong;
extern uint8 left_trueshortflag;
extern uint8 left_duan;
extern uint8 right_duan;
typedef struct
{

    int16 midline[120];
    int16 load_width[120];

    int16 leftline[120];
    int16 rightline[120];
    int16 leftlineflag[120];
    int16 rightlineflag[120];


    int16 leftstartpoint;
    int16 rightstartpoint;

    int16 endline;
    int16 R_endline;
    int16 L_endline;
    int8 loseflag;
//    int16 upline[160];
//    int16 uplineflag[160];
    float err[60];
//
//
//    float kp1;
//    float kd1;
//
//    float U;
//    float U1;
//
//    float llasterr;

    int16 r_lose;
    int16 l_lose;

} findline_TypeDef;
typedef struct
{
        int16 leftup[2];
        int16 rightup[2];
        int16 leftdown[2];
        int16 rightdown[2];
        int16 lcenter[2];
        int16 rcenter[2];
        int16 counter[1];
        float lenth;
        int16 leftdown_flag ;
        int16 leftup_flag;
        int16 rightdown_flag ;
        int16 rightup_flag ;
        int16 lcenter_flag ;
        int16 rcenter_flag;
}fp_TypeDef;

extern fp_TypeDef Findp;

extern findline_TypeDef Findline;


int my_abs(int value);
int16 limit1(int16 x, int16 y);
uint8 get_start_point(uint8 start_row);

void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest);

void get_left(uint16 total_L);
void get_right(uint16 total_R);
void image_filter(uint8(*bin_image)[image_w]);//形态学滤波，简单来说就是膨胀和腐蚀的思想
void image_draw_rectan(uint8(*bin_image)[image_w]);

void findp (void) ;
void Huandaochuli_left(void);
void Huandaochuli_right(void);

void shizi(void);
void connect_line(int16 x1, int16 y1, int16 x2, int16 y2, int type) ;
void Lengthen_right_line(int16 start, int16 end);
void Lengthen_left_line(int16 start, int16 end);
void right_addline(int16 start,int16 end);
void Continuity_change(int16 start, int16 end);
void find_right_line_new(void);
uint8 zuoluzhang_judge(int16 row);
extern void findline(void); //直接在中断或循环里调用此程序就可以循环执行了
extern void findline_left_right(void); //直接在中断或循环里调用此程序就可以循环执行了
float Slope_Calculate(uint8 begin, uint8 end, int16 *border);
void calculate_s_i(uint8 start, uint8 end, int16 *border, float *slope_rate, float *intercept);
void caculate_err(void);
void EmergencyStop(void);
void advanced_regression(uint8 type, uint8 startline1, uint8 endline1, uint8 startline2, uint8 endline2);//对两段图线进行拟合曲线 1为左线 2为中线 3为右线
void zhidaopanduan(void);
void youdaopanduan(void);
void zuodaopanduan(void);
void duanpanduan(uint8 startline, uint8 endline);
int16 Find_Left_Up_Point(int16 start, int16 end);
int16 Find_Left_Down_Point(int16 start, int16 end);
int16 Find_Right_Up_Point(int16 start, int16 end);
int16 Find_Right_Down_Point(int16 start, int16 end);
int16 Monotonicity_Change_Right(int16 start, int16 end);
int16 Monotonicity_Change_Left(int16 start, int16 end) ;
void blackbox(void);
void drawp (void);
void zhidaopanduan_jiasu(void);
float Err_Sum(void);
void lostupdown(void);
#endif /*_IMAGE_H*/
