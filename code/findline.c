#include "zf_common_headfile.h"
#include "headfile_code.h"

/*变量声明*/
uint8 image_thereshold;//图像分割阈值
//uint8 last_threshold;
float lastError;//偏差
int16 lose_line=0;//丢线信号

// TODO[4WHEEL-CAL]: 'qianzhan' is the look-ahead row index (image is 120 rows tall;
//                   higher index = closer, lower = farther). Mini default is 60.
//                   4-wheel cars run faster, so look further ahead -- try 70~85.
int16 qianzhan_init =44;
int16 qianzhan =44;


findline_TypeDef Findline = {{0}};
fp_TypeDef Findp= {{0}};

// TODO[4WHEEL-CAL]: starting speeds (m/s). Lowered from mini values for safe
//                   first run. After hardware/IMU/encoder/servo are calibrated,
//                   ramp up. 4-wheel target: straight ~3.0-4.5, normal ~2.5, ring ~2.0.
float speed_huandao     =  0.4 ; // 环岛速度2.8
float speed_normal      =  0.35 ;// 常速3.0 f
float speed_zhidao      =  1.3 ; //3.8
int16 left_hdflag=0;  // 左环岛标志位
int16 right_hdflag=0;  // 右环岛标志位

uint8 banmaxian_biaozhiwei = 0;  // 斑马线标志位
uint8 stop_flag=0;//丢线标志位

float ang_in=0;
int16 leftduan = 0;
int16 rightduan = 0;
uint8 hd_state=0;//左环岛数量
uint8 hd_right_state=0;//右环岛数量

float parameterA,parameterB;//参数A为截距 参数B为斜率
uint8 shortflag=0;
uint8 shortwrong = 0;
uint8 trueshortflag = 0;//直道标志位

uint8 right_shortflag=0;
uint8 right_shortwrong=0;
uint8 right_trueshortflag =0;//右道标志位

uint8 left_shortflag=0;
uint8 left_shortwrong=0;
uint8 left_trueshortflag=0;//左道标志位

uint8 left_duan=0;
uint8 right_duan=0;

////加权控制
//const uint8 Weight[70]=
//{
//        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端30 ——39 行权重
//        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端40 ——49 行权重
//        1, 1, 1, 1, 1, 1, 1, 3, 4, 5,              //图像最远端50 ——59 行权重
//        6, 7, 9,11,13,15,17,19,20,20,              //图像最远端60 ——69 行权重
//       19,17,15,13,11, 9, 7, 5, 3, 1,              //图像最远端70 ——79 行权重
//        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端80 ——89 行权重
//        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,              //图像最远端90 ——99 行权重
//};//权重只是选取某一范围内作为主要控制行，并且尽可能用上其他图像误差行
//
//float Err_Sum(void)
//{
//    int16 i;
////    float err=0;
//    float weight_count=0;
//    //常规误差
//    for(i=100;i>=30;i--)//常规误差计算
//    {
//        lastError+=(80-Findline.midline[i])*Weight[i];
//        weight_count+=Weight[i];
//    }
//    lastError=lastError/weight_count;
////    return lastError;
//}
void caculate_err(void)  //计算偏差
{
    int16 i,sum;
    //static float sum;
    lastError = Findline.err[0];
    sum = 0;
    for(i = 59; i >= 1; i--)
        Findline.err[i] = Findline.err[i - 1]; //偏差滤波
    Findline.err[0] = 0.0;
    //if(p_right_flag != 2)
        if(qianzhan > Findline.endline + 4)//<58
        {
            for(i = qianzhan; i > Findline.endline && i > qianzhan-7; i--) // 目前来说这个还是鸡肋不知道怎么砍掉
            {
                sum++;
                Findline.err[0] += (float)(Findline.midline[i] - 80);
            }
            Findline.err[0] = Findline.err[0] / sum;
        }
//        else
//            Findline.err[0] = (float)(Findline.midline[Findline.endline + 2] - 80);
//
//        if (lastError * Findline.err[0] < 0 && fabs(lastError - Findline.err[0]) > 40)
//        {
//            Findline.err[0] = lastError;
//        }
}

/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 limit_a_b(int16 x, int16 a, int16 b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}


int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}



uint8 Fast_otsuThreshold(uint8 *image)   //注意计算阈值的一定要是原图像
{
#define GrayScale 256
    int Pixel_Max=0;
    int Pixel_Min=255;
    uint16 width = MT9V03X_W;
    uint16 height = MT9V03X_H;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height/4;
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
            if(data[i * width + j]>Pixel_Max)   Pixel_Max=data[i * width + j];
            if(data[i * width + j]<Pixel_Min)   Pixel_Min=data[i * width + j];
        }
    }

    //计算每个像素值的点在整幅图像中的比例

    for (i = Pixel_Min; i < Pixel_Max; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;

    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;

    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = Pixel_Min; j < Pixel_Max; j++)
    {

        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;

        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }

    }

    return threshold;
}

// 大津法计算动态阈值
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;


    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
uint8 bin_image[image_h][image_w];//图像数组
void turn_to_bin(void)
{
  uint8 i,j;
 //image_thereshold = otsuThreshold(mt9v03x_image[0], image_w, image_h);
 image_thereshold=  Fast_otsuThreshold(mt9v03x_image[0]);
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(mt9v03x_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //从中间往左边，先找起点
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else {
        //printf("未找到起点\n");
        return 0;
    }
}

#define USE_num image_h*3   //定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针
//    static int8 square_5[24][2] = {
//    {-2,-2},{-1,-2},{0,-2},{+1,-2},{+2,-2},
//    {-2,-1},{-1,-1},{0,-1},{+1,-1},{+2,-1},
//    {-2,-0},{-1, 0},{+1, 0},{+2,-0},
//    {-2,+1},{-1,+1},{0,+1},{+1,+1},{+2,+1},
//    {-2,+2},{-1,+2},{0,+2},{+1,+2},{+2,+2}
//    };
    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},

    //这个是逆时针
//    int16 thres_r = 0;
//    int16 thres_l = 0;
    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
//            thres_l = 0;
//           for (uint8 j = 0; j <25; j++)
//           {
//               thres_l += image[search_filds_l[i][1] + square_5[j][1]][search_filds_l[i][0] + square_5[j][0]];
//           }
//           thres_l = thres_l + 201 + 200 + 200 + 201 + 200 + 199 + 196 + 194 + 192;
//           thres_l /= 33;//计算局部平均阈值
//           if (image[search_filds_l[i][1]][search_filds_l[i][0]] > thres_l)
//           {
//               bin_image[search_filds_l[i][1]][search_filds_l[i][0]] = 255;
//           }
//           else bin_image[search_filds_l[i][1]][search_filds_l[i][0]] = 0;
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }

        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y
        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            //printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {
                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }
    }
    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint16 k=0 ;
    uint8 h = 0;
    //初始化
    for (i = 0;i<image_h;i++)
    {
        Findline.leftline[i] = border_min;
    }
    h = image_h - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);

            if(Findline.leftline[points_l[j][1]] <= points_l[j][0]+1)
            {
            Findline.leftline[points_l[j][1]] = points_l[j][0]+1;
            }
    }
           for (i = 0;i<80;i++)
            {
                if(Findline.leftline[i]-Findline.leftline[i+1]<-40)
                {
                    Findline.L_endline = i+1;
                    for(k=i;k>1;k--)
                    {
                Findline.leftline[k] = Findline.leftline[i+1];
                    }
                    break;
                }
            }

}

void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint16 k =0;
    uint8 h = 0;
    for (i = 0; i < image_h; i++)
    {
        Findline.rightline[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
    if(Findline.rightline[points_r[j][1]] >= points_r[j][0]-1)
    {
    Findline.rightline[points_r[j][1]] = points_r[j][0]-1;
    }
    }

    for (i = 0;i<80;i++)
     {
         if(Findline.rightline[i]-Findline.rightline[i+1]>40)
         {
             Findline.R_endline = i+1;
             for(k=i;k>1;k--)
             {
         Findline.rightline[k] = Findline.rightline[i+1];
             }
             break;
         }
     }
}

//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5//此参数可根据自己的需求调节
#define threshold_min   255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                    bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
                + bin_image[i][j - 1] + bin_image[i][j + 1]
                + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {

                bin_image[i][j] = 0;//黑

            }

        }
    }

}


void image_draw_rectan(uint8(*bin_image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        bin_image[i][0] = 0;
        bin_image[i][1] = 0;
        bin_image[i][image_w - 1] = 0;
        bin_image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        bin_image[0][i] = 0;
        bin_image[1][i] = 0;
        bin_image[image_h-1][i] = 0;
//        bin_image[image_h-2][i] = 0;
    }
}



//判断斑马线函数
void banmaxian1(int16 start_point, int16 end_point)
{
        //变量标志位
        int16 banmaxian_kuandu;//斑马线宽度
        int16 banmaxian_hangshu;//斑马线行数
        int16 banmaxian_geshu;//斑马线个数（块）
        //从下往上扫描
        for (int16 y = end_point; y >= start_point; y--)
        {
             banmaxian_kuandu=0;
             banmaxian_hangshu=0;
             banmaxian_geshu=0;
             //从右往左扫描
            for (int16 x =140; x >=20; x--)
            {
                int16 baidian_heng=0;
                //扫描到黑色，就进判断
                if (bin_image[y][x] == 0)
                {
                    for(int16 a=x;x<x+15;x++)//从黑色点向左侧扫描
                    {
                        //找到白色点
                        if(bin_image[y][a] == 255)
                        {
                            //记录白色点的位置，跳出循环
                            baidian_heng=a;
                            break;
                        }
                    }//斑马线宽度等于黑白点的差
                    banmaxian_kuandu=x-baidian_heng;


                }
                else
                {   //斑马线的宽度在4~8之间认为它成立为斑马线黑色块
                    if (banmaxian_kuandu >= 2 && banmaxian_kuandu <= 8)
                    {
                        //斑马线黑色块++
                        banmaxian_geshu++;
                        //斑马线色块宽度清零，进行下一个黑色块的扫描计算
                        banmaxian_kuandu = 0;
                    }
                    else
                    {
                        //如果不满足对黑色块的认为要求就直接清零，去计算下一个黑色块
                        banmaxian_kuandu = 0;
                    }
                }
            }
            //如果色块的个数在6~12之间则认为这一行的斑马线满足要求，在去扫下一行
            if (banmaxian_geshu >= 6 && banmaxian_geshu <= 12) {banmaxian_hangshu++;}
        }
        //如果有大于等于4行的有效斑马线
        if(banmaxian_hangshu>=5)
        {
            //斑马线标准位置1
            banmaxian_biaozhiwei=1;
        }
        else{banmaxian_biaozhiwei=0;}
}

uint32 time_flag = 0;

void banmaxian2(void)
{
for(uint8 hang = 50;hang<119;hang++)
{
    uint8 garage_count= 0 ,region=0;
    uint8 white_black,black_white=0;
    for(int16 lie = Findline.leftline[hang];lie<Findline.rightline[hang];lie++)
     {
           if(bin_image[hang][lie]==255)//如果检测到255白
           {
               white_black=1;//白黑为1
           }
           else
   {
     white_black=0;//黑白为0
   }

           if(white_black!=black_white)
           {
             black_white = white_black;
               garage_count++;
           }
           if(garage_count>10)
         {
           region++;
         }
       }
           if(region>3)
             {banmaxian_biaozhiwei=1;break;}
           else banmaxian_biaozhiwei=0;
   }
//            if(e_distance>20&&e_distance<80)
//            {
//                connect_line(54,30,5,115,0);
//                connect_line(105,3.0,155,115,1);
//            }
}


//斑马线
void blackwhiteline(void)
{
    int16 i = 0, bwlineboard=0;
    for(i = 20; i < 140; i++)
    {
            if(bin_image[i-1][100]==0 && bin_image[i][100] == 0 && bin_image[i+1][100]==255 && bin_image[i+2][100]==255)
            {
                bwlineboard = bwlineboard + 1;
            }
    }
    if(bwlineboard>=3 && stop_flag ==0)
    {
        stop_flag = 1;
        stop();
    }
}
/**
* @brief 最小二乘法
* @param uint8 begin                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界首地址
*  @see CTest       Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, int16 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;

    }
    if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
    {
        result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}

/**
* @brief 计算斜率截距
* @param uint8 start                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界
* @param float *slope_rate          输入斜率地址
* @param float *intercept           输入截距地址
*  @see CTest       calculate_s_i(start, end, Findline.rightline, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, int16 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;
    uint16 xsum = 0, ysum = 0;
    float y_average, x_average;

    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }

    //计算各个平均数
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);

    }

    /*计算斜率*/
    *slope_rate = Slope_Calculate(start, end, border);//斜率
    *intercept = y_average - (*slope_rate)*x_average;//截距
}


void cross_fill(uint8(*image)[image_w], uint16 total_num_l, uint16 total_num_r, uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
    uint16 i;
    uint16 break_num_l_up = 0;
    uint16 break_num_r_up = 0;
    uint16 break_num_l_down = 0;
    uint16 break_num_r_down = 0;
    uint8 start, end;
    float slope_l_rate = 0, intercept_l = 0,slope_r_rate = 0, intercept_r = 0;
    for (i = 1; i < total_num_l; i++)
    {
        if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 4 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
        {
            break_num_l_up = points_l[i][1];//传递y坐标
            break;
        }
    }
    for (i = 1; i < total_num_r; i++)
    {
        if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 4 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
        {
            break_num_r_up = points_r[i][1];//传递y坐标
            break;
        }
    }

    for (i = 1; i < total_num_l; i++)
    {
        if (dir_l[i - 1] == 2 && dir_l[i] == 2 && dir_l[i + 3] == 2 && dir_l[i + 5] == 4 && dir_l[i + 7] == 4)
        {
            break_num_l_down = points_l[i][1];//传递y坐标
            break;
        }
    }

    for (i = 1; i < total_num_r; i++)
    {
        if (dir_r[i - 1] == 2 && dir_r[i] == 2 && dir_r[i + 3] == 2 && dir_r[i + 5] == 4 && dir_r[i + 7] == 4)
        {
            break_num_r_down = points_r[i][1];//传递y坐标
            break;
        }
    }
    //入十字
    if (break_num_l_up&&break_num_r_up&&break_num_l_down&&break_num_r_down/*&& left_duan==1&& right_duan==1*/)//两边生长方向都符合条件
    {
                start = break_num_l_down +5;
                end = break_num_l_down +9;
                end = limit_a_b(end, 0, image_h);
                calculate_s_i(start, end, Findline.leftline, &slope_l_rate, &intercept_l);
                //printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
                for (i = break_num_l_up - 3; i < break_num_l_down + 3; i++)
                {
                    Findline.leftline[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
                    Findline.leftline[i] = limit_a_b(Findline.leftline[i], border_min, border_max);//限幅
                }

                //计算斜率
                start = break_num_r_down +5;//起点
                end = break_num_r_down +9;//终点
                end = limit_a_b(end, 0, image_h);//限幅
                calculate_s_i(start, end, Findline.rightline, &slope_r_rate, &intercept_r);
                //printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
                for (i = break_num_r_up -3; i < break_num_r_down +3; i++)
                {
                    Findline.rightline[i] = slope_r_rate * (i)+intercept_r;
                    Findline.rightline[i] = limit_a_b(Findline.rightline[i], border_min, border_max);
                }    }
    //出十字
    if (break_num_l_up&&break_num_r_up&&image[image_h - 5][4] && image[image_h - 5][image_w - 4])//两边生长方向都符合条件
    {
        //计算斜率
        start = break_num_l_up - 15;
        start = limit_a_b(start, 0, image_h);
        end = break_num_l_up - 5;
        calculate_s_i(start, end, Findline.leftline, &slope_l_rate, &intercept_l);
        //printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
        for (i = break_num_l_up - 5; i < image_h - 1; i++)
        {
            Findline.leftline[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            Findline.leftline[i] = limit_a_b(Findline.leftline[i], border_min, border_max);//限幅
        }

        //计算斜率
        start = break_num_r_up - 15;//起点
        start = limit_a_b(start, 0, image_h);//限幅
        end = break_num_r_up - 5;//终点
        calculate_s_i(start, end, Findline.rightline, &slope_l_rate, &intercept_l);
        //printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
        for (i = break_num_r_up - 5; i < image_h - 1; i++)
        {
            Findline.rightline[i] = slope_l_rate * (i)+intercept_l;
            Findline.rightline[i] = limit_a_b(Findline.rightline[i], border_min, border_max);
        }


    }

}


void findline(void)
{
//    for(uint8 i=0;i<119;i++)
//    {
//        Findline.leftline[i]=0;
//        Findline.rightline[i]=159;
//    }

    uint8 hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
    /*这是离线调试用的*/
//    Get_image(bin_image);
    turn_to_bin();
    /*提取赛道边界*/
    image_filter(bin_image);//滤波
    image_draw_rectan(bin_image);//预处理
    //清零
    data_stastics_l = 0;
    data_stastics_r = 0;
    if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
    {
        //printf("正在开始八邻域\n");
        search_l_r((uint16)USE_num, &bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        //printf("八邻域已结束\n");
        // 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
        get_left(data_stastics_l);
        get_right(data_stastics_r);
    }
    zhidaopanduan();  // 直道判断
    youdaopanduan();  // 右直线
    zuodaopanduan();  // 左直线
    duanpanduan(35,70);
    findp ();
    //drawp();
    Huandaochuli_left();
    Huandaochuli_right();
    //banmaxian1(20,70);
    cross_fill(bin_image, data_stastics_l, data_stastics_r, dir_l, dir_r, &points_l, &points_r);
    zhidaopanduan_jiasu();
    banmaxian2();
    Continuity_change(100,20);
    uint16 i;
    for (i = hightest; i < image_h-1; i++)
        {
            Findline.midline[i] = (Findline.leftline[i] + Findline.rightline[i])/2;//求中线
            //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
            Findline.load_width[i]=Findline.rightline[i]-Findline.leftline[i];
        }
   // EmergencyStop();

}


void findline_left_right(void)  //根据图像寻找中线
{

    int16 i = 0, j = 0;//i是行 j是列
    //参数初始化
    lose_line = 0 ;
    Findline.midline[119] = 80;
    Findline.leftline[119] = 0;
    Findline.rightline[119] = 159;


    Findline.leftstartpoint = 0;
    Findline.rightstartpoint = 0;

    Findline.l_lose=0;
    Findline.r_lose=0;

    turn_to_bin();
    /*提取赛道边界*/
    image_filter(bin_image);//滤波
    image_draw_rectan(bin_image);//预处理
    //普通巡线
    {
        for(i = 118; i > 0; i--)
        {
            Findline.midline[i] = 0;
            Findline.leftline[i] = 0;
            Findline.rightline[i] = 159;
            Findline.leftlineflag[i] = 0;
            Findline.rightlineflag[i] = 0;
            Findline.endline = 0;
//            Findp.rightup[0]=0;
//                Findp.leftup[0]=0;
//                Findp.rightup[1]=0;
//                Findp.leftup[1]=0;
            //寻找左跳变点
            if(Findline.midline[i + 1] > 80)j = Findline.midline[i + 1] > 155 ? 155 : Findline.midline[i + 1];
            else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];  //先找到中间位置，从中间向左边开始找
            for(; j > 3; j--)
            {
                if(bin_image[i][j] == 255 && bin_image[i][j - 1] == 255 && bin_image[i][j - 2] == 0 && bin_image[i][j - 3] == 0)
                {
                    if(Findline.leftstartpoint == 0)
                        Findline.leftstartpoint = i;
                    Findline.leftline[i] = j - 1;
                    Findline.leftlineflag[i] = 1;
                    break;
                }
            }
            //寻找右跳变点
            if(Findline.midline[i + 1] > 80)j = Findline.midline[i + 1] > 155 ? 155 : Findline.midline[i + 1];
            else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];
            for(j = Findline.midline[i + 1]; j < 156; j++)
            {
                if(bin_image[i][j] == 255 && bin_image[i][j + 1] == 255 && bin_image[i][j + 2] == 0 && bin_image[i][j + 3] == 0)
                {
                    if(Findline.rightstartpoint == 0)
                        Findline.rightstartpoint = i;
                    Findline.rightline[i] = j + 1;
                    Findline.rightlineflag[i] = 1;
                    break;
                }
            }


            Findline.midline[i] = (Findline.leftline[i] + Findline.rightline[i]) / 2;


            //load_width[i] = -Findline.leftline[i] + Findline.rightline[i];


            //边线都没找到
            if((Findline.leftlineflag[i] == 0) && (Findline.rightlineflag[i] == 0))
            {
                lose_line++;
                Findline.midline[i] = Findline.midline[i + 1];

            }
            else if(lose_line > 0)
                lose_line--;

            if(Findline.leftlineflag[i] == 0)
                Findline.r_lose++;

            if(Findline.rightlineflag[i] == 0)
                Findline.l_lose++;

            //找顶点
            if(Findline.midline[i + 1] > 80)j = Findline.midline[i + 1] > 155 ? 155 : Findline.midline[i + 1];
            else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];

            if( bin_image[i - 1][j] == 0 && (bin_image[i - 1][j - 2] == 0 && bin_image[i - 1][j + 2] == 0)  &&  bin_image[i - 2][j] == 0 && (bin_image[i - 2][j - 2] == 0 && bin_image[i - 2][j + 2] == 0))
            {
                Findline.endline = i;
                break;
            }

        }
    }
    //EmergencyStop();
    //划线
}


void lostupdown(void)
{
    int16 i=0,j=0;
    for(i = 118; i > 0; i--)
    {
        //找顶点
        if(Findline.midline[i + 1] > 80)j = Findline.midline[i + 1] > 155 ? 155 : Findline.midline[i + 1];
        else j = Findline.midline[i + 1] < 4 ? 4 : Findline.midline[i + 1];

        if( bin_image[i - 1][j] == 0 && (bin_image[i - 1][j - 2] == 0 && bin_image[i - 1][j + 2] == 0)  &&  bin_image[i - 2][j] == 0 && (bin_image[i - 2][j - 2] == 0 && bin_image[i - 2][j + 2] == 0))
        {
            Findline.endline = i;
            break;
        }
    }
    for(i=5;i<118;i++)
    {
        if(Findline.leftline[i]==0)
        {
            Findline.l_lose++;
        }
        if(Findline.rightline[i]==0)
        {
            Findline.r_lose++;
        }
        if(Findline.rightline[i]==0&&Findline.leftline[i]==0)
        {
            Findline.loseflag++;
        }
    }
}
// 丢线停车
void EmergencyStop(void)
{
    //思路：丢线时近处的几行大部分都是黑色的，只要这部分的黑色像素累计大于一个阈值就认为丢线
    int16 i = 0, j = 0, count_black = 0 ;

    for(i = 90; i > 80; i--)
    {
        for(j = 130; j >= 30; j--)
        {
            if(bin_image[i][j] == 0)
            {
                count_black = count_black + 1;
            }
        }
    }
    if(count_black > 300 && stop_flag==0)  //300
    {
        Findline.loseflag = 1;
        stop();

        //bee_time = 200;
    }
}


void Huandaochuli_left()
{
    if(Findp.leftdown_flag==1 && Findp.lcenter_flag==1 && left_hdflag==0 && right_trueshortflag==1 &&left_trueshortflag==0 && left_duan==1 && right_duan==0 &&banmaxian_biaozhiwei==0&&e_distance>100/*&&(leftduan>50&&rightduan<35)*/)
    {

        left_hdflag=1;
        //speed.Stan = 1.6;
        bee_time = 200;
    }
    if(Findp.leftdown_flag==0 && Findp.lcenter_flag==1 && left_hdflag==1 && right_trueshortflag==1 && left_trueshortflag==0)
    {
        left_hdflag=2;
        gyro.PitchAngle_Integral=0;
        ang_in=gyro.PitchAngle_Integral;
    }
    if(Findp.leftup_flag==1&& left_hdflag==2/*&&(Findp.leftup[1]>Findp.lcenter[1])*/)
    {
        //connect_line(Findp.leftup[0],Findp.leftup[1],156,110,1);
        left_hdflag=3;
    }
    if(Findp.leftup[1]>45 && left_hdflag==3)
    {
        //find_right_line_new();
        //speed.Stan = speed_huandao;
        left_hdflag=4;
    }
    if(Findp.leftup_flag==0 && left_hdflag==4 &&ang_in-gyro.PitchAngle_Integral>10)
    {
        left_hdflag=5;
    }

    if(left_hdflag==5 && Findp.rightdown_flag==1&&ang_in-gyro.PitchAngle_Integral>45)
    {
        left_hdflag=6;
    }

      if(left_hdflag==6&&ang_in-gyro.PitchAngle_Integral>64)
      {
          left_hdflag=7;
          //speed.Stan = speed_normal;
          bee_time = 200;
      }//出环 陀螺仪积分

      if(left_hdflag==7&&Findp.leftup_flag==1)
      {
          left_hdflag=8;
      }
      if(left_hdflag==8&&Findp.leftup_flag==0)
      {
          left_hdflag=9;
          //speed.Stan = speed_normal;
      }
        switch(left_hdflag)
        {
            case 1:
                //connect_line(Findp.lcenter[0],Findp.lcenter[1],5,115,0);
                connect_line(55,30,5,115,0);
                break;
            case 2:
                connect_line(Findp.lcenter[0],Findp.lcenter[1]+10,5,115,0);
                //connect_line(45,30,5,115,0);
                break;
            case 3:
                connect_line(Findp.leftup[0],Findp.leftup[1],Findline.rightline[Findp.leftup[1]+50],Findp.leftup[1]+70,1);
                break;
            case 4:
                //speed.Stan = speed_huandao;
                connect_line(5,20,110,60,1);
                for(int i=0;i<115;i++)
                {
                    Findline.leftline[i]=4;
                }
                break;
            case 5:
                break;
            case 6:
                connect_line(29,20,119,90,1);//出环线
                break;
            case 8:
                connect_line(Findp.leftup[0],Findp.leftup[1],5,115,0);
                break;
            case 9:

//                if(hd_state==0)
//                {
//                    hd_state++;
                    left_hdflag=0;
//                }
//                else if (hd_state==2)
//                {
//                    left_hdflag=10;
//                    //speed.Stan = speed_normal;
//                }
                break;

        }
}


void Huandaochuli_right()
{   
    if(Findp.rightdown_flag==1 && Findp.rcenter_flag==1 && right_hdflag==0 &&right_trueshortflag==0 &&left_trueshortflag==1 && left_duan==0 && right_duan==1&&banmaxian_biaozhiwei==0 &&e_distance>50/*&&(leftduan>50&&rightduan<35)*/)
    {

        right_hdflag=1;
        //speed.Stan = 1.95;
        bee_time = 200;
    }
    if(Findp.rightdown_flag==0 && Findp.rcenter_flag==1 && right_hdflag==1 /*&& right_trueshortflag==0*/ && left_trueshortflag==1)
    {
        right_hdflag=2;
        gyro.PitchAngle_Integral=0;
        ang_in=gyro.PitchAngle_Integral;
    }
    if(Findp.rightup_flag==1&& right_hdflag==2/*&&(Findp.rightup[1]>Findp.rcenter[1])*/)
    {
        //connect_line(Findp.leftup[0],Findp.leftup[1],156,110,1);
        right_hdflag=3;
    }
    if(Findp.rightup[1]>45 && right_hdflag==3)//40
    {
        //find_right_line_new();
        //speed.Stan = speed_huandao;
        right_hdflag=4;
    }
    if(Findp.rightup_flag==0 && right_hdflag==4 &&ang_in-gyro.PitchAngle_Integral<-10)
    {
        right_hdflag=5;
    }
    if(right_hdflag==5 && Findp.leftdown_flag==1&&ang_in-gyro.PitchAngle_Integral<-45)
    {
        right_hdflag=6;
    }
    if(right_hdflag==6&&ang_in-gyro.PitchAngle_Integral<-64)
      {
          right_hdflag=7;
          //speed.Stan = 2.5;
          bee_time = 200;
      }//出环 陀螺仪积分

      if(right_hdflag==7&&Findp.rightup_flag==1)
      {
          right_hdflag=8;
      }
      if(right_hdflag==8&&Findp.rightup_flag==0)
      {
          right_hdflag=9;
          //speed.Stan = speed_normal;
      }
        switch(right_hdflag)
        {
            case 1:
//                connect_line(15,60,155,115,1);
                connect_line(105,30,155,115,1);
//                for(int16 i=10;i<120;i++)
//                {
//                    Findline.midline[i]=Findline.leftline[i]+Findline.load_width[i]/2;
//                }
// qianzhan=65;
                break;
            case 2:
                //speed.Stan = 0.3;
//                connect_line(115,30,155,115,1);
                connect_line(Findp.rcenter[0],Findp.rcenter[1]+10,155,115,1);
                break;
            case 3:
                connect_line(Findp.rightup[0],Findp.rightup[1],Findline.leftline[Findp.rightup[1]+50],Findp.rightup[1]+70,0);
//                if(Findp.rcenter[1]<Findp.rightup[1])
//                {
//                    connect_line(Findp.rcenter[0],Findp.rcenter[1],155,115,1);
//                }
                break;
            case 4:
                connect_line(155,20,50,110,0);
                for(int i=0;i<115;i++)
                {
                    Findline.rightline[i]=155;
                }
                break;
            case 5:
//                connect_line(135,5,1,110,0);
//                for(int i=0;i<115;i++)
//                {
//                    Findline.rightline[i]=0;
//                }
                break;
            case 6:
                connect_line(130,20,50,90,0);//connect_line(5,20,119,90,1);  connect_line(154,20,40,90,0)
                break;
            case 8:
                connect_line(Findp.rightup[0],Findp.rightup[1],154,115,1);
                break;
            case 9:
                //  Lengthen_left_line(Findp.leftup[1],100);
//                if(hd_right_state==0)
//                {
//                    hd_state++;
                    right_hdflag=0;
//                }
//                else if (hd_right_state==2)
//                {
//                    right_hdflag=10;
//                    //speed.Stan = speed_normal;
//                }
                    //qianzhan=60;
                break;
        }
}

//连续性判断
void Continuity_change(int16 start, int16 end) {
    int16 i;
    int16 t;

    if (start < end) { //反了就互换一下
        t = start;
        start = end;
        end = t;
    }
    for (i = start; i >= end; i--) {
        if (abs(Findline.rightline[i] - Findline.rightline[i - 1]) >= 5) { //阈值是3，可改
            Findline.r_lose = i;
            break;
        }
        if (abs(Findline.leftline[i] - Findline.leftline[i - 1]) >= 5) { //阈值是3，可改
            Findline.l_lose = i;
            break;
        }
    }
}



//角点寻找demo
void findp (void)
{
    //下角点和中拐点数组初始化，0纵坐标（行） 1横坐标（列）
    Findp.leftdown[0] = 0;
    Findp.rightdown[0] = 0;
    Findp.leftdown[1] = 0;
    Findp.rightdown[1] = 0;
    Findp.leftup[0] = 0;
    Findp.rightup[0] = 0;
    Findp.leftup[1] = 0;
    Findp.rightup[1] = 0;
    Findp.lcenter[0] = 0;
    Findp.lcenter[1] = 0;
    Findp.rcenter[0] = 0;
    Findp.rcenter[1] = 0;

    //角点拐点标志
    Findp.leftdown_flag = 0;
    Findp.leftup_flag = 0;
    Findp.rightdown_flag = 0;
    Findp.rightup_flag = 0;
    Findp.lcenter_flag = 0;
    Findp.rcenter_flag = 0;
    int16 i;
    for ( i = 118; i >40; i--)
    {

        //右下
        if (abs(Findline.rightline[i] - Findline.rightline[i + 1]) <= 5 && //角点的阈值可以更改
                abs(Findline.rightline[i + 1] - Findline.rightline[i + 2]) <= 5 &&
                abs(Findline.rightline[i + 2] - Findline.rightline[i + 3]) <= 5 &&
                (Findline.rightline[i] - Findline.rightline[i - 2]) <= -5 &&
                (Findline.rightline[i] - Findline.rightline[i - 3]) <= -5)
        {
            Findp.rightdown[0] = Findline.rightline[i];
            Findp.rightdown[1] = i;
            Findp.rightdown_flag = 1;
        }
        //左下
        if (abs(Findline.leftline[i] - Findline.leftline[i + 1]) <= 5 && //角点的阈值可以更改
                abs(Findline.leftline[i + 1] - Findline.leftline[i + 2]) <= 5 &&
                abs(Findline.leftline[i + 2] - Findline.leftline[i + 3]) <= 5 &&
                (Findline.leftline[i] - Findline.leftline[i - 2]) >= 3 &&
                (Findline.leftline[i] - Findline.leftline[i - 3]) >= 5) {
            Findp.leftdown[0] = Findline.leftline[i];
            Findp.leftdown[1] = i;
            Findp.leftdown_flag = 1;

        }
    }
    for ( i = 70; i >15; i--) //左上
    {
        /*左上:角点向上几个横坐标相差不大，向下几个横坐标相差较大，
             阈值可修改*/
        if (abs(Findline.leftline[i] - Findline.leftline[i - 1] <= 3) &&
                abs(Findline.leftline[i - 1] - Findline.leftline[i - 2] <= 3) &&
                abs(Findline.leftline[i - 2] - Findline.leftline[i - 3] <= 3) &&
                (Findline.leftline[i] - Findline.leftline[i + 1]) >= 8 &&
                (Findline.leftline[i] - Findline.leftline[i+2]) >= 10&&
                (Findline.leftline[i] - Findline.leftline[i+3]) >= 15)
        {
            Findp.leftup[0] = Findline.leftline[i];
            Findp.leftup[1] = i;
            Findp.leftup_flag = 1;
        }
        //右上：同理
        if (    abs(Findline.rightline[i] - Findline.rightline[i - 1]) <= 3 && //上面两行位置差不多
                abs(Findline.rightline[i - 1] - Findline.rightline[i - 2]) <= 3 &&
                abs(Findline.rightline[i - 2] - Findline.rightline[i - 3]) <= 3 &&
                (Findline.rightline[i] - Findline.rightline[i + 1]) <= -8 &&
                (Findline.rightline[i] - Findline.rightline[i + 2]) <= -10 &&
                (Findline.rightline[i] - Findline.rightline[i + 3]) <= -15
        ) {
            Findp.rightup[0] = Findline.rightline[i];
            Findp.rightup[1] = i;
            Findp.rightup_flag = 1;
        }
    }

    for( i=22 ;i<80;i++)
    {
        //左中拐点，环岛用，数组里的特点是在此点之前先增大，此点之后，减小
        if(Findline.leftline[i]==Findline.leftline[i+5]&&Findline.leftline[i]==Findline.leftline[i-5]&&
                Findline.leftline[i]==Findline.leftline[i+4]&&Findline.leftline[i]==Findline.leftline[i-4]&&
                Findline.leftline[i]==Findline.leftline[i+3]&&Findline.leftline[i]==Findline.leftline[i-3]&&
                Findline.leftline[i]==Findline.leftline[i+2]&&Findline.leftline[i]==Findline.leftline[i-2]&&
                Findline.leftline[i]==Findline.leftline[i+1]&&Findline.leftline[i]==Findline.leftline[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if
        (Findline.leftline[i]>=Findline.leftline[i+5]&&Findline.leftline[i]>=Findline.leftline[i-5]&&
                Findline.leftline[i]>=Findline.leftline[i+4]&&Findline.leftline[i]>=Findline.leftline[i-4]&&
                Findline.leftline[i]>=Findline.leftline[i+3]&&Findline.leftline[i]>=Findline.leftline[i-3]&&
                Findline.leftline[i]>=Findline.leftline[i+2]&&Findline.leftline[i]>=Findline.leftline[i-2]&&
                Findline.leftline[i]>=Findline.leftline[i+1]&&Findline.leftline[i]>=Findline.leftline[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            if(Findline.leftline[i]<=80 && i>15)
            {
            Findp.lcenter[0] = Findline.leftline[i];
            Findp.lcenter[1] = i;
            Findp.lcenter_flag = 1;
            break;
            }
        }
    }

    for( i=22;i<80;i++)
    {
        //右中拐点，环岛用，数组里的特点是在此点之前先减小，此点之后，增大

        if(Findline.rightline[i]==Findline.rightline[i+5]&&Findline.rightline[i]==Findline.rightline[i-5]&&
                Findline.rightline[i]==Findline.rightline[i+4]&&Findline.rightline[i]==Findline.rightline[i-4]&&
                Findline.rightline[i]==Findline.rightline[i+3]&&Findline.rightline[i]==Findline.rightline[i-3]&&
                Findline.rightline[i]==Findline.rightline[i+2]&&Findline.rightline[i]==Findline.rightline[i-2]&&
                Findline.rightline[i]==Findline.rightline[i+1]&&Findline.rightline[i]==Findline.rightline[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Findline.rightline[i]<=Findline.rightline[i+5]&&Findline.rightline[i]<=Findline.rightline[i-5]&&
                Findline.rightline[i]<=Findline.rightline[i+4]&&Findline.rightline[i]<=Findline.rightline[i-4]&&
                Findline.rightline[i]<=Findline.rightline[i+3]&&Findline.rightline[i]<=Findline.rightline[i-3]&&
                Findline.rightline[i]<=Findline.rightline[i+2]&&Findline.rightline[i]<=Findline.rightline[i-2]&&
                Findline.rightline[i]<=Findline.rightline[i+1]&&Findline.rightline[i]<=Findline.rightline[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            if(Findline.rightline[i]>=80 && i>15)
            {
            Findp.rcenter[0] = Findline.rightline[i];
            Findp.rcenter[1] = i;
            Findp.rcenter_flag = 1;
            break;}
        }
    }

}
//右连线函数(两点横纵坐标)其实改下下面的数组就可以左右线共用，可以多设一个状态输入，0左1右；
void connect_line(int16 x1, int16 y1, int16 x2, int16 y2, int type) {

    int16 i,  swap;
    float k;
    if (y1 > y2) {
        swap = x1;
        x1 = x2;
        x2 = swap;
        swap = y1;
        y1 = y2;
        y2 = swap;
    }
    else {
        k = (float)(x2 - x1) / (y2 - y1);
        for (i = y1; i <= y2; i++) {
            if (type == 1)
                Findline.rightline[i] =k*(i-y1)+x1 ; //   ov7725_uart_image_dec[i][(short)(x1+(i-y1)*k)]=0;
            else
                Findline.leftline[i] =k*(i-y1)+x1  ;
        }

    }
}

//右线斜率补线，延长起始行数，延长到某行，从起始点向上找3个点，算出斜率，向下延长，直至结束点
void Lengthen_right_line(int16 start, int16 end)
{
    int16 i, t;
    float k = 0;
    if (end < start) { //++访问，坐标互换
        t = end;
        end = start;
        start = t;
    }
//这里有bug，下方循环++循环，只进行y的互换，但是没有进行x的互换
//建议进行判断，根据a1和a2的大小关系，决定++或者--访问
    /*if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线（不确定有没有用）
    {
        connect_line(Findline.rightline[start],start,Findline.rightline[end],end);
    }
    else
    {*/
    k = (float)(Findline.rightline[start] - Findline.rightline[start - 4]) / 5.0; //这里的k是1/斜率
    for (i = start; i <= end; i++) {
        Findline.rightline[i] = (int16)(i - start) * k + Findline.rightline[start]; //(x=(y-y1)*k+x1),点斜式变形
        if (Findline.rightline[i] >= 158) {
            Findline.rightline[i] = 158;
        } else if (Findline.rightline[i] <= 1) {
            Findline.rightline[i] = 1;
        }
    }
    //}
}
//左边线斜率补线函数，同理
void Lengthen_left_line(int16 start, int16 end) {
    int16 i, t;
    float k = 0;

    if (start > end) { //--操作，start需要大
        t = start;
        start = end;
        end = t;
    }
    k = (float)(Findline.leftline[start] - Findline.leftline[start-4]) / 3.0;

    for (i = start; i <= end; i++) {
        Findline.leftline[i] = (int16)((i - start) / k + Findline.leftline[start]); //(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if (Findline.leftline[i] >= 158) {
            Findline.leftline[i] = 158;
        } else if (Findline.leftline[i] <= 1) {
            Findline.leftline[i] = 1;
        }
    }
}
void right_addline(int16 start,int16 end)//出环岛补线占用
{
    int16 i, t;
    float k = 0;

    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    k = (float)(Findline.rightline[start] - Findline.rightline[start+4]) / 4.0;

    for (i = start; i <= end; i--) {
        Findline.rightline[i] = (int16)((i - start) / k + Findline.rightline[start]); //(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if (Findline.rightline[i] >= 158) {
            Findline.rightline[i] = 158;
        } else if (Findline.rightline[i] <= 1) {
            Findline.rightline[i] = 1;
        }
    }
}



//对两段图线进行拟合曲线 1为左线 0为中线 2为右线
void advanced_regression(uint8 type, uint8 startline1, uint8 endline1, uint8 startline2, uint8 endline2)
{
    uint8 i = 0;
    uint8 sumlines1 = endline1 - startline1;
    uint8 sumlines2 = endline2 - startline2;
    uint16 sumX = 0;
    uint16 sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)  //拟合中线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += Findline.midline[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += Findline.midline[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (Findline.midline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (Findline.midline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else
        parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += Findline.leftline[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += Findline.leftline[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (Findline.leftline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (Findline.leftline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += Findline.rightline[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += Findline.rightline[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (Findline.rightline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (Findline.rightline[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}


float FMy_Abs(float a, float b)//求两数之差绝对值的浮点数
{

    if ((a - b) > 0)
        return ((float)(a - b));
    else return ((float)(b - a));
}

void zhidaopanduan(void)//可能需要修改参数的有第一段和第二段的斜线采样取值
{
    float kfirst,ksecond,kerror;
    advanced_regression(0, 10, 25, 45, 55);
    kfirst = parameterB;

    advanced_regression(0, 60, 64, 68, 72);
    ksecond = parameterB;

    if (FMy_Abs(0, kfirst) >= 0.35 || FMy_Abs(0, ksecond) >= 0.35) { shortwrong = 1; }//当前后两段斜率都极大时，此时方向大概是垂直于车子前进方向，判断为弯道
    else {shortwrong=0;}
    kerror = FMy_Abs(kfirst, ksecond);

    if (kerror <= 0.1) shortflag = shortflag + 1;//当有较小的斜率偏差时，给shortflag加1实现直道初步判断
    if (kerror >  0.1) { shortflag = 0; }

    if (shortwrong == 1) { shortflag = 0; }//弯道将shortflag清零

    if (shortflag >= 2) { shortflag = 2; trueshortflag = 1; }//当检测到连续2帧有，赋值直道标志位并置2继续计数
    if (shortflag < 2) trueshortflag = 0;//若没有则将直道标志位清零
}

void youdaopanduan(void)
{
    float kfirst,ksecond,kerror;
    advanced_regression(2, 15, 30, 35, 45);
    kfirst = parameterB;

    advanced_regression(2, 50, 55, 75, 85);
    ksecond = parameterB;

    //if (FMy_Abs(0, kfirst) >= 0.7 || FMy_Abs(0, ksecond) >= 0.7) { right_shortwrong = 1; }//当前后两段斜率都极大时，此时方向大概是垂直于车子前进方向，判断为弯道
    kerror = FMy_Abs(kfirst, ksecond);

    if (kerror <= 0.1) right_shortflag = right_shortflag + 1;//当有较小的斜率偏差时，给shortflag加1实现右线初步判断
    if (kerror > 0.1) { right_shortflag = 0; }

    if (right_shortwrong == 1) { right_shortflag = 0; }//弯道将shortflag清零

    if (right_shortflag >= 2) { right_shortflag = 2; right_trueshortflag = 1; }//当检测到连续2帧有，赋值直道标志位并置2继续计数
    if (right_shortflag < 2) right_trueshortflag = 0;//若没有则将直道标志位清零

}

void zuodaopanduan(void)
{
    float kfirst,ksecond,kerror;
    advanced_regression(1, 15, 30, 35, 45);
    kfirst = parameterB;

    advanced_regression(1, 50, 55, 75, 85);
    ksecond = parameterB;

    //if (FMy_Abs(0, kfirst) >= 0.7 || FMy_Abs(0, ksecond) >= 0.7) { right_shortwrong = 1; }//当前后两段斜率都极大时，此时方向大概是垂直于车子前进方向，判断为弯道
    kerror = FMy_Abs(kfirst, ksecond);

    if (kerror <= 0.1) left_shortflag = left_shortflag + 1;//当有较小的斜率偏差时，给shortflag加1实现右线初步判断
    if (kerror > 0.1) { left_shortflag = 0; }

    if (left_shortwrong == 1) { left_shortflag = 0; }//弯道将shortflag清零

    if (left_shortflag >= 2) { left_shortflag = 2; left_trueshortflag = 1; }//当检测到连续2帧有，赋值直道标志位并置2继续计数
    if (left_shortflag < 2) left_trueshortflag = 0;//若没有则将直道标志位清零


    }

void duanpanduan(uint8 startline, uint8 endline)
{   uint8 i=0;
    uint8 left_blank=0,right_blank=0;
    for (i = startline; i < endline; i++)
{
    if(Findline.leftline[i]<=5)
    {
        left_blank++;
    }
    if(Findline.rightline[i]>=154)
    {
        right_blank++;
    }
}
    if(left_blank >= 10)
    {left_duan=1;}
    else left_duan=0;

    if(right_blank >= 10)
    {right_duan=1;}
    else right_duan=0;
}


/*-------------------------------------------------------------------------------------------------------------------
    名称：左下角点检测
    输入：起始行，终止行
    返回值：返回角点所在的行数，找不到返回0
    注：角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Left_Down_Point(int16 start, int16 end) { //找左下角点，返回值是角点所在的行数
    int16 i, t;
    int16 left_down = 0;
    //目前好像没有丢线数先不加，加了稳定一点吧
//    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
//       return left_down;
    if (start < end) { //--访问，要保证start>end
        t = start;
        start = end;
        end = t;
    }
    if (start >= 115) //下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃；另一方面，当判断第i行时，会访问到i+3和i-4行，防止越界
        start = 115;
    if (end <= 5)
        end = 5;
    for (i = start; i >= end; i--) {
        if (left_down == 0 && //只找第一个符合条件的点
                abs(Findline.leftline[i] - Findline.leftline[i + 1]) <= 5 && //角点的阈值可以更改
                abs(Findline.leftline[i + 1] - Findline.leftline[i + 2]) <= 5 &&
                abs(Findline.leftline[i + 2] - Findline.leftline[i + 3]) <= 5 &&
                (Findline.leftline[i] - Findline.leftline[i - 2]) >= 5 &&
                (Findline.leftline[i] - Findline.leftline[i - 3]) >= 10 &&
                (Findline.leftline[i] - Findline.leftline[i - 4]) >= 10) {
            left_down = i;
            break;
        }
    }
    return left_down;
}

/*-------------------------------------------------------------------------------------------------------------------
    名称：左上角点检测
    输入：起始行，终止行
    返回值：返回角点所在的行数，找不到返回0
    注：角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Left_Up_Point(int16 start, int16 end) { //找四个角点，返回值是角点所在的行数
    int16 i, t;
    int16 left_up_line = 0;
//    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
//       return left_up_line;
    if (start < end) {
        t = start;
        start = end;
        end = t;
    }
    if (end <= 5) //舍弃部分点，防止数组越界
        end = 5;
    if (start >= 115)
        start = 115;
    for (i = start; i >= end; i--) {
        if (left_up_line == 0 && //只找第一个符合条件的点
                abs(Findline.leftline[i] - Findline.leftline[i - 1]) <= 5 &&
                abs(Findline.leftline[i - 1] - Findline.leftline[i - 2]) <= 5 &&
                abs(Findline.leftline[i - 2] - Findline.leftline[i - 3]) <= 5 &&
                (Findline.leftline[i] - Findline.leftline[i + 2]) >= 8 &&
                (Findline.leftline[i] - Findline.leftline[i + 3]) >= 15 &&
                (Findline.leftline[i] - Findline.leftline[i + 4]) >= 15) {
            left_up_line = i+7; //获取行数即可
            break;
        }
    }
    return left_up_line;//如果是MT9V03X_H-1，说明没有这么个拐点
}

/*-------------------------------------------------------------------------------------------------------------------
    名称：右下角点检测
    输入：起始行，终止行
    返回值：返回角点所在的行数，找不到返回0
    注：角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Right_Down_Point(int16 start, int16 end) { //找四个角点，返回值是角点所在的行数
    int16 i, t;
    int16 right_down_line = 0;
//    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
//        return right_down_line;
    if (start < end) {
        t = start;
        start = end;
        end = t;
    }
    if (start >= 115) //下面5行数据不稳定，不能作为边界点来判断，舍弃
        start =115;
    if (end <= 5)
        end = 5;
    for (i = start; i >= end; i--) {
        if (right_down_line == 0 && //只找第一个符合条件的点
                abs(Findline.rightline[i] - Findline.rightline[i + 1]) <= 5 && //角点的阈值可以更改
                abs(Findline.rightline[i + 1] - Findline.rightline[i + 2]) <= 5 &&
                abs(Findline.rightline[i + 2] - Findline.rightline[i + 3]) <= 5 &&
                (Findline.rightline[i] - Findline.rightline[i - 2]) <= -5 &&
                (Findline.rightline[i] - Findline.rightline[i - 3]) <= -10 &&
                (Findline.rightline[i] - Findline.rightline[i - 4]) <= -10) {
            right_down_line = i; //获取行数即可
            break;
        }
    }
    return right_down_line;
}

/*-------------------------------------------------------------------------------------------------------------------
    名称：右上角点检测
    输入：起始行，终止行
    返回值：返回角点所在的行数，找不到返回0
    注：角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Right_Up_Point(int16 start, int16 end) { //找四个角点，返回值是角点所在的行数
    int16 i, t;
    int16 right_up_line = 0;
//    if(Right_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有拐点判断的意义
//        return right_up_line;
    if (start < end) {
        t = start;
        start = end;
        end = t;
    }
    if (end <= 5)
        end = 5;
    if (start >= 115)
        start = 115;
    for (i = start; i >= end; i--) {
        if (right_up_line == 0 && //只找第一个符合条件的点
                abs(Findline.rightline[i] - Findline.rightline[i - 1]) <= 5 && //上面两行位置差不多
                abs(Findline.rightline[i - 1] - Findline.rightline[i - 2]) <= 5 &&
                abs(Findline.rightline[i - 2] - Findline.rightline[i - 3]) <= 5 &&
                (Findline.rightline[i] - Findline.rightline[i + 2]) <= -8 &&
                (Findline.rightline[i] - Findline.rightline[i + 3]) <= -15 &&
                (Findline.rightline[i] - Findline.rightline[i + 4]) <= -15) {
            right_up_line = i+7; //获取行数即可
            break;
        }
    }
    return right_up_line;
}

/*-------------------------------------------------------------------------------------------------------------------
    名称：单调性突变检测
    输入：起始点，终止行
    返回值：点所在的行数，找不到返回0
    注：前5后5它最大（最小），那此点就是圆弧中点//出环角点
-------------------------------------------------------------------------------------------------------------------*/
int16 Monotonicity_Change_Left(int16 start, int16 end) { //单调性改变，返回值是单调性改变点所在的行数
    int16 i;
    int16 monotonicity_change = 0;
//    if(Left_Lost_Time>=0.9*MT9V03X_H)//大部分都丢线，没有单调性判断的意义
//       return monotonicity_change_line;
    if (start >= 115) //数组越界保护，在判断第i个点时
        start = 115; //要访问它前后5个点，数组两头的点要不能作为起点终点
    if (end <= 5)
        end = 5;
    if (start <= end) //递减计算，入口反了，直接返回0
        return monotonicity_change;
    for (i = start; i >= end; i--) { //会读取前5后5数据，所以前面对输入范围有要求
        if (Findline.leftline[i] == Findline.leftline[i + 5] && Findline.leftline[i] == Findline.leftline[i - 5] &&
                Findline.leftline[i] == Findline.leftline[i + 4] && Findline.leftline[i] == Findline.leftline[i - 4] &&
                Findline.leftline[i] == Findline.leftline[i + 3] && Findline.leftline[i] == Findline.leftline[i - 3] &&
                Findline.leftline[i] == Findline.leftline[i + 2] && Findline.leftline[i] == Findline.leftline[i - 2] &&
                Findline.leftline[i] == Findline.leftline[i + 1] && Findline.leftline[i] == Findline.leftline[i - 1]) {
            //一堆数据一样，显然不能作为单调转折点
            continue;
        } else if (Findline.leftline[i] >= Findline.leftline[i + 5] && Findline.leftline[i] >= Findline.leftline[i - 5] &&
                   Findline.leftline[i] >= Findline.leftline[i + 4] && Findline.leftline[i] >= Findline.leftline[i - 4] &&
                   Findline.leftline[i] >= Findline.leftline[i + 3] && Findline.leftline[i] >= Findline.leftline[i - 3] &&
                   Findline.leftline[i] >= Findline.leftline[i + 2] && Findline.leftline[i] >= Findline.leftline[i - 2] &&
                   Findline.leftline[i] >= Findline.leftline[i + 1] && Findline.leftline[i] >= Findline.leftline[i - 1]) {
            //就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            monotonicity_change = i;
            break;
        }
    }
    return monotonicity_change;
}

/*-------------------------------------------------------------------------------------------------------------------
    名称：单调性突变检测
    输入：起始点，终止行
    返回值：点所在的行数，找不到返回0
    注：前5后5它最大（最小），那此点就是圆弧中点//出环角点
-------------------------------------------------------------------------------------------------------------------*/
int16 Monotonicity_Change_Right(int16 start, int16 end) { //单调性改变，返回值是单调性改变点所在的行数
    int16 i;
    int16 monotonicity_change_line = 0;

//    if (Right_Lost_Time >= 0.9 * MT9V03X_H) //大部分都丢线，没有单调性判断的意义
//        return monotonicity_change_line;
    if (start >= 115) //数组越界保护
        start = 115;
    if (end <= 5)
        end = 5;
    if (start <= end)
        return monotonicity_change_line;
    for (i = start; i >= end; i--) { //会读取前5后5数据，所以前面对输入范围有要求
        if (Findline.rightline[i] == Findline.rightline[i + 5] && Findline.rightline[i] == Findline.rightline[i - 5] &&
                Findline.rightline[i] == Findline.rightline[i + 4] && Findline.rightline[i] == Findline.rightline[i - 4] &&
                Findline.rightline[i] == Findline.rightline[i + 3] && Findline.rightline[i] == Findline.rightline[i - 3] &&
                Findline.rightline[i] == Findline.rightline[i + 2] && Findline.rightline[i] == Findline.rightline[i - 2] &&
                Findline.rightline[i] == Findline.rightline[i + 1] && Findline.rightline[i] == Findline.rightline[i - 1]) {
            //一堆数据一样，显然不能作为单调转折点
            continue;
        } else if (Findline.rightline[i] <= Findline.rightline[i + 5] && Findline.rightline[i] <= Findline.rightline[i - 5] &&
                   Findline.rightline[i] <= Findline.rightline[i + 4] && Findline.rightline[i] <= Findline.rightline[i - 4] &&
                   Findline.rightline[i] <= Findline.rightline[i + 3] && Findline.rightline[i] <= Findline.rightline[i - 3] &&
                   Findline.rightline[i] <= Findline.rightline[i + 2] && Findline.rightline[i] <= Findline.rightline[i - 2] &&
                   Findline.rightline[i] <= Findline.rightline[i + 1] && Findline.rightline[i] <= Findline.rightline[i - 1]) {
            //就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line = i;
            break;
        }
    }
    return monotonicity_change_line;
}

void blackbox(void)
{

    if(Findp.leftdown_flag==1&&Findp.leftup_flag==1&& Findp.lcenter_flag==0  && left_trueshortflag==0&& right_trueshortflag==1 &&left_duan==0 && right_duan==0 &&e_distance>50)
    {
        connect_line(Findp.leftdown[0],Findp.leftdown[1],5,110,1);
    }
    if(Findp.rightdown_flag==1&&Findp.rightup_flag==1&& Findp.rcenter_flag==0&& right_trueshortflag==0&& left_trueshortflag==1 &&left_duan==0 && right_duan==0 &&e_distance>50)
    {
        connect_line(Findp.rightdown[0],Findp.rightdown[1],156,110,1);
    }
}

void drawp (void)
{
        //下角点和中拐点数组初始化，0纵坐标（行） 1横坐标（列）
        Findp.leftdown[0] = 0;
        Findp.rightdown[0] = 0;
        Findp.leftdown[1] = 0;
        Findp.rightdown[1] = 0;
        Findp.leftup[0] = 0;
        Findp.rightup[0] = 0;
        Findp.leftup[1] = 0;
        Findp.rightup[1] = 0;
        Findp.lcenter[0] = 0;
        Findp.lcenter[1] = 0;
        Findp.rcenter[0] = 0;
        Findp.rcenter[1] = 0;

         //角点拐点标志
        Findp.leftdown_flag = 0;
        Findp.leftup_flag = 0;
        Findp.rightdown_flag = 0;
        Findp.rightup_flag = 0;
        Findp.lcenter_flag = 0;
        Findp.rcenter_flag = 0;
        //角点赋值

        Findp.leftdown[0]=Find_Left_Down_Point(MT9V03X_H-1,60);//左下
        Findp.leftdown[1]=Findline.leftline[Findp.leftdown[0]];

        Findp.rightdown[0]=Find_Right_Down_Point(MT9V03X_H-1,60);//右下
        Findp.rightdown[1]=Findline.rightline[Findp.rightdown[0]];

        Findp.leftup[0]=Find_Left_Up_Point(90,20);//左上
        Findp.leftup[1]=Findline.leftline[ Findp.leftup[0]];

        Findp.rightup[0]=Find_Right_Up_Point(90,20);//右上
        Findp.rightup[1]=Findline.rightline[Findp.rightup[0]];

//        Findp.lcenter[0] = Monotonicity_Change_Left(100,20);//左中间圆弧或右环岛出环左角点
//        Findp.lcenter[1] = Findline.leftline[Findp.lcenter[0]];
//
//        Findp.rcenter[0] = Monotonicity_Change_Right(100,20);//右中间圆弧或左环岛出环右角点
//        Findp.rcenter[1] = Findline.rightline[Findp.rcenter[0]];

//    Find_Left_Up_Point();
//    Monotonicity_Change_Left();
//    Monotonicity_Change_Right();
}
void zhidaopanduan_jiasu(void)
{
    if((e_distance>=400&&banmaxian_biaozhiwei) || (stop_flag==1))
    {
        speed.Stan = 0;
        stop_flag=1;
        L_SpeedLoop.KP = 0.4;
        L_SpeedLoop.KI = 0.01;
        L_SpeedLoop.KD = 0.0;

        R_SpeedLoop.KP = 0.2;
        R_SpeedLoop.KI = 0.01;
        R_SpeedLoop.KD = 0.0;

        //break;
    }
    else if(stop_flag == 0&&trueshortflag ==1 /*&& left_trueshortflag==1 && right_trueshortflag==1 && left_duan==0 && right_duan==0*/)
    {
        speed.Stan =speed_zhidao;
        // fan_speed = fan_speed_zhidao;
        ServPID1.P = 1.3;//1.3 0 1
        //qianzhan = 55 ;
    }
    else if((stop_flag == 0 &&left_hdflag>=3 && left_hdflag<=9)/*||(left_hdflag>=7 && left_hdflag<=9)*/||(right_hdflag>=3&& right_hdflag<=9)/*||(right_hdflag>=1 && right_hdflag<=3)*/)
    {
        speed.Stan =speed_huandao;
        // fan_speed = fan_speed_huandao;
        ServPID1.P = 1;//1.3 0 1
//        ServPID.P = 5;
//        ServPID.D = 1.5;
    }
   /* else if(stop_flag == 0 &&left_hdflag>=3 && left_hdflag<=9)
    {
        speed.Stan =speed_normal;
        // fan_speed = fan_speed_huandao;

    }*/
    else if(stop_flag== 0)
   {
        speed.Stan = speed_normal;
        // fan_speed = fan_speed_normal;
//        ServPID1.P = 0;
        ServPID1.P = 1.8;//1.3 0 1
       // qianzhan = 60;
   }

}

