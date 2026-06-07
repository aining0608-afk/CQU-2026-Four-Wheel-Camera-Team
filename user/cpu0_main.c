#include "zf_common_headfile.h"
#include "headfile_code.h"
#pragma section all "cpu0_dsram"

#define USE_Image_Transmission    0   // 是否使用图传
#define USE_Oscilloscope          0   // 是否使用虚拟示波器
#define USE_Parampter             0   // 是否使用远程调参

#if (USE_Image_Transmission|USE_Oscilloscope|USE_Parampter)
#define WIFI_SSID_TEST          "PRTS"
#define WIFI_PASSWORD_TEST      "12345678"  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NUL
#endif
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设


// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口

    ips200_init(IPS200_TYPE_SPI); //初始化显示屏
    MotorInit();       //初始化电机和舵机控制


//    LED 初始化
//    gpio_init(P21_4, GPO,0, GPO_PUSH_PULL);    //初始化引脚用于闪烁，判断程序是否卡死
    gpio_init(P21_5, GPO,0, GPO_PUSH_PULL);
//    gpio_init(P20_8, GPO,0, GPO_PUSH_PULL);
//    gpio_init(P20_9, GPO,0, GPO_PUSH_PULL);

    ips200_show_string(0, 16, "imu660ra init.");
    imu660ra_init();  //初始化陀螺仪
    ips200_show_string(0, 16, "code init.");
    speedcount_init(); //初始化编码器

#if(USE_Image_Transmission|USE_Oscilloscope|USE_Parampter)
    ips200_show_string(0, 16, "WIFI init.");
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
        {
            printf("\r\n connect wifi failed. \r\n");
            system_delay_ms(100);                                                   // 初始化失败 等待 100ms
        }
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
      {
          while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
              "TCP",                                                              // 指定使用TCP方式通讯
              WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
              WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
              WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
          {
              // 如果一直建立失败 考虑一下是不是没有接硬件复位
              printf("\r\n Connect TCP Servers error, try again.");
              system_delay_ms(100);                                               // 建立连接失败 等待 100ms
          }
      }
#endif

    ips200_show_string(0, 16, "mt9v03x init.");
    while(1)
    {
        if(mt9v03x_init())  //初始化摄像头
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
            system_delay_ms(100);
        }
        else
        {
            break;
        }
    }

#if(USE_Image_Transmission)
    // 逐飞助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    // 如果要发送图像信息，则务必调用seekfree_assistant_camera_information_config函数进行必要的参数设置
    // 如果需要发送边线则还需调用seekfree_assistant_camera_boundary_config函数设置边线的信息
    // 发送总钻风图像信息(仅包含原始图像信息)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY,120,Findline.rightline,Findline.leftline,Findline.midline,NULL,NULL,NULL);

#endif

#if(USE_Oscilloscope)
    // 逐飞助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    //seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.channel_num = 8;

#endif

#if(USE_Parampter)
    // 逐飞助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
#endif

    adc_init(ADC1_CH8_A24, ADC_8BIT); //初始化电压采集
    bee_init(); //初始化蜂鸣器  用法，bee_time = 100; 就是蜂鸣器响100ms
    key_init(100); //按键初始化
    Butterworth_Parameter_Init(); //低通滤波器初始化
    PID_init(); //控制参数初始化
    PIT_init();  //中断初始化
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    // !!! TEMP DIAG: MOTORTEST mode bypasses PID and encoders, hard-drives both motors at 4000.
    // !!! After verifying motors spin, change back to STAND.
    CarInfo.Mode =  MOTORTEST; //STAND MOTORTEST;
    ips200_clear();
    // === V2.5 motherboard dipswitch S5 = P33.12 / P33.13 ===
    // Mini original used P33.11 / P33.12; V2.5 motherboard shifts to P33.12 / P33.13.
    // External 4.7K pull-ups exist on V2.5 board, GPI_FLOATING_IN is safe.
    gpio_init(P33_12, GPI,0,GPI_FLOATING_IN);   // SW5-A UI toggle  (was P33_11)
    gpio_init(P33_13, GPI,0,GPI_FLOATING_IN);   // SW5-B Run enable (was P33_12)

    while (TRUE)
    {
        if (taskNum[0] == TASK_ENABLE) //500ms
       {
           taskNum[0] = 500;
           gpio_toggle_level(P21_5);  //反转引脚电平
       }
//       if(gpio_get_level(P33_12)==0)
//       {
       if (taskNum[1] == TASK_ENABLE)  //UI部分显示
        {
           taskNum[1] = 200;
           if(gpio_get_level(P33_12))  // V2.5 SW5-A (was P33_11), 这是拨码开关的一个引脚gpio_get_level(P33_12)
           {
               UI_Disp();
               keyScan();
           }
           else
               show_image();
        }
//       }

       if (taskNum[2] == TASK_ENABLE)  //暂时无作用
       {
           taskNum[2] = 5;
#if(USE_Oscilloscope)
           oscilloscope_data.data[0] = L_CarSpeed;
           oscilloscope_data.data[1] = R_CarSpeed;
           oscilloscope_data.data[2] = Findline.endline;
//           oscilloscope_data.data[3] = iR;
           oscilloscope_data.data[3] = Findline.err[0];
           oscilloscope_data.data[4] = CarAngle.Yawrate;
           oscilloscope_data.data[5] = Findline.R_endline;
           oscilloscope_data.data[6] = L_setSpeed;
           oscilloscope_data.data[7] = R_setSpeed;


           seekfree_assistant_oscilloscope_send(&oscilloscope_data);

#endif

#if(USE_Image_Transmission)
           // 发送图像
           seekfree_assistant_camera_send();
#endif

#if(USE_Parampter)
           // 滴答客解析接收到的数据
           seekfree_assistant_data_analysis();

           // 遍历
           for(uint8 i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
           {
               // 更新标志位
               if(seekfree_assistant_parameter_update_flag[i])
               {
                   seekfree_assistant_parameter_update_flag[i] = 0;

                   // 通过串口发送信息
//                   L_SpeedLoop.BaseKP=seekfree_assistant_parameter[0];
//                   L_SpeedLoop.BaseKI=seekfree_assistant_parameter[1];
//                   R_SpeedLoop.BaseKP=seekfree_assistant_parameter[2];
//                   R_SpeedLoop.BaseKI=seekfree_assistant_parameter[3];
//                   L_SpeedLoop.BaseKD=seekfree_assistant_parameter[4];
//                   R_SpeedLoop.BaseKD=seekfree_assistant_parameter[5];
//                   seekfree_assistant_parameter[6];
                   stop_flag=seekfree_assistant_parameter[7];
                   Kp_max=seekfree_assistant_parameter[0];
                   Kd_max=seekfree_assistant_parameter[1];
                   speed_normal=seekfree_assistant_parameter[2];
                   speed_zhidao=seekfree_assistant_parameter[3];
                   speed_huandao=seekfree_assistant_parameter[4];
                   speed_diff=seekfree_assistant_parameter[5];
                   printf("receive data channel : %d ", i);
                   printf("data : %f ", seekfree_assistant_parameter[i]);
                   printf("\r\n");
               }
           }

#endif
       }

    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
