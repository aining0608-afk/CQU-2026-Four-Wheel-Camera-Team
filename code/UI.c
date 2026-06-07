#include "zf_common_headfile.h"
#include "headfile_code.h"
#include "UI.h"
#include"isr.h"
void keyScan(void);
unsigned char ui_page0[10][17] =
{
    "   Showimage    ",
    "   Speed        ",
    "   encoder      ",
    "   POUT0        ",
    "   SERVPID     ",
    "   MotorPID     ",
    "   Angle        ",
    "                ",
    "                ",
    "   Votage:      "
};

unsigned char ui_page1[10][17] =
{
    " pageChange     ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    " page2          "
};

unsigned char ui_page2[10][17] =
{
    " pageChange     ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    " page3          "
};

unsigned char ui_page3[10][17] =
{
    " pageChange     ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    " page4          "
};

unsigned char ui_Sensor[10][17] =
{
    "   L :     ",
    "   SL:     ",
    "   M :     ",
    "   SR:     ",
    "   R :     ",
    "   LL:     ",
    "   RR:     ",
    "   HW:     ",
    "   err:    ",
    "   Sensor  "
};

unsigned char ui_Speed[10][17] =
{
    "   SPEED        ",
    "   SPEED_SET    ",
    "   speed_zhidao ",
    "   speed_normal ",
    "   speed_huandao",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};

unsigned char ui_encoder[10][17] =
{
    "   LeftCnt:     ",
    "                ",
    "   RightCnt:    ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};

unsigned char ui_OUTPD[10][17] =
{
    "   POUT0:       ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};
unsigned char ui_INPD[10][17] =
{
    "   INP:        ",
    "   IND:        ",
    "   ServPID1.P   ",
    "   ServPID1.D   ",
    "   Kp_max       ",
    "   Kd_max       ",
    "                ",
    "                ",
    "                ",
    "                "
};
unsigned char ui_MotorPID[13][17] =
{
    "   MotorP:   L: ",
    "             R: ",
    "   MotorI:   L: ",
    "             R: ",
    "   MotorD:   L: ",
    "             R: ",
    "   SPEED     L: ",
    "             R: ",
    "   SPEED_SET_AV ",
    "                "

};
unsigned char ui_angle[10][17] =
{
    "   Angle_Y:     ",
    "   Angle_Z:     ",
    "   SPEED_Z      ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                ",
    "                "
};

UI_CLASS ui =
{
    &UI_Disp,
    {0, 0, 0, 0}, 0, -1
};


static void UI_DispUIStrings(uint8 strings[10][17])
{
	int16 i;
	for (i = 0; i < 10; i++)
	{
		if (i == ui.cursor[ui.page])
		strings[i][2] = '>';
		else
		strings[i][2] = ' ';
		ips200_show_string(3, i*22, strings[i]);
	}
}
void num_add(float* num, float step)
{
*num += step;
}

void num_sub(float* num, float step)
{
*num -= step;
if(*num<0)
    *num = 0;
}

void draw_point(int16 x, int16 y, const uint16 color){
    ips200_draw_point(x,y,color);
    ips200_draw_point(x+1,y,color);
    ips200_draw_point(x-1,y,color);
    ips200_draw_point(x,y+1,color);
    ips200_draw_point(x,y-1,color);
    ips200_draw_point(x+1,y+1,color);
    ips200_draw_point(x-1,y-1,color);
    ips200_draw_point(x+1,y-1,color);
    ips200_draw_point(x-1,y+1,color);
}

void show_image(void)
{
    ips200_show_string(190,0,"FPS");
    ips200_show_uint(190, 20, (fps_cnt[0]), 2);
    ips200_show_gray_image(5, 5, bin_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    ips200_show_float(180,180,gyro.PitchAngle_Integral,3,2);
   for(int16 i = 119+5; i >= 5 && i > Findline.endline+5; i--)
   {
      ips200_draw_point(Findline.midline[i-5] + 5, i, RGB565_RED);
      ips200_draw_point(Findline.rightline[i-5] + 5, i, RGB565_RED);
      ips200_draw_point(Findline.leftline[i-5] + 5, i, RGB565_RED);
     }
//     draw_point(Findp.lcenter[0]+5,Findp.lcenter[1]+5,RGB565_GREEN);
//     draw_point(Findp.leftup[0]+5,Findp.leftup[1]+5,RGB565_GREEN);
//     draw_point(Findp.rightup[0]+5,Findp.rightup[1]+5,RGB565_BROWN);
//     draw_point(Findp.leftdown[0]+5,Findp.leftdown[1]+5,RGB565_PINK);
 //    draw_point(Findp.rightdown[0]+5,Findp.rightdown[1]+5,RGB565_PURPLE);

   ips200_draw_line(0, qianzhan+5, 20, qianzhan+5, RGB565_GREEN);
   ips200_show_string(190,40,"V");
     ips200_show_float(190, 60, (float)(3.3*5.8*adc_mean_filter_convert(ADC1_CH8_A24, 5)/256), 2, 2);
   ips200_show_uint(190, 120, dl1a_distance_mm, 4);
   ips200_show_string(190, 100, "mm");
//   ips200_show_int(10,160,leftduan,2);
//   ips200_show_int(20,180,rightduan,2);
//   //角点识别显示；
   ips200_show_int(10,160,(Findp.leftup_flag),2);
   ips200_show_int(30,160,(Findp.rightup_flag),2);
   ips200_show_int(10,180,(Findp.leftdown_flag),2);
   ips200_show_int(30,180,(Findp.rightdown_flag),2);
   ips200_show_int(10,200,(Findp.lcenter_flag),2);
   ips200_show_int(30,200,(Findp.rcenter_flag),2);
//   ips200_show_int(10,160,(Findp.leftup[0]),2);
//   ips200_show_int(40,160,(Findp.rightup[0]),2);
//   ips200_show_int(10,180,(Findp.leftdown[0]),2);
//   ips200_show_int(40,180,(Findp.rightdown[0]),2);
//   ips200_show_int(10,200,(Findp.lcenter[0]),2);
//   ips200_show_int(40,200,(Findp.rcenter[0]),2);
     ips200_show_int(190,200,left_hdflag,2);
   //角点划线显示
//   ips200_draw_line(Findp.leftdown[0]+5,Findp.leftdown[1],Findp.leftdown[0]+15,Findp.leftdown[1],RGB565_RED);
//   ips200_draw_line(Findp.leftup[0]+5,Findp.leftup[1],Findp.leftup[0]+15,Findp.leftup[1],RGB565_RED);
//   ips200_draw_line(Findp.rightdown[0],Findp.rightdown[1]+5,Findp.rightdown[0]+10,Findp.rightdown[1]+5,RGB565_RED);
//   ips200_draw_line(Findp.rightup[0],Findp.rightup[1]+5,Findp.rightup[0]+10,Findp.rightup[1]+5,RGB565_RED);
//   ips200_draw_line(Findp.lcenter[0]+5,Findp.lcenter[1],Findp.lcenter[0]+15,Findp.lcenter[1],RGB565_RED);
   //左右道显示，直道显示
   ips200_show_uint(10,220,left_trueshortflag,2);
   ips200_show_uint(20,220,trueshortflag,2);
   ips200_show_uint(30,220,right_trueshortflag,2);
//   //左右线断线显示
   ips200_show_uint(10,240,left_duan,2);
   ips200_show_uint(20,240,right_duan,2);
//   //斑马线标志
//   ips200_show_uint(10,260,banmaxian_biaozhiwei,2);
//   //距离显示
   ips200_show_float(10,280,e_distance,5,2);
//   //角度显示
   ips200_show_float(50,280,CarAngle.Yawrate,7,2);
   //偏差显示
   ips200_show_float(50,160,Findline.err[0],3,2);
   ips200_show_float(50,180,Findline.err[0]-lastError,3,2);
   ips200_show_uint(50,220, Findline.l_lose,3);
   ips200_show_uint(80,220, Findline.r_lose,3);

}
void UI_Disp(void)
{
  

	switch(ui.page)
	{
		case 0:
			if(ui.enter == -1)
			{
				UI_DispUIStrings(ui_page0);
				ips200_show_float(120, 198, (float)(3.3*5.8*adc_mean_filter_convert(ADC1_CH8_A24, 5)/256), 2, 2);
				ips200_show_string(190,0,"FPS");
				ips200_show_uint(190, 20, (fps_cnt[0]), 2);
			}
			else
			{
				if(ui.enter == 0)
				{
				    show_image();
				}
				else if(ui.enter == 1)
				{
					UI_DispUIStrings(ui_Speed);
					ips200_show_float(130, 0, speed_avr, 2, 2);
                    ips200_show_float(130, 22, speed.Stan, 2, 2);
                    ips200_show_float(130, 44, speed_zhidao, 2, 2);
                    ips200_show_float(130, 66, speed_normal, 2, 2);
                    ips200_show_float(130, 88,speed_huandao, 2, 2);

				}
				else if(ui.enter == 2)
				{
					UI_DispUIStrings(ui_encoder);
                    ips200_show_int(130, 0,  Left_count, 4);
                    ips200_show_int(130, 44,Right_count, 4);
					
				}
				else if(ui.enter == 3)
				{
					UI_DispUIStrings(ui_OUTPD);
                    ips200_show_int(130, 0, pout0, 4);
                    //ips200_show_float(130, 44, DirOutter.KD, 2, 2);
                                      

				}
				else if(ui.enter == 4)
				{
					UI_DispUIStrings(ui_INPD);
                    ips200_show_float(130, 0,  ServPID.P, 2, 2);
                    ips200_show_float(130, 22, ServPID.D, 2, 2);
                    ips200_show_float(130, 44, ServPID1.P, 2, 2);
                    ips200_show_float(130, 66, ServPID1.D, 2, 2);
                    ips200_show_float(130, 88,  Kp_max, 2, 2);
                    ips200_show_float(130, 110, Kd_max, 2, 2);
				}
				else if(ui.enter == 5)
				{
					UI_DispUIStrings(ui_MotorPID);
                    //ips200_show_float(130, 132, speed_avr, 2, 2);
                    ips200_show_float(130, 176, speed.Stan, 2, 2);

                    ips200_show_float(130, 0,  L_SpeedLoop.KP, 2, 2);
                    ips200_show_float(130, 22,  R_SpeedLoop.KP, 2, 2);
                    ips200_show_float(130, 44, L_SpeedLoop.KI, 2, 5);
                    ips200_show_float(130, 66,  R_SpeedLoop.KI, 2, 5);
                    ips200_show_float(130, 88, L_SpeedLoop.KD, 2, 3);
                    ips200_show_float(130, 110, R_SpeedLoop.KD, 2, 3);
                    ips200_show_float(130, 200, L_setSpeed, 2, 2);
                    ips200_show_float(130, 220, R_setSpeed, 2, 2);
                    ips200_show_float(130, 240, L_CarSpeed, 2, 2);
                    ips200_show_float(130, 260, R_CarSpeed, 2, 2);
				}
				else if(ui.enter ==6)
				{
				    UI_DispUIStrings(ui_angle);
                    ips200_show_float(130, 0,  Gravity_Angle, 2, 2);
                    ips200_show_float(130, 22,  gyro.PitchAngle_Integral, 2, 2);
                    ips200_show_float(130, 44, CarAngle.Yawrate, 2, 2);
					//UI_DispUIStrings(ui_FLAG);
					
				}
				else if(ui.enter ==7)
				{

					//UI_DispUIStrings(ui_FLAG);
					
				}
			}
			break;
			

	}
}

void keyScan(void)
{
    static unsigned char temp_page;
    if(key_get_state(KEY_2) == KEY_LONG_PRESS && ui.enter == -1) //key1_flag==0)
    {
        ui.enter = ui.cursor[ui.page];
        ips200_clear();
        temp_page = ui.cursor[ui.page];
        ui.cursor[ui.page] = 0;
        bee_time=50;
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS && ui.enter != -1)
    {
        ui.enter = -1;
        ips200_clear();
        ui.cursor[ui.page] = temp_page;
        temp_page = 0;
        bee_time=50;
    }

    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        ui.cursor[ui.page]++;
        bee_time=50;
       // ips200_clear();
        if (ui.cursor[ui.page] > 5 && ui.enter != -1)
        {
            ui.cursor[ui.page] = 0;
        }
        else if(ui.cursor[ui.page] > 6 && ui.enter == -1)
        {
            ui.cursor[ui.page] = 0;
        }

    }
    if(key_get_state(KEY_3) == KEY_SHORT_PRESS) //key3_flag==0&&switch1_state==1)
     {
        bee_time=50;
        if(ui.enter == 1 && ui.cursor[ui.page] == 2)
        {
            num_add(&speed_zhidao, 0.05);
        }
        if(ui.enter == 1 && ui.cursor[ui.page] == 3)
        {
            num_add(&speed_normal, 0.05);
        }
        if(ui.enter == 1 && ui.cursor[ui.page] == 4)
        {
            num_add(&speed_huandao, 0.05);
        }
        if(ui.enter == 3 && ui.cursor[ui.page] == 0)
        {
            num_add(&pout0, 10);
        }
        if(ui.enter == 3 && ui.cursor[ui.page] == 2)
        {
            //num_add(&DirOutter.KD, 0.1);
        }


        if(ui.enter == 4 && ui.cursor[ui.page] == 0)
        {
            num_add(&ServPID.P, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 1)
        {
            num_add(&ServPID.D, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 2)
        {
            num_add(&ServPID1.P, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 3)
        {
            num_add(&ServPID1.D, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 4)
        {
            num_add(&Kp_max, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 5)
        {
            num_add(&Kd_max, 0.01);
        }


        if(ui.enter == 5 && ui.cursor[ui.page] == 0)
        {
            num_add(&L_SpeedLoop.KP, 0.01);
        }

        if(ui.enter == 5 && ui.cursor[ui.page] == 1)
        {
            num_add(&R_SpeedLoop.KP, 0.01);
        }

        if(ui.enter == 5 && ui.cursor[ui.page] == 2)
        {
            num_add(&L_SpeedLoop.KI, 0.001);
        }

        if(ui.enter == 5 && ui.cursor[ui.page] == 3)
        {
            num_add(&R_SpeedLoop.KI, 0.001);
        }

        if(ui.enter == 5 && ui.cursor[ui.page] == 4)
        {
            num_add(&L_SpeedLoop.KD, 0.01);
        }

        if(ui.enter == 5 && ui.cursor[ui.page] == 5)
        {
            num_add(&R_SpeedLoop.KD, 0.01);
        }



     }

    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
     {
        bee_time=50;
        if(ui.enter == 1 && ui.cursor[ui.page] == 2)
        {
            num_sub(&speed_zhidao, 0.05);
        }
        if(ui.enter == 1 && ui.cursor[ui.page] == 3)
        {
            num_sub(&speed_normal, 0.05);
        }
        if(ui.enter == 1 && ui.cursor[ui.page] == 4)
        {
            num_sub(&speed_huandao, 0.05);
        }
        if(ui.enter == 3 && ui.cursor[ui.page] == 0)
        {
            num_sub(&pout0, 10);
        }
        if(ui.enter == 3 && ui.cursor[ui.page] == 2)
        {
            //num_sub(&DirOutter.KD, 0.1);
        }


        if(ui.enter == 4 && ui.cursor[ui.page] == 0)
        {
            num_sub(&ServPID.P, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 1)
        {
            num_sub(&ServPID.D, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 2)
        {
            num_sub(&ServPID1.P, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 3)
        {
            num_sub(&ServPID1.D, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 4)
        {
            num_sub(&Kp_max, 0.01);
        }
        if(ui.enter == 4 && ui.cursor[ui.page] == 5)
        {
            num_sub(&Kd_max, 0.01);
        }






        if(ui.enter == 5 && ui.cursor[ui.page] == 0)
        {
            num_sub(&L_SpeedLoop.KP, 0.01);
        }

        if(ui.enter == 5 && ui.cursor[ui.page] == 1)
        {
            num_sub(&R_SpeedLoop.KP, 0.01);
        }


        if(ui.enter == 5 && ui.cursor[ui.page] == 2)
        {
            num_sub(&L_SpeedLoop.KI, 0.001);
        }

        if(ui.enter == 5 && ui.cursor[ui.page] == 3)
        {
            num_sub(&R_SpeedLoop.KI, 0.001);
        }
        if(ui.enter == 5 && ui.cursor[ui.page] == 4)
        {
            num_sub(&L_SpeedLoop.KD, 0.01);
        }
        if(ui.enter == 5 && ui.cursor[ui.page] == 5)
        {
            num_sub(&R_SpeedLoop.KD, 0.01);
        }

     }
}


