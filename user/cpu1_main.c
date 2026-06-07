#include "zf_common_headfile.h"
#include "isr.h"
#include "headfile_code.h"
#pragma section all "cpu1_dsram"
#define TASK_ENABLE 0

// **************************** 代码区域 ****************************
void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
//        if(taskNum[3] == TASK_ENABLE) // && (begin_flag == 1 || begin_flag == 2 || begin_flag == 3))
//        {
//          taskNum[3] = 20;
          if(mt9v03x_finish_flag)
          {
              mt9v03x_finish_flag = 0;
              fps_cnt[2]++;
              findline();
              caculate_err(); //计算偏差
//              Err_Sum();
          }
//      }
    }
}
#pragma section all restore
