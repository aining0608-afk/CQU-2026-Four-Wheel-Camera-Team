#ifndef _UI_h
#define _UI_h
#include "zf_common_headfile.h"
typedef struct
{
    void (*Disp)(void);
    int  cursor[4], page, enter;
} UI_CLASS;

void UI_Disp(void);
void show_image(void);
void keyScan(void);
static void UI_DispUIStrings(uint8 strings[10][17]);

#endif

