/*
 * fuzzy.h
 *
 *  Created on: 2024年6月26日
 *      Author: 86139
 */

#ifndef CODE_FUZZY_H_
#define CODE_FUZZY_H_
#include "zf_common_headfile.h"

float Fuzzy_P(int  E,int EC);//第一个参数是误差，第二个是误差变化率
float Fuzzy_D(int  E,int EC);


#endif /* CODE_FUZZY_H_ */
