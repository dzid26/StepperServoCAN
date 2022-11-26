#ifndef __MAIN_H
#define __MAIN_H

#include <stdbool.h>

extern volatile bool runCalibration; //calibration running

//called from interrupt
void Motion_task(void); 
void Service_task(void);

#endif
