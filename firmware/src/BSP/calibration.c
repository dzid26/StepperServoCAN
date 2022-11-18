 /**
 * StepperServoCAN
 *
 * Copyright (c) 2020 Makerbase.
 * Copyright (C) 2018 MisfitTech LLC.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#include "calibration.h"
#include "nonvolatile.h"
#include "sine.h"
#include "A4950.h"
#include "encoder.h"
#include "delay.h"
#include "actuator_config.h"

static volatile CalData_t calData[CALIBRATION_TABLE_SIZE];

static void CalibrationTable_updateTableValue(uint16_t index, uint16_t value){
	calData[index].value =	value;
	calData[index].error = ANGLE_STEPS / CALIBRATION_TABLE_SIZE;
}

bool CalibrationTable_calValid(void){
	for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++){
		if (calData[i].error == CALIBRATION_ERROR_NOT_SET){
			return false;
		}
	}
	
	if (nvmFlashCalData->status != valid){ // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		CalibrationTable_saveToFlash();
	}
	return true;
}


static uint16_t interp(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x){//(0-65535)
	uint16_t dx;
	uint16_t dy;
	uint16_t dx2;
	uint16_t y;

	dx = x2 - x1;
	dy = y2 - y1;
	dx2 = x - x1;
	y = y1 + (uint16_t)((uint32_t)dx2 * dy / dx);
	return y;
}
static uint16_t CalibrationTable_reverseLookup(uint16_t encoderAngle){
	//guess matching index based on monotonicity and roughly regular spacing of the angle sensor
	uint16_t idx1 = (uint16_t)((uint32_t)(uint16_t)(encoderAngle - calData[0].value) * CALIBRATION_TABLE_SIZE / ANGLE_STEPS);
	//search precise match around guessed location
	//at SAMPLING_PERIOD_uS==40, CPU can handle about 50 loops
	uint16_t x = encoderAngle;
	for(uint16_t i = 0; i < 20U; i++){//todo - make this a define
		//compare the valeus
		uint16_t x1 = calData[idx1].value;//(0-65535)
		uint16_t x_x1 = x - x1;  //Utilize wrap around effect of subtraction to deal of with x2<x1 on calibration table wrap
		if (((int16_t)x_x1 >= 0)){//} && (x_x1 < (2U * (uint16_t)(ANGLE_STEPS / CALIBRATION_TABLE_SIZE)))){
			uint16_t idx2 = (idx1 + 1U)%CALIBRATION_TABLE_SIZE;
			uint16_t x2 = calData[idx2].value;//(0-65535)
			uint16_t x2_x = x2 - x;
			if((int16_t)x2_x > 0){ //match has been found
				//x1=a1  y1=b1=i*(65536/200)+0.5   x2=a2  y2=b2=(i+1)*(65536/200)+0.5   x=encoderAngle
				uint16_t y1 = (uint16_t)((uint32_t) idx1 * ANGLE_STEPS / CALIBRATION_TABLE_SIZE);
				uint16_t y2 = (uint16_t)((uint32_t) idx2 * ANGLE_STEPS / CALIBRATION_TABLE_SIZE);
				uint16_t y = interp(x1,y1,x2,y2,x); //y=y1+k(x-x1), k=(y2-y1)/(x2-x1)
				return y;//(0-65535)
			}else{//actual match is further to the right
				idx1=(idx1+1U)%CALIBRATION_TABLE_SIZE;
			}
		}else{//actual match is further to the left
			idx1=(idx1-1U)%CALIBRATION_TABLE_SIZE;
		}
	}
	return calData[idx1].value;	//calibration likely not linear - bail out with last best guess
}

uint16_t GetCorrectedAngle(uint16_t encoderAngle){ //(0-65535)
	uint16_t fastLook = CalibrationTable_reverseLookup(encoderAngle);
	return fastLook;//0-65535
}

void CalibrationTable_saveToFlash(void){
	FlashCalData_t data;
	for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++ ){
		data.FlashCalData[i] = calData[i].value;
	}
	data.status = valid;
	
	nvmWriteCalTable(&data); //CalTable
}

//Reading Calibration from Flash
static void CalibrationTable_loadFromFlash(void){
	for(uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++){
		calData[i].value = nvmFlashCalData->FlashCalData[i]; // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		calData[i].error = CALIBRATION_MIN_ERROR;
	}
}

void CalibrationTable_init(void){
	if(valid == nvmFlashCalData->status){  // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		CalibrationTable_loadFromFlash();
		
	}else{
		for(uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++){
			calData[i].value = 0;
			calData[i].error = CALIBRATION_ERROR_NOT_SET;
		}
	}
}

//We want to linearly interpolate between calibration table angle
static uint16_t CalibrationTable_getCal(uint16_t actualAngle){ //actualAngle - (0-65535)
	uint16_t index;
	index = (uint16_t) (((uint32_t)actualAngle * CALIBRATION_TABLE_SIZE) / ANGLE_STEPS);
	uint16_t x1 = (uint16_t) (((uint32_t)index * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE);
	uint16_t y1 = calData[index].value;
	
	index = (index+1U) % CALIBRATION_TABLE_SIZE;
	uint16_t x2 = (uint16_t) (((uint32_t)index * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE);
	uint16_t y2 = calData[index].value;
	
	uint16_t value = interp(x1, y1, x2, y2, actualAngle);

	return value;//(0-65535)
}


// return is angle in degreesx100 ie 360.0 is returned as 36000
float StepperCtrl_measureStepSize(void){
	uint16_t angle1;
	uint16_t angle2;
	A4950_enable(true);

	uint16_t stepCurrent = motorParams.currentMa;
	// Measure the full step size
	// Note we assume machine can take one step without issue///
	A4950_move(0, stepCurrent); //fix the stepper
	delay_ms(200);
	angle1 = OverSampleEncoderAngle(100U); //angle1 - (0-65535)
	
	A4950_move(A4950_STEP_MICROSTEPS, stepCurrent); //move one step 'forward'
	delay_ms(200);
	angle2 = OverSampleEncoderAngle(100U); //angle2 - (0-65535)

	// delta can be about 1.8 or 0.9 degree depending on the motor
	uint16_t angle_delta_unsigned = angle2 - angle1; //makes use of circular integer wrap around to subtract any any angles
	int16_t angle_delta = (int16_t) angle_delta_unsigned;
	float deg_delta = ANGLERAW_T0_DEGREES(angle_delta);
	
	//move back
	A4950_move(0,stepCurrent);
	A4950_enable(false);

	return deg_delta;
}



//The encoder needs to be calibrated to the motor.
// we will assume full step detents are correct,
// ex 1.8 degree motor will have 200 steps for 360 degrees.
// We also need to calibrate the phasing of the motor
// to the A4950. This requires that the A4950 "step angle" of
// zero is the first entry in the calibration table.
static uint16_t CalibrationMove(uint16_t pass_no, bool verifyOnly){
	static uint32_t electAngle;//electric angle

	uint16_t maxError = 0;
	uint16_t stepCurrent = motorParams.currentMa;
	uint8_t dir = (uint8_t)(pass_no % 2U);//forward during first pass, backward during second pass
		
	const uint16_t preRunSteps = motorParams.fullStepsPerRotation/2U; //do half rotation preRun to start calibration with max hysteresis
	const uint16_t passSteps = preRunSteps + CALIBRATION_TABLE_SIZE;
	for (uint16_t step = 0; step < passSteps; step++) //Starting calibration 
	{
		bool preRun = (step < preRunSteps); //rotate some to stabilize hysteresis before starting actual calibration
		
		uint16_t averageMeasurment; //average between passes
		if (!preRun) {
			delay_ms(60);
			uint16_t sampled = OverSampleEncoderAngle(200U); //collect angle every half step for 1.8 stepper
			uint16_t expectedAngle = (uint16_t)(ANGLE_STEPS * electAngle / A4950_STEP_MICROSTEPS / motorParams.fullStepsPerRotation);//convert to shaft angle
			uint16_t cal = (CalibrationTable_getCal(expectedAngle)); //(0-65535)
			int16_t delta = sampled - cal; //utlizes wrap around
			
			if(pass_no == 0U){//if first pass. This condition is not needed, but adds clarity
				averageMeasurment = sampled; //add half a distance to average
			}else{
				int16_t delta_weighted = (delta / (int16_t)pass_no);//cast to signed temporarily to perform division
				averageMeasurment = cal + (uint16_t)delta_weighted; //add half a distance to average
			}
			//store max error during second pass
			uint16_t dist_abs = (uint16_t) fastAbs(delta);
			maxError = (dist_abs > maxError) ? dist_abs : maxError;
		}
		if(!verifyOnly && !preRun){
			uint16_t calIdx;
			if(dir==0U){
				calIdx = step;
			}else{
				calIdx = passSteps - step; //index from the end on the way back
			}
			CalibrationTable_updateTableValue(calIdx % CALIBRATION_TABLE_SIZE, averageMeasurment);
		}
		//move half step at a time (even for 400 step motor). A full step move could cause a move backwards for uncalibrated controller
		uint8_t stepSize = (uint8_t) (CALIBRATION_TABLE_SIZE/motorParams.fullStepsPerRotation);
		for(uint16_t i = 0; i<stepSize; i++){
			if(dir==0U){
				electAngle += (uint32_t)A4950_STEP_MICROSTEPS/stepSize/2U; //it's ok if it overflows
			}else{
				electAngle -= (uint32_t)A4950_STEP_MICROSTEPS/stepSize/2U;
			}
			A4950_move((uint16_t)electAngle, stepCurrent);
			delay_ms(1);
		}
	}
	if (dir == 1U){ //reset static var after coming back
		electAngle = 0;
	}
	return maxError;
}


uint16_t StepperCtrl_calibrateEncoder(bool verifyOnly){
	uint16_t maxError;

	A4950_enable(true);

	A4950_move(0, motorParams.currentMa);
	delay_ms(50);
	//first calibration pass forward
	maxError = CalibrationMove(0U, verifyOnly);
	//second calibration pass backward - reduces influence of magnetic hysteresis
	delay_ms(1000);  	//give some time before motor starts to move the other direction
	if(!verifyOnly){
		maxError = CalibrationMove(1U, verifyOnly);
		CalibrationTable_saveToFlash(); //saves the calibration to flash
	}
	//measure new starting point
	A4950_move(0, 0); //release motor - 0mA

	return maxError;
}
