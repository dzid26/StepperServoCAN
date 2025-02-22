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
 * along with this program.  If not, see <www.gnu.org/licenses/>.
 *
 */
 
#include "calibration.h"
#include "nonvolatile.h"
#include "flash.h"
#include "motor.h"
#include "encoder.h"
#include "delay.h"
#include "actuator_config.h"
#include "main.h"
#include "utils.h"
#include "board.h"
#include <math.h>

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
float MeasureStepSize(void){
	uint16_t angle1;
	uint16_t angle2;
	A4950_enable(true);

	uint16_t stepCurrent = CALIBRATION_STEPPING_CURRENT;
	// Measure the full step size
	// Note we assume machine can take one step without issue///
	openloop_step(0, stepCurrent); //fix the stepper
	delay_ms(200);
	angle1 = OverSampleEncoderAngle(100U); //angle1 - (0-65535)
	
	openloop_step(FULLSTEP_ELECTRIC_ANGLE, stepCurrent); //move one step 'forward'
	delay_ms(200);
	angle2 = OverSampleEncoderAngle(100U); //angle2 - (0-65535)

	// delta can be about 1.8 or 0.9 degree depending on the motor
	uint16_t angle_delta_unsigned = angle2 - angle1; //makes use of circular integer wrap around to subtract any any angles
	int16_t angle_delta = (int16_t) angle_delta_unsigned;
	float deg_delta = ANGLERAW_T0_DEGREES(angle_delta);
	
	//move back
	openloop_step(0,stepCurrent);
	delay_ms(100);
	openloop_step(0, 0); //release the motor
	A4950_enable(false);

	return deg_delta;
}

bool Learn_StepSize_WiringPolarity(void){
	float x = MeasureStepSize();
	if (fabsf(x) < 0.5f){
		return false; //Motor may not have power or blocked
	}

	// detect wiring polarity
	if (x < 0.0f){
		// depending on wiring order, motor will rotate the other direction
		// flip phase polarity to avoid this
		liveMotorParams.invertedPhase = !liveMotorParams.invertedPhase;
	}

	// detect if the motor is 0.9 or 1.8 steps per rotation
	if (fabsf(x) <= 1.1){
		liveMotorParams.fullStepsPerRotation = FULLSTEPS_0_9;
	}else{
		liveMotorParams.fullStepsPerRotation = FULLSTEPS_1_8;
	}
	//Motor params are now good
	nvmMirror.motorParams = liveMotorParams;
	nvmWriteConfParms();
	return true;
}

//normalize the calData starting point regardles of what angle calibration was started at
static void CalibrationTable_normalizeStartIdx(void){
	uint16_t tempData[CALIBRATION_TABLE_SIZE];
	uint16_t wrapIdx=0;
	uint16_t wrapFinds=0;

	//find wrap point
	const uint16_t calData_2AngleSteps = ANGLE_STEPS / CALIBRATION_TABLE_SIZE * 2U;
	for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++ ){
		tempData[i] = calData[i].value; //copy

		//find wrap point
		if((i>0U) && (calData[CALIBRATION_TABLE_SIZE-i].value < calData_2AngleSteps) \
		&& (calData[CALIBRATION_TABLE_SIZE-i-1U].value > (ANGLE_MAX - calData_2AngleSteps))){
			wrapIdx = i; //wrap point counted from the end
			wrapFinds++; //should only find one wrap point, unless the calibration is erratic
		}
	}

	if(wrapFinds == 1U){//there shouldn't be more than one wrap point
		//full electric angle in terms of calibrations points 
		//electrical angle repeats every 4 full steps
		//for CALIBRATION_TABLE_SIZE=50 and 200steps, full elec angle is every single cal point, for 400steps it is every half a point
		uint16_t fullElecAngleCalPoints = (4U * CALIBRATION_TABLE_SIZE / liveMotorParams.fullStepsPerRotation); 
		uint16_t shiftBy;
		if(fullElecAngleCalPoints > 1U){
			shiftBy = wrapIdx - (wrapIdx % fullElecAngleCalPoints);
		}else{
			shiftBy = wrapIdx;
		}
		
		//shift data
		for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++ ){
			calData[(i + shiftBy)%CALIBRATION_TABLE_SIZE].value = tempData[i];
		}
	}
}


//The encoder needs to be calibrated to the motor.
// we will assume full step detents are correct,
// ex 1.8 degree motor will have 200 steps for 360 degrees.
// We also need to calibrate the phasing of the motor
// to the A4950. This requires that the A4950 "step angle" of
// zero is the first entry in the calibration table.
static uint16_t CalibrationRotation(int8_t dir, bool verifyOnly, bool firstPass){
	const uint16_t stepCurrent = CALIBRATION_STEPPING_CURRENT;
	const uint16_t microStepDelay = 30U;  	//[uS] controls calibration speed
	const uint16_t stabilizationDelay = 0U; //[uS] wait for taking measurements - some medium stopping time can cause resonance, long stopping time can cause oveheat
	const uint16_t stepOversampling = 3U;  		//measurements to take per point, note large will take some time
	const uint16_t microStep = FULLSTEP_ELECTRIC_ANGLE; //microsteping resolution in between taking measurements

	static int32_t electAngle;//electric angle - static carry value over between passes
	if (firstPass){
		electAngle = 0; //initialize angle only for the first pass
	}
	const uint16_t preRunSteps = CALIBRATION_TABLE_SIZE/2U; //half rotation preRun to saturate hysteresis of the angle sensor / magnet
	const uint16_t passSteps = preRunSteps + CALIBRATION_TABLE_SIZE;
	
	for (uint16_t step = 0; step < passSteps; ++step)
	{
		bool preRun = (step < preRunSteps); //rotate some to stabilize hysteresis before starting actual calibration
		if (!preRun) {
			delay_us(stabilizationDelay);
			volatile int16_t calcStep = (int16_t)(electAngle / (int16_t)FULLSTEP_ELECTRIC_ANGLE);
			volatile uint16_t expectedAngle = (uint16_t)(int32_t)((int32_t)calcStep * (int32_t)ANGLE_STEPS / (int16_t)liveMotorParams.fullStepsPerRotation);//convert to shaft angle
			volatile uint16_t cal = (CalibrationTable_getCal(expectedAngle)); //(0-65535) - this is necessary for the second pass
			
			volatile uint16_t sampled = OverSampleEncoderAngle(stepOversampling);
			volatile uint16_t anglePass; //stores average on the second pass
			if(firstPass){
				anglePass = sampled;
			}else{
				//average with wrap around
				//sampled - cal uses unsigned int wrap around math, then casts to signed to halve the distance,
				//then casts back to unsigned to allow wrap around math again
				volatile int16_t deltaCal = (int16_t)(uint16_t)(sampled - cal);
				if((deltaCal > CALIBRATION_MAX_HYSTERESIS) || (deltaCal < -CALIBRATION_MAX_HYSTERESIS)){
					debug_assert(0); //stop in debugger
					if (deltaCal < 0){ //abs
						deltaCal = -deltaCal;
					}
					if(deltaCal < CALIBRATION_MAX_ERROR){
						deltaCal = CALIBRATION_MAX_ERROR; //make sure it fails down the stream
					}
					return (ANGLE_STEPS/2U) + (uint16_t)deltaCal; 
				}
				anglePass = cal + (uint16_t)(int16_t)(deltaCal/2);
			}
			if(!verifyOnly){
				volatile int16_t calIdx;
				calIdx = (calcStep / (int16_t)(uint16_t)(liveMotorParams.fullStepsPerRotation / CALIBRATION_TABLE_SIZE));
				calIdx = (calIdx + (int16_t)CALIBRATION_TABLE_SIZE*2) % (int16_t)CALIBRATION_TABLE_SIZE; //adds 2*CALIBRATION_TABLE_SIZE, to make sure modulo gives positive value 
				CalibrationTable_updateTableValue((uint16_t)calIdx, anglePass);
			}
		}
		const uint8_t stepDivCal_q4 = (uint8_t)(((uint16_t)(CALIBRATION_TABLE_SIZE << 4U)) / liveMotorParams.fullStepsPerRotation);
		const uint16_t microSteps = (((uint16_t)(microStep << 4U)) / stepDivCal_q4);
		for(uint16_t i = 0; i<microSteps; ++i){	//move between measurements
			electAngle += dir * (int32_t)(uint16_t)(FULLSTEP_ELECTRIC_ANGLE/microStep);//dir can be negative on first pass depending on higher level settings
			openloop_step((uint16_t) electAngle, stepCurrent);
			delay_us(microStepDelay);
		}
	}

	//calculate average sensor offset
	int32_t sumCalOffset = 0;
	for(uint16_t idx = 0; idx < CALIBRATION_TABLE_SIZE; ++idx){
		uint16_t angleLinear = (uint16_t)(idx * ANGLE_STEPS / CALIBRATION_TABLE_SIZE);
		int16_t calOffset = (int16_t)(uint16_t)(calData[idx].value - angleLinear);
		sumCalOffset += calOffset;
	}
	uint16_t angleCalOffsetAvg = (uint16_t)(int32_t)(sumCalOffset/(int16_t)CALIBRATION_TABLE_SIZE);

	//find divergance from the average
	uint16_t maxError = 0;
	for(uint16_t idx = 0; idx < CALIBRATION_TABLE_SIZE; ++idx){
		uint16_t angleLinear = (uint16_t)(idx * ANGLE_STEPS / CALIBRATION_TABLE_SIZE);
		uint16_t dist_abs = (uint16_t)fastAbs((int16_t)(uint16_t)(calData[idx].value - angleCalOffsetAvg - angleLinear));
		maxError = (dist_abs > maxError) ? dist_abs : maxError;
	}
	return maxError;
}


uint16_t EncoderCalibrate(bool verifyOnly){
	uint16_t maxError;

	A4950_enable(true);

	openloop_step(0, CALIBRATION_STEPPING_CURRENT);
	delay_ms(50);

	//determine first pass direction
	int8_t dir;
	//check if dir sign matches - on the first pass rotate the same direction as positive torque request
	if((gearing_ratio > 0.f) == (liveSystemParams.dirRotation == CW_ROTATION)){
		dir = 1;
	}else{
		dir = -1;
	}
	maxError = CalibrationRotation(dir, verifyOnly, true);
	//wait holding two phases (half a step) for less heat generation before triggering second pass
	openloop_step(FULLSTEP_ELECTRIC_ANGLE/2U, CALIBRATION_STEPPING_CURRENT); //first calibration pass finishes at electAngle = 0, so adding half a step wont't ruin next pass
	delay_ms(1000);  	//give some time before motor starts to move the other direction
	if(!verifyOnly){
		//second calibration pass the other direction - reduces influence of magnetic hysteresis
		maxError = CalibrationRotation(-dir, verifyOnly, false);
		if(maxError < CALIBRATION_MAX_ERROR){
			CalibrationTable_normalizeStartIdx(); //this step is optional, but makes the calibration table more readable
			CalibrationTable_saveToFlash(); //saves the calibration to flash
		}
	}
	//measure new starting point
	openloop_step(0, 0); //release motor - 0mA

	return maxError;
}

// Estimate motor k_bemf with no load
int8_t Estimate_motor_k_bemf(void) {
	StepperCtrl_setMotionMode(STEPCTRL_OFF);
	if (GetMotorVoltage() < MIN_SUPPLY_VOLTAGE) {
		return -1;
	}
	
	//accelerate
	StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_KBEMF_ADAPT);
	StepperCtrl_setCurrent(MAX_CURRENT);
	//accelerate and observe (maximum) base speed
	uint32_t base_speed = 0;
	for (uint16_t i = 0; i < 300U; ++i) {
		base_speed = max(base_speed, fastAbs(speed_slow));
		delay_ms(1);
	}

	if (base_speed / ANGLE_STEPS < 1){ //require at least 1rev/s
		// failed to accelerate sufficiently
		StepperCtrl_setMotionMode(STEPCTRL_OFF);
		debug_assert(0);
		return -2;
	}
	uint32_t k_bemf = (uint32_t)(GetMotorVoltage_mV()+2*BODY_DIODE_DROP_mV) * ANGLE_STEPS / base_speed;

	if (k_bemf > 5555U) {
		// k_bemf not plausible
		StepperCtrl_setMotionMode(STEPCTRL_OFF);
		debug_assert(0);
		return -3;
	}

	// update motor_k_bemf
	motor_k_bemf = (int16_t)k_bemf;

	StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_KBEMF_ADAPT);
	StepperCtrl_setCurrent(MAX_CURRENT);

	// make sure motor can stop at with 0Nm torque command with the new motor_k_bemf
	// check both directions
	int16_t i = 0;
	for (int16_t pass = 0; pass <= 1; ++pass) {
		if (pass != 0) { //second pass
			StepperCtrl_setCurrent(-MAX_CURRENT); //accelerate
			delay_ms(300);
		}

		StepperCtrl_setCurrent(0);
		while ((fastAbs(speed_slow) > (base_speed * 3U / 4U)) && (i < 100)) {
			if(pass == 0) {++i;} else {--i;}
			debug_assert(i > 0); // if it takes longer to reduce speed on second run, that indicates board-magnet misalignment
			delay_ms(100);
			// decay motor_k_bemf
			motor_k_bemf = (int16_t)((int32_t)motor_k_bemf * 99 / 100);
		}
		delay_ms(200);
	}

	update_actuator_parameters(false); //update torque constant

	StepperCtrl_setMotionMode(STEPCTRL_OFF);
	return motor_k_bemf;
}
