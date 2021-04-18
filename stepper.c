/*
 * stepper_driver.c
 *
 *  Created on: Mar 24, 2021
 *  Author: Shyama Gandhi
 */

# include "stepper.h"
int idx;

void Stepper_PMOD_pins_to_output(){
	//set the GPIO pins to output direction
	//write the initial value of "0b0000" to the four bit motor signal
	XGpio_SetDataDirection(&PModMotorInst, 1, 0x00);
	XGpio_DiscreteWrite(&PModMotorInst, 1, WINDINGS_OFF);
}

void Stepper_Initialize(){

	//pmod motor signals
	_signals_to_motor[0] = 0;
	_signals_to_motor[1] = 0;
	_signals_to_motor[2] = 0;
	_signals_to_motor[3] = 0;

	currentPosition_InSteps = 0;
	desiredSpeed_InStepsPerSecond = 2048.0/4.0;  		//initial speed
	acceleration_InStepsPerSecondPerSecond = 2048.0/10; //initial acceleration
	currentStepPeriod = 0.0;
	stepPhase = 0;
}

/*
 * enter the current position of motor in steps and this function will not cause any motor rotation
 * Call this function only when the motor is at rest
 */
void Stepper_setCurrentPositionInSteps(long currentPositionInSteps){
	currentPosition_InSteps = currentPositionInSteps;
}

/*
 * This function gets the current position of the motor in steps when the motor is in motion
 * RETURNS: signed steps returned
 */
long Stepper_getCurrentPositionInSteps(){
	  return(currentPosition_InSteps);
}

void Stepper_SetupStop(){
	if (direction_Scaler > 0)
	  targetPosition_InSteps = currentPosition_InSteps + decelerationDistance_InSteps;
	else
	  targetPosition_InSteps = currentPosition_InSteps - decelerationDistance_InSteps;
}

/*
 * This function will set the maximum speed in steps/second that will be reached when in accelerating mode
 * ENTER: speedInStepsPerSecond = accelerate the motor to this speed speed and then the cruising/slewing mode is at this speed
 */
void Stepper_setSpeedInStepsPerSecond(float speedInStepsPerSecond){
	  desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
}

/*
 * set the acceleration in steps/second/second
 * ENTER: accelerationInStepsPerSecondPerSecond = ACCELERATION RATE. (units: steps/second/second)
 */
void Stepper_setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond){
	acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
}

/*
 * set the deceleration in steps/second/second
 * ENTER: decelerationInStepsPerSecondPerSecond = DECELERATION RATE. (units: steps/second/second)
 */
void Stepper_setDecelerationInStepsPerSecondPerSecond(float decelerationInStepsPerSecondPerSecond){
	deceleration_InStepsPerSecondPerSecond =decelerationInStepsPerSecondPerSecond;
}

/*
 * move relative to the current position in steps, this function does not return until the motor motion will be complete
 */
void Stepper_moveRelativeInSteps(long distanceToMoveInSteps){
	Stepper_SetupRelativeMoveInSteps(distanceToMoveInSteps);
	while(!Stepper_processMovement())
	  ;
}


void Stepper_SetupRelativeMoveInSteps(long distanceToMoveInSteps){
	Stepper_SetupMoveInSteps(currentPosition_InSteps + distanceToMoveInSteps);
}


void Stepper_moveToPositionInSteps(long absolutePositionToMoveToInSteps){
	Stepper_SetupMoveInSteps(absolutePositionToMoveToInSteps);
	while(!Stepper_processMovement())
	  ;
}

void Stepper_SetupMoveInSteps(long absolutePositionToMoveToInSteps){
	long distanceToTravel_InSteps;

	// save the target location
	targetPosition_InSteps = absolutePositionToMoveToInSteps;

	// determine the period of the first step
	ramp_InitialStepPeriod =
	    1000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);

	// determine the period between steps when going at the desired velocity
	desiredStepPeriod = 1000.0 / desiredSpeed_InStepsPerSecond;

	// determine the number of steps needed to go from the desired velocity down to a
	// velocity of 0,  Steps = Velocity^2 / (2 * Acceleration)
	decelerationDistance_InSteps = (long) round((desiredSpeed_InStepsPerSecond *
	    desiredSpeed_InStepsPerSecond) / (2.0 * deceleration_InStepsPerSecondPerSecond));

	// determine the distance and direction to travel
    distanceToTravel_InSteps = targetPosition_InSteps - currentPosition_InSteps;
	if (distanceToTravel_InSteps < 0){
	  distanceToTravel_InSteps = -distanceToTravel_InSteps;
	  direction_Scaler = -1;
	}
	else{
	  direction_Scaler = 1;
	}

	// check if travel distance is too short to accelerate up to the desired velocity
	if (distanceToTravel_InSteps <= (decelerationDistance_InSteps * 2L))
		decelerationDistance_InSteps = (distanceToTravel_InSteps / 2L);

	// start the acceleration ramp at the beginning
	ramp_NextStepPeriod = ramp_InitialStepPeriod;

	acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E6;
	deceleration_InStepsPerUSPerUS = deceleration_InStepsPerSecondPerSecond / 1E6;
	startNewMove = TRUE;
}

_Bool Stepper_processMovement(void){

	  unsigned long currentTime_InUS;
	  unsigned long periodSinceLastStep_InUS;
	  long distanceToTarget_InSteps;

	  // check if already at the target position
	  if (currentPosition_InSteps == targetPosition_InSteps)
		  return(TRUE);

	  // check if this is the first call to start this new move
	  if (startNewMove){
		  ramp_LastStepTime = xTaskGetTickCount();
		  startNewMove = FALSE;
	  }

	  // determine how much time has elapsed since the last step (Note 1: this method
	  // works even if the time has wrapped. Note 2: all variables must be unsigned)
	  currentTime_InUS = xTaskGetTickCount();

	  periodSinceLastStep_InUS = currentTime_InUS - ramp_LastStepTime;

	  // if it is not time for the next step, return
	  if (periodSinceLastStep_InUS < (unsigned long) ramp_NextStepPeriod)
		  return(FALSE);

	  // determine the distance from the current position to the target
	  distanceToTarget_InSteps = targetPosition_InSteps - currentPosition_InSteps;

	  if (distanceToTarget_InSteps < 0)
		  distanceToTarget_InSteps = -distanceToTarget_InSteps;

	  // test if it is time to start decelerating, if so change from accelerating to
	  // decelerating
	  if (distanceToTarget_InSteps == decelerationDistance_InSteps){
		  acceleration_InStepsPerUSPerUS = -deceleration_InStepsPerUSPerUS;
	  }


	  // execute the step on the rising edge
	  Stepper_setNextFullStep(direction_Scaler);

	  // update the current position and speed
	  currentPosition_InSteps += direction_Scaler;

	  currentStepPeriod = ramp_NextStepPeriod;

	  // compute the period for the next step
	  //   StepPeriod =
	  //   LastStepPeriod * (1 - AccelerationInStepsPerUSPerUS * LastStepPeriod^2)
	  ramp_NextStepPeriod = ramp_NextStepPeriod *
	                (1.0 - acceleration_InStepsPerUSPerUS * ramp_NextStepPeriod *
	                 ramp_NextStepPeriod);


	  // clip the speed so that it does not accelerate beyond the desired velocity

	  if (ramp_NextStepPeriod < desiredStepPeriod)
		  ramp_NextStepPeriod = desiredStepPeriod;

	  if(round(ramp_NextStepPeriod > 2)){
		  vTaskDelay((TickType_t)ramp_NextStepPeriod-1);
	  }

	  // update the acceleration ramp
	  ramp_LastStepTime = currentTime_InUS;

	  // check if the move has reached its final target position, return true if all done
	  if (currentPosition_InSteps == targetPosition_InSteps)
	  {
	    currentStepPeriod = 0.0;
	    return(TRUE);
	  }

	  return(FALSE);

}

/*
 * update the IO pins for the next full step
 * ENTER:  direction = 1 to step forward (CW), -1 to step backward (CCW)
 */
void Stepper_setNextFullStep(int direction){

	// compute the next phase number
	  //
	  stepPhase += (direction);
	  if (stepPhase <= -1)
		  stepPhase = 3;

	  if (stepPhase >= 4)
		  stepPhase = 0;

	  switch(stepPhase)
	  {
	  case 0: XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_1); break;
	  case 1: XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_2); break;
	  case 2: XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_3); break;
	  case 3: XGpio_DiscreteWrite(&PModMotorInst, 1, FULL_STEP_4); break;
	  }
}

/*
 * disable the motor, all the drive coils are turned off to save power
 * and reduce heat when motor is not in motion, any movement command will
 * automatically re-enable the stepper motor
 */

void Stepper_disableMotor()
{
	XGpio_DiscreteWrite(&PModMotorInst, 1, WINDINGS_OFF);
}

float Stepper_getCurrentVelocityInStepsPerSecond(){
	  if (currentStepPeriod == 0.0)
		  return(0);
	  else{
	    if (direction_Scaler > 0)
	    	return(1000.0 / currentStepPeriod);
	    else
	    	return(-1000.0 / currentStepPeriod);
	  }
}

_Bool Stepper_motionComplete(){
	if (currentPosition_InSteps == targetPosition_InSteps)
		return(TRUE);
	else
		return(FALSE);
}

