/*
 * main.c
 *
 *  Created on: Mar 24, 2021
 *  Author: Shyama Gandhi & Hetul Patel
 */

#include "sleep.h"
#include "xil_cache.h"
#include "stepper.h"
#include "xuartps.h" 		//UART definitions header file
#include "xgpio.h"			//GPIO functions definitions
#include "xparameters.h"	//DEVICE ID, UART BASEADDRESS, GPIO BASE ADDRESS definitions

static void _Task_Uart( void *pvParameters );
static TaskHandle_t xUarttask;

static void _Task_Motor( void *pvParameters );
static TaskHandle_t xMotortask;

static void _Task_Emerg( void *pvParameters );
static TaskHandle_t xEmergtask;

int Initialize_UART();

/************************* Queue Function definitions *************************/
static QueueHandle_t xQueue_FIFO1 = NULL;	//queue between task1 and task2

/************************* Global Variables ***********************************/

//GPIO Button Instance and DEVICE ID
XGpio BTNInst;
#define	EMERGENCY_STOP_BUTTON_DEVICE_ID		XPAR_PMOD_BUTTONS_DEVICE_ID

//GPIO RGB led Instance and DEVICE ID
XGpio Red_RGBInst;
#define RGB_LED_DEVICE_ID					XPAR_PMOD_RGB_DEVICE_ID

//struct for motor parameters
typedef struct {
	long  currentposition_in_steps;
	float rotational_speed;
	float rotational_acceleration;
	float rotational_deceleration;
	long  targetposition_in_steps;
} decision_parameters;	

decision_parameters motor_parameters;

typedef struct {
	int delay;
	long position;
} delay_struct;

delay_struct array[20];

int parameters_flag = 0;
int delay_flag = 0;
//----------------------------------------------------
// MAIN FUNCTION
//----------------------------------------------------
int main (void)
{
	int status;
	//------------------------------------------------------
	// INITIALIZE THE PMOD GPIO PERIPHERAL FOR STEPPER MOTOR, STOP BUTTON AND RGB LED(that will flash the red light when emergency stop button is pushed three times).
	//------------------------------------------------------

	// Initialize the PMOD for motor signals (JC PMOD is being used)
	status = XGpio_Initialize(&PModMotorInst, PMOD_MOTOR_DEVICE_ID);
	if(status != XST_SUCCESS){
	xil_printf("GPIO Initialization for PMOD unsuccessful.\r\n");
	return XST_FAILURE;
	}

	// button for emergency stop activation
	// Initialize the PMOD for getting the button value (btn0 is being used)
	status = XGpio_Initialize(&BTNInst, EMERGENCY_STOP_BUTTON_DEVICE_ID);
	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization for BUTTONS unsuccessful.\r\n");
		return XST_FAILURE;
	}

	// RGB Led for flasing the red light when stop button is activated
	// Initialize the PMOD for flashing the RED light on RGB LEDz
	status = XGpio_Initialize(&Red_RGBInst, RGB_LED_DEVICE_ID);
	if(status != XST_SUCCESS){
		xil_printf("GPIO Initialization for BUTTONS unsuccessful.\r\n");
		return XST_FAILURE;
	}

	//Initialize the UART
	status = Initialize_UART();
	if (status != XST_SUCCESS){
		xil_printf("UART Initialization failed\n");
	}


	// Set all buttons direction to inputs
	XGpio_SetDataDirection(&BTNInst, 1, 0xFF);
	// Set the RGB LED direction to output
	XGpio_SetDataDirection(&Red_RGBInst, 1, 0x00);
	
	
	// Initialization of motor parameter values here
	motor_parameters.currentposition_in_steps = 0;
	motor_parameters.rotational_speed = 500;
	motor_parameters.rotational_acceleration = 150;
	motor_parameters.rotational_deceleration = 150;
	motor_parameters.targetposition_in_steps = NO_OF_STEPS_PER_REVOLUTION_FULL_DRIVE;


	xil_printf("\nStepper motor Initialization Complete! Operational parameters can be changed below:\n\n");

	xTaskCreate( _Task_Uart,
				( const char * ) "Uart Task",
				configMINIMAL_STACK_SIZE*10,
				NULL,
				tskIDLE_PRIORITY+1,
				&xUarttask );

	xTaskCreate( _Task_Motor,
				( const char * ) "Motor Task",
				configMINIMAL_STACK_SIZE*10,
				NULL,
				tskIDLE_PRIORITY+2,
				&xMotortask );
	
	xTaskCreate( _Task_Emerg,
					( const char * ) "Emerg Task",
					configMINIMAL_STACK_SIZE*10,
					NULL,
					tskIDLE_PRIORITY+2,
					&xEmergtask );

	//the queue size if set to 25 right now, you can change this size later on based on your requirements.
	
	xQueue_FIFO1 = xQueueCreate( 25, sizeof(struct decision_parameters*) ); //connects task1 -> task2

	configASSERT(xQueue_FIFO1);

	vTaskStartScheduler();

	while(1);

	return 0;
}

static void _Task_Uart( void *pvParameters ){

	int message_flag = 0;
	int direction_ccw_flag=0; //this flag is to detect the case when a negative step value (for target position) is being entered by the user.

	while(1){

		if(message_flag == 0){
			if(parameters_flag == 0){
				xil_printf("Current position of the motor = %d steps\n", motor_parameters.currentposition_in_steps);
				xil_printf("Press <ENTER> to keep this value, or type a new starting position and then <ENTER>\n");
			}
			else if(parameters_flag == 1){
				printf("Current maximum speed of the motor = %0.1f steps/sec\n", motor_parameters.rotational_speed);
				xil_printf("Press <ENTER> to keep this value, or type a new maximum speed number and then <ENTER>\n");
			}
			else if(parameters_flag == 2){
				printf("Current maximum acceleration of the motor = %0.1f steps/sec/sec\n", motor_parameters.rotational_acceleration);
				xil_printf("Press <ENTER> to keep this value, or type a new maximum acceleration and then <ENTER>\n");
			}
			else if(parameters_flag == 3){
				printf("Current maximum deceleration of the motor = %0.1f steps/sec/sec\n", motor_parameters.rotational_deceleration);
				xil_printf("Press <ENTER> to keep this value, or type a new maximum deceleration and then <ENTER>\n");
			}
			else if(parameters_flag == 4){
				xil_printf("Destination position of the motor = %d steps\n", motor_parameters.targetposition_in_steps);
				xil_printf("Press <ENTER> to keep this value, or type a new destination position and then <ENTER>\n");
			}
			else if(parameters_flag == 5){
				xil_printf("Delay at that position of the motor = 0\n");
				xil_printf("Press <ENTER> to keep this value, or type a new delay and then <ENTER>\n");
		}


		char str_value_motor_value[] = "";
		char read_UART_character[100];	//an approximate size is being taken into consideration. You will use a larger size if you require. 
		int  invalid_input_flag=0;
		int cust_struct_count = 0;
		int keep_default_value_flag = 0;
		int idx=0;
		while (1){
			if(XUartPs_IsReceiveData(XPAR_XUARTPS_0_BASEADDR)){
				read_UART_character[idx] = XUartPs_ReadReg(XPAR_XUARTPS_0_BASEADDR, XUARTPS_FIFO_OFFSET);
				idx++;
				if(read_UART_character[idx-1] == 0x0D){
					break;
				}
			}
		}

		if(idx == 1){
			if(read_UART_character[idx-1] == 0x0D){
				keep_default_value_flag = 1;
				invalid_input_flag = 0;
			}
		}
		else{
			if(parameters_flag < 4){
				for(int i=0; i<idx-1; i++){

					if(!(read_UART_character[i] >= '0' && read_UART_character[i] <= '9')){
						invalid_input_flag = 1;
						break;
					}
					else{
						strncat(str_value_motor_value, &read_UART_character[i], 1);
						invalid_input_flag = 0;
					}
				}
			}
			else if(parameters_flag == 4){

				int iterate_index=0;
				if(read_UART_character[0] == '-'){
					direction_ccw_flag=1;
					iterate_index = 1;
				}
				else
					iterate_index = 0;

				for(int i=iterate_index; i<idx-1; i++){
					if(!(read_UART_character[i] >= '0' && read_UART_character[i] <= '9')){
						invalid_input_flag = 1;
						break;
					}
					else{
						strncat(str_value_motor_value, &read_UART_character[i], 1);
						invalid_input_flag = 0;
					}
				}
			}
			else if(parameters_flag == 5){

				int iterate_index=0;
				if(read_UART_character[0] == '-'){
					direction_ccw_flag=1;
					iterate_index = 1;
				}
				else
					iterate_index = 0;

				for(int i=iterate_index; i<idx-1; i++){
					if(!(read_UART_character[i] >= '0' && read_UART_character[i] <= '9')){
						invalid_input_flag = 1;
						break;
					}
					else{
						strncat(str_value_motor_value, &read_UART_character[i], 1);
						invalid_input_flag = 0;
					}
				}
			}
		}

		if(invalid_input_flag == 1){
			message_flag = 1;
			xil_printf("There was an invalid input from user except the valid inputs between 0-9\n");
			xil_printf("Please input the value of this parameter again!\n");
		}
		else{
			message_flag = 0;
			parameters_flag += 1;
			if(parameters_flag == 1){
				if(keep_default_value_flag == 1){
					xil_printf("User chooses to keep the default value of current position = %d steps\n\n",motor_parameters.currentposition_in_steps);
				}
				else{
					motor_parameters.currentposition_in_steps = atoi(str_value_motor_value);
					xil_printf("User entered the new current position = %d steps\n\n",motor_parameters.currentposition_in_steps);
				}
			}
			else if(parameters_flag == 2){
				if(keep_default_value_flag == 1){
					printf("User chooses to keep the default value of rotational speed = %0.1f steps/sec\n\n",motor_parameters.rotational_speed);
				}
				else{
					motor_parameters.rotational_speed = atoi(str_value_motor_value);
					printf("User entered the new rotational speed = %0.1f steps/sec\n\n",motor_parameters.rotational_speed);
				}
			}
			else if(parameters_flag == 3){
				if(keep_default_value_flag == 1){
					printf("User chooses to keep the default value of rotational acceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_acceleration);
				}
				else{
					motor_parameters.rotational_acceleration = atoi(str_value_motor_value);
					printf("User entered the new rotational acceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_acceleration);
				}
			}
			else if(parameters_flag == 4){
				if(keep_default_value_flag == 1){
					printf("User chooses to keep the default value of rotational deceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_deceleration);
				}
				else{
					motor_parameters.rotational_deceleration = atoi(str_value_motor_value);
					printf("User entered the new rotational deceleration = %0.1f steps/sec/sec\n\n",motor_parameters.rotational_deceleration);
				}
			}
			else if(parameters_flag == 5){
				if(keep_default_value_flag == 1){
					xil_printf("User chooses to keep the default value of destination position = %d\n\n",motor_parameters.targetposition_in_steps);
				}
				else{
					motor_parameters.targetposition_in_steps = atoi(str_value_motor_value);
					if(direction_ccw_flag==1){
						motor_parameters.targetposition_in_steps = -motor_parameters.targetposition_in_steps;
						direction_ccw_flag = 0;
					}
					xil_printf("User entered the new destination position = %d steps\n\n",motor_parameters.targetposition_in_steps);
				}
			}
			else if(parameters_flag == 6){
				if(keep_default_value_flag == 1){
					xil_printf("User chooses to keep the default delay of 0ms \n");
				}
				else{
					cust_struct_count++;
					int del = atoi(str_value_motor_value);
					long pos = motor_parameters.targetposition_in_steps;

					delay_struct temp = {.delay = del, .position = pos};
					array[0] = temp;



					int exit_step_flag = 1;
					int invalid_flag = 0;

					int neg_flag = 0;
					while(exit_step_flag < 20){
						xil_printf("Enter Another Pair? \n");
						long pos1;
						int delay1;
						char read_param_6[100];
						int idx6 = 0;
						char input_value[] = "";
						while (1){
							if(XUartPs_IsReceiveData(XPAR_XUARTPS_0_BASEADDR)){
								read_param_6[idx6] = XUartPs_ReadReg(XPAR_XUARTPS_0_BASEADDR, XUARTPS_FIFO_OFFSET);
								idx6++;
								if(read_param_6[idx6-1] == 0x0D){
									break;
								}
							}
						}
						if(idx6 == 1){
							if(read_param_6[idx6-1] == 0x0D){
								exit_step_flag = 21;
								continue;
							}
						}
						else{

							int iterate_index=0;

							if(read_param_6[0] == '-'){
								neg_flag = 1;
								iterate_index = 1;
							}
							else{
								iterate_index = 0;
							}
							for(int i=iterate_index; i<idx6-1; i++){
								if(!(read_param_6[i] >= '0' && read_param_6[i] <= '9')){
									invalid_flag = 1;
									break;
								}
								else{
									strncat(input_value, &read_param_6[i], 1);
									invalid_flag = 0;
								}
							}
						}

						if(invalid_flag == 1){
							xil_printf("Invalid input \n");
							continue;
						}
						else{

							pos1 = atoi(input_value);
							if(neg_flag == 1){
								pos1 = -pos1;
							}
							xil_printf("Position: %d\n", pos1);
						}
						xil_printf("Now input the delay for this position \n");
						int indx7 = 0;
						char read_param_7[100];
						char input_value1[] = "";
						while (1){
							if(XUartPs_IsReceiveData(XPAR_XUARTPS_0_BASEADDR)){
								read_param_7[indx7] = XUartPs_ReadReg(XPAR_XUARTPS_0_BASEADDR, XUARTPS_FIFO_OFFSET);
								indx7++;
								if(read_param_7[indx7-1] == 0x0D){
									break;
								}
							}
						}

						//add potential error checking here

						int iterate_index=0;

						for(int i=iterate_index; i<indx7-1; i++){
							if(!(read_param_7[i] >= '0' && read_param_7[i] <= '9')){
								invalid_flag = 1;
								break;
							}
							else{
								strncat(input_value1, &read_param_7[i], 1);
								invalid_flag = 0;
							}
						}

						if(invalid_flag == 1){
							xil_printf("Invalid input \n");
							continue;
						}
						else{
							delay1 = atoi(input_value1);
							xil_printf("Delay: %d\n", delay1);
						}

						cust_struct_count++;
						delay_struct temp = {.delay = delay1, .position = pos1};
						array[exit_step_flag] = temp;

						exit_step_flag++;

					}
				}
				/////////////////////////////////////////////////////////////////////////////
				xil_printf("\n****************************** MENU ******************************\n");
				xil_printf("1. Press m<ENTER> to change the motor parameters again.\n");
				xil_printf("2. Press g<ENTER> to start the movement of the motor.\n");

				char command_1_or_2_values[100];
				int index=0;
				char command;
				while (1){
					if(XUartPs_IsReceiveData(XPAR_XUARTPS_0_BASEADDR)){
						command_1_or_2_values[index] = XUartPs_ReadReg(XPAR_XUARTPS_0_BASEADDR, XUARTPS_FIFO_OFFSET);
						index++;
						if(command_1_or_2_values[index-1] == 0x0D){
							if((index>2) | (index==1)){
								index=0;
							}
							else if(index == 2){
								command = command_1_or_2_values[index-2];
								if((command == 'm') | (command == 'g')){
									break;
								}
								else{
									index = 0;
								}
							}
						}
					}
				}

				if(command == 'm'){
					parameters_flag = 0;
				}
				else if(command == 'g'){
					//decision_parameters * const pointer_to_motor_struct_values = &motor_parameters;
					//xQueueSendToBack(xQueue_FIFO1, &pointer_to_motor_struct_values, 0UL);
					if(cust_struct_count == 0){
						decision_parameters * const pointer_to_motor_struct_values = &motor_parameters;
						xQueueSendToBack(xQueue_FIFO1, &pointer_to_motor_struct_values, 0UL);
					}

					for(int i = 0; i < cust_struct_count; i++){
						xil_printf("Waiting at %d for %d ms\n", array[i].position, array[i].delay);
						motor_parameters.targetposition_in_steps = array[i].position;
						decision_parameters * const pointer_to_motor_struct_values = &motor_parameters;
						xQueueSendToBack(xQueue_FIFO1, &pointer_to_motor_struct_values, 0UL);
						while(!Stepper_motionComplete());

						vTaskDelay(array[i].delay);
					}


				}
			}
			}

		}

		vTaskDelay(1);
	}
}

/*-----------------------------------------------------------*/
static void _Task_Motor( void *pvParameters ){

	decision_parameters *read_motor_parameters_from_queue;



	while(1){

		xQueueReceive( 	xQueue_FIFO1,
						&read_motor_parameters_from_queue,
						portMAX_DELAY);

		
		Stepper_PMOD_pins_to_output();
		Stepper_Initialize();

		xil_printf("\nStarting the Motor Rotation...\n");
		Stepper_setSpeedInStepsPerSecond(read_motor_parameters_from_queue->rotational_speed);
		Stepper_setAccelerationInStepsPerSecondPerSecond(read_motor_parameters_from_queue->rotational_acceleration);
		Stepper_setDecelerationInStepsPerSecondPerSecond(read_motor_parameters_from_queue->rotational_deceleration);
		Stepper_setCurrentPositionInSteps(read_motor_parameters_from_queue->currentposition_in_steps);
		Stepper_SetupMoveInSteps(read_motor_parameters_from_queue->targetposition_in_steps);


		while(!Stepper_motionComplete()){

			Stepper_processMovement();

		}

		//xil_printf("\n\nCurrent position of the motor = %d steps\n", motor_parameters.currentposition_in_steps);
		//xil_printf("Press <ENTER> to keep this value, or type a new starting position and then <ENTER>\n");

		parameters_flag = 0;
		vTaskDelay(1);
	}
}

static void _Task_Emerg( void *pvParameters ){

	int emerg_flag = 0;

	while(1){

		unsigned int btn_value = XGpio_DiscreteRead(&BTNInst,1);

		if(btn_value == 1){
			xil_printf("%d\n", emerg_flag);
			emerg_flag++;
		}
		else{
			emerg_flag = 0;
		}

		if(emerg_flag == 3){

			while(1){
				emerg_flag = 0;
				xil_printf("Stopping\n");
				Stepper_SetupStop();
				XGpio_DiscreteWrite(&Red_RGBInst, 1, 0b0001);
				vTaskDelay(pdMS_TO_TICKS(500));
				XGpio_DiscreteClear(&Red_RGBInst, 1L, 0b0001);
				vTaskDelay(pdMS_TO_TICKS(500));
			}

		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
