// GopherCAN_devboard_example.c
//  This is a bare-bones module file that can be used in order to make a module main file

#include <steering_wheel.h>
#include "main.h"
//#include "gopher_sense.h"
#include "GopherCAN.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;


// Use this to define what module this board will be
#define THIS_MODULE_ID SWM_ID


// some global variables for examples
U8 last_button_state = 0;
U8 display_page = 1;


// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);
static void init_error(void);

BUTTON swUpshift = {
    .param = &swUpshift_state,
    .port = Up_Shift_In_GPIO_Port,
    .pin = Up_Shift_In_Pin
};

BUTTON swDownshift = {
    .param = &swDownshift_state,
    .port = Down_Shift_In_GPIO_Port,
    .pin = Down_Shift_In_Pin
};

BUTTON swButton0 = {
    .param = &swButton0_state,
    .port = Face_BTN0_In_GPIO_Port,
    .pin = Face_BTN0_In_Pin
};

BUTTON swButton1 = {
    .param = &swButton1_state,
    .port = Face_BTN1_In_GPIO_Port,
    .pin = Face_BTN1_In_Pin
};

BUTTON swButton2 = {
    .param = &swButton2_state,
    .port = Face_BTN2_In_GPIO_Port,
    .pin = Face_BTN2_In_Pin
};

BUTTON swButton3 = {
    .param = &swButton3_state,
    .port = Face_BTN3_In_GPIO_Port,
    .pin = Face_BTN3_In_Pin
};

BUTTON swButton4 = {
    .param = &swButton4_state,
    .port = Face_BTN4_In_GPIO_Port,
    .pin = Face_BTN4_In_Pin
};

BUTTON swButton5 = {
    .param = &swButton5_state,
    .port = Face_BTN5_In_GPIO_Port,
    .pin = Face_BTN5_In_Pin
};

BUTTON* buttons[NUM_OF_BUTTONS] = {
    &swUpshift,
    &swDownshift,
	&swButton0,
    &swButton1,
    &swButton2,
    &swButton3,
	&swButton4,
	&swButton5,
};

// init
//  What needs to happen on startup in order to run GopherCAN
void init(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	// NOTE: CAN will also need to be added in CubeMX and code must be generated
	// Check the STM_CAN repo for the file "F0xx CAN Config Settings.pptx" for the correct settings
	if (init_can(GCAN0, example_hcan, THIS_MODULE_ID, BXTYPE_MASTER))
	{
		init_error();
	}

	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
	// will be run whenever this can command is sent to the module
	if (add_custom_can_func(SET_LED_STATE, &change_led_state, TRUE, NULL))
	{
		init_error();
	}

	// lock param sending for all of the buttons
//	for (U8 i = 0; i < NUM_OF_BUTTONS; i++) {
//	    lock_param_sending(&buttons[i]->param->info);
//	}

//	lock_param_sending(&swDial_ul.info);
}


// can_buffer_handling_loop
//  This loop will handle CAN RX software task and CAN TX hardware task. Should be
//  called every 1ms or as often as received messages should be handled
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx(example_hcan);
}

U8 button_state;
U8 last_displayPageButton_state;

U8 rot_a_sw0_in;
U8 rot_a_sw1_in;
U8 rot_a_sw2_in;
U8 rot_a_sw3_in;

U8 rot_b_sw0_in;
U8 rot_b_sw1_in;
U8 rot_b_sw2_in;
U8 rot_b_sw3_in;

//will store the bitwise version of all rotary inputs, Ex: rot0 = 1, rot1 = 1, rot2 = 1, and rot3 =1  --> rot_rawResult = 00001111
U8 rot_a_result;
U8 rot_b_result;


static U8 BTN0, BTN1, BTN2, BTN3, BTN4, BTN5, BTN6, BTN7, ROTA1, ROTA2, ROTA3, ROTA4, ROTB1, ROTB2, ROTB3, ROTB4;


// main_loop
//  another loop. This includes logic for sending a CAN command. Designed to be
//  called every 10ms
void main_loop()
{
	static U32 lastHeartbeat = 0;

	//reading in steering wheel buttons through GPIO pots
	for (U8 i = 0; i < NUM_OF_BUTTONS; i++) {
	    BUTTON* btn = buttons[i];
	    U8 new_state = !HAL_GPIO_ReadPin(btn->port, btn->pin);
	    if (new_state != btn->param->data) {
			// button state has changed, send message immediately
			send_parameter(&btn->param->info);
	    }
	    btn->param->data = new_state;
    }

	U8 new_displayPageButton_state = !HAL_GPIO_ReadPin(DISPLAY_PAGE_CHANGE_BUTTON.port, DISPLAY_PAGE_CHANGE_BUTTON.pin);
	if(new_displayPageButton_state > last_displayPageButton_state) {
		display_page = (display_page + 1) % (NUM_DISPLAY_PAGES);
	}
	last_displayPageButton_state = new_displayPageButton_state;

	update_and_queue_param_u8(&displayPage_state, display_page + 1);

	rot_a_sw0_in = HAL_GPIO_ReadPin(Rot_A_SW0_In_GPIO_Port, Rot_A_SW0_In_Pin);
	rot_a_sw1_in = HAL_GPIO_ReadPin(Rot_A_SW1_In_GPIO_Port, Rot_A_SW1_In_Pin);
	rot_a_sw2_in = HAL_GPIO_ReadPin(Rot_A_SW2_In_GPIO_Port, Rot_A_SW2_In_Pin);
	rot_a_sw3_in = HAL_GPIO_ReadPin(Rot_A_SW3_In_GPIO_Port, Rot_A_SW3_In_Pin);
	rot_b_sw0_in = HAL_GPIO_ReadPin(Rot_B_SW0_In_GPIO_Port, Rot_B_SW0_In_Pin);
	rot_b_sw1_in = HAL_GPIO_ReadPin(Rot_B_SW1_In_GPIO_Port, Rot_B_SW1_In_Pin);
	rot_b_sw2_in = HAL_GPIO_ReadPin(Rot_B_SW2_In_GPIO_Port, Rot_B_SW2_In_Pin);
	rot_b_sw3_in = HAL_GPIO_ReadPin(Rot_B_SW3_In_GPIO_Port, Rot_B_SW3_In_Pin);

	//performing bitwise operations to read in rotary positons
	//left to right is increasing, restarts to 0 at end of range
	rot_a_result = (rot_a_sw3_in << 2) | (rot_a_sw2_in << 3) | (rot_a_sw1_in << 1) | rot_a_sw0_in;
	rot_a_result = 15 - rot_a_result;

	update_and_queue_param_u8(&swDial_a_ul, rot_a_result);

	rot_b_result = (rot_b_sw3_in << 2) | (rot_b_sw2_in << 3) | (rot_b_sw1_in << 1) | rot_b_sw0_in;
	rot_b_result = 15 - rot_b_result;

	update_and_queue_param_u8(&swDial_b_ul, rot_b_result);


	if (HAL_GetTick() - lastHeartbeat > HEARTBEAT_MS_BETWEEN)
	{
//		BTN0 = HAL_GPIO_ReadPin(Face_BTN0_In_GPIO_Port, Face_BTN0_In_Pin);
//		BTN1 = HAL_GPIO_ReadPin(Face_BTN1_In_GPIO_Port, Face_BTN1_In_Pin);
//		BTN2 = HAL_GPIO_ReadPin(Face_BTN2_In_GPIO_Port, Face_BTN2_In_Pin);
//		BTN3 = HAL_GPIO_ReadPin(Face_BTN3_In_GPIO_Port, Face_BTN3_In_Pin);
//		BTN4 = HAL_GPIO_ReadPin(Face_BTN4_In_GPIO_Port, Face_BTN4_In_Pin);
//		BTN5 = HAL_GPIO_ReadPin(Face_BTN5_In_GPIO_Port, Face_BTN5_In_Pin);
//		BTN6 = HAL_GPIO_ReadPin(Up_Shift_In_GPIO_Port, Up_Shift_In_Pin);
//		BTN7 = HAL_GPIO_ReadPin(Down_Shift_In_GPIO_Port, Down_Shift_In_Pin);
//		ROTA1 = HAL_GPIO_ReadPin(Rot_A_SW0_In_GPIO_Port, Rot_A_SW0_In_Pin);
//		ROTA2 = HAL_GPIO_ReadPin(Rot_A_SW1_In_GPIO_Port, Rot_A_SW1_In_Pin);
//		ROTA3 = HAL_GPIO_ReadPin(Rot_A_SW2_In_GPIO_Port, Rot_A_SW2_In_Pin);
//		ROTA4 = HAL_GPIO_ReadPin(Rot_A_SW3_In_GPIO_Port, Rot_A_SW3_In_Pin);
//		ROTB1 = HAL_GPIO_ReadPin(Rot_B_SW0_In_GPIO_Port, Rot_B_SW0_In_Pin);
//		ROTB2 = HAL_GPIO_ReadPin(Rot_B_SW1_In_GPIO_Port, Rot_B_SW1_In_Pin);
//		ROTB3 = HAL_GPIO_ReadPin(Rot_B_SW2_In_GPIO_Port, Rot_B_SW2_In_Pin);
//		ROTB4 = HAL_GPIO_ReadPin(Rot_B_SW3_In_GPIO_Port, Rot_B_SW3_In_Pin);

		lastHeartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HBEAT_LED_GPIO_Port, HBEAT_LED_Pin);
		HAL_GPIO_TogglePin(GSENSE_LED_GPIO_Port, GSENSE_LED_Pin);

	}
}


// can_callback_function example

// change_led_state
//  a custom function that will change the state of the LED specified
//  by parameter to remote_param. In this case parameter is a U16*, but
//  any data type can be pointed to, as long as it is configured and casted
//  correctly
static void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	//HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, !!remote_param);
	return;
}


// init_error
//  This function will stay in an infinite loop, blinking the LED in a 0.5sec period. Should only
//  be called from the init function before the RTOS starts
void init_error(void)
{
	while (1)
	{
		//HAL_GPIO_TogglePin(GRN_LED_GPIO_Port, GRN_LED_Pin);
		HAL_Delay(250);
	}
}

// end of GopherCAN_example.c
