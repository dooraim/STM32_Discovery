/**
  ******************************************************************************
  * @file    main.c
  * @author  Group 19. Casado Perez Alejandro, Del Prado Buznego Paula, Gañan Onieva Francisco Javier, Orlando Nico
  * @version V8.0
  * @date    8-June-2020
  * @brief   Final Project
  ******************************************************************************
*/

/*********** Includes  ****************/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis3dsh.h"
#include "math.h" //we include the <math.h> library to use atan2 and M_PI
#include <stdio.h>

#include "arm_math.h"
#include "math_helper.h"
#include "noarm_cmsis.h"

/*********** Defines   ****************/
#define DATA_STREAM_PERIOD_MS 200  //We stream data streaming through the UART at 5Hz
#define RESULT_STREAM_PERIOD_MS 50 //We stream result streaming through the UART at 20Hz
#define ACCEL_PERIOD_MS 10		   //save the data coming from the accelerometer at 100 Hz

/* ----------------------------------------------------------------------
* Defines for each of the tests performed
* ------------------------------------------------------------------- */
#define BLOCK_SIZE 5			  //The smaller amount of data we are going to filter is 100Hz read / 20 Hz transmitted
#define NUM_TAPS 10				  //we are going to use 10 coeffs in the filter
#define sampleWindow 5			  //when we do the mean, we do it of a sample window of 5, as specified in the problem
#define Data_Filtering_Amount 20  //sampling acc at 100Hz and streaming at 5Hz, we get 20 values from acc
#define Result_Filtering_Amount 5 //sampling acc at 100Hz and streaming at 20Hz, we get 5 values from acc

/*********** Declarations *************/
/*------ Function prototypes ---------*/
void USART_Config(void);
void Acc_Config(void);
float Calculate_Roll(float32_t X_axis, float32_t Z_axis);
float Calculate_Pitch(float32_t X_axis, float32_t Y_axis, float32_t Z_axis);
float Mean_sampleWindow(float32_t Data_Vector[], int i);
void ledOrientation(float X, float Y, float Z);
void print_easter_egg(void);
/*------ Global variables  -----------*/
u8 streamActive = 0; //Variable to start/stop the streaming of data {'0' Do not Stream || '1' Stream Data}

typedef enum
{ //structure to difference Data from Result streaming
	Streaming_Data,
	Streaming_Result
} Type_of_Streaming_TypeDef;

Type_of_Streaming_TypeDef Type_of_Streaming = Streaming_Data;

u8 Stream_Ready = 0;   //variable to indicate we can start streaming {'1' stream, '0' don't stream}
u8 dataReceived = 0;   //Variable to indicate we can stream through UART, put to 1 by systick each 200 ms
u8 chRX = 0;		   //Variable to save the character introduced by UART (Key from the computer in our case)
u8 Data_Acc_Ready = 0; //Variable to indicate that we can read the data from the Accelerometer, put to 1 by systick every 10 ms

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1), one for each axis
 * ------------------------------------------------------------------- */

static float32_t firStateF32X[BLOCK_SIZE + NUM_TAPS - 1];
static float32_t firStateF32Y[BLOCK_SIZE + NUM_TAPS - 1];
static float32_t firStateF32Z[BLOCK_SIZE + NUM_TAPS - 1];

/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 **
 **
 ** %Define sampling frequency
 ** Fs=100; %100Hz
 ** %Define cutoff frequency
 ** Cutoff_freq=5;
 ** %Calculate Nyquist frequency
 ** Nyq_frequency=Fs/2;
 ** Cutoff_norm=Cutoff_freq/Nyq_frequency;
 ** %FIR order
 ** order=9;            %x^9-x^0
 ** %calculate taps (coefficients)
 ** FIR_coeff=fir1(order,Cutoff_norm);
 **
 ** ------------------------------------------------------------------- */

const float32_t firCoeffs32[NUM_TAPS] = {
	+0.01198f, +0.03259f, +0.08880f, +0.15903f,
	+0.20758f, +0.20758f, +0.15903f, +0.08880f,
	+0.03259f, +0.01198f};

/* ----------------------------------------------------------------------
* Declare Global variables
* ------------------------------------------------------------------- */

uint32_t blockSize = BLOCK_SIZE; //variable in uint32_t of the Block size

/***********   Main  ******************/
int main(void)
{

	int i, k = 0;																		//variables that will act as counters for loops
	arm_fir_instance_f32 Fir_Instance_X_Axis, Fir_Instance_Y_Axis, Fir_Instance_Z_Axis; //instances of the filter
	int BUFFER_ACC_AXIS = Data_Filtering_Amount;										//the maximum amount of data obtained from the accumulator before we read it, by default we start streaming Data
	uint32_t numBlocks = BUFFER_ACC_AXIS / blockSize;									//number of blocks of data we need to filter, it will be 4 for Data streaming, 1 for Result streaming
	int16_t Data_Read_Acc[3] = {0, 0, 0};												//Auxiliary variable for the temporal data read from the accelerometer each 10ms

	/* -------------------------------------------------------------------
	 Vectors that will act as buffers storing the data read from the accelerometer before and after passing the filter
	 * ------------------------------------------------------------------- */
	float32_t Data_Stored_X_axis[BUFFER_ACC_AXIS];
	float32_t Data_Stored_Y_axis[BUFFER_ACC_AXIS];
	float32_t Data_Stored_Z_axis[BUFFER_ACC_AXIS];
	float32_t Data_Stored_Filtered_X_axis[BUFFER_ACC_AXIS];
	float32_t Data_Stored_Filtered_Y_axis[BUFFER_ACC_AXIS];
	float32_t Data_Stored_Filtered_Z_axis[BUFFER_ACC_AXIS];

	/* -------------------------------------------------------------------
	 Variables that will store the planar and orientation data of the board
	 * ------------------------------------------------------------------- */
	float Orientation_X_Axis;
	float Orientation_Y_Axis;
	float Orientation_Z_Axis;
	float roll = 0;
	float pitch = 0;

	// LED initialization
	STM_EVAL_LEDInit(LED3); // Orange
	STM_EVAL_LEDInit(LED4); // Green
	STM_EVAL_LEDInit(LED5); // Red
	STM_EVAL_LEDInit(LED6); // Blue

	/* USART configuration */
	USART_Config();

	/* Accelerometer configuration */
	Acc_Config();

	/* SysTick configuration */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1)
			;
	}

	/* Call FIR init function to initialize the instance structure of each axis. */
	arm_fir_init_f32(&Fir_Instance_X_Axis, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32X[0], blockSize);
	arm_fir_init_f32(&Fir_Instance_Y_Axis, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32Y[0], blockSize);
	arm_fir_init_f32(&Fir_Instance_Z_Axis, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32Z[0], blockSize);

	/* turn on green led, to indicate everything is initialized, until we get an user input */
	STM_EVAL_LEDOn(LED4);

	while (1)
	{

		if (dataReceived == 1)
		{
			/*
						 * By pressing the "s" button I start streaming the data, instead by pressing the "d" button I
						 * decide whether to print the mean of planar data or the results of the calculation on the "pitch" and "roll"
						 * By pressing "h" we get an easter egg
						 */
			if (chRX == 's')
			{
				streamActive = 1 - streamActive; //we toggle the streaming
				printf("Stream Toggle\r\n");
				STM_EVAL_LEDOff(LED4);
				if (streamActive == 0) //if we turn off the streaming we turn off all leds
				{
					STM_EVAL_LEDOff(LED3);
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOff(LED6);
				}
			}
			else
			{
				if (chRX == 'd')
				{
					Type_of_Streaming = !Type_of_Streaming; //we change the type of streaming
					printf("Result or Data Toggle\r\n");
					if (Type_of_Streaming == Streaming_Result)
					{
						printf("\nNow streaming Result\r\n");
						BUFFER_ACC_AXIS = Result_Filtering_Amount; //we change the amount of Data processed
					}
					else
					{
						printf("\nNow streaming Data\r\n");
						BUFFER_ACC_AXIS = Data_Filtering_Amount; //we change the amount of Data processed
					}
					numBlocks = BUFFER_ACC_AXIS / blockSize; //we update the number of blocks to be processed
				}
				else if (chRX == 'h')
					print_easter_egg();
				else
				{
					printf("Wrong command\r\n");
				}
			}
			dataReceived = 0; //we acknowledge that the data received has been processed, and start waiting for a new one
		}

		if (streamActive == 1)
		{
			if (Stream_Ready == 1)
			{
				/*
				 * Call the FIR process function for every blockSize samples  of each axis
				 * and evaluate the media of the sampleWindow elements filtered, storing it in The planar data variables
				 */

				for (i = 0; i < numBlocks; i++)
				{
					arm_fir_f32(&Fir_Instance_X_Axis, &Data_Stored_X_axis[i * blockSize], &Data_Stored_Filtered_X_axis[i * blockSize], blockSize);
					arm_fir_f32(&Fir_Instance_Y_Axis, &Data_Stored_Y_axis[i * blockSize], &Data_Stored_Filtered_Y_axis[i * blockSize], blockSize);
					arm_fir_f32(&Fir_Instance_Z_Axis, &Data_Stored_Z_axis[i * blockSize], &Data_Stored_Filtered_Z_axis[i * blockSize], blockSize);
					Orientation_X_Axis = Mean_sampleWindow(Data_Stored_Filtered_X_axis, i * blockSize);
					Orientation_Y_Axis = Mean_sampleWindow(Data_Stored_Filtered_Y_axis, i * blockSize);
					Orientation_Z_Axis = Mean_sampleWindow(Data_Stored_Filtered_Z_axis, i * blockSize);
					//if we first start streaming result, the first value streamed will be smaller as, the filter needs 10 values to have a complete output, and we just have 5
				}

				//calculate the "roll"
				roll = Calculate_Roll(Orientation_X_Axis, Orientation_Z_Axis);

				//calculate the "pitch"
				pitch = Calculate_Pitch(Orientation_X_Axis, Orientation_Y_Axis, Orientation_Z_Axis);

				if (Type_of_Streaming == Streaming_Data)
				{

					//Print data, every 5 Hz
					printf("X: %4d[mg]\t Y: %4d[mg]\t Z: %4d[mg]\r\n", (int)Orientation_X_Axis, (int)Orientation_Y_Axis, (int)Orientation_Z_Axis);
				}

				if (Type_of_Streaming == Streaming_Result)
				{

					//Print result for Roll and pitch, every 20 Hz
					printf("roll: %4d[degrees]\t pitch: %4d[degrees]\r\n", (int)roll, (int)pitch);
				}

				ledOrientation(Orientation_X_Axis, Orientation_Y_Axis, Orientation_Z_Axis); //we turn the led oriented to ground on
				Stream_Ready = 0;															//we acknowledge that the data has been streamed
			}

			if (Data_Acc_Ready == 1) //if a data to read from the accumulator is ready
			{
				LIS3DSH_ReadACC(Data_Read_Acc);						 //we save the values we read form the accumulator in Temp_Data, this function saves them in [mg]
				Data_Stored_X_axis[k] = (float32_t)Data_Read_Acc[0]; //we store the data in a vector that acts as a buffer
				Data_Stored_Y_axis[k] = (float32_t)Data_Read_Acc[1];
				Data_Stored_Z_axis[k] = (float32_t)Data_Read_Acc[2];
				k++;					  //increment the counter so values are not overwritten
				if (k >= BUFFER_ACC_AXIS) //if we get to the amount of data we are able to save in the time window before the filter requests it, we can start writing new data as the filter will have already read it
					k = 0;
				Data_Acc_Ready = 0; //we acknowledge the data from the acc has been stored
			}
		}
	}
}

/*********** Functions   ****************/

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RX Interrupt */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* USARTx configured as follows:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART initialization */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);
}

/**
  * @brief  This function initializes the SPI and the LIS3DSH Accelerometer.
  * @param  None
  * @retval None
  */
void Acc_Config(void)
{
	LIS3DSH_InitTypeDef AccInitStruct;

	AccInitStruct.Output_DataRate = LIS3DSH_DATARATE_100; //we introduce a sample period of 100Hz
	AccInitStruct.Axes_Enable = LIS3DSH_XYZ_ENABLE;		  //enable read from the 3 axes
	AccInitStruct.SPI_Wire = LIS3DSH_SERIALINTERFACE_4WIRE;
	AccInitStruct.Self_Test = LIS3DSH_SELFTEST_NORMAL;
	AccInitStruct.Full_Scale = LIS3DSH_FULLSCALE_2; // range +-2g
	AccInitStruct.Filter_BW = LIS3DSH_FILTER_BW_800;
	/*
					 * We decided to keep this filter as it will let us have a more clear output
					 * anyway if we couldn't use it, without it we have seen that the read values have an approximate offset of
					 * (X=-180 Y=-200 Z=510), that we would have to subtract to each axis of the data read from the accelerometer
					 * as we can leave it, and with 2 filters the values obtained have less noise we decided to keep it
					 */

	LIS3DSH_Init(&AccInitStruct);
}

/*********** IRQ Handlers   ****************/

void SysTick_Handler(void) //SysTick defined to execute once every ms
{
	static int counterStream_ms = 0; //counter for the streaming period
	static int counterAcc_ms = 0;	 //counter for the accumulator read period

	if (streamActive == 1) //if we are sending data
	{
		counterStream_ms++; //we increment the counters
		counterAcc_ms++;

		if (counterStream_ms >= DATA_STREAM_PERIOD_MS && Type_of_Streaming == Streaming_Data) //if we are streaming Data, we count until DATA_STREAM_PERIOD_MS
		{
			Stream_Ready = 1;	  //we indicate we can Stream Data or Result
			counterStream_ms = 0; //we reset the counter
		}
		if (counterStream_ms >= RESULT_STREAM_PERIOD_MS && Type_of_Streaming == Streaming_Result) //if we are streaming Result, we count until RESULT_STREAM_PERIOD_MS
		{
			Stream_Ready = 1;	  //we indicate we can Stream Data or Result
			counterStream_ms = 0; //we reset the counter
		}

		if (counterAcc_ms >= ACCEL_PERIOD_MS) //We will save the data from the Accelerometer when the counter is >= ACCEL_PERIOD_MS
		{
			Data_Acc_Ready = 1; //we indicate we can start reading the data from the accelerometer
			counterAcc_ms = 0;	//we reset the counter
		}
	}
}

void USART2_IRQHandler(void)
{
	/* RX interrupt */
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		chRX = USART_ReceiveData(USART2); //we save the read character in chRX
		dataReceived = 1;				  //we indicate we have received a character
	}
}

/*********** Functions Definition ****************/

/**
  * @brief  This function retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART2, (uint8_t)ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{
	}

	return ch;
}

/**
  * @brief  This function is used to calculate the "roll" parameter
  * @param  float32_t Data_Stored_Filtered_A_axis, float32_t Data_Stored_Filtered_B_axis
  * @retval Value of the "roll" parameter for the two axes in degrees
  */
float Calculate_Roll(float32_t X_axis, float32_t Z_axis)
{
	return (atan2(-X_axis, Z_axis)) * 180 / M_PI;
}

/**
  * @brief  This function is used to calculate the "pitch" parameter
  * for the pitch, as we can just get a value of the range [-90,90], by knowing the planar orientation signs,
  * we can made some changes so that it covers the whole range [-180,180]
  *
  * @param  float32_t Data_Stored_Filtered_A_axis, float32_t Data_Stored_Filtered_B_axis, float32_t Data_Stored_Filtered_C_axis
  * @retval Value of the "pitch" parameter in degrees
  */
float Calculate_Pitch(float32_t X_axis, float32_t Y_axis, float32_t Z_axis)
{
	float value = (atan2(Y_axis, sqrt(pow(X_axis, 2) + pow(Z_axis, 2)))) * 180 / M_PI;
	if (Z_axis > 0)
		return value;	//if we are inside[-90,90] we don't need to change anything
	else if (value > 0) //we do the relevant change, as the increment of angle comes from -Y axis, we change the orientation, and add 180 to get the correct angle
		return 180 - value;
	else //we do the relevant change, as the decrement of angle comes from -Y axis, we change the orientation, and subtract 180 to get the correct angle
		return (-value - 180);
}

/**
  * @brief  This function is used to calculate the mean of the [i, i + sampleWindow] components of a vector.
  * @param  float32_t Data_Vector[]
  * @retval MediaAccumulation/sampleWindow
  */
float Mean_sampleWindow(float32_t Data_Vector[], int i)
{
	float MediaAccumulation = 0;
	int k;
	for (k = i; k < i + sampleWindow; k++)
		MediaAccumulation += Data_Vector[i];
	return MediaAccumulation / sampleWindow;
}

/**
  * @brief  This function takes the value of the pitches, previously calculated, and according to their sign, turns on only the LED that points towards the ground.
  * @param  float X, float  Y , float  Z
  * @retval No data, but turns on the led pointing towards the ground
  */
void ledOrientation(float X, float Y, float Z)
{

	//button blue ground
	if (X < 0 && X < Y && X < -Y)
	{						   //if X is negative and it's absolute value is higher than the absolute value of Y, that means the ground is in the direction of the green led
		STM_EVAL_LEDOff(LED3); // Orange
		STM_EVAL_LEDOn(LED4);  // Green
		STM_EVAL_LEDOff(LED5); // Red
		STM_EVAL_LEDOff(LED6); // Blue
	}
	//usb ground
	if (Y > 0 && Y > X && Y > -X)
	{						   //if Y is positive and higher than the absolute value of X, that means the ground is in the direction of the orange led
		STM_EVAL_LEDOn(LED3);  // Orange
		STM_EVAL_LEDOff(LED4); // Green
		STM_EVAL_LEDOff(LED5); // Red
		STM_EVAL_LEDOff(LED6); // Blue
	}
	//button black ground
	if (X > 0 && X > Y && X > -Y)
	{						   //if X is positive and higher than the absolute value of Y, that means the ground is in the direction of the red led
		STM_EVAL_LEDOff(LED3); // Orange
		STM_EVAL_LEDOff(LED4); // Green
		STM_EVAL_LEDOn(LED5);  // Red
		STM_EVAL_LEDOff(LED6); // Blue
	}
	//jack ground
	if (Y < 0 && Y < X && Y < -X)
	{						   //if Y is positive and its absolute value is higher than the absolute value of X, that means the ground is in the direction of the blue led
		STM_EVAL_LEDOff(LED3); // Orange
		STM_EVAL_LEDOff(LED4); // Green
		STM_EVAL_LEDOff(LED5); // Red
		STM_EVAL_LEDOn(LED6);  // Blue
	}
}

void print_easter_egg(void)
{ //function that prints the easter egg

	printf("                            ,ooo888888888888888oooo,\r\n");
	printf("                          o8888YYYYYY77iiiiooo8888888o\r\n");
	printf("                         8888YYYY77iiYY8888888888888888\r\n");
	printf("                        [88YYY77iiY88888888888888888888]\r\n");
	printf("                        88YY7iYY888888888888888888888888\r\n");
	printf("                       [88YYi 88888888888888888888888888]\r\n");
	printf("                       i88Yo8888888888888888888888888888i\r\n");
	printf("                       i]        ^^^88888888^^^     o  [i\r\n");
	printf("                      oi8  i           o8o          i  8io\r\n");
	printf("                    ,77788o ^^  ,oooo8888888ooo,   ^ o88777,\r\n");
	printf("                    7777788888888888888888888888888888877777\r\n");
	printf("                     77777888888888888888888888888888877777\r\n");
	printf("                      77777788888888^7777777^8888888777777\r\n");
	printf("       ,oooo888 ooo   88888778888^7777ooooo7777^8887788888        ,o88^^^^888oo\r\n");
	printf("    o8888777788[];78 88888888888888888888888888888888888887 7;8^ 888888888oo^88\r\n");
	printf("   o888888iii788 ]; o 78888887788788888^;;^888878877888887 o7;[]88888888888888o\r\n");
	printf("   88888877 ii78[]8;7o 7888878^ ^8788^;;;;;;^878^ ^878877 o7;8 ]878888888888888\r\n");
	printf("  [88888888887888 87;7oo 777888o8888^;ii;;ii;^888o87777 oo7;7[]8778888888888888\r\n");
	printf("  88888888888888[]87;777oooooooooooooo888888oooooooooooo77;78]88877i78888888888\r\n");
	printf(" o88888888888888 877;7877788777iiiiiii;;;;;iiiiiiiii77877i;78] 88877i;788888888\r\n");
	printf(" 88^;iiii^88888 o87;78888888888888888888888888888888888887;778] 88877ii;7788888\r\n");
	printf(";;;iiiii7iiii^  87;;888888888888888888888888888888888888887;778] 888777ii;78888\r\n");
	printf(";iiiii7iiiii7iiii77;i88888888888888888888i7888888888888888877;77i 888877777ii78\r\n");
	printf("iiiiiiiiiii7iiii7iii;;;i7778888888888888ii7788888888888777i;;;;iiii 88888888888\r\n");
	printf("i;iiiiiiiiiiii7iiiiiiiiiiiiiiiiiiiiiiiiii8877iiiiiiiiiiiiiiiiiii877   88888\r\n");
	printf("ii;;iiiiiiiiiiiiii;;;ii^^^;;;ii77777788888888888887777iii;;  77777           78\r\n");
	printf("^ii;8iiiiiiii ';;;;ii;;;;;;;;;;;;;;;;;;^^oo ooooo^^^88888888;;i7          7;788\r\n");
	printf("o ^;;^^88888^     'i;;;;;;;;;;;;;;;;;;;;;;;;;;;^^^88oo^^^^888ii7         7;i788\r\n");
	printf("88ooooooooo         ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 788oo^;;          7;i888\r\n");
	printf("887ii8788888      ;;;;;;;ii;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;^87           7;788\r\n");
	printf("887i8788888^     ;;;;;;;ii;;;;;;;oo;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,,,      ;;888\r\n");
	printf("87787888888     ;;;;;;;ii;;;;;;;888888oo;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,,;i788\r\n");
	printf("87i8788888^       ';;;ii;;;;;;;8888878777ii8ooo;;;;;;;;;;;;;;;;;;;;;;;;;;i788 7\r\n");
	printf("77i8788888           ioo;;;;;;oo^^ooooo ^7i88^ooooo;;;;;;;;;;;;;;;;;;;;i7888 78\r\n");
	printf("7i87788888o         7;ii788887i7;7;788888ooooo7888888ooo;;;;;;;;;;;;;;oo ^^^ 78\r\n");
	printf("i; 7888888^      8888^o;ii778877;7;7888887;;7;7788878;878;;    ;;;;;;;i78888o ^\r\n");
	printf("i8 788888       [88888^^ ooo ^^^^^;;77888^^^^;;7787^^^^ ^^;;;;  iiii;i78888888\r\n");
	printf("^8 7888^        [87888 87 ^877i;i8ooooooo8778oooooo888877ii; iiiiiiii788888888\r\n");
	printf("  ^^^          [7i888 87;; ^8i;;i7888888888888888887888888   i7iiiiiii88888^^\r\n");
	printf("               87;88 o87;;;;o 87i;;;78888788888888888888^^ o 8ii7iiiiii;;\r\n");
	printf("               87;i8 877;77888o ^877;;;i7888888888888^^ 7888 78iii7iii7iiii\r\n");
	printf("               ^87; 877;778888887o 877;;88888888888^ 7ii7888 788oiiiiiiiii\r\n");
	printf("                 ^ 877;7 7888888887 877i;;8888887ii 87i78888 7888888888\r\n");
	printf("                  [87;;7 78888888887 87i;;888887i  87ii78888 7888888888]\r\n");
	printf("                  877;7 7788888888887 887i;887i^  87ii788888 78888888888\r\n");
	printf("                  87;i8 788888888888887 887ii;;^ 87ii7888888 78888888888\r\n");
	printf("                 [87;i8 7888888888888887 ^^^^   87ii77888888 78888888888\r\n");
	printf("                 87;;78 7888888888888887ii      87i78888888 778888888888\r\n");
	printf("                 87;788 7888888888888887i]      87i78888888 788888888888\r\n");
	printf("                87;;88 78888888888888887]       ii778888888 78888888888]\r\n");
	printf("                7;;788 7888888888888888]        i7888888888 78888888888'\r\n");
	printf("                7;;788 7888888888888888         'i788888888 78888888888\r\n");
	printf("                7;i788 788888888888888]          788888888 77888888888]\r\n");
	printf("                '7;788 778888888888888]         [788888888 78888888888'\r\n");
	printf("                ';77888 78888888888888          8888888888 7888888888]\r\n");
	printf("                 778888 78888888888888          8888888888 7888888888]\r\n");
	printf("                  78888 7888888888888]         [8888888888 7888888888\r\n");
	printf("                   7888 788888888888]          88888888888 788888888]\r\n");
	printf("                    778 78888888888]           ]888888888 778888888]\r\n");
	printf("                    oooooo ^88888^              ^88888^^^^^^^^8888]\r\n");
	printf("                   [877;i77888888888]          [;78887i8888878i7888;\r\n");
	printf("                    ^877;;ii7888ii788          ;i777;7788887787;778;\r\n");
	printf("                     ^87777;;;iiii777          ;77^^^^^^^^^^^^^^^^;;\r\n");
	printf("                        ^^^^^^^^^ii7]           ^ o88888888877iiioo\r\n");
	printf("                           77777o               [88777777iiiiii;;778\r\n");
	printf("                            77777iii            8877iiiii;;;77888888]\r\n");
	printf("                            77iiii;8           [77ii;778 788888888888\r\n");
	printf("                            7iii;;88           iii;78888 778888888888\r\n");
	printf("                           77i;78888]          ;;;;i88888 78888888888\r\n");
	printf("                          i;788888888           ;i7888888 7888888888\r\n");
	printf("                          ;788888888]           i77888888 788888888]\r\n");
	printf("                          ';88888888'           [77888888 788888888]\r\n");
	printf("                           [[8ooo88]             78888888 788888888\r\n");
	printf("                            [88888]              78888888 788888888\r\n");
	printf("                              ^^^                [7888888 77888888]\r\n");
	printf("                                                  88888888 7888887\r\n");
	printf("                                                  77888888 7888887\r\n");
	printf("                                                   ;i88888 788888i\r\n");
	printf("                                                  ,;;78888 788877i7\r\n");
	printf("                                                 ,7;;i;777777i7i;;7\r\n");
	printf("                                                 87778^^^ ^^^^87778\r\n");
	printf("                                                  ^^^^ o777777o ^^^\r\n");
	printf("                                                  o77777iiiiii7777o\r\n");
	printf("                                                 7777iiii88888iii777\r\n");
	printf("                                                ;;;i7778888888877ii;;\r\n");
	printf("                   Imperial Stormtrooper       [i77888888^^^^8888877i]\r\n");
	printf("                  (Standard Shock Trooper)     77888^oooo8888oooo^8887]\r\n");
	printf("                                              [788888888888888888888888]\r\n");
	printf("                                              88888888888888888888888888\r\n");
	printf("                                              ]8888888^iiiiiiiii^888888]\r\n");
	printf("                       Bob VanderClay           iiiiiiiiiiiiiiiiiiiiii\r\n");
	printf("                                                    ^^^^^^^^^^^^^\r\n");
}
