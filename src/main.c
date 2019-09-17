#include <stm32f4xx.h>
#include <arm_math.h>
#include <stm32f4_discovery.h>
#include <stm32f4_discovery_accelerometer.h>
#include <wolfson_pi_audio.h>
#include <diag/Trace.h>
#include <tests.h>
#include <dwt.h>
#include "filter.h"
//#include "math_helper.h"

#define NUM_TAPS   6
#define BLOCK_SIZE (WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE)/4

#undef CYCLE_COUNTER

int16_t TxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];
int16_t RxBuffer[WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE];

__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

__IO uint8_t Volume = 70;

uint32_t AcceleroTicks;
int16_t AcceleroAxis[3];

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
static q15_t firStateQ15[BLOCK_SIZE + NUM_TAPS - 1];

/* ----------------------------------------------------------------------
** FIR Coefficients buffer. Moving average.
** ------------------------------------------------------------------- */
const float32_t firCoeffs32[NUM_TAPS] = {0.2, 0.2, 0.2, 0.2, 0.2, 0};
const q15_t firCoeffsQ15[NUM_TAPS] = {6553, 6553, 6553, 6553, 6553, 0};  //num taps should be even and >4 for arm_filter_q15

/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */
FilterTypeDef filterType=FIR_Q15;

int main(int argc, char* argv[])
{
	UNUSED(argc);
	UNUSED(argv);

#ifdef CYCLE_COUNTER
	uint32_t cycleCount;
#endif
	uint32_t i, k;

	float32_t inputF32Buffer[BLOCK_SIZE];
	float32_t outputF32Buffer[BLOCK_SIZE];

	q15_t inputQ15Buffer[BLOCK_SIZE];
	q15_t outputQ15Buffer[BLOCK_SIZE];

#ifdef OS_USE_SEMIHOSTING
	//Semihosting example
	FILE *CoefficientsFile;
#ifdef CYCLE_COUNTER
	FILE *CycleFile;
#endif
	float Coefficients[NUM_TAPS];
#endif

	// Initialise the HAL Library; it must be the first
	// instruction to be executed in the main program.
	HAL_Init();

	DWT_Enable();

#ifdef OS_USE_SEMIHOSTING
	//Semihosting example
	CoefficientsFile = fopen("coefficients.txt", "r");
	if (!CoefficientsFile) {
		trace_printf("Error trying to open CoefficientsFile. Check the name/location.\n");
		while(1);
	}

	for(i=0; i<5; i++)
		fscanf(CoefficientsFile, "%f", &Coefficients[i]);

	for(i=0; i<5; i++)
		trace_printf("Coefficient %d: %f\n", i, Coefficients[i]);

	fclose(CoefficientsFile);

#ifdef CYCLE_COUNTER
	CycleFile = fopen("cyclecounter.txt", "w");
	if (!CycleFile) {
		trace_printf("Error trying to open cycle counter file\n.");
		while(1);
	}
#endif

#endif

	WOLFSON_PI_AUDIO_Init((INPUT_DEVICE_LINE_IN << 8) | OUTPUT_DEVICE_BOTH, 80, AUDIO_FREQUENCY_48K);

	WOLFSON_PI_AUDIO_SetInputMode(INPUT_DEVICE_LINE_IN);

	WOLFSON_PI_AUDIO_SetMute(AUDIO_MUTE_ON);

	WOLFSON_PI_AUDIO_Play(TxBuffer, RxBuffer, WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE);

	WOLFSON_PI_AUDIO_SetVolume(Volume);

	BSP_ACCELERO_Init();

	TEST_Init();

	arm_fir_instance_f32 S;
	float32_t  *inputF32, *outputF32;

	arm_fir_instance_q15 S15;
	q15_t *inputQ15, *outputQ15;
	arm_status status;

	/* Initialize input and output buffer pointers */
	inputF32 = &inputF32Buffer[0];
	outputF32 = &outputF32Buffer[0];

	inputQ15 = &inputQ15Buffer[0];
	outputQ15 = &outputQ15Buffer[0];

	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], BLOCK_SIZE);

	status=arm_fir_init_q15(&S15, NUM_TAPS, (q15_t *)&firCoeffsQ15[0], &firStateQ15[0], BLOCK_SIZE);
	if (status==ARM_MATH_ARGUMENT_ERROR) {
		trace_printf("Problem at arm_fir_init_q15. Check if num_taps is even and greater than 4.\n");
		while(1);
	}

	trace_printf("End of filter initialization.\n filterType is %d\n", filterType);

	while (1) {
		// Add your code here.
		if(buffer_offset == BUFFER_OFFSET_HALF)
		{
			DWT_Reset();

#ifdef CYCLE_COUNTER
			cycleCount = DWT_GetValue();
#endif
			if (filterType==FIR_FLOAT32) {
				for(i=0, k=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {
					if(i%2) {
						inputF32Buffer[k] = (float32_t)(RxBuffer[i]/32768.0);//convert to float LEFT
						k++;
					}
					else {
						TxBuffer[i] = RxBuffer[i];//   RIGHT (canal de baixo no OcenAudio)
					}
				}
				arm_fir_f32(&S, inputF32, outputF32, BLOCK_SIZE);
				for(i=0, k=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {
					if(i%2)	{
						TxBuffer[i] = (int16_t)(outputF32Buffer[k]*32768);//back to 1.15
						k++;
					}
				}
			}

			if (filterType==FIR_Q15) {
				for(i=0, k=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {
					if(i%2) {
						inputQ15Buffer[k] = (q15_t)(RxBuffer[i]);
						k++;
					}
					else {
						TxBuffer[i] = RxBuffer[i];//pass-through
					}
				}
				arm_fir_q15(&S15, inputQ15, outputQ15, BLOCK_SIZE);
				for(i=0, k=0; i<(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2); i++) {
					if(i%2)	{
						TxBuffer[i] = (int16_t)(outputQ15Buffer[k]);//back to 1.15
						k++;
					}
				}
			}

#ifdef CYCLE_COUNTER
			fprintf(CycleFile, "\nHALF: %lu", (DWT_GetValue()- cycleCount));
#endif

			buffer_offset = BUFFER_OFFSET_NONE;
		}

		if(buffer_offset == BUFFER_OFFSET_FULL)
		{
			DWT_Reset();

#ifdef CYCLE_COUNTER
			cycleCount = DWT_GetValue();
#endif

			if (filterType==FIR_FLOAT32) {
				for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {
					if(i%2) {
						inputF32Buffer[k] = (float32_t)(RxBuffer[i]/32768.0);//convert to float
						k++;
					}
					else {
						TxBuffer[i] = RxBuffer[i];//pass-through(int16_t)0.3*32768;//
					}
				}

				arm_fir_f32(&S, inputF32, outputF32, BLOCK_SIZE);
				for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {
					if(i%2)	{
						TxBuffer[i] = (int16_t)(outputF32Buffer[k]*32768.0);//back to 1.15
						k++;
					}
				}
			}

			if (filterType==FIR_Q15) {
				for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {
					if(i%2) {
						inputQ15Buffer[k] = (q15_t)(RxBuffer[i]);
						k++;
					}
					else {
						TxBuffer[i] = RxBuffer[i];//pass-through
					}
				}

				arm_fir_q15(&S15, inputQ15, outputQ15, BLOCK_SIZE);
				for(i=(WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE/2), k=0; i<WOLFSON_PI_AUDIO_TXRX_BUFFER_SIZE; i++) {
					if(i%2)	{
						TxBuffer[i] = (int16_t)(outputQ15Buffer[k]);//back to 1.15
						k++;
					}
				}
			}

#ifdef CYCLE_COUNTER
			fprintf(CycleFile, "\nFULL: %lu", (DWT_GetValue()- cycleCount));
#endif

			buffer_offset = BUFFER_OFFSET_NONE;
		}
		TEST_Main();
	}

#ifdef CYCLE_COUNTER
	fclose(CycleFile);
#endif
	return 0;
}

/*--------------------------------
Callbacks implementation:
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA full Transfer complete event.
  */
void WOLFSON_PI_AUDIO_TransferComplete_CallBack(void)
{
	buffer_offset = BUFFER_OFFSET_FULL;
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  */
void WOLFSON_PI_AUDIO_HalfTransfer_CallBack(void)
{
	  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
  * @brief  Manages the DMA FIFO error interrupt.
  * @param  None
  * @retval None
  */
void WOLFSON_PI_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1);
}
