/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
  {
    uint8_t numStages;         	// number of 2nd order stages in the filter.  Overall order is 2*numStages.
    float *pState;         		// points to the array of state coefficients.  The array is of length 2*numStages.
    float *pCoeffs;        		// points to the array of coefficients.  The array is of length 5*numStages.
    float gaindB;				// Gain in dB applied at the biquad cascade's output
    float f0;					// Filter's cut-off frequency
    float Q;					// Filter's quality factor
    float fs;					// Frequency sampling
    int type;					// Type of filter -> 0 = LP, 1 = HP, 2 = BP, 3 = PK, 4 = LS, 5 = HS.
  } biquad_Stereo_df2T_instance;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FS48_FLOAT 48000.0f
#define BLOCKSIZE_WORD 1024 					// Total 32-bit samples in transmit buffer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

/* USER CODE BEGIN PV */
int rxBuf_SAI1_A[BLOCKSIZE_WORD];
int txBuf_SAI2_A[BLOCKSIZE_WORD];
int txBuf_SAI2_B[BLOCKSIZE_WORD];

int inputBuffer[BLOCKSIZE_WORD];
int outputBufferLS[BLOCKSIZE_WORD];
int outputBufferLP[BLOCKSIZE_WORD];
int outputBufferHP[BLOCKSIZE_WORD];

float debugFloat[BLOCKSIZE_WORD/2];

// Declare biquad instance structure
biquad_Stereo_df2T_instance S_LS, S_LP, S_HP;

// Declare coefficients buffer for each biquad
float coeffLS[5];
float coeffLP[5];
float coeffHP[5];

// Declare state buffer for each biquad (containing previous output and current input samples)
float stateLS[4];
float stateLP[8];
float stateHP[8];

// Declare biquad's type and parameters
uint8_t type;
int numStage;
float gaindB;
float freqCut;
float Q;

// Debug
float debugFloatOneSample;
int debugIntOneSample;
int k = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI1_Init(void);
static void MX_SAI2_Init(void);
/* USER CODE BEGIN PFP */
void biquad_coeff_calculation(biquad_Stereo_df2T_instance * S);
void biquad_Stereo_df2T_init(biquad_Stereo_df2T_instance * S, uint8_t numStages, float *pCoeffs, float *pState, float freqCut, float Q, float fs, float gaindB, int type);
void biquad_Stereo_df2T (biquad_Stereo_df2T_instance * S, int *inputBuffer, int *outputBuffer, int size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */

  // Start receive and transmit DMAs
  HAL_SAI_Receive_DMA(&hsai_BlockA1,(uint8_t*)rxBuf_SAI1_A,BLOCKSIZE_WORD);
  HAL_SAI_Transmit_DMA(&hsai_BlockA2,(uint8_t*)txBuf_SAI2_A,BLOCKSIZE_WORD);
  HAL_SAI_Transmit_DMA(&hsai_BlockB2,(uint8_t*)txBuf_SAI2_B,BLOCKSIZE_WORD);

  // Lowshelf
  type = 4;
  numStage = 1;
  gaindB = 3.0f;
  freqCut = 100.0f;
  Q = 0.707f;
  biquad_Stereo_df2T_init(&S_LS, numStage, &coeffLS[0], &stateLS[0], freqCut, Q, FS48_FLOAT, gaindB, type);
  biquad_coeff_calculation(&S_LS); // Calculate biquad coefficients

  // Lowpass
  type = 0;
  numStage = 2;
  gaindB = 0.0f;
  freqCut = 2000.0f;
  Q = 0.5f;
  biquad_Stereo_df2T_init(&S_LP, numStage, &coeffLP[0], &stateLP[0], freqCut, Q, FS48_FLOAT, gaindB, type);
  biquad_coeff_calculation(&S_LP);

  // Highpass
  type = 1;
  numStage = 2;
  gaindB = -3.0f;
  freqCut = 2000.0f;
  Q = 0.5f;
  biquad_Stereo_df2T_init(&S_HP, numStage, &coeffHP[0], &stateHP[0], freqCut, Q, FS48_FLOAT, gaindB, type);
  biquad_coeff_calculation(&S_HP);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 16;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.ClockSource = SAI_CLKSOURCE_NA;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_IN_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODESLAVE_TX;
  hsai_BlockA2.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI1;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_TX;
  hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS_EXT_SAI1;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void biquad_Stereo_df2T_init(biquad_Stereo_df2T_instance * S, uint8_t numStages, float *pCoeffs, float *pState, float freqCut, float Q, float fs, float gaindB, int type)
{
  /* Assign filter stages */
  S->numStages = numStages;

  /* Assign coefficient pointer */
  S->pCoeffs = pCoeffs;

  /* Clear state buffer and size is always 2 * numStages */
  memset(pState, 0, (2U * (uint32_t) numStages) * sizeof(float));

  /* Assign state pointer */
  S->pState = pState;

  /* Assign filter's cut-off frequency */
  S->f0 = freqCut;

  /* Assign filter's quality factor */
  S->Q = Q;

  /* Assign frequency sampling */
  S->fs = fs;

  /* Assign output gain in dB */
  S->gaindB = gaindB;

  /* Assign filter type */
  S->type = type;

}

void biquad_Stereo_df2T (biquad_Stereo_df2T_instance * S, int * inputBuffer, int * outputBuffer, int size)
{
	/*
	/ Filter implementation is transposed direct form II - with scaling of output..
    / y(n) = b0*x(n) + s1(n-1)
    / s1(n) = s2(n-1) + b1*x(n) - a1*y(n)
    / s2(n) = b2*x(n) - a2*y(n)
    /
    /                   B0
    /   x(n)---(-)------>-------(+)------------------> Y(n)
    /           |                ^                |
    /           |                |                |
    /           |             +-----+             |
    /           |             |  -1 |             |
    /           |             | Z   |             |
    /           |             +-----+             |
    /           |                ^                |
    /           |       B1       |       A1       |
    /           -------->-------(+)-------<--------
    /           |                ^                |
    /           |                |                |
    /           |             +-----+             |
    /           |             |  -1 |             |
    /           |             | Z   |             |
    /           |             +-----+             |
    /           |                ^                |
    /           |       B2       |       A2       |
    /           -------->-------(+)-------<--------
    */

	// Get pointers and number of stages from the filter's instance
	uint32_t stage = S->numStages;         // loop counters
	float *pState = S->pState;             // State pointer
	float *pCoeffs = S->pCoeffs;           // Coefficient pointer

    float inLeftF, inRightF;
	float outLeftF, outRightF;

	// Load coefficients
	float b0 = pCoeffs[0];
	float b1 = pCoeffs[1];
	float b2 = pCoeffs[2];
	float a1 = pCoeffs[3];
	float a2 = pCoeffs[4];



	do{

		// Load state variables
	    float d1L = pState[0];
	    float d2L = pState[1];
	    float d1R = pState[2];
	    float d2R = pState[3];

		for (int i=0; i<size; i=i+2)
		{
			inLeftF = (float) inputBuffer[i+0];
			inRightF = (float) inputBuffer[i+1];

			outRightF = b0 * inRightF + d1R;
			d1R = b1 * inRightF - a1 * outRightF + d2R;
			d2R = b2 * inRightF - a2 * outRightF;

			outLeftF = b0 * inLeftF + d1L;
			d1L = b1 * inLeftF - a1 * outLeftF + d2L;
			d2L = b2 * inLeftF - a2 * outLeftF;

			outputBuffer[i+0] = (int) outLeftF;
			outputBuffer[i+1] = (int) outRightF;
		}

		// Save new state variable by incrementing state pointer
		*pState++ = d1L;
		*pState++ = d2L;
		*pState++ = d1R;
		*pState++ = d2R;

		// The current stage input is given as the output to the next stage
		inLeftF = outLeftF;
		inRightF = outRightF;

		stage--;

	} while(stage>0);
}



void biquad_coeff_calculation(biquad_Stereo_df2T_instance * S)
{
	float b0, b1, b2, a0, a1, a2, w0, A, alpha;

	float *coeffs 	= S->pCoeffs;
	float dBgain 	= S->gaindB;
	float f0	 	= S->f0;
	float Q 		= S->Q;
	float fs 		= S->fs;
	int type 		= S->type;
	int numStage 	= S->numStages;

    w0 = 2.0*PI*f0/fs;
    alpha = sinf(w0)/(2.0*Q);
    switch (type)
    {
        case 0: // LP
            A = powf(10.0, dBgain/20.0);
            A = powf(A, 1.0f/((float) numStage));
            b0 = A*(1.0 - cosf(w0))/2.0;
            b1 = A*(1.0 - cosf(w0));
            b2 = A*(1.0 - cosf(w0))/2.0;
            a0 = 1.0 + alpha;
            a1 = -2.0*cosf(w0);
            a2 = 1.0 - alpha;
            break;
        case 1: // HP
            A = powf(10.0, dBgain/20.0);
            A = powf(A, 1.0f/((float) numStage));
            b0 = A*(1.0 + cosf(w0))/2.0;
            b1 = -A*(1.0 + cosf(w0));
            b2 = A*(1.0 + cosf(w0))/2.0;
            a0 = 1.0 + alpha;
            a1 = -2.0*cosf(w0);
            a2 = 1.0 - alpha;
            break;
        case 2: // BP
            A = powf(10.0, dBgain/20.0);
            A = powf(A, 1.0f/((float) numStage));
            b0 = A*alpha;
            b1 = 0.0;
            b2 = -A*alpha;
            a0 = 1.0 + alpha;
            a1 = -2.0*cosf(w0);
            a2 = 1.0 - alpha;
            break;
        case 3: // PK
            A = powf(10.0, dBgain/40.0);
            A = powf(A, 1.0f/((float) numStage));
            b0 = 1.0 + alpha*A;
            b1 = -2.0*cosf(w0);
            b2 = 1.0 - alpha*A;
            a0 = 1.0 + alpha/A;
            a1 = -2.0*cosf(w0);
            a2 = 1.0 - alpha/A;
            break;
        case 4: // LS
            A = powf(10.0, dBgain/40.0);
            A = powf(A, 1.0f/((float) numStage));
            b0 = A*((A+1.0) - (A-1.0)*cosf(w0) + 2.0*sqrtf(A)*alpha);
            b1 = 2.0*A*((A-1.0) - (A+1.0)*cosf(w0));
            b2 = A*((A+1.0) - (A-1.0)*cosf(w0) - 2.0*sqrtf(A)*alpha);
            a0 = (A+1.0) + (A-1.0)*cosf(w0) + 2.0*sqrtf(A)*alpha;
            a1 = -2.0*((A-1.0) + (A+1.0)*cosf(w0));
            a2 = (A+1.0) + (A-1.0)*cosf(w0) - 2.0*sqrtf(A)*alpha;
            break;
        case 5: // HS
            A = powf(10.0, dBgain/40.0);
            A = powf(A, 1.0f/((float) numStage));
            b0 = A*((A+1.0) + (A-1.0)*cosf(w0) + 2.0*sqrtf(A)*alpha);
            b1 = -2.0*A*((A-1.0) + (A+1.0)*cosf(w0));
            b2 = A*( (A+1.0) + (A-1.0)*cosf(w0) - 2.0*sqrtf(A)*alpha);
            a0 = (A+1.0) - (A-1.0)*cosf(w0) + 2.0*sqrtf(A)*alpha;
            a1 = 2.0*((A-1.0) - (A+1.0)*cosf(w0));
            a2 = (A+1.0) - (A-1.0)*cosf(w0) - 2.0*sqrtf(A)*alpha;
            break;
        default:
            b0 = 1.0;
            b1 = 0.0;
            b2 = 0.0;
            a0 = 1.0;
            a1 = 0.0;
            a2 = 0.0;
            break;
    }

    b0 = b0/a0;
    b1 = b1/a0;
    b2 = b2/a0;
    a1 = a1/a0;
    a2 = a2/a0;

    coeffs[0] = b0;
    coeffs[1] = b1;
    coeffs[2] = b2;
    coeffs[3] = a1;
    coeffs[4] = a2;

}

void HAL_SAI_RxHalfCpltCallback (SAI_HandleTypeDef * hsai){

	// Get input data from receive DMA destination buffer
	for (int i=0; i < BLOCKSIZE_WORD/2; i=i+2)
	{
		inputBuffer[i+0] = (int) rxBuf_SAI1_A[i]>>1;
		inputBuffer[i+1] = (int) rxBuf_SAI1_A[i+1]>>1;
	}

	// Perform filtering
	biquad_Stereo_df2T(&S_LS, &inputBuffer[0], &outputBufferLS[0], BLOCKSIZE_WORD/2);
	biquad_Stereo_df2T(&S_LP, &outputBufferLS[0], &outputBufferLP[0], BLOCKSIZE_WORD/2);
	biquad_Stereo_df2T(&S_HP, &inputBuffer[0], &outputBufferHP[0], BLOCKSIZE_WORD/2);

	// Send filtered to transmit DMA source buffer
	for (int i=0; i < BLOCKSIZE_WORD/2; i=i+2)
	{
		txBuf_SAI2_A[i+0] = (int) outputBufferLP[i];
		txBuf_SAI2_A[i+1] = (int) -outputBufferHP[i];

		txBuf_SAI2_B[i+0] = (int) outputBufferLP[i+1];
		txBuf_SAI2_B[i+1] = (int) -outputBufferHP[i+1];
	}

}

void HAL_SAI_RxCpltCallback (SAI_HandleTypeDef * hsai){

	// Get input data from receive DMA destination buffer
	for (int i=BLOCKSIZE_WORD/2; i < BLOCKSIZE_WORD; i=i+2)
	{
		inputBuffer[i+0] = (int) rxBuf_SAI1_A[i]>>1;
		inputBuffer[i+1] = (int) rxBuf_SAI1_A[i+1]>>1;
	}

	// Perform filtering
	biquad_Stereo_df2T(&S_LS, &inputBuffer[BLOCKSIZE_WORD/2], &outputBufferLS[BLOCKSIZE_WORD/2], BLOCKSIZE_WORD/2);
	biquad_Stereo_df2T(&S_LP, &outputBufferLS[BLOCKSIZE_WORD/2], &outputBufferLP[BLOCKSIZE_WORD/2], BLOCKSIZE_WORD/2);
	biquad_Stereo_df2T(&S_HP, &inputBuffer[BLOCKSIZE_WORD/2], &outputBufferHP[BLOCKSIZE_WORD/2], BLOCKSIZE_WORD/2);

	// Send filtered to transmit DMA source buffer
	for (int i=BLOCKSIZE_WORD/2; i < BLOCKSIZE_WORD; i=i+2)
	{
		txBuf_SAI2_A[i+0] = (int) outputBufferLP[i];
		txBuf_SAI2_A[i+1] = (int) -outputBufferHP[i];

		txBuf_SAI2_B[i+0] = (int) outputBufferLP[i+1];
		txBuf_SAI2_B[i+1] = (int) -outputBufferHP[i+1];
	}

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
