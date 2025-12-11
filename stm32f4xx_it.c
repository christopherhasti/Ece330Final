/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */
extern char paddle_pos; 
extern int Game_State; 
extern int Score; 
extern int Miss_Count; 
extern int Max_Misses; 
extern int Tempo_Increment;
extern int get_display_for_note(int note); 
/* USER CODE END EV */

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

 /* RTC Alarm Logic */


COUNT++;  // Increment note duration counter
Vibrato_Count++; // Increment the note vibrato effect counter

/* This code applies vibrato to the current note that is playing  */
if (Vibrato_Count >= Vibrato_Rate)
{
	Vibrato_Count = 0;
	if (Song[INDEX].note > 0)
		{
			Song[INDEX].note += Vibrato_Depth;
			if (Song[INDEX].note > (Save_Note + Vibrato_Depth)) Song[INDEX].note = Save_Note - Vibrato_Depth;

		}
}

if (Animate_On > 0)
{
	Delay_counter++;
	if (Delay_counter > Delay_msec)
	{
		Delay_counter = 0;
		Seven_Segment_Digit(7,*(Message_Pointer),0);
		Seven_Segment_Digit(6,*(Message_Pointer+1),0);
		Seven_Segment_Digit(5,*(Message_Pointer+2),0);
		Seven_Segment_Digit(4,*(Message_Pointer+3),0);
		Seven_Segment_Digit(3,*(Message_Pointer+4),0);
		Seven_Segment_Digit(2,*(Message_Pointer+5),0);
		Seven_Segment_Digit(1,*(Message_Pointer+6),0);
		Seven_Segment_Digit(0,*(Message_Pointer+7),0);
		Message_Pointer++;
		if ((Message_Pointer - Save_Pointer) >= (Message_Length-8)) Message_Pointer = Save_Pointer;

	}
}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC alarms A and B interrupt through EXTI line 17.
  */
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */


  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

	/* Increment TONE counter and dimming ramp counter */
	TONE++;
	ramp++;

	if ((Music_ON > 0) && (Game_State == 1))
	{
		// Condition 1: Note is currently playing (TONE check for sound output)
		if ((Song[INDEX].note > 0) && ((Song[INDEX].tempo/Song[INDEX].size - Song[INDEX].space) > COUNT))
		{
			if (Song[INDEX].note <= TONE)
			{
				int display_for_note = get_display_for_note(Song[INDEX].note);

				if (display_for_note == paddle_pos)
				{
					GPIOD->ODR ^= 1; 
				}
				else
				{
					GPIOD->ODR &= ~1; 
				}

				TONE = 0;
			}
		}
		// Condition 2: Space/Rest duration
		else if (Song[INDEX].tempo/Song[INDEX].size > COUNT)
		{
			TONE = 0;
			GPIOD->ODR &= ~1;
		}
		// Condition 3: End of note duration: Check for Hit/Miss
		else if (Song[INDEX].tempo/Song[INDEX].size == COUNT)
		{
			GPIOD->ODR &= ~1; 

			int note_is_rest = (Song[INDEX].note == rest);

			if (!note_is_rest) 
			{
				int display_for_note = get_display_for_note(Song[INDEX].note);

				if (display_for_note == paddle_pos)
				{
					Score++; 
					Miss_Count = 0; 
				}
				else
				{
					Miss_Count++; 
				}
			}

			COUNT = 0;
			TONE = 0;

			if (!(Song[INDEX].end))
			{
				INDEX++;
				Save_Note = Song[INDEX].note; 
			}
			else 
			{
				if (Miss_Count >= Max_Misses) 
				{
					Game_State = 3; 
				}
				else
				{
					Game_State = 2; 
					
					for (int i = 0; i < 100; i++)
					{
						if (Song[i].end == 1) break;
						Song[i].tempo -= Tempo_Increment; 
						if (Song[i].tempo < 500) Song[i].tempo = 500; 
					}
					
					Save_Note = Song[0].note;
					INDEX = 0;
				}
			}
		}
	}
	else if (Music_ON == 0)
	{
		TONE = 0;
		COUNT = 0;
	}


	/* This code dims the RGB LEDs using PWM */
	if (DIM_Enable > 0)
	{
		if (RED_BRT <= ramp)
		{
			GPIOD->ODR |= (1 << 15);
		}
		else
		{
			GPIOD->ODR &= ~(1 << 15);
		}
		if (BLUE_BRT <= ramp)
		{
			GPIOD->ODR |= (1 << 14);
		}
		else
		{
			GPIOD->ODR &= ~(1 << 14);
		}
		if (GREEN_BRT <= ramp)
		{
			GPIOD->ODR |= (1 << 13);
		}
		else
		{
			GPIOD->ODR &= ~(1 << 13);
		}
	}
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */