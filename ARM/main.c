/**
 ******************************************************************************
 * @file    GPIO/GPIO_IOToggle/Src/main.c
 * @author  MCD Application Team
 * @version V1.1.1
 * @date    16-October-2015
 * @brief   This example describes how to configure and use GPIOs through
 *          the STM32L4xx HAL API.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Extern functions ----------------------------------------------------------*/
extern void Nucleo_UART_Init(void);
extern void SystemClock_Config(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UPDATE_INTERVAL 	15 //refresh rate: 1/0.015ms = 66Hz
#define TASK_INTERVAL			10000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TimerHandle_t xTimerUpdate;

TaskHandle_t xCircularRingHandler;
TaskHandle_t xHeartBeatHandler;
TaskHandle_t xAllColorsHandler;
TaskHandle_t xColorWheelHandler;
TaskHandle_t xPatternMoveHandler;
TaskHandle_t xFullEmptyHandler;
TaskHandle_t xAlternateColorsHandler;

/* Private function prototypes -----------------------------------------------*/
void Main_Task(void * pvParameters);

void CircularRing_Task(void * pvParameters);
void HeartBeat_Task(void * pvParameters);
void AllColors_Task(void * pvParameters);
void ColorWheel_Task(void * pvParameters);
void PatternMove_Task(void * pvParameters);
void FullEmpty_Task(void * pvParameters);
void AlternateColors_Task(void * pvParameters);

void vTimerUpdateCallback(TimerHandle_t pxTimer);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	/* STM32L4xx HAL library initialization:
	 - Configure the Flash prefetch
	 - Systick timer is configured by default as source of time base, but user 
	 can eventually implement his proper time base source (a general purpose 
	 timer for example or other time source), keeping in mind that Time base 
	 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
	 handled in milliseconds basis.
	 - Set NVIC Group Priority to 4
	 - Low Level Initialization
	 */
	HAL_Init();

	/* Configure the system clock to 80 MHz */
	SystemClock_Config();

	/* Init LED */
	BSP_LED_Init(LED2);

	/* Init uart */
	Nucleo_UART_Init();

	/* Send hello message through uart */
	printf("Starting...\n");

	ws2812_init();

	xTimerUpdate = xTimerCreate("UpdTimer", UPDATE_INTERVAL, pdTRUE, (void *) 1,
			vTimerUpdateCallback);
	xTimerStart(xTimerUpdate, 0);

	/* Create task */
	xTaskCreate(Main_Task, "MAIN_TASK", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(CircularRing_Task, "CircularRing", configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY + 1, &xCircularRingHandler);
	xTaskCreate(HeartBeat_Task, "HeartBeat", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + 1, &xHeartBeatHandler);
	xTaskCreate(AllColors_Task, "AllColors", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + 1, &xAllColorsHandler);
	xTaskCreate(ColorWheel_Task, "ColorWheel", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + 1, &xColorWheelHandler);
	xTaskCreate(PatternMove_Task, "PatternMove", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + 1, &xPatternMoveHandler);
	xTaskCreate(FullEmpty_Task, "FullEmpty", configMINIMAL_STACK_SIZE, NULL,
			tskIDLE_PRIORITY + 1, &xFullEmptyHandler);
	xTaskCreate(AlternateColors_Task, "AlternateColors",
			configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1,
			&xAlternateColorsHandler);

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	/* Catch error */
	Error_Handler();
}

void Main_Task(void * pvParameters) {
	vTaskSuspend(xCircularRingHandler);
	vTaskSuspend(xHeartBeatHandler);
	vTaskSuspend(xAllColorsHandler);
	vTaskSuspend(xColorWheelHandler);
	vTaskSuspend(xPatternMoveHandler);
	vTaskSuspend(xFullEmptyHandler);
	vTaskSuspend(xAlternateColorsHandler);

	while (1) {
		vTaskResume(xCircularRingHandler);
		vTaskDelay(TASK_INTERVAL);
		vTaskSuspend(xCircularRingHandler);

		vTaskResume(xHeartBeatHandler);
		vTaskDelay(TASK_INTERVAL);
		vTaskSuspend(xHeartBeatHandler);

		vTaskResume(xAllColorsHandler);
		vTaskDelay(TASK_INTERVAL);
		vTaskSuspend(xAllColorsHandler);

		vTaskResume(xColorWheelHandler);
		vTaskDelay(TASK_INTERVAL);
		vTaskSuspend(xColorWheelHandler);

		vTaskResume(xPatternMoveHandler);
		vTaskDelay(TASK_INTERVAL);
		vTaskSuspend(xPatternMoveHandler);

		vTaskResume(xFullEmptyHandler);
		vTaskDelay(TASK_INTERVAL);
		vTaskSuspend(xFullEmptyHandler);

		vTaskResume(xAlternateColorsHandler);
		vTaskDelay(TASK_INTERVAL);
		vTaskSuspend(xAlternateColorsHandler);
	}
}

void CircularRing_Task(void * pvParameters) {
	stripEffect_CircularRing(50, 0, 0, 20);
}

void HeartBeat_Task(void * pvParameters) {
	stripEffect_HeartBeat(700, 64, 0, 16);
}

void AllColors_Task(void * pvParameters) {
	stripEffect_AllColors(10);
}

void ColorWheel_Task(void * pvParameters) {
	stripEffect_ColorWheel(50);
}

void PatternMove_Task(void * pvParameters) {
	stripEffect_PatternMove(50, 2, 10, 10, 10);
}

void FullEmpty_Task(void * pvParameters) {
	stripEffect_FullEmpty(50, 20, 20, 20);
}

void AlternateColors_Task(void * pvParameters) {
	stripEffect_AlternateColors(1000, 10, 50, 0, 0, 0, 0, 50);
}

void vTimerUpdateCallback(TimerHandle_t pxTimer) {
	ws2812_update();
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
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
