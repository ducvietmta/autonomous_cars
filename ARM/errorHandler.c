#include "main.h"

void Error_Handler(void);

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	BSP_LED_Init(LED2);
  while (1)
  {
    BSP_LED_On(LED2);
  }
}
