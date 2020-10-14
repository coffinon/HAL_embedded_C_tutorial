#include "main.h"
#include "stm32f1xx_it.h"

extern mode;

void NMI_Handler(void)
{

}


void HardFault_Handler(void)
{

  while (1)
  {

  }
}


void MemManage_Handler(void)
{

  while (1)
  {

  }
}


void BusFault_Handler(void)
{

  while (1)
  {

  }
}


void UsageFault_Handler(void)
{

  while (1)
  {

  }
}


void SVC_Handler(void)
{

}


void DebugMon_Handler(void)
{

}


void PendSV_Handler(void)
{

}


void SysTick_Handler(void)
{

  HAL_IncTick();

}


void EXTI15_10_IRQHandler(void)
{

  if(mode == 0) mode = 1;
  else if(mode == 1) mode = 2;
  else mode = 0;

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);

}

