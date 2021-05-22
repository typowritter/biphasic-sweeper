#include "gpio.h"
#include "led.h"

void LED_SetColor(led_color color)
{
  HAL_GPIO_WritePin(GPIOB, LED_G_Pin|LED_R_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

  switch (color)
  {
    case LED_R: HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET); break;
    case LED_G: HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); break;
    case LED_B: HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); break;
    default: break;
  }
}