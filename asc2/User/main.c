#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Timer.h"
#include "Menu.h"
#include "Encoder.h"

// uint16_t LEDStatus;

int main(void)
{
	Key_Init();
	LED_Init();
	OLED_Init();
	Timer_Init();
	Encoder_Init();
	Menu_Init();
	while (1)
	{
		Menu_Show();
		uint8_t Key = Key_GetNum();
		Menu_Option(Key);
		LED_DirSet(Menu_LED_Direction());
		LED_SpeedSet(Menu_LED_Speed());
	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Key_Tick();
		LED_Tick();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
