#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Timer.h"
#include "Encoder.h"
#include "Serial.h"


int16_t Speed;

int main(void)
{
	Key_Init();
	OLED_Init();
	Timer_Init();
	Encoder_Init();
	Serial_Init();
	while (1)
	{
		OLED_ShowSignedNum(1, 7, Speed, 4);
		Serial_Printf("%d\n", Speed);
	}
}

void TIM2_IRQHandler(void)
{
	static uint16_t Speed_Tim = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Key_Tick();
		Speed_Tim++;
		if (Speed_Tim == 100 - 1)
		{
			Speed = Encoder_Get();
			Speed_Tim = 0;
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
