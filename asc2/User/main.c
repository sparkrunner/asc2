#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Timer.h"
#include "Encoder.h"
#include "Serial.h"
#include "Motor.h"


int16_t Speed, SSpeed;

int main(void)
{ 
	Key_Init();
	OLED_Init();
	Timer_Init();
	Encoder_Init();
	Motor_Init();
	Serial_Init();
	while (1)
	{
		if (Key_Check(3, KEY_UP))
		{
			Speed += 20;
			if (Speed > 60) Speed = 0;
		}
		Motor_SetSpeed(Speed);
		OLED_ShowSignedNum(1, 7, Speed, 4);
		OLED_ShowSignedNum(2, 7, SSpeed, 4);
//		Serial_Printf("%d\n", Speed);
	}
}

void TIM4_IRQHandler(void)
{
	static uint32_t Speed_Tim = 0;
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		Key_Tick();
		Speed_Tim++;
		if (Speed_Tim == 10)
		{
			Speed_Tim = 0;
			SSpeed = Encoder_Get();
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
