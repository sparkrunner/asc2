#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Timer.h"
#include "Encoder.h"
#include "Serial.h"
#include "Motor.h"


int16_t Speed, SSpeed, SSpeed2;

int main(void)
{ 
	Key_Init();
	OLED_Init();
	Timer_Init();
	Encoder_Init();
	Motor_Init();
	Serial_Init();
	OLED_ShowChar(2, 2, '!');
	while (1)
	{
		if (Key_Check(2, KEY_UP))
		{
			Speed += 30;
			if (Speed > 60) Speed = 0;
		}
		Motor1_SetSpeed(Speed);
		Motor2_SetSpeed(Speed);
		OLED_ShowSignedNum(1, 7, Speed, 4);
		OLED_ShowSignedNum(2, 7, SSpeed, 4);
		OLED_ShowSignedNum(3, 7, SSpeed2, 4);
		Serial_Printf("%d,%d\n", SSpeed, SSpeed2);
	}
}

void TIM1_UP_IRQHandler(void) {
    static uint32_t Speed_Tim = 0;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Key_Tick();
		Speed_Tim++;
		if (Speed_Tim == 100)
		{
			Speed_Tim = 0;
			SSpeed = Encoder1_Get();
			SSpeed2 = Encoder2_Get();
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
