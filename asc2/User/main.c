#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Timer.h"
#include "Encoder.h"
#include "Serial.h"
#include "Motor.h"
#include <string.h>

int32_t Speed, SSpeed1, SSpeed2, Angle1, Angle2;

struct PID {
	float Target, Actual, Out;			//目标值，实际值，输出值
	float Kp, Ki, Kd;					//比例项，积分项，微分项的权重
	float Error0, Error1, Error2;		//本次误差，上次误差，上上次误差
} M1, M2, MInit, A1, A2, AInit;

void PID_Init_M1(void)
{
	M1 = MInit;
	M1.Kp = 1.8;
	M1.Ki = 0.8;
	M1.Kd = 0.1;
}

void PID_Init_M2(void)
{
	M2 = MInit;
	M2.Kp = 1.8;
	M2.Ki = 0.8;
	M2.Kd = 0.1;
}

void PID_SetSpeed1(int32_t Speed)
{
	M1.Target = Speed;
	M1.Actual = SSpeed1;
	M1.Error2 = M1.Error1;			//获取上上次误差
	M1.Error1 = M1.Error0;			//获取上次误差
	M1.Error0 = M1.Target - M1.Actual;	//获取本次误差，目标值减实际值，即为误差值
	
	M1.Out += M1.Kp * (M1.Error0 - M1.Error1) + M1.Ki * M1.Error0
			+ M1.Kd * (M1.Error0 - 2 * M1.Error1 + M1.Error2);
	
	/*输出限幅*/
	if (M1.Out > 100) {M1.Out = 100;}		//限制输出值最大为100
	if (M1.Out < -100) {M1.Out = -100;}	//限制输出值最小为100
	
	Motor1_SetSpeed(M1.Out);
}

void PID_SetSpeed2(int32_t Speed)
{
	M2.Target = Speed;
	M2.Actual = SSpeed2;
	
	/*获取本次误差、上次误差和上上次误差*/
	M2.Error2 = M2.Error1;			//获取上上次误差
	M2.Error1 = M2.Error0;			//获取上次误差
	M2.Error0 = M2.Target - M2.Actual;	//获取本次误差，目标值减实际值，即为误差值
	
	/*PID计算*/
	/*使用增量式PID公式，计算得到输出值*/
	M2.Out += M2.Kp * (M2.Error0 - M2.Error1) + M2.Ki * M2.Error0
			+ M2.Kd * (M2.Error0 - 2 * M2.Error1 + M2.Error2);
	
	/*输出限幅*/
	if (M2.Out > 100) {M2.Out = 100;}		//限制输出值最大为100
	if (M2.Out < -100) {M2.Out = -100;}	//限制输出值最小为100
	
	Motor2_SetSpeed(M2.Out);
}

void Task1(void)
{
	if (Serial_RxFlag == 1)
	{
		if (strncmp(Serial_RxPacket, "@speed%", sizeof(char) * 7))
		{
			int16_t num = 0, flag = 1, len = strlen(Serial_RxPacket), i = 6;
			if (Serial_RxPacket[i] == '-')
			{
				flag = -1;
				i++;
			}
			for (; i < len; i++)
			{
				num = num * 10 + Serial_RxPacket[i] - '0';
			}
			num *= flag;
			OLED_ShowSignedNum(4, 7, num, 4);
			Speed = num;
		}
		Serial_RxFlag = 0;
	}
	PID_SetSpeed1(Speed);
	PID_SetSpeed2(Speed);
	Serial_Printf("%d,%d,%d\n", 0, SSpeed1, SSpeed2);
}

void Task2(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
	}
	Angle1 = Encoder1_Get();
	PID_SetSpeed2(Angle1);
	Serial_Printf("%d,%d,%d\n", 0, SSpeed1, SSpeed2);
}

int Flag_Tim;

void TaskClear(void)
{
	Speed = 0;
	Angle1 = 0;
	Angle2 = 0;
	Motor1_SetSpeed(0);
	Motor2_SetSpeed(0);
	PID_Init_M1();
	PID_Init_M2();
}

int main(void)
{
	Key_Init();
	OLED_Init();
	Timer_Init();
	Encoder_Init();
	Motor_Init();
	Serial_Init();
	PID_Init_M1();
	PID_Init_M2();
	int TaskMode = 1;
	OLED_ShowString(1, 1, "Task1");
	while (1)
	{
		if (TaskMode == 1)
		{
			Task1();
		}
		else
		{
			Task2();
		}
		if (Key_Check(2, KEY_UP))
		{
			TaskMode ^= 1;
			if (TaskMode == 1)
			{
				OLED_ShowString(1, 1, "Task1");
			}
			else
			{
				OLED_ShowString(1, 1, "Task2");
			}
		}
		if (Flag_Tim)
		{
			Flag_Tim = 0;
		}
	}
}

void TIM1_UP_IRQHandler(void)
{
    static uint32_t Speed_Tim = 0;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) 
	{
		Key_Tick();
		Speed_Tim++;
		if (Speed_Tim >= 10)
		{
			Speed_Tim = 0;
			Flag_Tim = 0;
			SSpeed1 = Encoder1_Get();
			SSpeed2 = Encoder2_Get();
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
