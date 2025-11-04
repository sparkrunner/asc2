#include "stm32f10x.h"                  // Device header
#include "OLED.h"

uint16_t Row, Col;

void Print_Init(void)
{
	Row = 1;
	Col = 1;
}

void Print_String(char *str)
{
	OLED_ShowString(Row, Col, str);
	
}
