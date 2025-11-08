#include "stm32f10x.h"

// 全局变量：计时毫秒数
uint32_t TIM1_TimeMs = 0;

void Timer_Init(void) {
    // 1. 使能时钟（APB2总线和TIM1）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    // 2. 配置时基参数
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 7200 - 1;       // 预分频器：72MHz/(7199+1)=10kHz
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStruct.TIM_Period = 10 - 1;             // 自动重装载值：10次计数=1ms
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频（计时无需配置）
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;  // 重复计数器（高级定时器特有，计时用0）
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
    
    // 3. 使能更新中断（溢出时触发）
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
    // 4. 配置NVIC中断（注意TIM1的中断通道名）
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_IRQn;    // TIM1更新中断通道（区别于TIM6）
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级（按需设置）
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;    // 子优先级（可与TIM6区分）
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 5. 启动定时器（高级定时器需额外使能计数器）
    TIM_Cmd(TIM1, ENABLE);
}
