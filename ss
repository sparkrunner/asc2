warning: in the working copy of 'asc2/Project.uvoptx', LF will be replaced by CRLF the next time Git touches it
warning: in the working copy of 'asc2/Project.uvprojx', LF will be replaced by CRLF the next time Git touches it
[1mdiff --git a/asc2/Hardware/Encoder.c b/asc2/Hardware/Encoder.c[m
[1mindex ebba08b..e1876ac 100644[m
[1m--- a/asc2/Hardware/Encoder.c[m
[1m+++ b/asc2/Hardware/Encoder.c[m
[36m@@ -1,78 +1,42 @@[m
 #include "stm32f10x.h"                  // Device header[m
 [m
[31m-int16_t Encoder_Count;[m
[31m-[m
 void Encoder_Init(void)[m
 {[m
[31m-	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);[m
[31m-	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);[m
[32m+[m	[32mRCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);[m
[32m+[m	[32mRCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);[m
 	[m
 	GPIO_InitTypeDef GPIO_InitStructure;[m
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;[m
[31m-	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;[m
[32m+[m	[32mGPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;[m
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;[m
[31m-	GPIO_Init(GPIOB, &GPIO_InitStructure);[m
[31m-	[m
[31m-	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);[m
[31m-	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);[m
[32m+[m	[32mGPIO_Init(GPIOA, &GPIO_InitStructure);[m
[32m+[m[41m		[m
[32m+[m	[32mTIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;[m
[32m+[m	[32mTIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;[m
[32m+[m	[32mTIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;[m
[32m+[m	[32mTIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;		//ARR[m
[32m+[m	[32mTIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;		//PSC[m
[32m+[m	[32mTIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;[m
[32m+[m	[32mTIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);[m
 	[m
[31m-	EXTI_InitTypeDef EXTI_InitStructure;[m
[31m-	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;[m
[31m-	EXTI_InitStructure.EXTI_LineCmd = ENABLE;[m
[31m-	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;[m
[31m-	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;[m
[31m-	EXTI_Init(&EXTI_InitStructure);[m
[32m+[m	[32mTIM_ICInitTypeDef TIM_ICInitStructure;[m
[32m+[m	[32mTIM_ICStructInit(&TIM_ICInitStructure);[m
[32m+[m	[32mTIM_ICInitStructure.TIM_Channel = TIM_Channel_1;[m
[32m+[m	[32mTIM_ICInitStructure.TIM_ICFilter = 0xF;[m
[32m+[m	[32mTIM_ICInit(TIM3, &TIM_ICInitStructure);[m
[32m+[m	[32mTIM_ICInitStructure.TIM_Channel = TIM_Channel_2;[m
[32m+[m	[32mTIM_ICInitStructure.TIM_ICFilter = 0xF;[m
[32m+[m	[32mTIM_ICInit(TIM3, &TIM_ICInitStructure);[m
 	[m
[31m-	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);[m
[32m+[m	[32mTIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);[m
 	[m
[31m-	NVIC_InitTypeDef NVIC_InitStructure;[m
[31m-	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;[m
[31m-	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;[m
[31m-	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;[m
[31m-	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;[m
[31m-	NVIC_Init(&NVIC_InitStructure);[m
[31m-[m
[31m-	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;[m
[31m-	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;[m
[31m-	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;[m
[31m-	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;[m
[31m-	NVIC_Init(&NVIC_InitStructure);[m
[32m+[m	[32mTIM_Cmd(TIM3, ENABLE);[m
 }[m
 [m
 int16_t Encoder_Get(void)[m
 {[m
 	int16_t Temp;[m
[31m-	Temp = Encoder_Count;[m
[31m-	Encoder_Count = 0;[m
[32m+[m	[32mTemp = TIM_GetCounter(TIM3);[m
[32m+[m	[32mTIM_SetCounter(TIM3, 0);[m
 	return Temp;[m
 }[m
[31m-[m
[31m-void EXTI0_IRQHandler(void)[m
[31m-{[m
[31m-	if (EXTI_GetITStatus(EXTI_Line0) == SET)[m
[31m-	{[m
[31m-		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)[m
[31m-		{[m
[31m-			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)[m
[31m-			{[m
[31m-				Encoder_Count--;[m
[31m-			}[m
[31m-		}[m
[31m-		EXTI_ClearITPendingBit(EXTI_Line0);[m
[31m-	}[m
[31m-}[m
[31m-[m
[31m-void EXTI1_IRQHandler(void)[m
[31m-{[m
[31m-	if (EXTI_GetITStatus(EXTI_Line1) == SET)[m
[31m-	{[m
[31m-		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)[m
[31m-		{[m
[31m-			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)[m
[31m-			{[m
[31m-				Encoder_Count++;[m
[31m-			}[m
[31m-		}[m
[31m-		EXTI_ClearITPendingBit(EXTI_Line1);[m
[31m-	}[m
[31m-}[m
[1mdiff --git a/asc2/Listings/Project.map b/asc2/Listings/Project.map[m
[1mindex 2dddd4a..74f8e2f 100644[m
[1m--- a/asc2/Listings/Project.map[m
[1m+++ b/asc2/Listings/Project.map[m
[36m@@ -18,9 +18,8 @@[m [mSection Cross References[m
     startup_stm32f10x_md.o(RESET) refers to stm32f10x_it.o(i.DebugMon_Handler) for DebugMon_Handler[m
     startup_stm32f10x_md.o(RESET) refers to stm32f10x_it.o(i.PendSV_Handler) for PendSV_Handler[m
     startup_stm32f10x_md.o(RESET) refers to stm32f10x_it.o(i.SysTick_Handler) for SysTick_Handler[m
[31m-    startup_stm32f10x_md.o(RESET) refers to encoder.o(i.EXTI0_IRQHandler) for EXTI0_IRQHandler[m
[31m-    startup_stm32f10x_md.o(RESET) refers to encoder.o(i.EXTI1_IRQHandler) for EXTI1_IRQHandler[m
     startup_stm32f10x_md.o(RESET) refers to main.o(i.TIM2_IRQHandler) for TIM2_IRQHandler[m
[32m+[m[32m    startup_stm32f10x_md.o(RESET) refers to serial.o(i.USART1_IRQHandler) for USART1_IRQHandler[m
     startup_stm32f10x_md.o(.text) refers (Special) to heapauxi.o(.text) for __use_two_region_memory[m
     startup_stm32f10x_md.o(.text) refers to system_stm32f10x.o(i.SystemInit) for SystemInit[m
     startup_stm32f10x_md.o(.text) refers to __main.o(!!!main) for __main[m
[36m@@ -141,7 +140,7 @@[m [mSection Cross References[m
     oled.o(i.OLED_ShowFloatNum) refers to _printf_f.o(.ARM.Collect$$_printf_percent$$00000003) for _printf_f[m
     oled.o(i.OLED_ShowFloatNum) refers to printf1.o(x$fpl$printf1) for _printf_fp_dec[m
     oled.o(i.OLED_ShowFloatNum) refers to f2d.o(x$fpl$f2d) for __aeabi_f2d[m
[31m-    oled.o(i.OLED_ShowFloatNum) refers to noretval__2sprintf.o(.text) for __2sprintf[m
[32m+[m[32m    oled.o(i.OLED_ShowFloatNum) refers to __2sprintf.o(.text) for __2sprintf[m
     oled.o(i.OLED_ShowFloatNum) refers to oled.o(i.OLED_ShowString) for OLED_ShowString[m
     oled.o(i.OLED_ShowHexNum) refers to oled.o(i.OLED_Pow) for OLED_Pow[m
     oled.o(i.OLED_ShowHexNum) refers to oled.o(i.OLED_ShowChar) for OLED_ShowChar[m
[36m@@ -186,38 +185,86 @@[m [mSection Cross References[m
     printer.o(i.Print_Init) refers to printer.o(.data) for Row[m
     printer.o(i.Print_String) refers to oled.o(i.OLED_ShowString) for OLED_ShowString[m
     printer.o(i.Print_String) refers to printer.o(.data) for Col[m
[31m-    encoder.o(i.EXTI0_IRQHandler) refers to stm32f10x_exti.o(i.EXTI_GetITStatus) for EXTI_GetITStatus[m
[31m-    encoder.o(i.EXTI0_IRQHandler) refers to stm32f10x_gpio.o(i.GPIO_ReadInputDataBit) for GPIO_ReadInputDataBit[m
[31m-    encoder.o(i.EXTI0_IRQHandler) refers to stm32f10x_exti.o(i.EXTI_ClearITPendingBit) for EXTI_ClearITPendingBit[m
[31m-    encoder.o(i.EXTI0_IRQHandler) refers to encoder.o(.data) for Encoder_Count[m
[31m-    encoder.o(i.EXTI1_IRQHandler) refers to stm32f10x_exti.o(i.EXTI_GetITStatus) for EXTI_GetITStatus[m
[31m-    encoder.o(i.EXTI1_IRQHandler) refers to stm32f10x_gpio.o(i.GPIO_ReadInputDataBit) for GPIO_ReadInputDataBit[m
[31m-    encoder.o(i.EXTI1_IRQHandler) refers to stm32f10x_exti.o(i.EXTI_ClearITPendingBit) for EXTI_ClearITPendingBit[m
[31m-    encoder.o(i.EXTI1_IRQHandler) refers to encoder.o(.data) for Encoder_Count[m
[31m-    encoder.o(i.Encoder_Get) refers to encoder.o(.data) for Encoder_Count[m
[32m+[m[32m    encoder.o(i.Encoder_Get) refers to stm32f10x_tim.o(i.TIM_GetCounter) for TIM_GetCounter[m
[32m+[m[32m    encoder.o(i.Encoder_Get) refers to stm32f10x_tim.o(i.TIM_SetCounter) for TIM_SetCounter[m
[32m+[m[32m    encoder.o(i.Encoder_Init) refers to stm32f10x_rcc.o(i.RCC_APB1PeriphClockCmd) for RCC_APB1PeriphClockCmd[m
     encoder.o(i.Encoder_Init) refers to stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd) for RCC_APB2PeriphClockCmd[m
     encoder.o(i.Encoder_Init) refers to stm32f10x_gpio.o(i.GPIO_Init) for GPIO_Init[m
[31m-    encoder.o(i.Encoder_Init) refers to stm32f10x_gpio.o(i.GPIO_EXTILineConfig) for GPIO_EXTILineConfig[m
[31m-    encoder.o(i.Encoder_Init) refers to stm32f10x_exti.o(i.EXTI_Init) for EXTI_Init[m
[31m-    encoder.o(i.Encoder_Init) refers to misc.o(i.NVIC_PriorityGroupConfig) for NVIC_PriorityGroupConfig[m
[31m-    encoder.o(i.Encoder_Init) refers to misc.o(i.NVIC_Init) for NVIC_Init[m
[32m+[m[32m    encoder.o(i.Encoder_Init) refers to stm32f10x_tim.o(i.TIM_TimeBaseInit) for TIM_TimeBaseInit[m
[32m+[m[32m    encoder.o(i.Encoder_Init) refers to stm32f10x_tim.o(i.TIM_ICStructInit) for TIM_ICStructInit[m
[32m+[m[32m    encoder.o(i.Encoder_Init) refers to stm32f10x_tim.o(i.TIM_ICInit) for TIM_ICInit[m
[32m+[m[32m    encoder.o(i.Encoder_Init) refers to stm32f10x_tim.o(i.TIM_EncoderInterfaceConfig) for TIM_EncoderInterfaceConfig[m
[32m+[m[32m    encoder.o(i.Encoder_Init) refers to stm32f10x_tim.o(i.TIM_Cmd) for TIM_Cmd[m
[32m+[m[32m    serial.o(i.Serial_Init) refers to stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd) for RCC_APB2PeriphClockCmd[m
[32m+[m[32m    serial.o(i.Serial_Init) refers to stm32f10x_gpio.o(i.GPIO_Init) for GPIO_Init[m
[32m+[m[32m    serial.o(i.Serial_Init) refers to stm32f10x_usart.o(i.USART_Init) for USART_Init[m
[32m+[m[32m    serial.o(i.Serial_Init) refers to stm32f10x_usart.o(i.USART_ITConfig) for USART_ITConfig[m
[32m+[m[32m    serial.o(i.Serial_Init) refers to misc.o(i.NVIC_PriorityGroupConfig) for NVIC_PriorityGroupConfig[m
[32m+[m[32m    serial.o(i.Serial_Init) refers to misc.o(i.NVIC_Init) for NVIC_Init[m
[32m+[m[32m    serial.o(i.Serial_Init) refers to stm32f10x_usart.o(i.USART_Cmd) for USART_Cmd[m
[32m+[m[32m    serial.o(i.Serial_Printf) refers to vsprintf.o(.text) for vsprintf[m
[32m+[m[32m    serial.o(i.Serial_Printf) refers to serial.o(i.Serial_SendString) for Serial_SendString[m
[32m+[m[32m    serial.o(i.Serial_SendArray) refers to serial.o(i.Serial_SendByte) for Serial_SendByte[m
[32m+[m[32m    serial.o(i.Serial_SendByte) refers to stm32f10x_usart.o(i.USART_SendData) for USART_SendData[m
[32m+[m[32m    serial.o(i.Serial_SendByte) refers to stm32f10x_usart.o(i.USART_GetFlagStatus) for USART_GetFlagStatus[m
[32m+[m[32m    serial.o(i.Serial_SendNumber) refers to serial.o(i.Serial_Pow) for Serial_Pow[m
[32m+[m[32m    serial.o(i.Serial_SendNumber) refers to serial.o(i.Serial_SendByte) for Serial_SendByte[m
[32m+[m[32m    serial.o(i.Serial_SendString) refers to serial.o(i.Serial_SendByte) for Serial_SendByte[m
[32m+[m[32m    serial.o(i.USART1_IRQHandler) refers to stm32f10x_usart.o(i.USART_GetITStatus) for USART_GetITStatus[m
[32m+[m[32m    serial.o(i.USART1_IRQHandler) refers to stm32f10x_usart.o(i.USART_ReceiveData) for USART_ReceiveData[m
[32m+[m[32m    serial.o(i.USART1_IRQHandler) refers to stm32f10x_usart.o(i.USART_ClearITPendingBit) for USART_ClearITPendingBit[m
[32m+[m[32m    serial.o(i.USART1_IRQHandler) refers to serial.o(.data) for RxState[m
[32m+[m[32m    serial.o(i.USART1_IRQHandler) refers to serial.o(.bss) for Serial_RxPacket[m
[32m+[m[32m    serial.o(i.fputc) refers to serial.o(i.Serial_SendByte) for Serial_SendByte[m
     main.o(i.TIM2_IRQHandler) refers to stm32f10x_tim.o(i.TIM_GetITStatus) for TIM_GetITStatus[m
     main.o(i.TIM2_IRQHandler) refers to key.o(i.Key_Tick) for Key_Tick[m
[31m-    main.o(i.TIM2_IRQHandler) refers to led.o(i.LED_Tick) for LED_Tick[m
[32m+[m[32m    main.o(i.TIM2_IRQHandler) refers to encoder.o(i.Encoder_Get) for Encoder_Get[m
     main.o(i.TIM2_IRQHandler) refers to stm32f10x_tim.o(i.TIM_ClearITPendingBit) for TIM_ClearITPendingBit[m
[32m+[m[32m    main.o(i.TIM2_IRQHandler) refers to main.o(.data) for Speed_Tim[m
     main.o(i.main) refers to key.o(i.Key_Init) for Key_Init[m
[31m-    main.o(i.main) refers to led.o(i.LED_Init) for LED_Init[m
     main.o(i.main) refers to oled.o(i.OLED_Init) for OLED_Init[m
     main.o(i.main) refers to timer.o(i.Timer_Init) for Timer_Init[m
     main.o(i.main) refers to encoder.o(i.Encoder_Init) for Encoder_Init[m
[31m-    main.o(i.main) refers to menu.o(i.Menu_Init) for Menu_Init[m
[31m-    main.o(i.main) refers to menu.o(i.Menu_Show) for Menu_Show[m
[31m-    main.o(i.main) refers to key.o(i.Key_GetNum) for Key_GetNum[m
[31m-    main.o(i.main) refers to menu.o(i.Menu_Option) for Menu_Option[m
[31m-    main.o(i.main) refers to menu.o(i.Menu_LED_Direction) for Menu_LED_Direction[m
[31m-    main.o(i.main) refers to led.o(i.LED_DirSet) for LED_DirSet[m
[31m-    main.o(i.main) refers to menu.o(i.Menu_LED_Speed) for Menu_LED_Speed[m
[31m-    main.o(i.main) refers to led.o(i.LED_SpeedSet) for LED_SpeedSet[m
[32m+[m[32m    main.o(i.main) refers to serial.o(i.Serial_Init) for Serial_Init[m
[32m+[m[32m    main.o(i.main) refers to oled.o(i.OLED_ShowSignedNum) for OLED_ShowSignedNum[m
[32m+[m[32m    main.o(i.main) refers to serial.o(i.Serial_Printf) for Serial_Printf[m
[32m+[m[32m    main.o(i.main) refers to main.o(.data) for Speed[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_a.o(.ARM.Collect$$_printf_percent$$00000006) for _printf_a[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_c.o(.ARM.Collect$$_printf_percent$$00000013) for _printf_c[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_charcount.o(.text) for _printf_charcount[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_d.o(.ARM.Collect$$_printf_percent$$00000009) for _printf_d[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_e.o(.ARM.Collect$$_printf_percent$$00000004) for _printf_e[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_f.o(.ARM.Collect$$_printf_percent$$00000003) for _printf_f[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to printf1.o(x$fpl$printf1) for _printf_fp_dec[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to printf2.o(x$fpl$printf2) for _printf_fp_hex[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_g.o(.ARM.Collect$$_printf_percent$$00000005) for _printf_g[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_i.o(.ARM.Collect$$_printf_percent$$00000008) for _printf_i[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_dec.o(.text) for _printf_int_dec[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_l.o(.ARM.Collect$$_printf_percent$$00000012) for _printf_l[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_lc.o(.ARM.Collect$$_printf_percent$$00000015) for _printf_lc[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007) for _printf_ll[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_lld.o(.ARM.Collect$$_printf_percent$$0000000E) for _printf_lld[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_lli.o(.ARM.Collect$$_printf_percent$$0000000D) for _printf_lli[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_llo.o(.ARM.Collect$$_printf_percent$$00000010) for _printf_llo[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_llu.o(.ARM.Collect$$_printf_percent$$0000000F) for _printf_llu[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_llx.o(.ARM.Collect$$_printf_percent$$00000011) for _printf_llx[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_longlong_dec.o(.text) for _printf_longlong_dec[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_hex_int_ll_ptr.o(.text) for _printf_longlong_hex[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_oct_int_ll.o(.text) for _printf_longlong_oct[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_ls.o(.ARM.Collect$$_printf_percent$$00000016) for _printf_ls[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_n.o(.ARM.Collect$$_printf_percent$$00000001) for _printf_n[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_o.o(.ARM.Collect$$_printf_percent$$0000000B) for _printf_o[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_p.o(.ARM.Collect$$_printf_percent$$00000002) for _printf_p[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_percent.o(.ARM.Collect$$_printf_percent$$00000000) for _printf_percent[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_pad.o(.text) for _printf_post_padding[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_s.o(.ARM.Collect$$_printf_percent$$00000014) for _printf_s[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_str.o(.text) for _printf_str[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_truncate.o(.text) for _printf_truncate_signed[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_u.o(.ARM.Collect$$_printf_percent$$0000000A) for _printf_u[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_wctomb.o(.text) for _printf_wctomb[m
[32m+[m[32m    vsprintf.o(.text) refers (Special) to _printf_x.o(.ARM.Collect$$_printf_percent$$0000000C) for _printf_x[m
[32m+[m[32m    vsprintf.o(.text) refers to _printf_char_common.o(.text) for _printf_char_common[m
[32m+[m[32m    vsprintf.o(.text) refers to _sputc.o(.text) for _sputc[m
     __2sprintf.o(.text) refers to _printf_char_common.o(.text) for _printf_char_common[m
     __2sprintf.o(.text) refers to _sputc.o(.text) for _sputc[m
     noretval__2sprintf.o(.text) refers to _printf_char_common.o(.text) for _printf_char_common[m
[36m@@ -259,6 +306,12 @@[m [mSection Cross References[m
     __rtentry.o(.ARM.Collect$$rtentry$$00000000) refers (Special) to __rtentry2.o(.ARM.Collect$$rtentry$$00000009) for __rt_entry_postsh_1[m
     __rtentry.o(.ARM.Collect$$rtentry$$00000000) refers (Special) to __rtentry2.o(.ARM.Collect$$rtentry$$00000002) for __rt_entry_presh_1[m
     __rtentry.o(.ARM.Collect$$rtentry$$00000000) refers (Special) to __rtentry4.o(.ARM.Collect$$rtentry$$00000004) for __rt_entry_sh[m
[32m+[m[32m    _printf_str.o(.text) refers (Special) to _printf_char.o(.text) for _printf_cs_common[m
[32m+[m[32m    _printf_str.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_str.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_post_padding[m
[32m+[m[32m    _printf_dec.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_signed[m
[32m+[m[32m    _printf_dec.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_unsigned[m
[32m+[m[32m    _printf_dec.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
     _printf_fp_dec.o(.text) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
     _printf_fp_dec.o(.text) refers (Special) to lc_numeric_c.o(locale$$code) for _get_lc_numeric[m
     _printf_fp_dec.o(.text) refers to bigflt0.o(.text) for _btod_etento[m
[36m@@ -268,11 +321,73 @@[m [mSection Cross References[m
     _printf_fp_dec.o(.text) refers to lludiv10.o(.text) for _ll_udiv10[m
     _printf_fp_dec.o(.text) refers to fpclassify.o(i.__ARM_fpclassify) for __ARM_fpclassify[m
     _printf_fp_dec.o(.text) refers to _printf_fp_infnan.o(.text) for _printf_fp_infnan[m
[32m+[m[32m    _printf_fp_dec.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_fp_dec.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
     _printf_fp_dec.o(.text) refers to rt_locale_intlibspace.o(.text) for __rt_locale[m
[31m-    _printf_char_common.o(.text) refers to __printf_wp.o(.text) for __printf[m
[32m+[m[32m    _printf_fp_dec.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_post_padding[m
[32m+[m[32m    _printf_char_common.o(.text) refers to __printf_flags_ss_wp.o(.text) for __printf[m
[32m+[m[32m    _printf_wctomb.o(.text) refers (Special) to _printf_wchar.o(.text) for _printf_lcs_common[m
[32m+[m[32m    _printf_wctomb.o(.text) refers to _wcrtomb.o(.text) for _wcrtomb[m
[32m+[m[32m    _printf_wctomb.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_wctomb.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_post_padding[m
[32m+[m[32m    _printf_wctomb.o(.text) refers to _printf_wctomb.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_wctomb.o(.constdata) refers (Special) to _printf_wchar.o(.text) for _printf_lcs_common[m
[32m+[m[32m    _printf_longlong_dec.o(.text) refers to lludiv10.o(.text) for _ll_udiv10[m
[32m+[m[32m    _printf_longlong_dec.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_oct_ll.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_oct_int.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_unsigned[m
[32m+[m[32m    _printf_oct_int.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_oct_int_ll.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_oct_int_ll.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_unsigned[m
[32m+[m[32m    _printf_hex_ll.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_hex_ll.o(.text) refers to _printf_hex_ll.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_hex_int.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_unsigned[m
[32m+[m[32m    _printf_hex_int.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_hex_int.o(.text) refers to _printf_hex_int.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_hex_int_ll.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_hex_int_ll.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_unsigned[m
[32m+[m[32m    _printf_hex_int_ll.o(.text) refers to _printf_hex_int_ll.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_hex_ptr.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_hex_ptr.o(.text) refers to _printf_hex_ptr.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_hex_int_ptr.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_hex_int_ptr.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_unsigned[m
[32m+[m[32m    _printf_hex_int_ptr.o(.text) refers to _printf_hex_int_ptr.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_hex_ll_ptr.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_hex_ll_ptr.o(.text) refers to _printf_hex_ll_ptr.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_hex_int_ll_ptr.o(.text) refers to _printf_intcommon.o(.text) for _printf_int_common[m
[32m+[m[32m    _printf_hex_int_ll_ptr.o(.text) refers (Weak) to _printf_truncate.o(.text) for _printf_truncate_unsigned[m
[32m+[m[32m    _printf_hex_int_ll_ptr.o(.text) refers to _printf_hex_int_ll_ptr.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_c.o(.ARM.Collect$$_printf_percent$$00000013) refers (Weak) to _printf_char.o(.text) for _printf_char[m
[32m+[m[32m    _printf_s.o(.ARM.Collect$$_printf_percent$$00000014) refers (Weak) to _printf_char.o(.text) for _printf_string[m
[32m+[m[32m    _printf_n.o(.ARM.Collect$$_printf_percent$$00000001) refers (Weak) to _printf_charcount.o(.text) for _printf_charcount[m
[32m+[m[32m    _printf_x.o(.ARM.Collect$$_printf_percent$$0000000C) refers (Weak) to _printf_hex_int_ll_ptr.o(.text) for _printf_int_hex[m
[32m+[m[32m    _printf_p.o(.ARM.Collect$$_printf_percent$$00000002) refers (Weak) to _printf_hex_int_ll_ptr.o(.text) for _printf_hex_ptr[m
[32m+[m[32m    _printf_o.o(.ARM.Collect$$_printf_percent$$0000000B) refers (Weak) to _printf_oct_int_ll.o(.text) for _printf_int_oct[m
[32m+[m[32m    _printf_i.o(.ARM.Collect$$_printf_percent$$00000008) refers (Weak) to _printf_dec.o(.text) for _printf_int_dec[m
[32m+[m[32m    _printf_d.o(.ARM.Collect$$_printf_percent$$00000009) refers (Weak) to _printf_dec.o(.text) for _printf_int_dec[m
[32m+[m[32m    _printf_u.o(.ARM.Collect$$_printf_percent$$0000000A) refers (Weak) to _printf_dec.o(.text) for _printf_int_dec[m
[32m+[m[32m    _printf_e.o(.ARM.Collect$$_printf_percent$$00000004) refers (Weak) to printf1.o(x$fpl$printf1) for _printf_fp_dec[m
[32m+[m[32m    _printf_g.o(.ARM.Collect$$_printf_percent$$00000005) refers (Weak) to printf1.o(x$fpl$printf1) for _printf_fp_dec[m
[32m+[m[32m    _printf_a.o(.ARM.Collect$$_printf_percent$$00000006) refers (Weak) to printf2.o(x$fpl$printf2) for _printf_fp_hex[m
[32m+[m[32m    _printf_lli.o(.ARM.Collect$$_printf_percent$$0000000D) refers (Special) to _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007) for _printf_ll[m
[32m+[m[32m    _printf_lli.o(.ARM.Collect$$_printf_percent$$0000000D) refers (Weak) to _printf_longlong_dec.o(.text) for _printf_longlong_dec[m
[32m+[m[32m    _printf_lld.o(.ARM.Collect$$_printf_percent$$0000000E) refers (Special) to _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007) for _printf_ll[m
[32m+[m[32m    _printf_lld.o(.ARM.Collect$$_printf_percent$$0000000E) refers (Weak) to _printf_longlong_dec.o(.text) for _printf_longlong_dec[m
[32m+[m[32m    _printf_llu.o(.ARM.Collect$$_printf_percent$$0000000F) refers (Special) to _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007) for _printf_ll[m
[32m+[m[32m    _printf_llu.o(.ARM.Collect$$_printf_percent$$0000000F) refers (Weak) to _printf_longlong_dec.o(.text) for _printf_longlong_dec[m
[32m+[m[32m    _printf_lc.o(.ARM.Collect$$_printf_percent$$00000015) refers (Special) to _printf_l.o(.ARM.Collect$$_printf_percent$$00000012) for _printf_l[m
[32m+[m[32m    _printf_lc.o(.ARM.Collect$$_printf_percent$$00000015) refers (Weak) to _printf_wchar.o(.text) for _printf_wchar[m
[32m+[m[32m    _printf_ls.o(.ARM.Collect$$_printf_percent$$00000016) refers (Special) to _printf_l.o(.ARM.Collect$$_printf_percent$$00000012) for _printf_l[m
[32m+[m[32m    _printf_ls.o(.ARM.Collect$$_printf_percent$$00000016) refers (Weak) to _printf_wchar.o(.text) for _printf_wstring[m
[32m+[m[32m    _printf_llo.o(.ARM.Collect$$_printf_percent$$00000010) refers (Special) to _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007) for _printf_ll[m
[32m+[m[32m    _printf_llo.o(.ARM.Collect$$_printf_percent$$00000010) refers (Weak) to _printf_oct_int_ll.o(.text) for _printf_ll_oct[m
[32m+[m[32m    _printf_llx.o(.ARM.Collect$$_printf_percent$$00000011) refers (Special) to _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007) for _printf_ll[m
[32m+[m[32m    _printf_llx.o(.ARM.Collect$$_printf_percent$$00000011) refers (Weak) to _printf_hex_int_ll_ptr.o(.text) for _printf_ll_hex[m
     dretinf.o(x$fpl$dretinf) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
     fnaninf.o(x$fpl$fnaninf) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
     fretinf.o(x$fpl$fretinf) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
[32m+[m[32m    printf2.o(x$fpl$printf2) refers to _printf_fp_hex.o(.text) for _printf_fp_hex_real[m
[32m+[m[32m    printf2b.o(x$fpl$printf2) refers to _printf_fp_hex.o(.text) for _printf_fp_hex_real[m
     __rtentry2.o(.ARM.Collect$$rtentry$$00000008) refers to boardinit2.o(.text) for _platform_post_stackheap_init[m
     __rtentry2.o(.ARM.Collect$$rtentry$$0000000A) refers to libinit.o(.ARM.Collect$$libinit$$00000000) for __rt_lib_init[m
     __rtentry2.o(.ARM.Collect$$rtentry$$0000000B) refers to boardinit3.o(.text) for _platform_post_lib_init[m
[36m@@ -287,6 +402,21 @@[m [mSection Cross References[m
     __rtentry4.o(.ARM.exidx) refers to __rtentry4.o(.ARM.Collect$$rtentry$$00000004) for .ARM.Collect$$rtentry$$00000004[m
     rt_locale.o(.text) refers to rt_locale.o(.bss) for __rt_locale_data[m
     rt_locale_intlibspace.o(.text) refers to libspace.o(.bss) for __libspace_start[m
[32m+[m[32m    _printf_intcommon.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_intcommon.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_intcommon.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_post_padding[m
[32m+[m[32m    _printf_fp_hex.o(.text) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
[32m+[m[32m    _printf_fp_hex.o(.text) refers to fpclassify.o(i.__ARM_fpclassify) for __ARM_fpclassify[m
[32m+[m[32m    _printf_fp_hex.o(.text) refers to _printf_fp_infnan.o(.text) for _printf_fp_infnan[m
[32m+[m[32m    _printf_fp_hex.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_fp_hex.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_fp_hex.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_post_padding[m
[32m+[m[32m    _printf_fp_hex.o(.text) refers to _printf_fp_hex.o(.constdata) for .constdata[m
[32m+[m[32m    _printf_fp_hex.o(.constdata) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
[32m+[m[32m    _printf_fp_infnan.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_pre_padding[m
[32m+[m[32m    _printf_fp_infnan.o(.text) refers (Weak) to _printf_pad.o(.text) for _printf_post_padding[m
[32m+[m[32m    _printf_char.o(.text) refers (Weak) to _printf_str.o(.text) for _printf_str[m
[32m+[m[32m    _printf_wchar.o(.text) refers (Weak) to _printf_wctomb.o(.text) for _printf_wctomb[m
     bigflt0.o(.text) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
     bigflt0.o(.text) refers to btod.o(CL$$btod_emul) for _btod_emul[m
     bigflt0.o(.text) refers to btod.o(CL$$btod_ediv) for _btod_ediv[m
[36m@@ -314,6 +444,7 @@[m [mSection Cross References[m
     btod.o(CL$$btod_e2d) refers to btod.o(CL$$btod_e2e) for _e2e[m
     btod.o(CL$$btod_mult_common) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
     btod.o(CL$$btod_div_common) refers (Special) to usenofp.o(x$fpl$usenofp) for __I$use$fp[m
[32m+[m[32m    _wcrtomb.o(.text) refers to rt_ctype_table.o(.text) for __rt_ctype_table[m
     lc_numeric_c.o(locale$$data) refers (Special) to libinit2.o(.ARM.Collect$$libinit$$00000016) for __rt_lib_init_lc_numeric_2[m
     lc_numeric_c.o(locale$$code) refers (Special) to libinit2.o(.ARM.Collect$$libinit$$00000016) for __rt_lib_init_lc_numeric_2[m
     lc_numeric_c.o(locale$$code) refers to strcmpv7m.o(.text) for strcmp[m
[36m@@ -323,6 +454,8 @@[m [mSection Cross References[m
     libspace.o(.text) refers to libspace.o(.bss) for __libspace_start[m
     sys_stackheap_outer.o(.text) refers to libspace.o(.text) for __user_perproc_libspace[m
     sys_stackheap_outer.o(.text) refers to startup_stm32f10x_md.o(.text) for __user_initial_stackheap[m
[32m+[m[32m    rt_ctype_table.o(.text) refers to rt_locale_intlibspace.o(.text) for __rt_locale[m
[32m+[m[32m    rt_ctype_table.o(.text) refers to lc_ctype_c.o(locale$$code) for _get_lc_ctype[m
     exit.o(.text) refers to rtexit.o(.ARM.Collect$$rtexit$$00000000) for __rt_exit[m
     libinit.o(.ARM.Collect$$libinit$$00000000) refers (Special) to libinit2.o(.ARM.Collect$$libinit$$0000002E) for __rt_lib_init_alloca_1[m
     libinit.o(.ARM.Collect$$libinit$$00000000) refers (Special) to libinit2.o(.ARM.Collect$$libinit$$0000002C) for __rt_lib_init_argv_1[m
[36m@@ -348,6 +481,7 @@[m [mSection Cross References[m
     libinit2.o(.ARM.Collect$$libinit$$0000000F) refers (Weak) to rt_locale_intlibspace.o(.text) for __rt_locale[m
     libinit2.o(.ARM.Collect$$libinit$$00000010) refers to libinit2.o(.ARM.Collect$$libinit$$0000000F) for .ARM.Collect$$libinit$$0000000F[m
     libinit2.o(.ARM.Collect$$libinit$$00000012) refers to libinit2.o(.ARM.Collect$$libinit$$0000000F) for .ARM.Collect$$libinit$$0000000F[m
[32m+[m[32m    libinit2.o(.ARM.Collect$$libinit$$00000012) refers (Weak) to lc_ctype_c.o(locale$$code) for _get_lc_ctype[m
     libinit2.o(.ARM.Collect$$libinit$$00000014) refers to libinit2.o(.ARM.Collect$$libinit$$0000000F) for .ARM.Collect$$libinit$$0000000F[m
     libinit2.o(.ARM.Collect$$libinit$$00000016) refers to libinit2.o(.ARM.Collect$$libinit$$0000000F) for .ARM.Collect$$libinit$$0000000F[m
     libinit2.o(.ARM.Collect$$libinit$$00000016) refers (Weak) to lc_numeric_c.o(locale$$code) for _get_lc_numeric[m
[36m@@ -361,6 +495,10 @@[m [mSection Cross References[m
     rtexit.o(.ARM.exidx) refers (Special) to rtexit2.o(.ARM.Collect$$rtexit$$00000003) for __rt_exit_ls[m
     rtexit.o(.ARM.exidx) refers (Special) to rtexit2.o(.ARM.Collect$$rtexit$$00000002) for __rt_exit_prels_1[m
     rtexit.o(.ARM.exidx) refers to rtexit.o(.ARM.Collect$$rtexit$$00000000) for .ARM.Collect$$rtexit$$00000000[m
[32m+[m[32m    lc_ctype_c.o(locale$$data) refers (Special) to libinit2.o(.ARM.Collect$$libinit$$00000012) for __rt_lib_init_lc_ctype_2[m
[32m+[m[32m    lc_ctype_c.o(locale$$code) refers (Special) to libinit2.o(.ARM.Collect$$libinit$$00000012) for __rt_lib_init_lc_ctype_2[m
[32m+[m[32m    lc_ctype_c.o(locale$$code) refers to strcmpv7m.o(.text) for strcmp[m
[32m+[m[32m    lc_ctype_c.o(locale$$code) refers to lc_ctype_c.o(locale$$data) for __lcctype_c_name[m
     argv_veneer.o(.emb_text) refers to no_argv.o(.text) for __ARM_get_argv[m
     rtexit2.o(.ARM.Collect$$rtexit$$00000003) refers to libshutdown.o(.ARM.Collect$$libshutdown$$00000000) for __rt_lib_shutdown[m
     rtexit2.o(.ARM.Collect$$rtexit$$00000004) refers to sys_exit.o(.text) for _sys_exit[m
[36m@@ -419,27 +557,19 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_wwdg.o(i.WWDG_SetPrescaler), (24 bytes).[m
     Removing stm32f10x_wwdg.o(i.WWDG_SetWindowValue), (40 bytes).[m
     Removing stm32f10x_usart.o(i.USART_ClearFlag), (18 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_ClearITPendingBit), (30 bytes).[m
     Removing stm32f10x_usart.o(i.USART_ClockInit), (34 bytes).[m
     Removing stm32f10x_usart.o(i.USART_ClockStructInit), (12 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_Cmd), (24 bytes).[m
     Removing stm32f10x_usart.o(i.USART_DMACmd), (18 bytes).[m
     Removing stm32f10x_usart.o(i.USART_DeInit), (156 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_GetFlagStatus), (26 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_GetITStatus), (84 bytes).[m
     Removing stm32f10x_usart.o(i.USART_HalfDuplexCmd), (24 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_ITConfig), (74 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_Init), (216 bytes).[m
     Removing stm32f10x_usart.o(i.USART_IrDACmd), (24 bytes).[m
     Removing stm32f10x_usart.o(i.USART_IrDAConfig), (18 bytes).[m
     Removing stm32f10x_usart.o(i.USART_LINBreakDetectLengthConfig), (18 bytes).[m
     Removing stm32f10x_usart.o(i.USART_LINCmd), (24 bytes).[m
     Removing stm32f10x_usart.o(i.USART_OneBitMethodCmd), (24 bytes).[m
     Removing stm32f10x_usart.o(i.USART_OverSampling8Cmd), (22 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_ReceiveData), (10 bytes).[m
     Removing stm32f10x_usart.o(i.USART_ReceiverWakeUpCmd), (24 bytes).[m
     Removing stm32f10x_usart.o(i.USART_SendBreak), (10 bytes).[m
[31m-    Removing stm32f10x_usart.o(i.USART_SendData), (8 bytes).[m
     Removing stm32f10x_usart.o(i.USART_SetAddress), (18 bytes).[m
     Removing stm32f10x_usart.o(i.USART_SetGuardTime), (16 bytes).[m
     Removing stm32f10x_usart.o(i.USART_SetPrescaler), (16 bytes).[m
[36m@@ -447,10 +577,6 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_usart.o(i.USART_SmartCardNACKCmd), (24 bytes).[m
     Removing stm32f10x_usart.o(i.USART_StructInit), (24 bytes).[m
     Removing stm32f10x_usart.o(i.USART_WakeUpConfig), (18 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TI1_Config), (128 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TI2_Config), (152 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TI3_Config), (144 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TI4_Config), (152 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_ARRPreloadConfig), (24 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_BDTRConfig), (32 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_BDTRStructInit), (18 bytes).[m
[36m@@ -469,7 +595,6 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_tim.o(i.TIM_ETRClockMode1Config), (54 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_ETRClockMode2Config), (32 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_ETRConfig), (28 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_EncoderInterfaceConfig), (66 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_ForcedOC1Config), (18 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_ForcedOC2Config), (26 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_ForcedOC3Config), (18 bytes).[m
[36m@@ -479,11 +604,8 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_tim.o(i.TIM_GetCapture2), (6 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_GetCapture3), (6 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_GetCapture4), (8 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_GetCounter), (6 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_GetFlagStatus), (18 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_GetPrescaler), (6 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_ICInit), (172 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_ICStructInit), (18 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_ITRxExternalClockConfig), (24 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_OC1FastConfig), (18 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_OC1Init), (152 bytes).[m
[36m@@ -522,11 +644,6 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_tim.o(i.TIM_SetCompare2), (4 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_SetCompare3), (4 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_SetCompare4), (6 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_SetCounter), (4 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_SetIC1Prescaler), (18 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_SetIC2Prescaler), (26 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_SetIC3Prescaler), (18 bytes).[m
[31m-    Removing stm32f10x_tim.o(i.TIM_SetIC4Prescaler), (26 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_TIxExternalClockConfig), (62 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_TimeBaseStructInit), (18 bytes).[m
     Removing stm32f10x_tim.o(i.TIM_UpdateDisableConfig), (24 bytes).[m
[36m@@ -608,7 +725,6 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_rcc.o(i.RCC_ClearITPendingBit), (12 bytes).[m
     Removing stm32f10x_rcc.o(i.RCC_ClockSecuritySystemCmd), (12 bytes).[m
     Removing stm32f10x_rcc.o(i.RCC_DeInit), (76 bytes).[m
[31m-    Removing stm32f10x_rcc.o(i.RCC_GetClocksFreq), (212 bytes).[m
     Removing stm32f10x_rcc.o(i.RCC_GetFlagStatus), (60 bytes).[m
     Removing stm32f10x_rcc.o(i.RCC_GetITStatus), (24 bytes).[m
     Removing stm32f10x_rcc.o(i.RCC_GetSYSCLKSource), (16 bytes).[m
[36m@@ -628,7 +744,6 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_rcc.o(i.RCC_SYSCLKConfig), (24 bytes).[m
     Removing stm32f10x_rcc.o(i.RCC_USBCLKConfig), (12 bytes).[m
     Removing stm32f10x_rcc.o(i.RCC_WaitForHSEStartUp), (56 bytes).[m
[31m-    Removing stm32f10x_rcc.o(.data), (20 bytes).[m
     Removing stm32f10x_pwr.o(i.PWR_BackupAccessCmd), (12 bytes).[m
     Removing stm32f10x_pwr.o(i.PWR_ClearFlag), (20 bytes).[m
     Removing stm32f10x_pwr.o(i.PWR_DeInit), (22 bytes).[m
[36m@@ -680,6 +795,7 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_gpio.o(i.GPIO_AFIODeInit), (20 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_DeInit), (200 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_ETH_MediaInterfaceConfig), (12 bytes).[m
[32m+[m[32m    Removing stm32f10x_gpio.o(i.GPIO_EXTILineConfig), (64 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_EventOutputCmd), (12 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_EventOutputConfig), (32 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_PinLockConfig), (18 bytes).[m
[36m@@ -688,6 +804,7 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_gpio.o(i.GPIO_ReadOutputData), (8 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_ReadOutputDataBit), (18 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_ResetBits), (4 bytes).[m
[32m+[m[32m    Removing stm32f10x_gpio.o(i.GPIO_SetBits), (4 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_StructInit), (16 bytes).[m
     Removing stm32f10x_gpio.o(i.GPIO_Write), (4 bytes).[m
     Removing stm32f10x_fsmc.o(i.FSMC_ClearFlag), (64 bytes).[m
[36m@@ -738,9 +855,12 @@[m [mRemoving Unused input sections from the image.[m
     Removing stm32f10x_flash.o(i.FLASH_WaitForLastBank1Operation), (38 bytes).[m
     Removing stm32f10x_flash.o(i.FLASH_WaitForLastOperation), (38 bytes).[m
     Removing stm32f10x_exti.o(i.EXTI_ClearFlag), (12 bytes).[m
[32m+[m[32m    Removing stm32f10x_exti.o(i.EXTI_ClearITPendingBit), (12 bytes).[m
     Removing stm32f10x_exti.o(i.EXTI_DeInit), (36 bytes).[m
     Removing stm32f10x_exti.o(i.EXTI_GenerateSWInterrupt), (16 bytes).[m
     Removing stm32f10x_exti.o(i.EXTI_GetFlagStatus), (24 bytes).[m
[32m+[m[32m    Removing stm32f10x_exti.o(i.EXTI_GetITStatus), (40 bytes).[m
[32m+[m[32m    Removing stm32f10x_exti.o(i.EXTI_Init), (148 bytes).[m
     Removing stm32f10x_exti.o(i.EXTI_StructInit), (16 bytes).[m
     Removing stm32f10x_dma.o(i.DMA_ClearFlag), (28 bytes).[m
     Removing stm32f10x_dma.o(i.DMA_ClearITPendingBit), (28 bytes).[m
[36m@@ -867,15 +987,41 @@[m [mRemoving Unused input sections from the image.[m
     Removing delay.o(i.Delay_ms), (24 bytes).[m
     Removing delay.o(i.Delay_s), (24 bytes).[m
     Removing delay.o(i.Delay_us), (46 bytes).[m
[32m+[m[32m    Removing led.o(i.FloodLED_LeftTurn), (96 bytes).[m
[32m+[m[32m    Removing led.o(i.FloodLED_RightTurn), (96 bytes).[m
[32m+[m[32m    Removing led.o(i.LED_DirSet), (12 bytes).[m
[32m+[m[32m    Removing led.o(i.LED_Init), (56 bytes).[m
[32m+[m[32m    Removing led.o(i.LED_SpeedSet), (20 bytes).[m
[32m+[m[32m    Removing led.o(i.LED_Tick), (60 bytes).[m
[32m+[m[32m    Removing led.o(.constdata), (18 bytes).[m
[32m+[m[32m    Removing led.o(.data), (12 bytes).[m
[32m+[m[32m    Removing key.o(i.Key_Check), (36 bytes).[m
[32m+[m[32m    Removing key.o(i.Key_GetNum), (62 bytes).[m
     Removing oled.o(i.OLED_ShowBinNum), (62 bytes).[m
[32m+[m[32m    Removing oled.o(i.OLED_ShowFloatNum), (60 bytes).[m
     Removing oled.o(i.OLED_ShowHexNum), (84 bytes).[m
[31m-    Removing oled.o(i.OLED_ShowSignedNum), (102 bytes).[m
[32m+[m[32m    Removing oled.o(i.OLED_ShowNum), (68 bytes).[m
[32m+[m[32m    Removing oled.o(i.OLED_ShowString), (40 bytes).[m
[32m+[m[32m    Removing menu.o(i.Menu_Init), (284 bytes).[m
[32m+[m[32m    Removing menu.o(i.Menu_LED_Direction), (20 bytes).[m
[32m+[m[32m    Removing menu.o(i.Menu_LED_Speed), (20 bytes).[m
     Removing menu.o(i.Menu_Mode), (12 bytes).[m
[32m+[m[32m    Removing menu.o(i.Menu_Option), (396 bytes).[m
[32m+[m[32m    Removing menu.o(i.Menu_Show), (292 bytes).[m
[32m+[m[32m    Removing menu.o(i.Numlen), (28 bytes).[m
[32m+[m[32m    Removing menu.o(.bss), (320 bytes).[m
[32m+[m[32m    Removing menu.o(.constdata), (320 bytes).[m
[32m+[m[32m    Removing menu.o(.conststring), (83 bytes).[m
[32m+[m[32m    Removing menu.o(.data), (60 bytes).[m
     Removing printer.o(i.Print_Init), (20 bytes).[m
     Removing printer.o(i.Print_String), (28 bytes).[m
     Removing printer.o(.data), (4 bytes).[m
[32m+[m[32m    Removing serial.o(i.Serial_Pow), (20 bytes).[m
[32m+[m[32m    Removing serial.o(i.Serial_SendArray), (26 bytes).[m
[32m+[m[32m    Removing serial.o(i.Serial_SendNumber), (58 bytes).[m
[32m+[m[32m    Removing serial.o(i.fputc), (16 bytes).[m
 [m
[31m-467 unused section(s) (total 18784 bytes) removed from the image.[m
[32m+[m[32m475 unused section(s) (total 19895 bytes) removed from the image.[m
 [m
 ==============================================================================[m
 [m
[36m@@ -886,77 +1032,123 @@[m [mImage Symbol Table[m
     Symbol Name                              Value     Ov Type        Size  Object(Section)[m
 [m
     ../clib/angel/boardlib.s                 0x00000000   Number         0  boardinit3.o ABSOLUTE[m
[31m-    ../clib/angel/boardlib.s                 0x00000000   Number         0  boardshut.o ABSOLUTE[m
     ../clib/angel/boardlib.s                 0x00000000   Number         0  boardinit2.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/boardlib.s                 0x00000000   Number         0  boardshut.o ABSOLUTE[m
     ../clib/angel/boardlib.s                 0x00000000   Number         0  boardinit1.o ABSOLUTE[m
     ../clib/angel/handlers.s                 0x00000000   Number         0  __scatter_copy.o ABSOLUTE[m
     ../clib/angel/handlers.s                 0x00000000   Number         0  __scatter_zi.o ABSOLUTE[m
[31m-    ../clib/angel/kernel.s                   0x00000000   Number         0  __rtentry2.o ABSOLUTE[m
[31m-    ../clib/angel/kernel.s                   0x00000000   Number         0  rtexit2.o ABSOLUTE[m
[31m-    ../clib/angel/kernel.s                   0x00000000   Number         0  __rtentry.o ABSOLUTE[m
     ../clib/angel/kernel.s                   0x00000000   Number         0  rtexit.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/kernel.s                   0x00000000   Number         0  __rtentry2.o ABSOLUTE[m
     ../clib/angel/kernel.s                   0x00000000   Number         0  __rtentry4.o ABSOLUTE[m
[31m-    ../clib/angel/rt.s                       0x00000000   Number         0  rt_raise.o ABSOLUTE[m
[31m-    ../clib/angel/rt.s                       0x00000000   Number         0  rt_locale_intlibspace.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/kernel.s                   0x00000000   Number         0  __rtentry.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/kernel.s                   0x00000000   Number         0  rtexit2.o ABSOLUTE[m
     ../clib/angel/rt.s                       0x00000000   Number         0  rt_locale.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/rt.s                       0x00000000   Number         0  rt_locale_intlibspace.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/rt.s                       0x00000000   Number         0  rt_raise.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/rt.s                       0x00000000   Number         0  rt_ctype_table.o ABSOLUTE[m
     ../clib/angel/scatter.s                  0x00000000   Number         0  __scatter.o ABSOLUTE[m
     ../clib/angel/startup.s                  0x00000000   Number         0  __main.o ABSOLUTE[m
[31m-    ../clib/angel/sys.s                      0x00000000   Number         0  use_no_semi.o ABSOLUTE[m
[31m-    ../clib/angel/sys.s                      0x00000000   Number         0  indicate_semi.o ABSOLUTE[m
     ../clib/angel/sys.s                      0x00000000   Number         0  sys_stackheap_outer.o ABSOLUTE[m
     ../clib/angel/sys.s                      0x00000000   Number         0  libspace.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/sys.s                      0x00000000   Number         0  use_no_semi.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/sys.s                      0x00000000   Number         0  indicate_semi.o ABSOLUTE[m
     ../clib/angel/sysapp.c                   0x00000000   Number         0  sys_exit.o ABSOLUTE[m
[31m-    ../clib/angel/sysapp.c                   0x00000000   Number         0  sys_command.o ABSOLUTE[m
     ../clib/angel/sysapp.c                   0x00000000   Number         0  sys_wrch.o ABSOLUTE[m
[32m+[m[32m    ../clib/angel/sysapp.c                   0x00000000   Number         0  sys_command.o ABSOLUTE[m
[32m+[m[32m    ../clib/armsys.c                         0x00000000   Number         0  argv_veneer.o ABSOLUTE[m
     ../clib/armsys.c                         0x00000000   Number         0  argv_veneer.o ABSOLUTE[m
     ../clib/armsys.c                         0x00000000   Number         0  _get_argv_nomalloc.o ABSOLUTE[m
     ../clib/armsys.c                         0x00000000   Number         0  no_argv.o ABSOLUTE[m
[31m-    ../clib/armsys.c                         0x00000000   Number         0  argv_veneer.o ABSOLUTE[m
     ../clib/bigflt.c                         0x00000000   Number         0  bigflt0.o ABSOLUTE[m
     ../clib/btod.s                           0x00000000   Number         0  btod.o ABSOLUTE[m
     ../clib/heapalloc.c                      0x00000000   Number         0  hrguard.o ABSOLUTE[m
     ../clib/heapaux.c                        0x00000000   Number         0  heapauxi.o ABSOLUTE[m
[31m-    ../clib/libinit.s                        0x00000000   Number         0  libshutdown.o ABSOLUTE[m
     ../clib/libinit.s                        0x00000000   Number         0  libinit2.o ABSOLUTE[m
     ../clib/libinit.s                        0x00000000   Number         0  libshutdown2.o ABSOLUTE[m
     ../clib/libinit.s                        0x00000000   Number         0  libinit.o ABSOLUTE[m
[32m+[m[32m    ../clib/libinit.s                        0x00000000   Number         0  libshutdown.o ABSOLUTE[m
[32m+[m[32m    ../clib/locale.c                         0x00000000   Number         0  _wcrtomb.o ABSOLUTE[m
     ../clib/locale.s                         0x00000000   Number         0  lc_numeric_c.o ABSOLUTE[m
[32m+[m[32m    ../clib/locale.s                         0x00000000   Number         0  lc_ctype_c.o ABSOLUTE[m
     ../clib/longlong.s                       0x00000000   Number         0  lludiv10.o ABSOLUTE[m
     ../clib/memcpset.s                       0x00000000   Number         0  strcmpv7m.o ABSOLUTE[m
     ../clib/memcpset.s                       0x00000000   Number         0  rt_memcpy_w.o ABSOLUTE[m
     ../clib/misc.s                           0x00000000   Number         0  printf_stubs.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf_flags_ss_wp.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf_ss_wp.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf_flags_wp.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf_wp.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf_flags_ss.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf_ss.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf_flags.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  noretval__2sprintf.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_oct_int.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_oct_int_ll.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _sputc.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_char_common.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_hex_ll.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_hex_int.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_hex_int_ll.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_hex_ptr.o ABSOLUTE[m
     ../clib/printf.c                         0x00000000   Number         0  __2sprintf.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  __printf.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_fp_dec.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_charcount.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_dec.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_hex_int_ptr.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_hex_ll_ptr.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_str.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_char.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_truncate.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_pad.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_wchar.o ABSOLUTE[m
     ../clib/printf.c                         0x00000000   Number         0  __printf_nopercent.o ABSOLUTE[m
     ../clib/printf.c                         0x00000000   Number         0  _printf_fp_infnan.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  _printf_fp_dec.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  _printf_char_common.o ABSOLUTE[m
[31m-    ../clib/printf.c                         0x00000000   Number         0  _sputc.o ABSOLUTE[m
[31m-    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_percent.o ABSOLUTE[m
[31m-    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_f.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_fp_hex.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_intcommon.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_oct_ll.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  vsprintf.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  noretval__2sprintf.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_longlong_dec.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf_flags.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf_ss.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf_flags_ss.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf_wp.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf_flags_wp.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf_ss_wp.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf_flags_ss_wp.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_hex_int_ll_ptr.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  _printf_wctomb.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf.c                         0x00000000   Number         0  __printf.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_s.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_c.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_llo.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_ls.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_lc.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_l.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_ll.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_llu.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_lld.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_lli.o ABSOLUTE[m
     ../clib/printf_percent.s                 0x00000000   Number         0  _printf_percent_end.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_rtmem_formal.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_a.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_g.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_e.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_u.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_d.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_i.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_o.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_f.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_p.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_percent.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_llx.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_x.o ABSOLUTE[m
[32m+[m[32m    ../clib/printf_percent.s                 0x00000000   Number         0  _printf_n.o ABSOLUTE[m
     ../clib/signal.c                         0x00000000   Number         0  defsig_rtmem_outer.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_other.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_segv_inner.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_cppl_inner.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_rtmem_formal.o ABSOLUTE[m
     ../clib/signal.c                         0x00000000   Number         0  defsig_pvfn_inner.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_cppl_inner.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_segv_inner.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_exit.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_rtmem_inner.o ABSOLUTE[m
     ../clib/signal.c                         0x00000000   Number         0  defsig_stak_inner.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_abrt_inner.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_rtred_inner.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_other.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  __raise.o ABSOLUTE[m
     ../clib/signal.c                         0x00000000   Number         0  defsig_fpe_inner.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_exit.o ABSOLUTE[m
     ../clib/signal.c                         0x00000000   Number         0  defsig_general.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  __raise.o ABSOLUTE[m
[31m-    ../clib/signal.c                         0x00000000   Number         0  defsig_rtmem_inner.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_rtred_inner.o ABSOLUTE[m
[32m+[m[32m    ../clib/signal.c                         0x00000000   Number         0  defsig_abrt_inner.o ABSOLUTE[m
     ../clib/signal.s                         0x00000000   Number         0  defsig.o ABSOLUTE[m
     ../clib/stdlib.c                         0x00000000   Number         0  exit.o ABSOLUTE[m
     ../fplib/dretinf.s                       0x00000000   Number         0  dretinf.o ABSOLUTE[m
[36m@@ -968,6 +1160,9 @@[m [mImage Symbol Table[m
     ../fplib/fretinf.s                       0x00000000   Number         0  fretinf.o ABSOLUTE[m
     ../fplib/istatus.s                       0x00000000   Number         0  istatus.o ABSOLUTE[m
     ../fplib/printf1.s                       0x00000000   Number         0  printf1.o ABSOLUTE[m
[32m+[m[32m    ../fplib/printf2.s                       0x00000000   Number         0  printf2.o ABSOLUTE[m
[32m+[m[32m    ../fplib/printf2a.s                      0x00000000   Number         0  printf2a.o ABSOLUTE[m
[32m+[m[32m    ../fplib/printf2b.s                      0x00000000   Number         0  printf2b.o ABSOLUTE[m
     ../fplib/usenofp.s                       0x00000000   Number         0  usenofp.o ABSOLUTE[m
     ../mathlib/fpclassify.c                  0x00000000   Number         0  fpclassify.o ABSOLUTE[m
     Hardware\Key.c                           0x00000000   Number         0  key.o ABSOLUTE[m
[36m@@ -975,6 +1170,7 @@[m [mImage Symbol Table[m
     Hardware\Menu.c                          0x00000000   Number         0  menu.o ABSOLUTE[m
     Hardware\OLED.c                          0x00000000   Number         0  oled.o ABSOLUTE[m
     Hardware\Printer.c                       0x00000000   Number         0  printer.o ABSOLUTE[m
[32m+[m[32m    Hardware\Serial.c                        0x00000000   Number         0  serial.o ABSOLUTE[m
     Library\misc.c                           0x00000000   Number         0  misc.o ABSOLUTE[m
     Library\stm32f10x_adc.c                  0x00000000   Number         0  stm32f10x_adc.o ABSOLUTE[m
     Library\stm32f10x_bkp.c                  0x00000000   Number         0  stm32f10x_bkp.o ABSOLUTE[m
[36m@@ -1014,195 +1210,251 @@[m [mImage Symbol Table[m
     !!handler_copy                           0x08000128   Section       26  __scatter_copy.o(!!handler_copy)[m
     !!handler_zi                             0x08000144   Section       28  __scatter_zi.o(!!handler_zi)[m
     .ARM.Collect$$_printf_percent$$00000000  0x08000160   Section        0  _printf_percent.o(.ARM.Collect$$_printf_percent$$00000000)[m
[31m-    .ARM.Collect$$_printf_percent$$00000003  0x08000160   Section        6  _printf_f.o(.ARM.Collect$$_printf_percent$$00000003)[m
[31m-    .ARM.Collect$$_printf_percent$$00000017  0x08000166   Section        4  _printf_percent_end.o(.ARM.Collect$$_printf_percent$$00000017)[m
[31m-    .ARM.Collect$$libinit$$00000000          0x0800016a   Section        2  libinit.o(.ARM.Collect$$libinit$$00000000)[m
[31m-    .ARM.Collect$$libinit$$00000002          0x0800016c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000002)[m
[31m-    .ARM.Collect$$libinit$$00000004          0x0800016c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000004)[m
[31m-    .ARM.Collect$$libinit$$0000000A          0x0800016c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000000A)[m
[31m-    .ARM.Collect$$libinit$$0000000C          0x0800016c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000000C)[m
[31m-    .ARM.Collect$$libinit$$0000000E          0x0800016c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000000E)[m
[31m-    .ARM.Collect$$libinit$$0000000F          0x0800016c   Section        6  libinit2.o(.ARM.Collect$$libinit$$0000000F)[m
[31m-    .ARM.Collect$$libinit$$00000011          0x08000172   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000011)[m
[31m-    .ARM.Collect$$libinit$$00000013          0x08000172   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000013)[m
[31m-    .ARM.Collect$$libinit$$00000015          0x08000172   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000015)[m
[31m-    .ARM.Collect$$libinit$$00000016          0x08000172   Section       10  libinit2.o(.ARM.Collect$$libinit$$00000016)[m
[31m-    .ARM.Collect$$libinit$$00000017          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000017)[m
[31m-    .ARM.Collect$$libinit$$00000019          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000019)[m
[31m-    .ARM.Collect$$libinit$$0000001B          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000001B)[m
[31m-    .ARM.Collect$$libinit$$0000001D          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000001D)[m
[31m-    .ARM.Collect$$libinit$$0000001F          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000001F)[m
[31m-    .ARM.Collect$$libinit$$00000021          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000021)[m
[31m-    .ARM.Collect$$libinit$$00000023          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000023)[m
[31m-    .ARM.Collect$$libinit$$00000025          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000025)[m
[31m-    .ARM.Collect$$libinit$$0000002C          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000002C)[m
[31m-    .ARM.Collect$$libinit$$0000002E          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000002E)[m
[31m-    .ARM.Collect$$libinit$$00000030          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000030)[m
[31m-    .ARM.Collect$$libinit$$00000032          0x0800017c   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000032)[m
[31m-    .ARM.Collect$$libinit$$00000033          0x0800017c   Section        2  libinit2.o(.ARM.Collect$$libinit$$00000033)[m
[31m-    .ARM.Collect$$libshutdown$$00000000      0x0800017e   Section        2  libshutdown.o(.ARM.Collect$$libshutdown$$00000000)[m
[31m-    .ARM.Collect$$libshutdown$$00000002      0x08000180   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000002)[m
[31m-    .ARM.Collect$$libshutdown$$00000004      0x08000180   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000004)[m
[31m-    .ARM.Collect$$libshutdown$$00000007      0x08000180   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000007)[m
[31m-    .ARM.Collect$$libshutdown$$0000000A      0x08000180   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000A)[m
[31m-    .ARM.Collect$$libshutdown$$0000000C      0x08000180   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000C)[m
[31m-    .ARM.Collect$$libshutdown$$0000000F      0x08000180   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000F)[m
[31m-    .ARM.Collect$$libshutdown$$00000010      0x08000180   Section        2  libshutdown2.o(.ARM.Collect$$libshutdown$$00000010)[m
[31m-    .ARM.Collect$$rtentry$$00000000          0x08000182   Section        0  __rtentry.o(.ARM.Collect$$rtentry$$00000000)[m
[31m-    .ARM.Collect$$rtentry$$00000002          0x08000182   Section        0  __rtentry2.o(.ARM.Collect$$rtentry$$00000002)[m
[31m-    .ARM.Collect$$rtentry$$00000004          0x08000182   Section        6  __rtentry4.o(.ARM.Collect$$rtentry$$00000004)[m
[31m-    .ARM.Collect$$rtentry$$00000009          0x08000188   Section        0  __rtentry2.o(.ARM.Collect$$rtentry$$00000009)[m
[31m-    .ARM.Collect$$rtentry$$0000000A          0x08000188   Section        4  __rtentry2.o(.ARM.Collect$$rtentry$$0000000A)[m
[31m-    .ARM.Collect$$rtentry$$0000000C          0x0800018c   Section        0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000C)[m
[31m-    .ARM.Collect$$rtentry$$0000000D          0x0800018c   Section        8  __rtentry2.o(.ARM.Collect$$rtentry$$0000000D)[m
[31m-    .ARM.Collect$$rtexit$$00000000           0x08000194   Section        2  rtexit.o(.ARM.Collect$$rtexit$$00000000)[m
[31m-    .ARM.Collect$$rtexit$$00000002           0x08000196   Section        0  rtexit2.o(.ARM.Collect$$rtexit$$00000002)[m
[31m-    .ARM.Collect$$rtexit$$00000003           0x08000196   Section        4  rtexit2.o(.ARM.Collect$$rtexit$$00000003)[m
[31m-    .ARM.Collect$$rtexit$$00000004           0x0800019a   Section        6  rtexit2.o(.ARM.Collect$$rtexit$$00000004)[m
[31m-    .text                                    0x080001a0   Section       64  startup_stm32f10x_md.o(.text)[m
[31m-    .text                                    0x080001e0   Section        0  noretval__2sprintf.o(.text)[m
[31m-    .text                                    0x08000208   Section        0  __printf_wp.o(.text)[m
[31m-    .text                                    0x08000316   Section      100  rt_memcpy_w.o(.text)[m
[31m-    .text                                    0x0800037c   Section      128  strcmpv7m.o(.text)[m
[31m-    .text                                    0x080003fc   Section        0  heapauxi.o(.text)[m
[31m-    .text                                    0x08000402   Section        0  _printf_fp_dec.o(.text)[m
[31m-    _fp_digits                               0x08000405   Thumb Code   432  _printf_fp_dec.o(.text)[m
[31m-    .text                                    0x08000820   Section        0  _printf_char_common.o(.text)[m
[31m-    _printf_input_char                       0x08000821   Thumb Code    10  _printf_char_common.o(.text)[m
[31m-    .text                                    0x08000850   Section        0  _sputc.o(.text)[m
[31m-    .text                                    0x0800085c   Section        8  rt_locale_intlibspace.o(.text)[m
[31m-    .text                                    0x08000864   Section      138  lludiv10.o(.text)[m
[31m-    .text                                    0x080008f0   Section        0  _printf_fp_infnan.o(.text)[m
[31m-    .text                                    0x08000970   Section        0  bigflt0.o(.text)[m
[31m-    .text                                    0x08000a54   Section        8  libspace.o(.text)[m
[31m-    .text                                    0x08000a5c   Section       74  sys_stackheap_outer.o(.text)[m
[31m-    .text                                    0x08000aa6   Section        0  exit.o(.text)[m
[31m-    .text                                    0x08000ab8   Section        0  sys_exit.o(.text)[m
[31m-    .text                                    0x08000ac4   Section        2  use_no_semi.o(.text)[m
[31m-    .text                                    0x08000ac6   Section        0  indicate_semi.o(.text)[m
[31m-    CL$$btod_d2e                             0x08000ac6   Section       62  btod.o(CL$$btod_d2e)[m
[31m-    CL$$btod_d2e_denorm_low                  0x08000b04   Section       70  btod.o(CL$$btod_d2e_denorm_low)[m
[31m-    CL$$btod_d2e_norm_op1                    0x08000b4a   Section       96  btod.o(CL$$btod_d2e_norm_op1)[m
[31m-    CL$$btod_div_common                      0x08000baa   Section      824  btod.o(CL$$btod_div_common)[m
[31m-    CL$$btod_e2e                             0x08000ee2   Section      220  btod.o(CL$$btod_e2e)[m
[31m-    CL$$btod_ediv                            0x08000fbe   Section       42  btod.o(CL$$btod_ediv)[m
[31m-    CL$$btod_emul                            0x08000fe8   Section       42  btod.o(CL$$btod_emul)[m
[31m-    CL$$btod_mult_common                     0x08001012   Section      580  btod.o(CL$$btod_mult_common)[m
[31m-    i.BusFault_Handler                       0x08001256   Section        0  stm32f10x_it.o(i.BusFault_Handler)[m
[31m-    i.DebugMon_Handler                       0x0800125a   Section        0  stm32f10x_it.o(i.DebugMon_Handler)[m
[31m-    i.EXTI0_IRQHandler                       0x0800125c   Section        0  encoder.o(i.EXTI0_IRQHandler)[m
[31m-    i.EXTI1_IRQHandler                       0x08001298   Section        0  encoder.o(i.EXTI1_IRQHandler)[m
[31m-    i.EXTI_ClearITPendingBit                 0x080012d4   Section        0  stm32f10x_exti.o(i.EXTI_ClearITPendingBit)[m
[31m-    i.EXTI_GetITStatus                       0x080012e0   Section        0  stm32f10x_exti.o(i.EXTI_GetITStatus)[m
[31m-    i.EXTI_Init                              0x08001308   Section        0  stm32f10x_exti.o(i.EXTI_Init)[m
[31m-    i.Encoder_Get                            0x0800139c   Section        0  encoder.o(i.Encoder_Get)[m
[31m-    i.Encoder_Init                           0x080013b0   Section        0  encoder.o(i.Encoder_Init)[m
[31m-    i.FloodLED_LeftTurn                      0x0800144c   Section        0  led.o(i.FloodLED_LeftTurn)[m
[31m-    i.FloodLED_RightTurn                     0x080014ac   Section        0  led.o(i.FloodLED_RightTurn)[m
[31m-    i.GPIO_EXTILineConfig                    0x0800150c   Section        0  stm32f10x_gpio.o(i.GPIO_EXTILineConfig)[m
[31m-    i.GPIO_Init                              0x0800154c   Section        0  stm32f10x_gpio.o(i.GPIO_Init)[m
[31m-    i.GPIO_ReadInputDataBit                  0x08001662   Section        0  stm32f10x_gpio.o(i.GPIO_ReadInputDataBit)[m
[31m-    i.GPIO_SetBits                           0x08001674   Section        0  stm32f10x_gpio.o(i.GPIO_SetBits)[m
[31m-    i.GPIO_WriteBit                          0x08001678   Section        0  stm32f10x_gpio.o(i.GPIO_WriteBit)[m
[31m-    i.HardFault_Handler                      0x08001682   Section        0  stm32f10x_it.o(i.HardFault_Handler)[m
[31m-    i.Key_Check                              0x08001688   Section        0  key.o(i.Key_Check)[m
[31m-    i.Key_GetNum                             0x080016ac   Section        0  key.o(i.Key_GetNum)[m
[31m-    i.Key_GetState                           0x080016ec   Section        0  key.o(i.Key_GetState)[m
[31m-    i.Key_Init                               0x08001744   Section        0  key.o(i.Key_Init)[m
[31m-    i.Key_Tick                               0x08001798   Section        0  key.o(i.Key_Tick)[m
[31m-    i.LED_DirSet                             0x08001a48   Section        0  led.o(i.LED_DirSet)[m
[31m-    i.LED_Init                               0x08001a54   Section        0  led.o(i.LED_Init)[m
[31m-    i.LED_SpeedSet                           0x08001a8c   Section        0  led.o(i.LED_SpeedSet)[m
[31m-    i.LED_Tick                               0x08001aa0   Section        0  led.o(i.LED_Tick)[m
[31m-    i.MemManage_Handler                      0x08001adc   Section        0  stm32f10x_it.o(i.MemManage_Handler)[m
[31m-    i.Menu_Init                              0x08001ae0   Section        0  menu.o(i.Menu_Init)[m
[31m-    i.Menu_LED_Direction                     0x08001bfc   Section        0  menu.o(i.Menu_LED_Direction)[m
[31m-    i.Menu_LED_Speed                         0x08001c10   Section        0  menu.o(i.Menu_LED_Speed)[m
[31m-    i.Menu_Option                            0x08001c24   Section        0  menu.o(i.Menu_Option)[m
[31m-    i.Menu_Show                              0x08001db0   Section        0  menu.o(i.Menu_Show)[m
[31m-    i.NMI_Handler                            0x08001ed4   Section        0  stm32f10x_it.o(i.NMI_Handler)[m
[31m-    i.NVIC_Init                              0x08001ed8   Section        0  misc.o(i.NVIC_Init)[m
[31m-    i.NVIC_PriorityGroupConfig               0x08001f48   Section        0  misc.o(i.NVIC_PriorityGroupConfig)[m
[31m-    i.Numlen                                 0x08001f5c   Section        0  menu.o(i.Numlen)[m
[31m-    i.OLED_Clear                             0x08001f78   Section        0  oled.o(i.OLED_Clear)[m
[31m-    i.OLED_I2C_Init                          0x08001fa4   Section        0  oled.o(i.OLED_I2C_Init)[m
[31m-    i.OLED_I2C_SendByte                      0x08001ff4   Section        0  oled.o(i.OLED_I2C_SendByte)[m
[31m-    i.OLED_I2C_Start                         0x08002050   Section        0  oled.o(i.OLED_I2C_Start)[m
[31m-    i.OLED_I2C_Stop                          0x08002084   Section        0  oled.o(i.OLED_I2C_Stop)[m
[31m-    i.OLED_Init                              0x080020ac   Section        0  oled.o(i.OLED_Init)[m
[31m-    i.OLED_Pow                               0x0800215a   Section        0  oled.o(i.OLED_Pow)[m
[31m-    i.OLED_SetCursor                         0x0800216e   Section        0  oled.o(i.OLED_SetCursor)[m
[31m-    i.OLED_ShowChar                          0x08002190   Section        0  oled.o(i.OLED_ShowChar)[m
[31m-    i.OLED_ShowFloatNum                      0x08002204   Section        0  oled.o(i.OLED_ShowFloatNum)[m
[31m-    i.OLED_ShowNum                           0x08002240   Section        0  oled.o(i.OLED_ShowNum)[m
[31m-    i.OLED_ShowString                        0x08002284   Section        0  oled.o(i.OLED_ShowString)[m
[31m-    i.OLED_WriteCommand                      0x080022ac   Section        0  oled.o(i.OLED_WriteCommand)[m
[31m-    i.OLED_WriteData                         0x080022cc   Section        0  oled.o(i.OLED_WriteData)[m
[31m-    i.PendSV_Handler                         0x080022ec   Section        0  stm32f10x_it.o(i.PendSV_Handler)[m
[31m-    i.RCC_APB1PeriphClockCmd                 0x080022f0   Section        0  stm32f10x_rcc.o(i.RCC_APB1PeriphClockCmd)[m
[31m-    i.RCC_APB2PeriphClockCmd                 0x08002310   Section        0  stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd)[m
[31m-    i.SVC_Handler                            0x08002330   Section        0  stm32f10x_it.o(i.SVC_Handler)[m
[31m-    i.SetSysClock                            0x08002332   Section        0  system_stm32f10x.o(i.SetSysClock)[m
[31m-    SetSysClock                              0x08002333   Thumb Code     8  system_stm32f10x.o(i.SetSysClock)[m
[31m-    i.SetSysClockTo72                        0x0800233c   Section        0  system_stm32f10x.o(i.SetSysClockTo72)[m
[31m-    SetSysClockTo72                          0x0800233d   Thumb Code   214  system_stm32f10x.o(i.SetSysClockTo72)[m
[31m-    i.SysTick_Handler                        0x0800241c   Section        0  stm32f10x_it.o(i.SysTick_Handler)[m
[31m-    i.SystemInit                             0x08002420   Section        0  system_stm32f10x.o(i.SystemInit)[m
[31m-    i.TIM2_IRQHandler                        0x08002480   Section        0  main.o(i.TIM2_IRQHandler)[m
[31m-    i.TIM_ClearFlag                          0x080024a0   Section        0  stm32f10x_tim.o(i.TIM_ClearFlag)[m
[31m-    i.TIM_ClearITPendingBit                  0x080024a6   Section        0  stm32f10x_tim.o(i.TIM_ClearITPendingBit)[m
[31m-    i.TIM_Cmd                                0x080024ac   Section        0  stm32f10x_tim.o(i.TIM_Cmd)[m
[31m-    i.TIM_GetITStatus                        0x080024c4   Section        0  stm32f10x_tim.o(i.TIM_GetITStatus)[m
[31m-    i.TIM_ITConfig                           0x080024e6   Section        0  stm32f10x_tim.o(i.TIM_ITConfig)[m
[31m-    i.TIM_InternalClockConfig                0x080024f8   Section        0  stm32f10x_tim.o(i.TIM_InternalClockConfig)[m
[31m-    i.TIM_TimeBaseInit                       0x08002504   Section        0  stm32f10x_tim.o(i.TIM_TimeBaseInit)[m
[31m-    i.Timer_Init                             0x080025a8   Section        0  timer.o(i.Timer_Init)[m
[31m-    i.UsageFault_Handler                     0x08002624   Section        0  stm32f10x_it.o(i.UsageFault_Handler)[m
[31m-    i.__ARM_fpclassify                       0x08002628   Section        0  fpclassify.o(i.__ARM_fpclassify)[m
[31m-    i._is_digit                              0x08002650   Section        0  __printf_wp.o(i._is_digit)[m
[31m-    i.main                                   0x0800265e   Section        0  main.o(i.main)[m
[31m-    locale$$code                             0x080026a0   Section       44  lc_numeric_c.o(locale$$code)[m
[31m-    x$fpl$dretinf                            0x080026cc   Section       12  dretinf.o(x$fpl$dretinf)[m
[31m-    x$fpl$f2d                                0x080026d8   Section       86  f2d.o(x$fpl$f2d)[m
[31m-    x$fpl$fdiv                               0x08002730   Section      388  fdiv.o(x$fpl$fdiv)[m
[31m-    _fdiv1                                   0x08002731   Thumb Code     0  fdiv.o(x$fpl$fdiv)[m
[31m-    x$fpl$fflt                               0x080028b4   Section       48  fflt_clz.o(x$fpl$fflt)[m
[31m-    x$fpl$fnaninf                            0x080028e4   Section      140  fnaninf.o(x$fpl$fnaninf)[m
[31m-    x$fpl$fretinf                            0x08002970   Section       10  fretinf.o(x$fpl$fretinf)[m
[31m-    x$fpl$printf1                            0x0800297a   Section        4  printf1.o(x$fpl$printf1)[m
[31m-    .constdata                               0x0800297e   Section       18  led.o(.constdata)[m
[31m-    x$fpl$usenofp                            0x0800297e   Section        0  usenofp.o(x$fpl$usenofp)[m
[31m-    .constdata                               0x08002990   Section     1520  oled.o(.constdata)[m
[31m-    .constdata                               0x08002f80   Section      320  menu.o(.constdata)[m
[31m-    .constdata                               0x080030c0   Section      148  bigflt0.o(.constdata)[m
[31m-    tenpwrs_x                                0x080030c0   Data          60  bigflt0.o(.constdata)[m
[31m-    tenpwrs_i                                0x080030fc   Data          64  bigflt0.o(.constdata)[m
[31m-    .conststring                             0x08003154   Section       82  menu.o(.conststring)[m
[31m-    locale$$data                             0x080031c8   Section       28  lc_numeric_c.o(locale$$data)[m
[31m-    __lcnum_c_name                           0x080031cc   Data           2  lc_numeric_c.o(locale$$data)[m
[31m-    __lcnum_c_start                          0x080031d4   Data           0  lc_numeric_c.o(locale$$data)[m
[31m-    __lcnum_c_point                          0x080031e0   Data           0  lc_numeric_c.o(locale$$data)[m
[31m-    __lcnum_c_thousands                      0x080031e2   Data           0  lc_numeric_c.o(locale$$data)[m
[31m-    __lcnum_c_grouping                       0x080031e3   Data           0  lc_numeric_c.o(locale$$data)[m
[31m-    __lcnum_c_end                            0x080031e4   Data           0  lc_numeric_c.o(locale$$data)[m
[31m-    .data                                    0x20000000   Section       12  led.o(.data)[m
[31m-    Tim                                      0x20000008   Data           4  led.o(.data)[m
[31m-    .data                                    0x2000000c   Section       26  key.o(.data)[m
[31m-    Count                                    0x20000010   Data           1  key.o(.data)[m
[31m-    i                                        0x20000011   Data           1  key.o(.data)[m
[31m-    CurrState                                0x20000012   Data           4  key.o(.data)[m
[31m-    PrevState                                0x20000016   Data           4  key.o(.data)[m
[31m-    S                                        0x2000001a   Data           4  key.o(.data)[m
[31m-    Time                                     0x2000001e   Data           8  key.o(.data)[m
[31m-    .data                                    0x20000028   Section       60  menu.o(.data)[m
[31m-    .data                                    0x20000064   Section        2  encoder.o(.data)[m
[31m-    .bss                                     0x20000068   Section      320  menu.o(.bss)[m
[31m-    .bss                                     0x200001a8   Section       96  libspace.o(.bss)[m
[31m-    HEAP                                     0x20000208   Section      512  startup_stm32f10x_md.o(HEAP)[m
[31m-    Heap_Mem                                 0x20000208   Data         512  startup_stm32f10x_md.o(HEAP)[m
[31m-    STACK                                    0x20000408   Section     1024  startup_stm32f10x_md.o(STACK)[m
[31m-    Stack_Mem                                0x20000408   Data        1024  startup_stm32f10x_md.o(STACK)[m
[31m-    __initial_sp                             0x20000808   Data           0  startup_stm32f10x_md.o(STACK)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000001  0x08000160   Section        6  _printf_n.o(.ARM.Collect$$_printf_percent$$00000001)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000002  0x08000166   Section        6  _printf_p.o(.ARM.Collect$$_printf_percent$$00000002)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000003  0x0800016c   Section        6  _printf_f.o(.ARM.Collect$$_printf_percent$$00000003)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000004  0x08000172   Section        6  _printf_e.o(.ARM.Collect$$_printf_percent$$00000004)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000005  0x08000178   Section        6  _printf_g.o(.ARM.Collect$$_printf_percent$$00000005)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000006  0x0800017e   Section        6  _printf_a.o(.ARM.Collect$$_printf_percent$$00000006)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000007  0x08000184   Section       10  _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000008  0x0800018e   Section        6  _printf_i.o(.ARM.Collect$$_printf_percent$$00000008)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000009  0x08000194   Section        6  _printf_d.o(.ARM.Collect$$_printf_percent$$00000009)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$0000000A  0x0800019a   Section        6  _printf_u.o(.ARM.Collect$$_printf_percent$$0000000A)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$0000000B  0x080001a0   Section        6  _printf_o.o(.ARM.Collect$$_printf_percent$$0000000B)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$0000000C  0x080001a6   Section        6  _printf_x.o(.ARM.Collect$$_printf_percent$$0000000C)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$0000000D  0x080001ac   Section        6  _printf_lli.o(.ARM.Collect$$_printf_percent$$0000000D)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$0000000E  0x080001b2   Section        6  _printf_lld.o(.ARM.Collect$$_printf_percent$$0000000E)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$0000000F  0x080001b8   Section        6  _printf_llu.o(.ARM.Collect$$_printf_percent$$0000000F)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000010  0x080001be   Section        6  _printf_llo.o(.ARM.Collect$$_printf_percent$$00000010)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000011  0x080001c4   Section        6  _printf_llx.o(.ARM.Collect$$_printf_percent$$00000011)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000012  0x080001ca   Section       10  _printf_l.o(.ARM.Collect$$_printf_percent$$00000012)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000013  0x080001d4   Section        6  _printf_c.o(.ARM.Collect$$_printf_percent$$00000013)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000014  0x080001da   Section        6  _printf_s.o(.ARM.Collect$$_printf_percent$$00000014)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000015  0x080001e0   Section        6  _printf_lc.o(.ARM.Collect$$_printf_percent$$00000015)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000016  0x080001e6   Section        6  _printf_ls.o(.ARM.Collect$$_printf_percent$$00000016)[m
[32m+[m[32m    .ARM.Collect$$_printf_percent$$00000017  0x080001ec   Section        4  _printf_percent_end.o(.ARM.Collect$$_printf_percent$$00000017)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000000          0x080001f0   Section        2  libinit.o(.ARM.Collect$$libinit$$00000000)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000002          0x080001f2   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000002)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000004          0x080001f2   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000004)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000000A          0x080001f2   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000000A)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000000C          0x080001f2   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000000C)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000000E          0x080001f2   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000000E)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000000F          0x080001f2   Section        6  libinit2.o(.ARM.Collect$$libinit$$0000000F)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000011          0x080001f8   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000011)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000012          0x080001f8   Section       12  libinit2.o(.ARM.Collect$$libinit$$00000012)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000013          0x08000204   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000013)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000015          0x08000204   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000015)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000016          0x08000204   Section       10  libinit2.o(.ARM.Collect$$libinit$$00000016)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000017          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000017)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000019          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000019)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000001B          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000001B)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000001D          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000001D)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000001F          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000001F)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000021          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000021)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000023          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000023)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000025          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000025)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000002C          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000002C)[m
[32m+[m[32m    .ARM.Collect$$libinit$$0000002E          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$0000002E)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000030          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000030)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000032          0x0800020e   Section        0  libinit2.o(.ARM.Collect$$libinit$$00000032)[m
[32m+[m[32m    .ARM.Collect$$libinit$$00000033          0x0800020e   Section        2  libinit2.o(.ARM.Collect$$libinit$$00000033)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$00000000      0x08000210   Section        2  libshutdown.o(.ARM.Collect$$libshutdown$$00000000)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$00000002      0x08000212   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000002)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$00000004      0x08000212   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000004)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$00000007      0x08000212   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000007)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$0000000A      0x08000212   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000A)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$0000000C      0x08000212   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000C)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$0000000F      0x08000212   Section        0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000F)[m
[32m+[m[32m    .ARM.Collect$$libshutdown$$00000010      0x08000212   Section        2  libshutdown2.o(.ARM.Collect$$libshutdown$$00000010)[m
[32m+[m[32m    .ARM.Collect$$rtentry$$00000000          0x08000214   Section        0  __rtentry.o(.ARM.Collect$$rtentry$$00000000)[m
[32m+[m[32m    .ARM.Collect$$rtentry$$00000002          0x08000214   Section        0  __rtentry2.o(.ARM.Collect$$rtentry$$00000002)[m
[32m+[m[32m    .ARM.Collect$$rtentry$$00000004          0x08000214   Section        6  __rtentry4.o(.ARM.Collect$$rtentry$$00000004)[m
[32m+[m[32m    .ARM.Collect$$rtentry$$00000009          0x0800021a   Section        0  __rtentry2.o(.ARM.Collect$$rtentry$$00000009)[m
[32m+[m[32m    .ARM.Collect$$rtentry$$0000000A          0x0800021a   Section        4  __rtentry2.o(.ARM.Collect$$rtentry$$0000000A)[m
[32m+[m[32m    .ARM.Collect$$rtentry$$0000000C          0x0800021e   Section        0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000C)[m
[32m+[m[32m    .ARM.Collect$$rtentry$$0000000D          0x0800021e   Section        8  __rtentry2.o(.ARM.Collect$$rtentry$$0000000D)[m
[32m+[m[32m    .ARM.Collect$$rtexit$$00000000           0x08000226   Section        2  rtexit.o(.ARM.Collect$$rtexit$$00000000)[m
[32m+[m[32m    .ARM.Collect$$rtexit$$00000002           0x08000228   Section        0  rtexit2.o(.ARM.Collect$$rtexit$$00000002)[m
[32m+[m[32m    .ARM.Collect$$rtexit$$00000003           0x08000228   Section        4  rtexit2.o(.ARM.Collect$$rtexit$$00000003)[m
[32m+[m[32m    .ARM.Collect$$rtexit$$00000004           0x0800022c   Section        6  rtexit2.o(.ARM.Collect$$rtexit$$00000004)[m
[32m+[m[32m    .text                                    0x08000234   Section       64  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    .text                                    0x08000274   Section        0  vsprintf.o(.text)[m
[32m+[m[32m    .text                                    0x08000298   Section        0  __printf_flags_ss_wp.o(.text)[m
[32m+[m[32m    .text                                    0x08000420   Section      128  strcmpv7m.o(.text)[m
[32m+[m[32m    .text                                    0x080004a0   Section        0  heapauxi.o(.text)[m
[32m+[m[32m    .text                                    0x080004a6   Section        0  _printf_pad.o(.text)[m
[32m+[m[32m    .text                                    0x080004f4   Section        0  _printf_truncate.o(.text)[m
[32m+[m[32m    .text                                    0x08000518   Section        0  _printf_str.o(.text)[m
[32m+[m[32m    .text                                    0x0800056c   Section        0  _printf_dec.o(.text)[m
[32m+[m[32m    .text                                    0x080005e4   Section        0  _printf_charcount.o(.text)[m
[32m+[m[32m    .text                                    0x0800060c   Section        0  _printf_fp_dec.o(.text)[m
[32m+[m[32m    _fp_digits                               0x0800060f   Thumb Code   432  _printf_fp_dec.o(.text)[m
[32m+[m[32m    .text                                    0x08000a2c   Section        0  _printf_char_common.o(.text)[m
[32m+[m[32m    _printf_input_char                       0x08000a2d   Thumb Code    10  _printf_char_common.o(.text)[m
[32m+[m[32m    .text                                    0x08000a5c   Section        0  _sputc.o(.text)[m
[32m+[m[32m    .text                                    0x08000a68   Section        0  _printf_wctomb.o(.text)[m
[32m+[m[32m    .text                                    0x08000b24   Section        0  _printf_longlong_dec.o(.text)[m
[32m+[m[32m    .text                                    0x08000ba0   Section        0  _printf_oct_int_ll.o(.text)[m
[32m+[m[32m    _printf_longlong_oct_internal            0x08000ba1   Thumb Code     0  _printf_oct_int_ll.o(.text)[m
[32m+[m[32m    .text                                    0x08000c10   Section        0  _printf_hex_int_ll_ptr.o(.text)[m
[32m+[m[32m    _printf_hex_common                       0x08000c11   Thumb Code     0  _printf_hex_int_ll_ptr.o(.text)[m
[32m+[m[32m    .text                                    0x08000ca4   Section        8  rt_locale_intlibspace.o(.text)[m
[32m+[m[32m    .text                                    0x08000cac   Section      138  lludiv10.o(.text)[m
[32m+[m[32m    .text                                    0x08000d36   Section        0  _printf_intcommon.o(.text)[m
[32m+[m[32m    .text                                    0x08000de8   Section        0  _printf_fp_hex.o(.text)[m
[32m+[m[32m    .text                                    0x080010e4   Section        0  _printf_fp_infnan.o(.text)[m
[32m+[m[32m    .text                                    0x08001164   Section        0  _printf_char.o(.text)[m
[32m+[m[32m    .text                                    0x08001190   Section        0  _printf_wchar.o(.text)[m
[32m+[m[32m    .text                                    0x080011bc   Section        0  bigflt0.o(.text)[m
[32m+[m[32m    .text                                    0x080012a0   Section        0  _wcrtomb.o(.text)[m
[32m+[m[32m    .text                                    0x080012e0   Section        8  libspace.o(.text)[m
[32m+[m[32m    .text                                    0x080012e8   Section       74  sys_stackheap_outer.o(.text)[m
[32m+[m[32m    .text                                    0x08001334   Section       16  rt_ctype_table.o(.text)[m
[32m+[m[32m    .text                                    0x08001344   Section        0  exit.o(.text)[m
[32m+[m[32m    .text                                    0x08001358   Section        0  sys_exit.o(.text)[m
[32m+[m[32m    .text                                    0x08001364   Section        2  use_no_semi.o(.text)[m
[32m+[m[32m    .text                                    0x08001366   Section        0  indicate_semi.o(.text)[m
[32m+[m[32m    CL$$btod_d2e                             0x08001366   Section       62  btod.o(CL$$btod_d2e)[m
[32m+[m[32m    CL$$btod_d2e_denorm_low                  0x080013a4   Section       70  btod.o(CL$$btod_d2e_denorm_low)[m
[32m+[m[32m    CL$$btod_d2e_norm_op1                    0x080013ea   Section       96  btod.o(CL$$btod_d2e_norm_op1)[m
[32m+[m[32m    CL$$btod_div_common                      0x0800144a   Section      824  btod.o(CL$$btod_div_common)[m
[32m+[m[32m    CL$$btod_e2e                             0x08001782   Section      220  btod.o(CL$$btod_e2e)[m
[32m+[m[32m    CL$$btod_ediv                            0x0800185e   Section       42  btod.o(CL$$btod_ediv)[m
[32m+[m[32m    CL$$btod_emul                            0x08001888   Section       42  btod.o(CL$$btod_emul)[m
[32m+[m[32m    CL$$btod_mult_common                     0x080018b2   Section      580  btod.o(CL$$btod_mult_common)[m
[32m+[m[32m    i.BusFault_Handler                       0x08001af6   Section        0  stm32f10x_it.o(i.BusFault_Handler)[m
[32m+[m[32m    i.DebugMon_Handler                       0x08001afa   Section        0  stm32f10x_it.o(i.DebugMon_Handler)[m
[32m+[m[32m    i.Encoder_Get                            0x08001afc   Section        0  encoder.o(i.Encoder_Get)[m
[32m+[m[32m    i.Encoder_Init                           0x08001b18   Section        0  encoder.o(i.Encoder_Init)[m
[32m+[m[32m    i.GPIO_Init                              0x08001bb8   Section        0  stm32f10x_gpio.o(i.GPIO_Init)[m
[32m+[m[32m    i.GPIO_ReadInputDataBit                  0x08001cce   Section        0  stm32f10x_gpio.o(i.GPIO_ReadInputDataBit)[m
[32m+[m[32m    i.GPIO_WriteBit                          0x08001ce0   Section        0  stm32f10x_gpio.o(i.GPIO_WriteBit)[m
[32m+[m[32m    i.HardFault_Handler                      0x08001cea   Section        0  stm32f10x_it.o(i.HardFault_Handler)[m
[32m+[m[32m    i.Key_GetState                           0x08001cf0   Section        0  key.o(i.Key_GetState)[m
[32m+[m[32m    i.Key_Init                               0x08001d48   Section        0  key.o(i.Key_Init)[m
[32m+[m[32m    i.Key_Tick                               0x08001d9c   Section        0  key.o(i.Key_Tick)[m
[32m+[m[32m    i.MemManage_Handler                      0x0800204c   Section        0  stm32f10x_it.o(i.MemManage_Handler)[m
[32m+[m[32m    i.NMI_Handler                            0x08002050   Section        0  stm32f10x_it.o(i.NMI_Handler)[m
[32m+[m[32m    i.NVIC_Init                              0x08002054   Section        0  misc.o(i.NVIC_Init)[m
[32m+[m[32m    i.NVIC_PriorityGroupConfig               0x080020c4   Section        0  misc.o(i.NVIC_PriorityGroupConfig)[m
[32m+[m[32m    i.OLED_Clear                             0x080020d8   Section        0  oled.o(i.OLED_Clear)[m
[32m+[m[32m    i.OLED_I2C_Init                          0x08002104   Section        0  oled.o(i.OLED_I2C_Init)[m
[32m+[m[32m    i.OLED_I2C_SendByte                      0x08002154   Section        0  oled.o(i.OLED_I2C_SendByte)[m
[32m+[m[32m    i.OLED_I2C_Start                         0x080021b0   Section        0  oled.o(i.OLED_I2C_Start)[m
[32m+[m[32m    i.OLED_I2C_Stop                          0x080021e4   Section        0  oled.o(i.OLED_I2C_Stop)[m
[32m+[m[32m    i.OLED_Init                              0x0800220c   Section        0  oled.o(i.OLED_Init)[m
[32m+[m[32m    i.OLED_Pow                               0x080022ba   Section        0  oled.o(i.OLED_Pow)[m
[32m+[m[32m    i.OLED_SetCursor                         0x080022ce   Section        0  oled.o(i.OLED_SetCursor)[m
[32m+[m[32m    i.OLED_ShowChar                          0x080022f0   Section        0  oled.o(i.OLED_ShowChar)[m
[32m+[m[32m    i.OLED_ShowSignedNum                     0x08002364   Section        0  oled.o(i.OLED_ShowSignedNum)[m
[32m+[m[32m    i.OLED_WriteCommand                      0x080023ca   Section        0  oled.o(i.OLED_WriteCommand)[m
[32m+[m[32m    i.OLED_WriteData                         0x080023ea   Section        0  oled.o(i.OLED_WriteData)[m
[32m+[m[32m    i.PendSV_Handler                         0x0800240a   Section        0  stm32f10x_it.o(i.PendSV_Handler)[m
[32m+[m[32m    i.RCC_APB1PeriphClockCmd                 0x0800240c   Section        0  stm32f10x_rcc.o(i.RCC_APB1PeriphClockCmd)[m
[32m+[m[32m    i.RCC_APB2PeriphClockCmd                 0x0800242c   Section        0  stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd)[m
[32m+[m[32m    i.RCC_GetClocksFreq                      0x0800244c   Section        0  stm32f10x_rcc.o(i.RCC_GetClocksFreq)[m
[32m+[m[32m    i.SVC_Handler                            0x08002520   Section        0  stm32f10x_it.o(i.SVC_Handler)[m
[32m+[m[32m    i.Serial_Init                            0x08002524   Section        0  serial.o(i.Serial_Init)[m
[32m+[m[32m    i.Serial_Printf                          0x080025dc   Section        0  serial.o(i.Serial_Printf)[m
[32m+[m[32m    i.Serial_SendByte                        0x08002600   Section        0  serial.o(i.Serial_SendByte)[m
[32m+[m[32m    i.Serial_SendString                      0x08002620   Section        0  serial.o(i.Serial_SendString)[m
[32m+[m[32m    i.SetSysClock                            0x0800263a   Section        0  system_stm32f10x.o(i.SetSysClock)[m
[32m+[m[32m    SetSysClock                              0x0800263b   Thumb Code     8  system_stm32f10x.o(i.SetSysClock)[m
[32m+[m[32m    i.SetSysClockTo72                        0x08002644   Section        0  system_stm32f10x.o(i.SetSysClockTo72)[m
[32m+[m[32m    SetSysClockTo72                          0x08002645   Thumb Code   214  system_stm32f10x.o(i.SetSysClockTo72)[m
[32m+[m[32m    i.SysTick_Handler                        0x08002724   Section        0  stm32f10x_it.o(i.SysTick_Handler)[m
[32m+[m[32m    i.SystemInit                             0x08002728   Section        0  system_stm32f10x.o(i.SystemInit)[m
[32m+[m[32m    i.TI1_Config                             0x08002788   Section        0  stm32f10x_tim.o(i.TI1_Config)[m
[32m+[m[32m    TI1_Config                               0x08002789   Thumb Code   108  stm32f10x_tim.o(i.TI1_Config)[m
[32m+[m[32m    i.TI2_Config                             0x08002808   Section        0  stm32f10x_tim.o(i.TI2_Config)[m
[32m+[m[32m    TI2_Config                               0x08002809   Thumb Code   130  stm32f10x_tim.o(i.TI2_Config)[m
[32m+[m[32m    i.TI3_Config                             0x080028a0   Section        0  stm32f10x_tim.o(i.TI3_Config)[m
[32m+[m[32m    TI3_Config                               0x080028a1   Thumb Code   122  stm32f10x_tim.o(i.TI3_Config)[m
[32m+[m[32m    i.TI4_Config                             0x08002930   Section        0  stm32f10x_tim.o(i.TI4_Config)[m
[32m+[m[32m    TI4_Config                               0x08002931   Thumb Code   130  stm32f10x_tim.o(i.TI4_Config)[m
[32m+[m[32m    i.TIM2_IRQHandler                        0x080029c8   Section        0  main.o(i.TIM2_IRQHandler)[m
[32m+[m[32m    i.TIM_ClearFlag                          0x08002a0c   Section        0  stm32f10x_tim.o(i.TIM_ClearFlag)[m
[32m+[m[32m    i.TIM_ClearITPendingBit                  0x08002a12   Section        0  stm32f10x_tim.o(i.TIM_ClearITPendingBit)[m
[32m+[m[32m    i.TIM_Cmd                                0x08002a18   Section        0  stm32f10x_tim.o(i.TIM_Cmd)[m
[32m+[m[32m    i.TIM_EncoderInterfaceConfig             0x08002a30   Section        0  stm32f10x_tim.o(i.TIM_EncoderInterfaceConfig)[m
[32m+[m[32m    i.TIM_GetCounter                         0x08002a72   Section        0  stm32f10x_tim.o(i.TIM_GetCounter)[m
[32m+[m[32m    i.TIM_GetITStatus                        0x08002a78   Section        0  stm32f10x_tim.o(i.TIM_GetITStatus)[m
[32m+[m[32m    i.TIM_ICInit                             0x08002a9c   Section        0  stm32f10x_tim.o(i.TIM_ICInit)[m
[32m+[m[32m    i.TIM_ICStructInit                       0x08002b48   Section        0  stm32f10x_tim.o(i.TIM_ICStructInit)[m
[32m+[m[32m    i.TIM_ITConfig                           0x08002b5a   Section        0  stm32f10x_tim.o(i.TIM_ITConfig)[m
[32m+[m[32m    i.TIM_InternalClockConfig                0x08002b6c   Section        0  stm32f10x_tim.o(i.TIM_InternalClockConfig)[m
[32m+[m[32m    i.TIM_SetCounter                         0x08002b78   Section        0  stm32f10x_tim.o(i.TIM_SetCounter)[m
[32m+[m[32m    i.TIM_SetIC1Prescaler                    0x08002b7c   Section        0  stm32f10x_tim.o(i.TIM_SetIC1Prescaler)[m
[32m+[m[32m    i.TIM_SetIC2Prescaler                    0x08002b8e   Section        0  stm32f10x_tim.o(i.TIM_SetIC2Prescaler)[m
[32m+[m[32m    i.TIM_SetIC3Prescaler                    0x08002ba8   Section        0  stm32f10x_tim.o(i.TIM_SetIC3Prescaler)[m
[32m+[m[32m    i.TIM_SetIC4Prescaler                    0x08002bba   Section        0  stm32f10x_tim.o(i.TIM_SetIC4Prescaler)[m
[32m+[m[32m    i.TIM_TimeBaseInit                       0x08002bd4   Section        0  stm32f10x_tim.o(i.TIM_TimeBaseInit)[m
[32m+[m[32m    i.Timer_Init                             0x08002c78   Section        0  timer.o(i.Timer_Init)[m
[32m+[m[32m    i.USART1_IRQHandler                      0x08002cf4   Section        0  serial.o(i.USART1_IRQHandler)[m
[32m+[m[32m    i.USART_ClearITPendingBit                0x08002d94   Section        0  stm32f10x_usart.o(i.USART_ClearITPendingBit)[m
[32m+[m[32m    i.USART_Cmd                              0x08002db2   Section        0  stm32f10x_usart.o(i.USART_Cmd)[m
[32m+[m[32m    i.USART_GetFlagStatus                    0x08002dca   Section        0  stm32f10x_usart.o(i.USART_GetFlagStatus)[m
[32m+[m[32m    i.USART_GetITStatus                      0x08002de4   Section        0  stm32f10x_usart.o(i.USART_GetITStatus)[m
[32m+[m[32m    i.USART_ITConfig                         0x08002e38   Section        0  stm32f10x_usart.o(i.USART_ITConfig)[m
[32m+[m[32m    i.USART_Init                             0x08002e84   Section        0  stm32f10x_usart.o(i.USART_Init)[m
[32m+[m[32m    i.USART_ReceiveData                      0x08002f5c   Section        0  stm32f10x_usart.o(i.USART_ReceiveData)[m
[32m+[m[32m    i.USART_SendData                         0x08002f66   Section        0  stm32f10x_usart.o(i.USART_SendData)[m
[32m+[m[32m    i.UsageFault_Handler                     0x08002f6e   Section        0  stm32f10x_it.o(i.UsageFault_Handler)[m
[32m+[m[32m    i.__ARM_fpclassify                       0x08002f72   Section        0  fpclassify.o(i.__ARM_fpclassify)[m
[32m+[m[32m    i._is_digit                              0x08002f9a   Section        0  __printf_wp.o(i._is_digit)[m
[32m+[m[32m    i.main                                   0x08002fa8   Section        0  main.o(i.main)[m
[32m+[m[32m    locale$$code                             0x08002fe4   Section       44  lc_numeric_c.o(locale$$code)[m
[32m+[m[32m    locale$$code                             0x08003010   Section       44  lc_ctype_c.o(locale$$code)[m
[32m+[m[32m    x$fpl$printf1                            0x0800303c   Section        4  printf1.o(x$fpl$printf1)[m
[32m+[m[32m    x$fpl$printf2                            0x08003040   Section        4  printf2.o(x$fpl$printf2)[m
[32m+[m[32m    .constdata                               0x08003044   Section     1520  oled.o(.constdata)[m
[32m+[m[32m    x$fpl$usenofp                            0x08003044   Section        0  usenofp.o(x$fpl$usenofp)[m
[32m+[m[32m    .constdata                               0x08003634   Section       17  __printf_flags_ss_wp.o(.constdata)[m
[32m+[m[32m    maptable                                 0x08003634   Data          17  __printf_flags_ss_wp.o(.constdata)[m
[32m+[m[32m    .constdata                               0x08003648   Section        8  _printf_wctomb.o(.constdata)[m
[32m+[m[32m    initial_mbstate                          0x08003648   Data           8  _printf_wctomb.o(.constdata)[m
[32m+[m[32m    .constdata                               0x08003650   Section       40  _printf_hex_int_ll_ptr.o(.constdata)[m
[32m+[m[32m    uc_hextab                                0x08003650   Data          20  _printf_hex_int_ll_ptr.o(.constdata)[m
[32m+[m[32m    lc_hextab                                0x08003664   Data          20  _printf_hex_int_ll_ptr.o(.constdata)[m
[32m+[m[32m    .constdata                               0x08003678   Section       38  _printf_fp_hex.o(.constdata)[m
[32m+[m[32m    lc_hextab                                0x08003678   Data          19  _printf_fp_hex.o(.constdata)[m
[32m+[m[32m    uc_hextab                                0x0800368b   Data          19  _printf_fp_hex.o(.constdata)[m
[32m+[m[32m    .constdata                               0x080036a0   Section      148  bigflt0.o(.constdata)[m
[32m+[m[32m    tenpwrs_x                                0x080036a0   Data          60  bigflt0.o(.constdata)[m
[32m+[m[32m    tenpwrs_i                                0x080036dc   Data          64  bigflt0.o(.constdata)[m
[32m+[m[32m    locale$$data                             0x08003754   Section       28  lc_numeric_c.o(locale$$data)[m
[32m+[m[32m    __lcnum_c_name                           0x08003758   Data           2  lc_numeric_c.o(locale$$data)[m
[32m+[m[32m    __lcnum_c_start                          0x08003760   Data           0  lc_numeric_c.o(locale$$data)[m
[32m+[m[32m    __lcnum_c_point                          0x0800376c   Data           0  lc_numeric_c.o(locale$$data)[m
[32m+[m[32m    __lcnum_c_thousands                      0x0800376e   Data           0  lc_numeric_c.o(locale$$data)[m
[32m+[m[32m    __lcnum_c_grouping                       0x0800376f   Data           0  lc_numeric_c.o(locale$$data)[m
[32m+[m[32m    locale$$data                             0x08003770   Section      272  lc_ctype_c.o(locale$$data)[m
[32m+[m[32m    __lcnum_c_end                            0x08003770   Data           0  lc_numeric_c.o(locale$$data)[m
[32m+[m[32m    __lcctype_c_name                         0x08003774   Data           2  lc_ctype_c.o(locale$$data)[m
[32m+[m[32m    __lcctype_c_start                        0x0800377c   Data           0  lc_ctype_c.o(locale$$data)[m
[32m+[m[32m    __lcctype_c_end                          0x08003880   Data           0  lc_ctype_c.o(locale$$data)[m
[32m+[m[32m    .data                                    0x20000000   Section       20  stm32f10x_rcc.o(.data)[m
[32m+[m[32m    APBAHBPrescTable                         0x20000000   Data          16  stm32f10x_rcc.o(.data)[m
[32m+[m[32m    ADCPrescTable                            0x20000010   Data           4  stm32f10x_rcc.o(.data)[m
[32m+[m[32m    .data                                    0x20000014   Section       26  key.o(.data)[m
[32m+[m[32m    Count                                    0x20000018   Data           1  key.o(.data)[m
[32m+[m[32m    i                                        0x20000019   Data           1  key.o(.data)[m
[32m+[m[32m    CurrState                                0x2000001a   Data           4  key.o(.data)[m
[32m+[m[32m    PrevState                                0x2000001e   Data           4  key.o(.data)[m
[32m+[m[32m    S                                        0x20000022   Data           4  key.o(.data)[m
[32m+[m[32m    Time                                     0x20000026   Data           8  key.o(.data)[m
[32m+[m[32m    .data                                    0x2000002e   Section        3  serial.o(.data)[m
[32m+[m[32m    RxState                                  0x2000002f   Data           1  serial.o(.data)[m
[32m+[m[32m    pRxPacket                                0x20000030   Data           1  serial.o(.data)[m
[32m+[m[32m    .data                                    0x20000032   Section        4  main.o(.data)[m
[32m+[m[32m    Speed_Tim                                0x20000034   Data           2  main.o(.data)[m
[32m+[m[32m    .bss                                     0x20000038   Section      100  serial.o(.bss)[m
[32m+[m[32m    .bss                                     0x2000009c   Section       96  libspace.o(.bss)[m
[32m+[m[32m    HEAP                                     0x20000100   Section      512  startup_stm32f10x_md.o(HEAP)[m
[32m+[m[32m    Heap_Mem                                 0x20000100   Data         512  startup_stm32f10x_md.o(HEAP)[m
[32m+[m[32m    STACK                                    0x20000300   Section     1024  startup_stm32f10x_md.o(STACK)[m
[32m+[m[32m    Stack_Mem                                0x20000300   Data        1024  startup_stm32f10x_md.o(STACK)[m
[32m+[m[32m    __initial_sp                             0x20000700   Data           0  startup_stm32f10x_md.o(STACK)[m
 [m
     Global Symbols[m
 [m
[36m@@ -1226,7 +1478,6 @@[m [mImage Symbol Table[m
     _fp_trap_init                             - Undefined Weak Reference[m
     _fp_trap_shutdown                         - Undefined Weak Reference[m
     _get_lc_collate                           - Undefined Weak Reference[m
[31m-    _get_lc_ctype                             - Undefined Weak Reference[m
     _get_lc_monetary                          - Undefined Weak Reference[m
     _get_lc_time                              - Undefined Weak Reference[m
     _getenv_init                              - Undefined Weak Reference[m
[36m@@ -1234,8 +1485,8 @@[m [mImage Symbol Table[m
     _init_alloc                               - Undefined Weak Reference[m
     _init_user_alloc                          - Undefined Weak Reference[m
     _initio                                   - Undefined Weak Reference[m
[31m-    _printf_post_padding                      - Undefined Weak Reference[m
[31m-    _printf_pre_padding                       - Undefined Weak Reference[m
[32m+[m[32m    _printf_mbtowc                            - Undefined Weak Reference[m
[32m+[m[32m    _printf_wc                                - Undefined Weak Reference[m
     _rand_init                                - Undefined Weak Reference[m
     _signal_finish                            - Undefined Weak Reference[m
     _signal_init                              - Undefined Weak Reference[m
[36m@@ -1252,241 +1503,258 @@[m [mImage Symbol Table[m
     __scatterload_null                       0x08000103   Thumb Code     0  __scatter.o(!!!scatter)[m
     __scatterload_copy                       0x08000129   Thumb Code    26  __scatter_copy.o(!!handler_copy)[m
     __scatterload_zeroinit                   0x08000145   Thumb Code    28  __scatter_zi.o(!!handler_zi)[m
[31m-    _printf_f                                0x08000161   Thumb Code     0  _printf_f.o(.ARM.Collect$$_printf_percent$$00000003)[m
[32m+[m[32m    _printf_n                                0x08000161   Thumb Code     0  _printf_n.o(.ARM.Collect$$_printf_percent$$00000001)[m
     _printf_percent                          0x08000161   Thumb Code     0  _printf_percent.o(.ARM.Collect$$_printf_percent$$00000000)[m
[31m-    _printf_percent_end                      0x08000167   Thumb Code     0  _printf_percent_end.o(.ARM.Collect$$_printf_percent$$00000017)[m
[31m-    __rt_lib_init                            0x0800016b   Thumb Code     0  libinit.o(.ARM.Collect$$libinit$$00000000)[m
[31m-    __rt_lib_init_fp_1                       0x0800016d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000002)[m
[31m-    __rt_lib_init_heap_1                     0x0800016d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000A)[m
[31m-    __rt_lib_init_lc_common                  0x0800016d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000F)[m
[31m-    __rt_lib_init_preinit_1                  0x0800016d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000004)[m
[31m-    __rt_lib_init_rand_1                     0x0800016d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000E)[m
[31m-    __rt_lib_init_user_alloc_1               0x0800016d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000C)[m
[31m-    __rt_lib_init_lc_collate_1               0x08000173   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000011)[m
[31m-    __rt_lib_init_lc_ctype_1                 0x08000173   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000013)[m
[31m-    __rt_lib_init_lc_monetary_1              0x08000173   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000015)[m
[31m-    __rt_lib_init_lc_numeric_2               0x08000173   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000016)[m
[31m-    __rt_lib_init_alloca_1                   0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000002E)[m
[31m-    __rt_lib_init_argv_1                     0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000002C)[m
[31m-    __rt_lib_init_atexit_1                   0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000001B)[m
[31m-    __rt_lib_init_clock_1                    0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000021)[m
[31m-    __rt_lib_init_cpp_1                      0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000032)[m
[31m-    __rt_lib_init_exceptions_1               0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000030)[m
[31m-    __rt_lib_init_fp_trap_1                  0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000001F)[m
[31m-    __rt_lib_init_getenv_1                   0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000023)[m
[31m-    __rt_lib_init_lc_numeric_1               0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000017)[m
[31m-    __rt_lib_init_lc_time_1                  0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000019)[m
[31m-    __rt_lib_init_return                     0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000033)[m
[31m-    __rt_lib_init_signal_1                   0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000001D)[m
[31m-    __rt_lib_init_stdio_1                    0x0800017d   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000025)[m
[31m-    __rt_lib_shutdown                        0x0800017f   Thumb Code     0  libshutdown.o(.ARM.Collect$$libshutdown$$00000000)[m
[31m-    __rt_lib_shutdown_cpp_1                  0x08000181   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000002)[m
[31m-    __rt_lib_shutdown_fp_trap_1              0x08000181   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000007)[m
[31m-    __rt_lib_shutdown_heap_1                 0x08000181   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000F)[m
[31m-    __rt_lib_shutdown_return                 0x08000181   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000010)[m
[31m-    __rt_lib_shutdown_signal_1               0x08000181   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000A)[m
[31m-    __rt_lib_shutdown_stdio_1                0x08000181   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000004)[m
[31m-    __rt_lib_shutdown_user_alloc_1           0x08000181   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000C)[m
[31m-    __rt_entry                               0x08000183   Thumb Code     0  __rtentry.o(.ARM.Collect$$rtentry$$00000000)[m
[31m-    __rt_entry_presh_1                       0x08000183   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$00000002)[m
[31m-    __rt_entry_sh                            0x08000183   Thumb Code     0  __rtentry4.o(.ARM.Collect$$rtentry$$00000004)[m
[31m-    __rt_entry_li                            0x08000189   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000A)[m
[31m-    __rt_entry_postsh_1                      0x08000189   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$00000009)[m
[31m-    __rt_entry_main                          0x0800018d   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000D)[m
[31m-    __rt_entry_postli_1                      0x0800018d   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000C)[m
[31m-    __rt_exit                                0x08000195   Thumb Code     0  rtexit.o(.ARM.Collect$$rtexit$$00000000)[m
[31m-    __rt_exit_ls                             0x08000197   Thumb Code     0  rtexit2.o(.ARM.Collect$$rtexit$$00000003)[m
[31m-    __rt_exit_prels_1                        0x08000197   Thumb Code     0  rtexit2.o(.ARM.Collect$$rtexit$$00000002)[m
[31m-    __rt_exit_exit                           0x0800019b   Thumb Code     0  rtexit2.o(.ARM.Collect$$rtexit$$00000004)[m
[31m-    Reset_Handler                            0x080001a1   Thumb Code     8  startup_stm32f10x_md.o(.text)[m
[31m-    ADC1_2_IRQHandler                        0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    CAN1_RX1_IRQHandler                      0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    CAN1_SCE_IRQHandler                      0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    DMA1_Channel1_IRQHandler                 0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    DMA1_Channel2_IRQHandler                 0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    DMA1_Channel3_IRQHandler                 0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    DMA1_Channel4_IRQHandler                 0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    DMA1_Channel5_IRQHandler                 0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    DMA1_Channel6_IRQHandler                 0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    DMA1_Channel7_IRQHandler                 0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    EXTI15_10_IRQHandler                     0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    EXTI2_IRQHandler                         0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    EXTI3_IRQHandler                         0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    EXTI4_IRQHandler                         0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    EXTI9_5_IRQHandler                       0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    FLASH_IRQHandler                         0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    I2C1_ER_IRQHandler                       0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    I2C1_EV_IRQHandler                       0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    I2C2_ER_IRQHandler                       0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    I2C2_EV_IRQHandler                       0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    PVD_IRQHandler                           0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    RCC_IRQHandler                           0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    RTCAlarm_IRQHandler                      0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    RTC_IRQHandler                           0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    SPI1_IRQHandler                          0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    SPI2_IRQHandler                          0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    TAMPER_IRQHandler                        0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    TIM1_BRK_IRQHandler                      0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    TIM1_CC_IRQHandler                       0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    TIM1_TRG_COM_IRQHandler                  0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    TIM1_UP_IRQHandler                       0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    TIM3_IRQHandler                          0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    TIM4_IRQHandler                          0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    USART1_IRQHandler                        0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    USART2_IRQHandler                        0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    USART3_IRQHandler                        0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    USBWakeUp_IRQHandler                     0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    USB_HP_CAN1_TX_IRQHandler                0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    USB_LP_CAN1_RX0_IRQHandler               0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    WWDG_IRQHandler                          0x080001bb   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    __user_initial_stackheap                 0x080001bd   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[31m-    __2sprintf                               0x080001e1   Thumb Code    34  noretval__2sprintf.o(.text)[m
[31m-    __printf                                 0x08000209   Thumb Code   270  __printf_wp.o(.text)[m
[31m-    __aeabi_memcpy4                          0x08000317   Thumb Code     0  rt_memcpy_w.o(.text)[m
[31m-    __aeabi_memcpy8                          0x08000317   Thumb Code     0  rt_memcpy_w.o(.text)[m
[31m-    __rt_memcpy_w                            0x08000317   Thumb Code   100  rt_memcpy_w.o(.text)[m
[31m-    _memcpy_lastbytes_aligned                0x0800035f   Thumb Code     0  rt_memcpy_w.o(.text)[m
[31m-    strcmp                                   0x0800037d   Thumb Code   128  strcmpv7m.o(.text)[m
[31m-    __use_two_region_memory                  0x080003fd   Thumb Code     2  heapauxi.o(.text)[m
[31m-    __rt_heap_escrow$2region                 0x080003ff   Thumb Code     2  heapauxi.o(.text)[m
[31m-    __rt_heap_expand$2region                 0x08000401   Thumb Code     2  heapauxi.o(.text)[m
[31m-    __lib_sel_fp_printf                      0x08000403   Thumb Code     2  _printf_fp_dec.o(.text)[m
[31m-    _printf_fp_dec_real                      0x080005b5   Thumb Code   620  _printf_fp_dec.o(.text)[m
[31m-    _printf_char_common                      0x0800082b   Thumb Code    32  _printf_char_common.o(.text)[m
[31m-    _sputc                                   0x08000851   Thumb Code    10  _sputc.o(.text)[m
[31m-    __rt_locale                              0x0800085d   Thumb Code     8  rt_locale_intlibspace.o(.text)[m
[31m-    _ll_udiv10                               0x08000865   Thumb Code   138  lludiv10.o(.text)[m
[31m-    _printf_fp_infnan                        0x080008f1   Thumb Code   112  _printf_fp_infnan.o(.text)[m
[31m-    _btod_etento                             0x08000971   Thumb Code   224  bigflt0.o(.text)[m
[31m-    __user_libspace                          0x08000a55   Thumb Code     8  libspace.o(.text)[m
[31m-    __user_perproc_libspace                  0x08000a55   Thumb Code     0  libspace.o(.text)[m
[31m-    __user_perthread_libspace                0x08000a55   Thumb Code     0  libspace.o(.text)[m
[31m-    __user_setup_stackheap                   0x08000a5d   Thumb Code    74  sys_stackheap_outer.o(.text)[m
[31m-    exit                                     0x08000aa7   Thumb Code    18  exit.o(.text)[m
[31m-    _sys_exit                                0x08000ab9   Thumb Code     8  sys_exit.o(.text)[m
[31m-    __I$use$semihosting                      0x08000ac5   Thumb Code     0  use_no_semi.o(.text)[m
[31m-    __use_no_semihosting_swi                 0x08000ac5   Thumb Code     2  use_no_semi.o(.text)[m
[31m-    __semihosting_library_function           0x08000ac7   Thumb Code     0  indicate_semi.o(.text)[m
[31m-    _btod_d2e                                0x08000ac7   Thumb Code    62  btod.o(CL$$btod_d2e)[m
[31m-    _d2e_denorm_low                          0x08000b05   Thumb Code    70  btod.o(CL$$btod_d2e_denorm_low)[m
[31m-    _d2e_norm_op1                            0x08000b4b   Thumb Code    96  btod.o(CL$$btod_d2e_norm_op1)[m
[31m-    __btod_div_common                        0x08000bab   Thumb Code   696  btod.o(CL$$btod_div_common)[m
[31m-    _e2e                                     0x08000ee3   Thumb Code   220  btod.o(CL$$btod_e2e)[m
[31m-    _btod_ediv                               0x08000fbf   Thumb Code    42  btod.o(CL$$btod_ediv)[m
[31m-    _btod_emul                               0x08000fe9   Thumb Code    42  btod.o(CL$$btod_emul)[m
[31m-    __btod_mult_common                       0x08001013   Thumb Code   580  btod.o(CL$$btod_mult_common)[m
[31m-    BusFault_Handler                         0x08001257   Thumb Code     4  stm32f10x_it.o(i.BusFault_Handler)[m
[31m-    DebugMon_Handler                         0x0800125b   Thumb Code     2  stm32f10x_it.o(i.DebugMon_Handler)[m
[31m-    EXTI0_IRQHandler                         0x0800125d   Thumb Code    52  encoder.o(i.EXTI0_IRQHandler)[m
[31m-    EXTI1_IRQHandler                         0x08001299   Thumb Code    52  encoder.o(i.EXTI1_IRQHandler)[m
[31m-    EXTI_ClearITPendingBit                   0x080012d5   Thumb Code     6  stm32f10x_exti.o(i.EXTI_ClearITPendingBit)[m
[31m-    EXTI_GetITStatus                         0x080012e1   Thumb Code    34  stm32f10x_exti.o(i.EXTI_GetITStatus)[m
[31m-    EXTI_Init                                0x08001309   Thumb Code   142  stm32f10x_exti.o(i.EXTI_Init)[m
[31m-    Encoder_Get                              0x0800139d   Thumb Code    14  encoder.o(i.Encoder_Get)[m
[31m-    Encoder_Init                             0x080013b1   Thumb Code   150  encoder.o(i.Encoder_Init)[m
[31m-    FloodLED_LeftTurn                        0x0800144d   Thumb Code    86  led.o(i.FloodLED_LeftTurn)[m
[31m-    FloodLED_RightTurn                       0x080014ad   Thumb Code    86  led.o(i.FloodLED_RightTurn)[m
[31m-    GPIO_EXTILineConfig                      0x0800150d   Thumb Code    60  stm32f10x_gpio.o(i.GPIO_EXTILineConfig)[m
[31m-    GPIO_Init                                0x0800154d   Thumb Code   278  stm32f10x_gpio.o(i.GPIO_Init)[m
[31m-    GPIO_ReadInputDataBit                    0x08001663   Thumb Code    18  stm32f10x_gpio.o(i.GPIO_ReadInputDataBit)[m
[31m-    GPIO_SetBits                             0x08001675   Thumb Code     4  stm32f10x_gpio.o(i.GPIO_SetBits)[m
[31m-    GPIO_WriteBit                            0x08001679   Thumb Code    10  stm32f10x_gpio.o(i.GPIO_WriteBit)[m
[31m-    HardFault_Handler                        0x08001683   Thumb Code     4  stm32f10x_it.o(i.HardFault_Handler)[m
[31m-    Key_Check                                0x08001689   Thumb Code    32  key.o(i.Key_Check)[m
[31m-    Key_GetNum                               0x080016ad   Thumb Code    62  key.o(i.Key_GetNum)[m
[31m-    Key_GetState                             0x080016ed   Thumb Code    80  key.o(i.Key_GetState)[m
[31m-    Key_Init                                 0x08001745   Thumb Code    74  key.o(i.Key_Init)[m
[31m-    Key_Tick                                 0x08001799   Thumb Code   660  key.o(i.Key_Tick)[m
[31m-    LED_DirSet                               0x08001a49   Thumb Code     6  led.o(i.LED_DirSet)[m
[31m-    LED_Init                                 0x08001a55   Thumb Code    50  led.o(i.LED_Init)[m
[31m-    LED_SpeedSet                             0x08001a8d   Thumb Code    12  led.o(i.LED_SpeedSet)[m
[31m-    LED_Tick                                 0x08001aa1   Thumb Code    48  led.o(i.LED_Tick)[m
[31m-    MemManage_Handler                        0x08001add   Thumb Code     4  stm32f10x_it.o(i.MemManage_Handler)[m
[31m-    Menu_Init                                0x08001ae1   Thumb Code   230  menu.o(i.Menu_Init)[m
[31m-    Menu_LED_Direction                       0x08001bfd   Thumb Code    14  menu.o(i.Menu_LED_Direction)[m
[31m-    Menu_LED_Speed                           0x08001c11   Thumb Code    14  menu.o(i.Menu_LED_Speed)[m
[31m-    Menu_Option                              0x08001c25   Thumb Code   382  menu.o(i.Menu_Option)[m
[31m-    Menu_Show                                0x08001db1   Thumb Code   266  menu.o(i.Menu_Show)[m
[31m-    NMI_Handler                              0x08001ed5   Thumb Code     2  stm32f10x_it.o(i.NMI_Handler)[m
[31m-    NVIC_Init                                0x08001ed9   Thumb Code   100  misc.o(i.NVIC_Init)[m
[31m-    NVIC_PriorityGroupConfig                 0x08001f49   Thumb Code    10  misc.o(i.NVIC_PriorityGroupConfig)[m
[31m-    Numlen                                   0x08001f5d   Thumb Code    28  menu.o(i.Numlen)[m
[31m-    OLED_Clear                               0x08001f79   Thumb Code    42  oled.o(i.OLED_Clear)[m
[31m-    OLED_I2C_Init                            0x08001fa5   Thumb Code    76  oled.o(i.OLED_I2C_Init)[m
[31m-    OLED_I2C_SendByte                        0x08001ff5   Thumb Code    88  oled.o(i.OLED_I2C_SendByte)[m
[31m-    OLED_I2C_Start                           0x08002051   Thumb Code    48  oled.o(i.OLED_I2C_Start)[m
[31m-    OLED_I2C_Stop                            0x08002085   Thumb Code    36  oled.o(i.OLED_I2C_Stop)[m
[31m-    OLED_Init                                0x080020ad   Thumb Code   174  oled.o(i.OLED_Init)[m
[31m-    OLED_Pow                                 0x0800215b   Thumb Code    20  oled.o(i.OLED_Pow)[m
[31m-    OLED_SetCursor                           0x0800216f   Thumb Code    34  oled.o(i.OLED_SetCursor)[m
[31m-    OLED_ShowChar                            0x08002191   Thumb Code   110  oled.o(i.OLED_ShowChar)[m
[31m-    OLED_ShowFloatNum                        0x08002205   Thumb Code    50  oled.o(i.OLED_ShowFloatNum)[m
[31m-    OLED_ShowNum                             0x08002241   Thumb Code    68  oled.o(i.OLED_ShowNum)[m
[31m-    OLED_ShowString                          0x08002285   Thumb Code    40  oled.o(i.OLED_ShowString)[m
[31m-    OLED_WriteCommand                        0x080022ad   Thumb Code    32  oled.o(i.OLED_WriteCommand)[m
[31m-    OLED_WriteData                           0x080022cd   Thumb Code    32  oled.o(i.OLED_WriteData)[m
[31m-    PendSV_Handler                           0x080022ed   Thumb Code     2  stm32f10x_it.o(i.PendSV_Handler)[m
[31m-    RCC_APB1PeriphClockCmd                   0x080022f1   Thumb Code    26  stm32f10x_rcc.o(i.RCC_APB1PeriphClockCmd)[m
[31m-    RCC_APB2PeriphClockCmd                   0x08002311   Thumb Code    26  stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd)[m
[31m-    SVC_Handler                              0x08002331   Thumb Code     2  stm32f10x_it.o(i.SVC_Handler)[m
[31m-    SysTick_Handler                          0x0800241d   Thumb Code     2  stm32f10x_it.o(i.SysTick_Handler)[m
[31m-    SystemInit                               0x08002421   Thumb Code    78  system_stm32f10x.o(i.SystemInit)[m
[31m-    TIM2_IRQHandler                          0x08002481   Thumb Code    32  main.o(i.TIM2_IRQHandler)[m
[31m-    TIM_ClearFlag                            0x080024a1   Thumb Code     6  stm32f10x_tim.o(i.TIM_ClearFlag)[m
[31m-    TIM_ClearITPendingBit                    0x080024a7   Thumb Code     6  stm32f10x_tim.o(i.TIM_ClearITPendingBit)[m
[31m-    TIM_Cmd                                  0x080024ad   Thumb Code    24  stm32f10x_tim.o(i.TIM_Cmd)[m
[31m-    TIM_GetITStatus                          0x080024c5   Thumb Code    34  stm32f10x_tim.o(i.TIM_GetITStatus)[m
[31m-    TIM_ITConfig                             0x080024e7   Thumb Code    18  stm32f10x_tim.o(i.TIM_ITConfig)[m
[31m-    TIM_InternalClockConfig                  0x080024f9   Thumb Code    12  stm32f10x_tim.o(i.TIM_InternalClockConfig)[m
[31m-    TIM_TimeBaseInit                         0x08002505   Thumb Code   122  stm32f10x_tim.o(i.TIM_TimeBaseInit)[m
[31m-    Timer_Init                               0x080025a9   Thumb Code   124  timer.o(i.Timer_Init)[m
[31m-    UsageFault_Handler                       0x08002625   Thumb Code     4  stm32f10x_it.o(i.UsageFault_Handler)[m
[31m-    __ARM_fpclassify                         0x08002629   Thumb Code    40  fpclassify.o(i.__ARM_fpclassify)[m
[31m-    _is_digit                                0x08002651   Thumb Code    14  __printf_wp.o(i._is_digit)[m
[31m-    main                                     0x0800265f   Thumb Code    66  main.o(i.main)[m
[31m-    _get_lc_numeric                          0x080026a1   Thumb Code    44  lc_numeric_c.o(locale$$code)[m
[31m-    __fpl_dretinf                            0x080026cd   Thumb Code    12  dretinf.o(x$fpl$dretinf)[m
[31m-    __aeabi_f2d                              0x080026d9   Thumb Code     0  f2d.o(x$fpl$f2d)[m
[31m-    _f2d                                     0x080026d9   Thumb Code    86  f2d.o(x$fpl$f2d)[m
[31m-    __aeabi_fdiv                             0x08002731   Thumb Code     0  fdiv.o(x$fpl$fdiv)[m
[31m-    _fdiv                                    0x08002731   Thumb Code   384  fdiv.o(x$fpl$fdiv)[m
[31m-    __aeabi_i2f                              0x080028b5   Thumb Code     0  fflt_clz.o(x$fpl$fflt)[m
[31m-    _fflt                                    0x080028b5   Thumb Code    48  fflt_clz.o(x$fpl$fflt)[m
[31m-    __fpl_fnaninf                            0x080028e5   Thumb Code   140  fnaninf.o(x$fpl$fnaninf)[m
[31m-    __fpl_fretinf                            0x08002971   Thumb Code    10  fretinf.o(x$fpl$fretinf)[m
[31m-    _printf_fp_dec                           0x0800297b   Thumb Code     4  printf1.o(x$fpl$printf1)[m
[31m-    FloodLED_Total                           0x0800297e   Data           2  led.o(.constdata)[m
[31m-    __I$use$fp                               0x0800297e   Number         0  usenofp.o(x$fpl$usenofp)[m
[31m-    FloodLED                                 0x08002980   Data           8  led.o(.constdata)[m
[31m-    LED_Speed_Total                          0x08002988   Data           2  led.o(.constdata)[m
[31m-    LED_Speed_Option                         0x0800298a   Data           6  led.o(.constdata)[m
[31m-    OLED_F8x16                               0x08002990   Data        1520  oled.o(.constdata)[m
[31m-    Region$$Table$$Base                      0x080031a8   Number         0  anon$$obj.o(Region$$Table)[m
[31m-    Region$$Table$$Limit                     0x080031c8   Number         0  anon$$obj.o(Region$$Table)[m
[31m-    Period                                   0x20000000   Data           4  led.o(.data)[m
[31m-    LEDStatus                                0x20000004   Data           1  led.o(.data)[m
[31m-    Key_Flag                                 0x2000000c   Data           4  key.o(.data)[m
[31m-    cur                                      0x20000028   Data           4  menu.o(.data)[m
[31m-    main_Menu_sons                           0x2000002c   Data          16  menu.o(.data)[m
[31m-    LED_Control_sons                         0x2000003c   Data           8  menu.o(.data)[m
[31m-    PID_sons                                 0x20000044   Data          12  menu.o(.data)[m
[31m-    Image_sons                               0x20000050   Data           4  menu.o(.data)[m
[31m-    Angle_sons                               0x20000054   Data           4  menu.o(.data)[m
[31m-    Mode_Name                                0x20000058   Data           8  menu.o(.data)[m
[31m-    modeIndex                                0x20000060   Data           4  menu.o(.data)[m
[31m-    Encoder_Count                            0x20000064   Data           2  encoder.o(.data)[m
[31m-    main_Menu                                0x20000068   Data          32  menu.o(.bss)[m
[31m-    LED_Control                              0x20000088   Data          32  menu.o(.bss)[m
[31m-    Image                                    0x200000a8   Data          32  menu.o(.bss)[m
[31m-    PID                                      0x200000c8   Data          32  menu.o(.bss)[m
[31m-    LED_Speed                                0x200000e8   Data          32  menu.o(.bss)[m
[31m-    LED_Direction                            0x20000108   Data          32  menu.o(.bss)[m
[31m-    Angle                                    0x20000128   Data          32  menu.o(.bss)[m
[31m-    Set_KP                                   0x20000148   Data          32  menu.o(.bss)[m
[31m-    Set_KI                                   0x20000168   Data          32  menu.o(.bss)[m
[31m-    Set_KD                                   0x20000188   Data          32  menu.o(.bss)[m
[31m-    __libspace_start                         0x200001a8   Data          96  libspace.o(.bss)[m
[31m-    __temporary_stack_top$libspace           0x20000208   Data           0  libspace.o(.bss)[m
[32m+[m[32m    _printf_p                                0x08000167   Thumb Code     0  _printf_p.o(.ARM.Collect$$_printf_percent$$00000002)[m
[32m+[m[32m    _printf_f                                0x0800016d   Thumb Code     0  _printf_f.o(.ARM.Collect$$_printf_percent$$00000003)[m
[32m+[m[32m    _printf_e                                0x08000173   Thumb Code     0  _printf_e.o(.ARM.Collect$$_printf_percent$$00000004)[m
[32m+[m[32m    _printf_g                                0x08000179   Thumb Code     0  _printf_g.o(.ARM.Collect$$_printf_percent$$00000005)[m
[32m+[m[32m    _printf_a                                0x0800017f   Thumb Code     0  _printf_a.o(.ARM.Collect$$_printf_percent$$00000006)[m
[32m+[m[32m    _printf_ll                               0x08000185   Thumb Code     0  _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007)[m
[32m+[m[32m    _printf_i                                0x0800018f   Thumb Code     0  _printf_i.o(.ARM.Collect$$_printf_percent$$00000008)[m
[32m+[m[32m    _printf_d                                0x08000195   Thumb Code     0  _printf_d.o(.ARM.Collect$$_printf_percent$$00000009)[m
[32m+[m[32m    _printf_u                                0x0800019b   Thumb Code     0  _printf_u.o(.ARM.Collect$$_printf_percent$$0000000A)[m
[32m+[m[32m    _printf_o                                0x080001a1   Thumb Code     0  _printf_o.o(.ARM.Collect$$_printf_percent$$0000000B)[m
[32m+[m[32m    _printf_x                                0x080001a7   Thumb Code     0  _printf_x.o(.ARM.Collect$$_printf_percent$$0000000C)[m
[32m+[m[32m    _printf_lli                              0x080001ad   Thumb Code     0  _printf_lli.o(.ARM.Collect$$_printf_percent$$0000000D)[m
[32m+[m[32m    _printf_lld                              0x080001b3   Thumb Code     0  _printf_lld.o(.ARM.Collect$$_printf_percent$$0000000E)[m
[32m+[m[32m    _printf_llu                              0x080001b9   Thumb Code     0  _printf_llu.o(.ARM.Collect$$_printf_percent$$0000000F)[m
[32m+[m[32m    _printf_llo                              0x080001bf   Thumb Code     0  _printf_llo.o(.ARM.Collect$$_printf_percent$$00000010)[m
[32m+[m[32m    _printf_llx                              0x080001c5   Thumb Code     0  _printf_llx.o(.ARM.Collect$$_printf_percent$$00000011)[m
[32m+[m[32m    _printf_l                                0x080001cb   Thumb Code     0  _printf_l.o(.ARM.Collect$$_printf_percent$$00000012)[m
[32m+[m[32m    _printf_c                                0x080001d5   Thumb Code     0  _printf_c.o(.ARM.Collect$$_printf_percent$$00000013)[m
[32m+[m[32m    _printf_s                                0x080001db   Thumb Code     0  _printf_s.o(.ARM.Collect$$_printf_percent$$00000014)[m
[32m+[m[32m    _printf_lc                               0x080001e1   Thumb Code     0  _printf_lc.o(.ARM.Collect$$_printf_percent$$00000015)[m
[32m+[m[32m    _printf_ls                               0x080001e7   Thumb Code     0  _printf_ls.o(.ARM.Collect$$_printf_percent$$00000016)[m
[32m+[m[32m    _printf_percent_end                      0x080001ed   Thumb Code     0  _printf_percent_end.o(.ARM.Collect$$_printf_percent$$00000017)[m
[32m+[m[32m    __rt_lib_init                            0x080001f1   Thumb Code     0  libinit.o(.ARM.Collect$$libinit$$00000000)[m
[32m+[m[32m    __rt_lib_init_fp_1                       0x080001f3   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000002)[m
[32m+[m[32m    __rt_lib_init_heap_1                     0x080001f3   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000A)[m
[32m+[m[32m    __rt_lib_init_lc_common                  0x080001f3   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000F)[m
[32m+[m[32m    __rt_lib_init_preinit_1                  0x080001f3   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000004)[m
[32m+[m[32m    __rt_lib_init_rand_1                     0x080001f3   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000E)[m
[32m+[m[32m    __rt_lib_init_user_alloc_1               0x080001f3   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000000C)[m
[32m+[m[32m    __rt_lib_init_lc_collate_1               0x080001f9   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000011)[m
[32m+[m[32m    __rt_lib_init_lc_ctype_2                 0x080001f9   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000012)[m
[32m+[m[32m    __rt_lib_init_lc_ctype_1                 0x08000205   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000013)[m
[32m+[m[32m    __rt_lib_init_lc_monetary_1              0x08000205   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000015)[m
[32m+[m[32m    __rt_lib_init_lc_numeric_2               0x08000205   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000016)[m
[32m+[m[32m    __rt_lib_init_alloca_1                   0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000002E)[m
[32m+[m[32m    __rt_lib_init_argv_1                     0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000002C)[m
[32m+[m[32m    __rt_lib_init_atexit_1                   0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000001B)[m
[32m+[m[32m    __rt_lib_init_clock_1                    0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000021)[m
[32m+[m[32m    __rt_lib_init_cpp_1                      0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000032)[m
[32m+[m[32m    __rt_lib_init_exceptions_1               0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000030)[m
[32m+[m[32m    __rt_lib_init_fp_trap_1                  0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000001F)[m
[32m+[m[32m    __rt_lib_init_getenv_1                   0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000023)[m
[32m+[m[32m    __rt_lib_init_lc_numeric_1               0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000017)[m
[32m+[m[32m    __rt_lib_init_lc_time_1                  0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000019)[m
[32m+[m[32m    __rt_lib_init_return                     0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000033)[m
[32m+[m[32m    __rt_lib_init_signal_1                   0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$0000001D)[m
[32m+[m[32m    __rt_lib_init_stdio_1                    0x0800020f   Thumb Code     0  libinit2.o(.ARM.Collect$$libinit$$00000025)[m
[32m+[m[32m    __rt_lib_shutdown                        0x08000211   Thumb Code     0  libshutdown.o(.ARM.Collect$$libshutdown$$00000000)[m
[32m+[m[32m    __rt_lib_shutdown_cpp_1                  0x08000213   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000002)[m
[32m+[m[32m    __rt_lib_shutdown_fp_trap_1              0x08000213   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000007)[m
[32m+[m[32m    __rt_lib_shutdown_heap_1                 0x08000213   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000F)[m
[32m+[m[32m    __rt_lib_shutdown_return                 0x08000213   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000010)[m
[32m+[m[32m    __rt_lib_shutdown_signal_1               0x08000213   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000A)[m
[32m+[m[32m    __rt_lib_shutdown_stdio_1                0x08000213   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$00000004)[m
[32m+[m[32m    __rt_lib_shutdown_user_alloc_1           0x08000213   Thumb Code     0  libshutdown2.o(.ARM.Collect$$libshutdown$$0000000C)[m
[32m+[m[32m    __rt_entry                               0x08000215   Thumb Code     0  __rtentry.o(.ARM.Collect$$rtentry$$00000000)[m
[32m+[m[32m    __rt_entry_presh_1                       0x08000215   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$00000002)[m
[32m+[m[32m    __rt_entry_sh                            0x08000215   Thumb Code     0  __rtentry4.o(.ARM.Collect$$rtentry$$00000004)[m
[32m+[m[32m    __rt_entry_li                            0x0800021b   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000A)[m
[32m+[m[32m    __rt_entry_postsh_1                      0x0800021b   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$00000009)[m
[32m+[m[32m    __rt_entry_main                          0x0800021f   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000D)[m
[32m+[m[32m    __rt_entry_postli_1                      0x0800021f   Thumb Code     0  __rtentry2.o(.ARM.Collect$$rtentry$$0000000C)[m
[32m+[m[32m    __rt_exit                                0x08000227   Thumb Code     0  rtexit.o(.ARM.Collect$$rtexit$$00000000)[m
[32m+[m[32m    __rt_exit_ls                             0x08000229   Thumb Code     0  rtexit2.o(.ARM.Collect$$rtexit$$00000003)[m
[32m+[m[32m    __rt_exit_prels_1                        0x08000229   Thumb Code     0  rtexit2.o(.ARM.Collect$$rtexit$$00000002)[m
[32m+[m[32m    __rt_exit_exit                           0x0800022d   Thumb Code     0  rtexit2.o(.ARM.Collect$$rtexit$$00000004)[m
[32m+[m[32m    Reset_Handler                            0x08000235   Thumb Code     8  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    ADC1_2_IRQHandler                        0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    CAN1_RX1_IRQHandler                      0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    CAN1_SCE_IRQHandler                      0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    DMA1_Channel1_IRQHandler                 0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    DMA1_Channel2_IRQHandler                 0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    DMA1_Channel3_IRQHandler                 0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    DMA1_Channel4_IRQHandler                 0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    DMA1_Channel5_IRQHandler                 0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    DMA1_Channel6_IRQHandler                 0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    DMA1_Channel7_IRQHandler                 0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    EXTI0_IRQHandler                         0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    EXTI15_10_IRQHandler                     0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    EXTI1_IRQHandler                         0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    EXTI2_IRQHandler                         0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    EXTI3_IRQHandler                         0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    EXTI4_IRQHandler                         0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    EXTI9_5_IRQHandler                       0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    FLASH_IRQHandler                         0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    I2C1_ER_IRQHandler                       0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    I2C1_EV_IRQHandler                       0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    I2C2_ER_IRQHandler                       0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    I2C2_EV_IRQHandler                       0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    PVD_IRQHandler                           0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    RCC_IRQHandler                           0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    RTCAlarm_IRQHandler                      0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    RTC_IRQHandler                           0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    SPI1_IRQHandler                          0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    SPI2_IRQHandler                          0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    TAMPER_IRQHandler                        0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    TIM1_BRK_IRQHandler                      0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    TIM1_CC_IRQHandler                       0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    TIM1_TRG_COM_IRQHandler                  0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    TIM1_UP_IRQHandler                       0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    TIM3_IRQHandler                          0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    TIM4_IRQHandler                          0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    USART2_IRQHandler                        0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    USART3_IRQHandler                        0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    USBWakeUp_IRQHandler                     0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    USB_HP_CAN1_TX_IRQHandler                0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    USB_LP_CAN1_RX0_IRQHandler               0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    WWDG_IRQHandler                          0x0800024f   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    __user_initial_stackheap                 0x08000251   Thumb Code     0  startup_stm32f10x_md.o(.text)[m
[32m+[m[32m    vsprintf                                 0x08000275   Thumb Code    32  vsprintf.o(.text)[m
[32m+[m[32m    __printf                                 0x08000299   Thumb Code   388  __printf_flags_ss_wp.o(.text)[m
[32m+[m[32m    strcmp                                   0x08000421   Thumb Code   128  strcmpv7m.o(.text)[m
[32m+[m[32m    __use_two_region_memory                  0x080004a1   Thumb Code     2  heapauxi.o(.text)[m
[32m+[m[32m    __rt_heap_escrow$2region                 0x080004a3   Thumb Code     2  heapauxi.o(.text)[m
[32m+[m[32m    __rt_heap_expand$2region                 0x080004a5   Thumb Code     2  heapauxi.o(.text)[m
[32m+[m[32m    _printf_pre_padding                      0x080004a7   Thumb Code    44  _printf_pad.o(.text)[m
[32m+[m[32m    _printf_post_padding                     0x080004d3   Thumb Code    34  _printf_pad.o(.text)[m
[32m+[m[32m    _printf_truncate_signed                  0x080004f5   Thumb Code    18  _printf_truncate.o(.text)[m
[32m+[m[32m    _printf_truncate_unsigned                0x08000507   Thumb Code    18  _printf_truncate.o(.text)[m
[32m+[m[32m    _printf_str                              0x08000519   Thumb Code    82  _printf_str.o(.text)[m
[32m+[m[32m    _printf_int_dec                          0x0800056d   Thumb Code   104  _printf_dec.o(.text)[m
[32m+[m[32m    _printf_charcount                        0x080005e5   Thumb Code    40  _printf_charcount.o(.text)[m
[32m+[m[32m    __lib_sel_fp_printf                      0x0800060d   Thumb Code     2  _printf_fp_dec.o(.text)[m
[32m+[m[32m    _printf_fp_dec_real                      0x080007bf   Thumb Code   620  _printf_fp_dec.o(.text)[m
[32m+[m[32m    _printf_char_common                      0x08000a37   Thumb Code    32  _printf_char_common.o(.text)[m
[32m+[m[32m    _sputc                                   0x08000a5d   Thumb Code    10  _sputc.o(.text)[m
[32m+[m[32m    _printf_wctomb                           0x08000a69   Thumb Code   182  _printf_wctomb.o(.text)[m
[32m+[m[32m    _printf_longlong_dec                     0x08000b25   Thumb Code   108  _printf_longlong_dec.o(.text)[m
[32m+[m[32m    _printf_longlong_oct                     0x08000ba1   Thumb Code    66  _printf_oct_int_ll.o(.text)[m
[32m+[m[32m    _printf_int_oct                          0x08000be3   Thumb Code    24  _printf_oct_int_ll.o(.text)[m
[32m+[m[32m    _printf_ll_oct                           0x08000bfb   Thumb Code    12  _printf_oct_int_ll.o(.text)[m
[32m+[m[32m    _printf_longlong_hex                     0x08000c11   Thumb Code    86  _printf_hex_int_ll_ptr.o(.text)[m
[32m+[m[32m    _printf_int_hex                          0x08000c67   Thumb Code    28  _printf_hex_int_ll_ptr.o(.text)[m
[32m+[m[32m    _printf_ll_hex                           0x08000c83   Thumb Code    12  _printf_hex_int_ll_ptr.o(.text)[m
[32m+[m[32m    _printf_hex_ptr                          0x08000c8f   Thumb Code    18  _printf_hex_int_ll_ptr.o(.text)[m
[32m+[m[32m    __rt_locale                              0x08000ca5   Thumb Code     8  rt_locale_intlibspace.o(.text)[m
[32m+[m[32m    _ll_udiv10                               0x08000cad   Thumb Code   138  lludiv10.o(.text)[m
[32m+[m[32m    _printf_int_common                       0x08000d37   Thumb Code   178  _printf_intcommon.o(.text)[m
[32m+[m[32m    _printf_fp_hex_real                      0x08000de9   Thumb Code   756  _printf_fp_hex.o(.text)[m
[32m+[m[32m    _printf_fp_infnan                        0x080010e5   Thumb Code   112  _printf_fp_infnan.o(.text)[m
[32m+[m[32m    _printf_cs_common                        0x08001165   Thumb Code    20  _printf_char.o(.text)[m
[32m+[m[32m    _printf_char                             0x08001179   Thumb Code    16  _printf_char.o(.text)[m
[32m+[m[32m    _printf_string                           0x08001189   Thumb Code     8  _printf_char.o(.text)[m
[32m+[m[32m    _printf_lcs_common                       0x08001191   Thumb Code    20  _printf_wchar.o(.text)[m
[32m+[m[32m    _printf_wchar                            0x080011a5   Thumb Code    16  _printf_wchar.o(.text)[m
[32m+[m[32m    _printf_wstring                          0x080011b5   Thumb Code     8  _printf_wchar.o(.text)[m
[32m+[m[32m    _btod_etento                             0x080011bd   Thumb Code   224  bigflt0.o(.text)[m
[32m+[m[32m    _wcrtomb                                 0x080012a1   Thumb Code    64  _wcrtomb.o(.text)[m
[32m+[m[32m    __user_libspace                          0x080012e1   Thumb Code     8  libspace.o(.text)[m
[32m+[m[32m    __user_perproc_libspace                  0x080012e1   Thumb Code     0  libspace.o(.text)[m
[32m+[m[32m    __user_perthread_libspace                0x080012e1   Thumb Code     0  libspace.o(.text)[m
[32m+[m[32m    __user_setup_stackheap                   0x080012e9   Thumb Code    74  sys_stackheap_outer.o(.text)[m
[32m+[m[32m    __rt_ctype_table                         0x08001335   Thumb Code    16  rt_ctype_table.o(.text)[m
[32m+[m[32m    exit                                     0x08001345   Thumb Code    18  exit.o(.text)[m
[32m+[m[32m    _sys_exit                                0x08001359   Thumb Code     8  sys_exit.o(.text)[m
[32m+[m[32m    __I$use$semihosting                      0x08001365   Thumb Code     0  use_no_semi.o(.text)[m
[32m+[m[32m    __use_no_semihosting_swi                 0x08001365   Thumb Code     2  use_no_semi.o(.text)[m
[32m+[m[32m    __semihosting_library_function           0x08001367   Thumb Code     0  indicate_semi.o(.text)[m
[32m+[m[32m    _btod_d2e                                0x08001367   Thumb Code    62  btod.o(CL$$btod_d2e)[m
[32m+[m[32m    _d2e_denorm_low                          0x080013a5   Thumb Code    70  btod.o(CL$$btod_d2e_denorm_low)[m
[32m+[m[32m    _d2e_norm_op1                            0x080013eb   Thumb Code    96  btod.o(CL$$btod_d2e_norm_op1)[m
[32m+[m[32m    __btod_div_common                        0x0800144b   Thumb Code   696  btod.o(CL$$btod_div_common)[m
[32m+[m[32m    _e2e                                     0x08001783   Thumb Code   220  btod.o(CL$$btod_e2e)[m
[32m+[m[32m    _btod_ediv                               0x0800185f   Thumb Code    42  btod.o(CL$$btod_ediv)[m
[32m+[m[32m    _btod_emul                               0x08001889   Thumb Code    42  btod.o(CL$$btod_emul)[m
[32m+[m[32m    __btod_mult_common                       0x080018b3   Thumb Code   580  btod.o(CL$$btod_mult_common)[m
[32m+[m[32m    BusFault_Handler                         0x08001af7   Thumb Code     4  stm32f10x_it.o(i.BusFault_Handler)[m
[32m+[m[32m    DebugMon_Handler                         0x08001afb   Thumb Code     2  stm32f10x_it.o(i.DebugMon_Handler)[m
[32m+[m[32m    Encoder_Get                              0x08001afd   Thumb Code    22  encoder.o(i.Encoder_Get)[m
[32m+[m[32m    Encoder_Init                             0x08001b19   Thumb Code   152  encoder.o(i.Encoder_Init)[m
[32m+[m[32m    GPIO_Init                                0x08001bb9   Thumb Code   278  stm32f10x_gpio.o(i.GPIO_Init)[m
[32m+[m[32m    GPIO_ReadInputDataBit                    0x08001ccf   Thumb Code    18  stm32f10x_gpio.o(i.GPIO_ReadInputDataBit)[m
[32m+[m[32m    GPIO_WriteBit                            0x08001ce1   Thumb Code    10  stm32f10x_gpio.o(i.GPIO_WriteBit)[m
[32m+[m[32m    HardFault_Handler                        0x08001ceb   Thumb Code     4  stm32f10x_it.o(i.HardFault_Handler)[m
[32m+[m[32m    Key_GetState                             0x08001cf1   Thumb Code    80  key.o(i.Key_GetState)[m
[32m+[m[32m    Key_Init                                 0x08001d49   Thumb Code    74  key.o(i.Key_Init)[m
[32m+[m[32m    Key_Tick                                 0x08001d9d   Thumb Code   660  key.o(i.Key_Tick)[m
[32m+[m[32m    MemManage_Handler                        0x0800204d   Thumb Code     4  stm32f10x_it.o(i.MemManage_Handler)[m
[32m+[m[32m    NMI_Handler                              0x08002051   Thumb Code     2  stm32f10x_it.o(i.NMI_Handler)[m
[32m+[m[32m    NVIC_Init                                0x08002055   Thumb Code   100  misc.o(i.NVIC_Init)[m
[32m+[m[32m    NVIC_PriorityGroupConfig                 0x080020c5   Thumb Code    10  misc.o(i.NVIC_PriorityGroupConfig)[m
[32m+[m[32m    OLED_Clear                               0x080020d9   Thumb Code    42  oled.o(i.OLED_Clear)[m
[32m+[m[32m    OLED_I2C_Init                            0x08002105   Thumb Code    76  oled.o(i.OLED_I2C_Init)[m
[32m+[m[32m    OLED_I2C_SendByte                        0x08002155   Thumb Code    88  oled.o(i.OLED_I2C_SendByte)[m
[32m+[m[32m    OLED_I2C_Start                           0x080021b1   Thumb Code    48  oled.o(i.OLED_I2C_Start)[m
[32m+[m[32m    OLED_I2C_Stop                            0x080021e5   Thumb Code    36  oled.o(i.OLED_I2C_Stop)[m
[32m+[m[32m    OLED_Init                                0x0800220d   Thumb Code   174  oled.o(i.OLED_Init)[m
[32m+[m[32m    OLED_Pow                                 0x080022bb   Thumb Code    20  oled.o(i.OLED_Pow)[m
[32m+[m[32m    OLED_SetCursor                           0x080022cf   Thumb Code    34  oled.o(i.OLED_SetCursor)[m
[32m+[m[32m    OLED_ShowChar                            0x080022f1   Thumb Code   110  oled.o(i.OLED_ShowChar)[m
[32m+[m[32m    OLED_ShowSignedNum                       0x08002365   Thumb Code   102  oled.o(i.OLED_ShowSignedNum)[m
[32m+[m[32m    OLED_WriteCommand                        0x080023cb   Thumb Code    32  oled.o(i.OLED_WriteCommand)[m
[32m+[m[32m    OLED_WriteData                           0x080023eb   Thumb Code    32  oled.o(i.OLED_WriteData)[m
[32m+[m[32m    PendSV_Handler                           0x0800240b   Thumb Code     2  stm32f10x_it.o(i.PendSV_Handler)[m
[32m+[m[32m    RCC_APB1PeriphClockCmd                   0x0800240d   Thumb Code    26  stm32f10x_rcc.o(i.RCC_APB1PeriphClockCmd)[m
[32m+[m[32m    RCC_APB2PeriphClockCmd                   0x0800242d   Thumb Code    26  stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd)[m
[32m+[m[32m    RCC_GetClocksFreq                        0x0800244d   Thumb Code   192  stm32f10x_rcc.o(i.RCC_GetClocksFreq)[m
[32m+[m[32m    SVC_Handler                              0x08002521   Thumb Code     2  stm32f10x_it.o(i.SVC_Handler)[m
[32m+[m[32m    Serial_Init                              0x08002525   Thumb Code   174  serial.o(i.Serial_Init)[m
[32m+[m[32m    Serial_Printf                            0x080025dd   Thumb Code    36  serial.o(i.Serial_Printf)[m
[32m+[m[32m    Serial_SendByte                          0x08002601   Thumb Code    28  serial.o(i.Serial_SendByte)[m
[32m+[m[32m    Serial_SendString                        0x08002621   Thumb Code    26  serial.o(i.Serial_SendString)[m
[32m+[m[32m    SysTick_Handler                          0x08002725   Thumb Code     2  stm32f10x_it.o(i.SysTick_Handler)[m
[32m+[m[32m    SystemInit                               0x08002729   Thumb Code    78  system_stm32f10x.o(i.SystemInit)[m
[32m+[m[32m    TIM2_IRQHandler                          0x080029c9   Thumb Code    60  main.o(i.TIM2_IRQHandler)[m
[32m+[m[32m    TIM_ClearFlag                            0x08002a0d   Thumb Code     6  stm32f10x_tim.o(i.TIM_ClearFlag)[m
[32m+[m[32m    TIM_ClearITPendingBit                    0x08002a13   Thumb Code     6  stm32f10x_tim.o(i.TIM_ClearITPendingBit)[m
[32m+[m[32m    TIM_Cmd                                  0x08002a19   Thumb Code    24  stm32f10x_tim.o(i.TIM_Cmd)[m
[32m+[m[32m    TIM_EncoderInterfaceConfig               0x08002a31   Thumb Code    66  stm32f10x_tim.o(i.TIM_EncoderInterfaceConfig)[m
[32m+[m[32m    TIM_GetCounter                           0x08002a73   Thumb Code     6  stm32f10x_tim.o(i.TIM_GetCounter)[m
[32m+[m[32m    TIM_GetITStatus                          0x08002a79   Thumb Code    34  stm32f10x_tim.o(i.TIM_GetITStatus)[m
[32m+[m[32m    TIM_ICInit                               0x08002a9d   Thumb Code   150  stm32f10x_tim.o(i.TIM_ICInit)[m
[32m+[m[32m    TIM_ICStructInit                         0x08002b49   Thumb Code    18  stm32f10x_tim.o(i.TIM_ICStructInit)[m
[32m+[m[32m    TIM_ITConfig                             0x08002b5b   Thumb Code    18  stm32f10x_tim.o(i.TIM_ITConfig)[m
[32m+[m[32m    TIM_InternalClockConfig                  0x08002b6d   Thumb Code    12  stm32f10x_tim.o(i.TIM_InternalClockConfig)[m
[32m+[m[32m    TIM_SetCounter                           0x08002b79   Thumb Code     4  stm32f10x_tim.o(i.TIM_SetCounter)[m
[32m+[m[32m    TIM_SetIC1Prescaler                      0x08002b7d   Thumb Code    18  stm32f10x_tim.o(i.TIM_SetIC1Prescaler)[m
[32m+[m[32m    TIM_SetIC2Prescaler                      0x08002b8f   Thumb Code    26  stm32f10x_tim.o(i.TIM_SetIC2Prescaler)[m
[32m+[m[32m    TIM_SetIC3Prescaler                      0x08002ba9   Thumb Code    18  stm32f10x_tim.o(i.TIM_SetIC3Prescaler)[m
[32m+[m[32m    TIM_SetIC4Prescaler                      0x08002bbb   Thumb Code    26  stm32f10x_tim.o(i.TIM_SetIC4Prescaler)[m
[32m+[m[32m    TIM_TimeBaseInit                         0x08002bd5   Thumb Code   122  stm32f10x_tim.o(i.TIM_TimeBaseInit)[m
[32m+[m[32m    Timer_Init                               0x08002c79   Thumb Code   124  timer.o(i.Timer_Init)[m
[32m+[m[32m    USART1_IRQHandler                        0x08002cf5   Thumb Code   140  serial.o(i.USART1_IRQHandler)[m
[32m+[m[32m    USART_ClearITPendingBit                  0x08002d95   Thumb Code    30  stm32f10x_usart.o(i.USART_ClearITPendingBit)[m
[32m+[m[32m    USART_Cmd                                0x08002db3   Thumb Code    24  stm32f10x_usart.o(i.USART_Cmd)[m
[32m+[m[32m    USART_GetFlagStatus                      0x08002dcb   Thumb Code    26  stm32f10x_usart.o(i.USART_GetFlagStatus)[m
[32m+[m[32m    USART_GetITStatus                        0x08002de5   Thumb Code    84  stm32f10x_usart.o(i.USART_GetITStatus)[m
[32m+[m[32m    USART_ITConfig                           0x08002e39   Thumb Code    74  stm32f10x_usart.o(i.USART_ITConfig)[m
[32m+[m[32m    USART_Init                               0x08002e85   Thumb Code   210  stm32f10x_usart.o(i.USART_Init)[m
[32m+[m[32m    USART_ReceiveData                        0x08002f5d   Thumb Code    10  stm32f10x_usart.o(i.USART_ReceiveData)[m
[32m+[m[32m    USART_SendData                           0x08002f67   Thumb Code     8  stm32f10x_usart.o(i.USART_SendData)[m
[32m+[m[32m    UsageFault_Handler                       0x08002f6f   Thumb Code     4  stm32f10x_it.o(i.UsageFault_Handler)[m
[32m+[m[32m    __ARM_fpclassify                         0x08002f73   Thumb Code    40  fpclassify.o(i.__ARM_fpclassify)[m
[32m+[m[32m    _is_digit                                0x08002f9b   Thumb Code    14  __printf_wp.o(i._is_digit)[m
[32m+[m[32m    main                                     0x08002fa9   Thumb Code    52  main.o(i.main)[m
[32m+[m[32m    _get_lc_numeric                          0x08002fe5   Thumb Code    44  lc_numeric_c.o(locale$$code)[m
[32m+[m[32m    _get_lc_ctype                            0x08003011   Thumb Code    44  lc_ctype_c.o(locale$$code)[m
[32m+[m[32m    _printf_fp_dec                           0x0800303d   Thumb Code     4  printf1.o(x$fpl$printf1)[m
[32m+[m[32m    _printf_fp_hex                           0x08003041   Thumb Code     4  printf2.o(x$fpl$printf2)[m
[32m+[m[32m    OLED_F8x16                               0x08003044   Data        1520  oled.o(.constdata)[m
[32m+[m[32m    __I$use$fp                               0x08003044   Number         0  usenofp.o(x$fpl$usenofp)[m
[32m+[m[32m    Region$$Table$$Base                      0x08003734   Number         0  anon$$obj.o(Region$$Table)[m
[32m+[m[32m    Region$$Table$$Limit                     0x08003754   Number         0  anon$$obj.o(Region$$Table)[m
[32m+[m[32m    __ctype                                  0x0800377d   Data           0  lc_ctype_c.o(locale$$data)[m
[32m+[m[32m    Key_Flag                                 0x20000014   Data           4  key.o(.data)[m
[32m+[m[32m    Serial_RxFlag                            0x2000002e   Data           1  serial.o(.data)[m
[32m+[m[32m    Speed                                    0x20000032   Data           2  main.o(.data)[m
[32m+[m[32m    Serial_RxPacket                          0x20000038   Data         100  serial.o(.bss)[m
[32m+[m[32m    __libspace_start                         0x2000009c   Data          96  libspace.o(.bss)[m
[32m+[m[32m    __temporary_stack_top$libspace           0x200000fc   Data           0  libspace.o(.bss)[m
 [m
 [m
 [m
[36m@@ -1496,206 +1764,249 @@[m [mMemory Map of the image[m
 [m
   Image Entry point : 0x080000ed[m
 [m
[31m-  Load Region LR_IROM1 (Base: 0x08000000, Size: 0x0000324c, Max: 0x00010000, ABSOLUTE)[m
[32m+[m[32m  Load Region LR_IROM1 (Base: 0x08000000, Size: 0x000038b8, Max: 0x00010000, ABSOLUTE)[m
 [m
[31m-    Execution Region ER_IROM1 (Exec base: 0x08000000, Load base: 0x08000000, Size: 0x000031e4, Max: 0x00010000, ABSOLUTE)[m
[32m+[m[32m    Execution Region ER_IROM1 (Exec base: 0x08000000, Load base: 0x08000000, Size: 0x00003880, Max: 0x00010000, ABSOLUTE)[m
 [m
     Exec Addr    Load Addr    Size         Type   Attr      Idx    E Section Name        Object[m
 [m
     0x08000000   0x08000000   0x000000ec   Data   RO            3    RESET               startup_stm32f10x_md.o[m
[31m-    0x080000ec   0x080000ec   0x00000008   Code   RO         3777  * !!!main             c_w.l(__main.o)[m
[31m-    0x080000f4   0x080000f4   0x00000034   Code   RO         4005    !!!scatter          c_w.l(__scatter.o)[m
[31m-    0x08000128   0x08000128   0x0000001a   Code   RO         4007    !!handler_copy      c_w.l(__scatter_copy.o)[m
[32m+[m[32m    0x080000ec   0x080000ec   0x00000008   Code   RO         3922  * !!!main             c_w.l(__main.o)[m
[32m+[m[32m    0x080000f4   0x080000f4   0x00000034   Code   RO         4235    !!!scatter          c_w.l(__scatter.o)[m
[32m+[m[32m    0x08000128   0x08000128   0x0000001a   Code   RO         4237    !!handler_copy      c_w.l(__scatter_copy.o)[m
     0x08000142   0x08000142   0x00000002   PAD[m
[31m-    0x08000144   0x08000144   0x0000001c   Code   RO         4009    !!handler_zi        c_w.l(__scatter_zi.o)[m
[31m-    0x08000160   0x08000160   0x00000000   Code   RO         3770    .ARM.Collect$$_printf_percent$$00000000  c_w.l(_printf_percent.o)[m
[31m-    0x08000160   0x08000160   0x00000006   Code   RO         3769    .ARM.Collect$$_printf_percent$$00000003  c_w.l(_printf_f.o)[m
[31m-    0x08000166   0x08000166   0x00000004   Code   RO         3800    .ARM.Collect$$_printf_percent$$00000017  c_w.l(_printf_percent_end.o)[m
[31m-    0x0800016a   0x0800016a   0x00000002   Code   RO         3878    .ARM.Collect$$libinit$$00000000  c_w.l(libinit.o)[m
[31m-    0x0800016c   0x0800016c   0x00000000   Code   RO         3880    .ARM.Collect$$libinit$$00000002  c_w.l(libinit2.o)[m
[31m-    0x0800016c   0x0800016c   0x00000000   Code   RO         3882    .ARM.Collect$$libinit$$00000004  c_w.l(libinit2.o)[m
[31m-    0x0800016c   0x0800016c   0x00000000   Code   RO         3885    .ARM.Collect$$libinit$$0000000A  c_w.l(libinit2.o)[m
[31m-    0x0800016c   0x0800016c   0x00000000   Code   RO         3887    .ARM.Collect$$libinit$$0000000C  c_w.l(libinit2.o)[m
[31m-    0x0800016c   0x0800016c   0x00000000   Code   RO         3889    .ARM.Collect$$libinit$$0000000E  c_w.l(libinit2.o)[m
[31m-    0x0800016c   0x0800016c   0x00000006   Code   RO         3890    .ARM.Collect$$libinit$$0000000F  c_w.l(libinit2.o)[m
[31m-    0x08000172   0x08000172   0x00000000   Code   RO         3892    .ARM.Collect$$libinit$$00000011  c_w.l(libinit2.o)[m
[31m-    0x08000172   0x08000172   0x00000000   Code   RO         3894    .ARM.Collect$$libinit$$00000013  c_w.l(libinit2.o)[m
[31m-    0x08000172   0x08000172   0x00000000   Code   RO         3896    .ARM.Collect$$libinit$$00000015  c_w.l(libinit2.o)[m
[31m-    0x08000172   0x08000172   0x0000000a   Code   RO         3897    .ARM.Collect$$libinit$$00000016  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3898    .ARM.Collect$$libinit$$00000017  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3900    .ARM.Collect$$libinit$$00000019  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3902    .ARM.Collect$$libinit$$0000001B  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3904    .ARM.Collect$$libinit$$0000001D  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3906    .ARM.Collect$$libinit$$0000001F  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3908    .ARM.Collect$$libinit$$00000021  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3910    .ARM.Collect$$libinit$$00000023  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3912    .ARM.Collect$$libinit$$00000025  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3916    .ARM.Collect$$libinit$$0000002C  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3918    .ARM.Collect$$libinit$$0000002E  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3920    .ARM.Collect$$libinit$$00000030  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000000   Code   RO         3922    .ARM.Collect$$libinit$$00000032  c_w.l(libinit2.o)[m
[31m-    0x0800017c   0x0800017c   0x00000002   Code   RO         3923    .ARM.Collect$$libinit$$00000033  c_w.l(libinit2.o)[m
[31m-    0x0800017e   0x0800017e   0x00000002   Code   RO         3953    .ARM.Collect$$libshutdown$$00000000  c_w.l(libshutdown.o)[m
[31m-    0x08000180   0x08000180   0x00000000   Code   RO         3964    .ARM.Collect$$libshutdown$$00000002  c_w.l(libshutdown2.o)[m
[31m-    0x08000180   0x08000180   0x00000000   Code   RO         3966    .ARM.Collect$$libshutdown$$00000004  c_w.l(libshutdown2.o)[m
[31m-    0x08000180   0x08000180   0x00000000   Code   RO         3969    .ARM.Collect$$libshutdown$$00000007  c_w.l(libshutdown2.o)[m
[31m-    0x08000180   0x08000180   0x00000000   Code   RO         3972    .ARM.Collect$$libshutdown$$0000000A  c_w.l(libshutdown2.o)[m
[31m-    0x08000180   0x08000180   0x00000000   Code   RO         3974    .ARM.Collect$$libshutdown$$0000000C  c_w.l(libshutdown2.o)[m
[31m-    0x08000180   0x08000180   0x00000000   Code   RO         3977    .ARM.Collect$$libshutdown$$0000000F  c_w.l(libshutdown2.o)[m
[31m-    0x08000180   0x08000180   0x00000002   Code   RO         3978    .ARM.Collect$$libshutdown$$00000010  c_w.l(libshutdown2.o)[m
[31m-    0x08000182   0x08000182   0x00000000   Code   RO         3793    .ARM.Collect$$rtentry$$00000000  c_w.l(__rtentry.o)[m
[31m-    0x08000182   0x08000182   0x00000000   Code   RO         3809    .ARM.Collect$$rtentry$$00000002  c_w.l(__rtentry2.o)[m
[31m-    0x08000182   0x08000182   0x00000006   Code   RO         3821    .ARM.Collect$$rtentry$$00000004  c_w.l(__rtentry4.o)[m
[31m-    0x08000188   0x08000188   0x00000000   Code   RO         3811    .ARM.Collect$$rtentry$$00000009  c_w.l(__rtentry2.o)[m
[31m-    0x08000188   0x08000188   0x00000004   Code   RO         3812    .ARM.Collect$$rtentry$$0000000A  c_w.l(__rtentry2.o)[m
[31m-    0x0800018c   0x0800018c   0x00000000   Code   RO         3814    .ARM.Collect$$rtentry$$0000000C  c_w.l(__rtentry2.o)[m
[31m-    0x0800018c   0x0800018c   0x00000008   Code   RO         3815    .ARM.Collect$$rtentry$$0000000D  c_w.l(__rtentry2.o)[m
[31m-    0x08000194   0x08000194   0x00000002   Code   RO         3924    .ARM.Collect$$rtexit$$00000000  c_w.l(rtexit.o)[m
[31m-    0x08000196   0x08000196   0x00000000   Code   RO         3931    .ARM.Collect$$rtexit$$00000002  c_w.l(rtexit2.o)[m
[31m-    0x08000196   0x08000196   0x00000004   Code   RO         3932    .ARM.Collect$$rtexit$$00000003  c_w.l(rtexit2.o)[m
[31m-    0x0800019a   0x0800019a   0x00000006   Code   RO         3933    .ARM.Collect$$rtexit$$00000004  c_w.l(rtexit2.o)[m
[31m-    0x080001a0   0x080001a0   0x00000040   Code   RO            4    .text               startup_stm32f10x_md.o[m
[31m-    0x080001e0   0x080001e0   0x00000028   Code   RO         3745    .text               c_w.l(noretval__2sprintf.o)[m
[31m-    0x08000208   0x08000208   0x0000010e   Code   RO         3757    .text               c_w.l(__printf_wp.o)[m
[31m-    0x08000316   0x08000316   0x00000064   Code   RO         3771    .text               c_w.l(rt_memcpy_w.o)[m
[31m-    0x0800037a   0x0800037a   0x00000002   PAD[m
[31m-    0x0800037c   0x0800037c   0x00000080   Code   RO         3773    .text               c_w.l(strcmpv7m.o)[m
[31m-    0x080003fc   0x080003fc   0x00000006   Code   RO         3775    .text               c_w.l(heapauxi.o)[m
[31m-    0x08000402   0x08000402   0x0000041e   Code   RO         3794    .text               c_w.l(_printf_fp_dec.o)[m
[31m-    0x08000820   0x08000820   0x00000030   Code   RO         3796    .text               c_w.l(_printf_char_common.o)[m
[31m-    0x08000850   0x08000850   0x0000000a   Code   RO         3798    .text               c_w.l(_sputc.o)[m
[31m-    0x0800085a   0x0800085a   0x00000002   PAD[m
[31m-    0x0800085c   0x0800085c   0x00000008   Code   RO         3826    .text               c_w.l(rt_locale_intlibspace.o)[m
[31m-    0x08000864   0x08000864   0x0000008a   Code   RO         3828    .text               c_w.l(lludiv10.o)[m
[31m-    0x080008ee   0x080008ee   0x00000002   PAD[m
[31m-    0x080008f0   0x080008f0   0x00000080   Code   RO         3830    .text               c_w.l(_printf_fp_infnan.o)[m
[31m-    0x08000970   0x08000970   0x000000e4   Code   RO         3834    .text               c_w.l(bigflt0.o)[m
[31m-    0x08000a54   0x08000a54   0x00000008   Code   RO         3866    .text               c_w.l(libspace.o)[m
[31m-    0x08000a5c   0x08000a5c   0x0000004a   Code   RO         3869    .text               c_w.l(sys_stackheap_outer.o)[m
[31m-    0x08000aa6   0x08000aa6   0x00000012   Code   RO         3871    .text               c_w.l(exit.o)[m
[31m-    0x08000ab8   0x08000ab8   0x0000000c   Code   RO         3943    .text               c_w.l(sys_exit.o)[m
[31m-    0x08000ac4   0x08000ac4   0x00000002   Code   RO         3954    .text               c_w.l(use_no_semi.o)[m
[31m-    0x08000ac6   0x08000ac6   0x00000000   Code   RO         3956    .text               c_w.l(indicate_semi.o)[m
[31m-    0x08000ac6   0x08000ac6   0x0000003e   Code   RO         3837    CL$$btod_d2e        c_w.l(btod.o)[m
[31m-    0x08000b04   0x08000b04   0x00000046   Code   RO         3839    CL$$btod_d2e_denorm_low  c_w.l(btod.o)[m
[31m-    0x08000b4a   0x08000b4a   0x00000060   Code   RO         3838    CL$$btod_d2e_norm_op1  c_w.l(btod.o)[m
[31m-    0x08000baa   0x08000baa   0x00000338   Code   RO         3847    CL$$btod_div_common  c_w.l(btod.o)[m
[31m-    0x08000ee2   0x08000ee2   0x000000dc   Code   RO         3844    CL$$btod_e2e        c_w.l(btod.o)[m
[31m-    0x08000fbe   0x08000fbe   0x0000002a   Code   RO         3841    CL$$btod_ediv       c_w.l(btod.o)[m
[31m-    0x08000fe8   0x08000fe8   0x0000002a   Code   RO         3840    CL$$btod_emul       c_w.l(btod.o)[m
[31m-    0x08001012   0x08001012   0x00000244   Code   RO         3846    CL$$btod_mult_common  c_w.l(btod.o)[m
[31m-    0x08001256   0x08001256   0x00000004   Code   RO         3678    i.BusFault_Handler  stm32f10x_it.o[m
[31m-    0x0800125a   0x0800125a   0x00000002   Code   RO         3679    i.DebugMon_Handler  stm32f10x_it.o[m
[31m-    0x0800125c   0x0800125c   0x0000003c   Code   RO         3609    i.EXTI0_IRQHandler  encoder.o[m
[31m-    0x08001298   0x08001298   0x0000003c   Code   RO         3610    i.EXTI1_IRQHandler  encoder.o[m
[31m-    0x080012d4   0x080012d4   0x0000000c   Code   RO         2316    i.EXTI_ClearITPendingBit  stm32f10x_exti.o[m
[31m-    0x080012e0   0x080012e0   0x00000028   Code   RO         2320    i.EXTI_GetITStatus  stm32f10x_exti.o[m
[31m-    0x08001308   0x08001308   0x00000094   Code   RO         2321    i.EXTI_Init         stm32f10x_exti.o[m
[31m-    0x0800139c   0x0800139c   0x00000014   Code   RO         3611    i.Encoder_Get       encoder.o[m
[31m-    0x080013b0   0x080013b0   0x0000009c   Code   RO         3612    i.Encoder_Init      encoder.o[m
[31m-    0x0800144c   0x0800144c   0x00000060   Code   RO         3315    i.FloodLED_LeftTurn  led.o[m
[31m-    0x080014ac   0x080014ac   0x00000060   Code   RO         3316    i.FloodLED_RightTurn  led.o[m
[31m-    0x0800150c   0x0800150c   0x00000040   Code   RO         1901    i.GPIO_EXTILineConfig  stm32f10x_gpio.o[m
[31m-    0x0800154c   0x0800154c   0x00000116   Code   RO         1904    i.GPIO_Init         stm32f10x_gpio.o[m
[31m-    0x08001662   0x08001662   0x00000012   Code   RO         1908    i.GPIO_ReadInputDataBit  stm32f10x_gpio.o[m
[31m-    0x08001674   0x08001674   0x00000004   Code   RO         1912    i.GPIO_SetBits      stm32f10x_gpio.o[m
[31m-    0x08001678   0x08001678   0x0000000a   Code   RO         1915    i.GPIO_WriteBit     stm32f10x_gpio.o[m
[31m-    0x08001682   0x08001682   0x00000004   Code   RO         3680    i.HardFault_Handler  stm32f10x_it.o[m
[31m-    0x08001686   0x08001686   0x00000002   PAD[m
[31m-    0x08001688   0x08001688   0x00000024   Code   RO         3361    i.Key_Check         key.o[m
[31m-    0x080016ac   0x080016ac   0x0000003e   Code   RO         3362    i.Key_GetNum        key.o[m
[31m-    0x080016ea   0x080016ea   0x00000002   PAD[m
[31m-    0x080016ec   0x080016ec   0x00000058   Code   RO         3363    i.Key_GetState      key.o[m
[31m-    0x08001744   0x08001744   0x00000054   Code   RO         3364    i.Key_Init          key.o[m
[31m-    0x08001798   0x08001798   0x000002b0   Code   RO         3365    i.Key_Tick          key.o[m
[31m-    0x08001a48   0x08001a48   0x0000000c   Code   RO         3317    i.LED_DirSet        led.o[m
[31m-    0x08001a54   0x08001a54   0x00000038   Code   RO         3318    i.LED_Init          led.o[m
[31m-    0x08001a8c   0x08001a8c   0x00000014   Code   RO         3319    i.LED_SpeedSet      led.o[m
[31m-    0x08001aa0   0x08001aa0   0x0000003c   Code   RO         3320    i.LED_Tick          led.o[m
[31m-    0x08001adc   0x08001adc   0x00000004   Code   RO         3681    i.MemManage_Handler  stm32f10x_it.o[m
[31m-    0x08001ae0   0x08001ae0   0x0000011c   Code   RO         3521    i.Menu_Init         menu.o[m
[31m-    0x08001bfc   0x08001bfc   0x00000014   Code   RO         3522    i.Menu_LED_Direction  menu.o[m
[31m-    0x08001c10   0x08001c10   0x00000014   Code   RO         3523    i.Menu_LED_Speed    menu.o[m
[31m-    0x08001c24   0x08001c24   0x0000018c   Code   RO         3525    i.Menu_Option       menu.o[m
[31m-    0x08001db0   0x08001db0   0x00000124   Code   RO         3526    i.Menu_Show         menu.o[m
[31m-    0x08001ed4   0x08001ed4   0x00000002   Code   RO         3682    i.NMI_Handler       stm32f10x_it.o[m
[31m-    0x08001ed6   0x08001ed6   0x00000002   PAD[m
[31m-    0x08001ed8   0x08001ed8   0x00000070   Code   RO         3157    i.NVIC_Init         misc.o[m
[31m-    0x08001f48   0x08001f48   0x00000014   Code   RO         3158    i.NVIC_PriorityGroupConfig  misc.o[m
[31m-    0x08001f5c   0x08001f5c   0x0000001c   Code   RO         3527    i.Numlen            menu.o[m
[31m-    0x08001f78   0x08001f78   0x0000002a   Code   RO         3403    i.OLED_Clear        oled.o[m
[31m-    0x08001fa2   0x08001fa2   0x00000002   PAD[m
[31m-    0x08001fa4   0x08001fa4   0x00000050   Code   RO         3404    i.OLED_I2C_Init     oled.o[m
[31m-    0x08001ff4   0x08001ff4   0x0000005c   Code   RO         3405    i.OLED_I2C_SendByte  oled.o[m
[31m-    0x08002050   0x08002050   0x00000034   Code   RO         3406    i.OLED_I2C_Start    oled.o[m
[31m-    0x08002084   0x08002084   0x00000028   Code   RO         3407    i.OLED_I2C_Stop     oled.o[m
[31m-    0x080020ac   0x080020ac   0x000000ae   Code   RO         3408    i.OLED_Init         oled.o[m
[31m-    0x0800215a   0x0800215a   0x00000014   Code   RO         3409    i.OLED_Pow          oled.o[m
[31m-    0x0800216e   0x0800216e   0x00000022   Code   RO         3410    i.OLED_SetCursor    oled.o[m
[31m-    0x08002190   0x08002190   0x00000074   Code   RO         3412    i.OLED_ShowChar     oled.o[m
[31m-    0x08002204   0x08002204   0x0000003c   Code   RO         3413    i.OLED_ShowFloatNum  oled.o[m
[31m-    0x08002240   0x08002240   0x00000044   Code   RO         3415    i.OLED_ShowNum      oled.o[m
[31m-    0x08002284   0x08002284   0x00000028   Code   RO         3417    i.OLED_ShowString   oled.o[m
[31m-    0x080022ac   0x080022ac   0x00000020   Code   RO         3418    i.OLED_WriteCommand  oled.o[m
[31m-    0x080022cc   0x080022cc   0x00000020   Code   RO         3419    i.OLED_WriteData    oled.o[m
[31m-    0x080022ec   0x080022ec   0x00000002   Code   RO         3683    i.PendSV_Handler    stm32f10x_it.o[m
[31m-    0x080022ee   0x080022ee   0x00000002   PAD[m
[31m-    0x080022f0   0x080022f0   0x00000020   Code   RO         1382    i.RCC_APB1PeriphClockCmd  stm32f10x_rcc.o[m
[31m-    0x08002310   0x08002310   0x00000020   Code   RO         1384    i.RCC_APB2PeriphClockCmd  stm32f10x_rcc.o[m
[31m-    0x08002330   0x08002330   0x00000002   Code   RO         3684    i.SVC_Handler       stm32f10x_it.o[m
[31m-    0x08002332   0x08002332   0x00000008   Code   RO           24    i.SetSysClock       system_stm32f10x.o[m
[31m-    0x0800233a   0x0800233a   0x00000002   PAD[m
[31m-    0x0800233c   0x0800233c   0x000000e0   Code   RO           25    i.SetSysClockTo72   system_stm32f10x.o[m
[31m-    0x0800241c   0x0800241c   0x00000002   Code   RO         3685    i.SysTick_Handler   stm32f10x_it.o[m
[31m-    0x0800241e   0x0800241e   0x00000002   PAD[m
[31m-    0x08002420   0x08002420   0x00000060   Code   RO           27    i.SystemInit        system_stm32f10x.o[m
[31m-    0x08002480   0x08002480   0x00000020   Code   RO         3642    i.TIM2_IRQHandler   main.o[m
[31m-    0x080024a0   0x080024a0   0x00000006   Code   RO          410    i.TIM_ClearFlag     stm32f10x_tim.o[m
[31m-    0x080024a6   0x080024a6   0x00000006   Code   RO          411    i.TIM_ClearITPendingBit  stm32f10x_tim.o[m
[31m-    0x080024ac   0x080024ac   0x00000018   Code   RO          416    i.TIM_Cmd           stm32f10x_tim.o[m
[31m-    0x080024c4   0x080024c4   0x00000022   Code   RO          437    i.TIM_GetITStatus   stm32f10x_tim.o[m
[31m-    0x080024e6   0x080024e6   0x00000012   Code   RO          441    i.TIM_ITConfig      stm32f10x_tim.o[m
[31m-    0x080024f8   0x080024f8   0x0000000c   Code   RO          443    i.TIM_InternalClockConfig  stm32f10x_tim.o[m
[31m-    0x08002504   0x08002504   0x000000a4   Code   RO          487    i.TIM_TimeBaseInit  stm32f10x_tim.o[m
[31m-    0x080025a8   0x080025a8   0x0000007c   Code   RO         3220    i.Timer_Init        timer.o[m
[31m-    0x08002624   0x08002624   0x00000004   Code   RO         3686    i.UsageFault_Handler  stm32f10x_it.o[m
[31m-    0x08002628   0x08002628   0x00000028   Code   RO         3864    i.__ARM_fpclassify  m_ws.l(fpclassify.o)[m
[31m-    0x08002650   0x08002650   0x0000000e   Code   RO         3759    i._is_digit         c_w.l(__printf_wp.o)[m
[31m-    0x0800265e   0x0800265e   0x00000042   Code   RO         3643    i.main              main.o[m
[31m-    0x080026a0   0x080026a0   0x0000002c   Code   RO         3860    locale$$code        c_w.l(lc_numeric_c.o)[m
[31m-    0x080026cc   0x080026cc   0x0000000c   Code   RO         3801    x$fpl$dretinf       fz_ws.l(dretinf.o)[m
[31m-    0x080026d8   0x080026d8   0x00000056   Code   RO         3779    x$fpl$f2d           fz_ws.l(f2d.o)[m
[31m-    0x0800272e   0x0800272e   0x00000002   PAD[m
[31m-    0x08002730   0x08002730   0x00000184   Code   RO         3782    x$fpl$fdiv          fz_ws.l(fdiv.o)[m
[31m-    0x080028b4   0x080028b4   0x00000030   Code   RO         3786    x$fpl$fflt          fz_ws.l(fflt_clz.o)[m
[31m-    0x080028e4   0x080028e4   0x0000008c   Code   RO         3803    x$fpl$fnaninf       fz_ws.l(fnaninf.o)[m
[31m-    0x08002970   0x08002970   0x0000000a   Code   RO         3805    x$fpl$fretinf       fz_ws.l(fretinf.o)[m
[31m-    0x0800297a   0x0800297a   0x00000004   Code   RO         3791    x$fpl$printf1       fz_ws.l(printf1.o)[m
[31m-    0x0800297e   0x0800297e   0x00000000   Code   RO         3807    x$fpl$usenofp       fz_ws.l(usenofp.o)[m
[31m-    0x0800297e   0x0800297e   0x00000012   Data   RO         3321    .constdata          led.o[m
[31m-    0x08002990   0x08002990   0x000005f0   Data   RO         3420    .constdata          oled.o[m
[31m-    0x08002f80   0x08002f80   0x00000140   Data   RO         3529    .constdata          menu.o[m
[31m-    0x080030c0   0x080030c0   0x00000094   Data   RO         3835    .constdata          c_w.l(bigflt0.o)[m
[31m-    0x08003154   0x08003154   0x00000052   Data   RO         3530    .conststring        menu.o[m
[31m-    0x080031a6   0x080031a6   0x00000002   PAD[m
[31m-    0x080031a8   0x080031a8   0x00000020   Data   RO         4003    Region$$Table       anon$$obj.o[m
[31m-    0x080031c8   0x080031c8   0x0000001c   Data   RO         3859    locale$$data        c_w.l(lc_numeric_c.o)[m
[32m+[m[32m    0x08000144   0x08000144   0x0000001c   Code   RO         4239    !!handler_zi        c_w.l(__scatter_zi.o)[m
[32m+[m[32m    0x08000160   0x08000160   0x00000000   Code   RO         3915    .ARM.Collect$$_printf_percent$$00000000  c_w.l(_printf_percent.o)[m
[32m+[m[32m    0x08000160   0x08000160   0x00000006   Code   RO         3989    .ARM.Collect$$_printf_percent$$00000001  c_w.l(_printf_n.o)[m
[32m+[m[32m    0x08000166   0x08000166   0x00000006   Code   RO         3991    .ARM.Collect$$_printf_percent$$00000002  c_w.l(_printf_p.o)[m
[32m+[m[32m    0x0800016c   0x0800016c   0x00000006   Code   RO         3914    .ARM.Collect$$_printf_percent$$00000003  c_w.l(_printf_f.o)[m
[32m+[m[32m    0x08000172   0x08000172   0x00000006   Code   RO         3996    .ARM.Collect$$_printf_percent$$00000004  c_w.l(_printf_e.o)[m
[32m+[m[32m    0x08000178   0x08000178   0x00000006   Code   RO         3997    .ARM.Collect$$_printf_percent$$00000005  c_w.l(_printf_g.o)[m
[32m+[m[32m    0x0800017e   0x0800017e   0x00000006   Code   RO         3998    .ARM.Collect$$_printf_percent$$00000006  c_w.l(_printf_a.o)[m
[32m+[m[32m    0x08000184   0x08000184   0x0000000a   Code   RO         4003    .ARM.Collect$$_printf_percent$$00000007  c_w.l(_printf_ll.o)[m
[32m+[m[32m    0x0800018e   0x0800018e   0x00000006   Code   RO         3993    .ARM.Collect$$_printf_percent$$00000008  c_w.l(_printf_i.o)[m
[32m+[m[32m    0x08000194   0x08000194   0x00000006   Code   RO         3994    .ARM.Collect$$_printf_percent$$00000009  c_w.l(_printf_d.o)[m
[32m+[m[32m    0x0800019a   0x0800019a   0x00000006   Code   RO         3995    .ARM.Collect$$_printf_percent$$0000000A  c_w.l(_printf_u.o)[m
[32m+[m[32m    0x080001a0   0x080001a0   0x00000006   Code   RO         3992    .ARM.Collect$$_printf_percent$$0000000B  c_w.l(_printf_o.o)[m
[32m+[m[32m    0x080001a6   0x080001a6   0x00000006   Code   RO         3990    .ARM.Collect$$_printf_percent$$0000000C  c_w.l(_printf_x.o)[m
[32m+[m[32m    0x080001ac   0x080001ac   0x00000006   Code   RO         4000    .ARM.Collect$$_printf_percent$$0000000D  c_w.l(_printf_lli.o)[m
[32m+[m[32m    0x080001b2   0x080001b2   0x00000006   Code   RO         4001    .ARM.Collect$$_printf_percent$$0000000E  c_w.l(_printf_lld.o)[m
[32m+[m[32m    0x080001b8   0x080001b8   0x00000006   Code   RO         4002    .ARM.Collect$$_printf_percent$$0000000F  c_w.l(_printf_llu.o)[m
[32m+[m[32m    0x080001be   0x080001be   0x00000006   Code   RO         4007    .ARM.Collect$$_printf_percent$$00000010  c_w.l(_printf_llo.o)[m
[32m+[m[32m    0x080001c4   0x080001c4   0x00000006   Code   RO         4008    .ARM.Collect$$_printf_percent$$00000011  c_w.l(_printf_llx.o)[m
[32m+[m[32m    0x080001ca   0x080001ca   0x0000000a   Code   RO         4004    .ARM.Collect$$_printf_percent$$00000012  c_w.l(_printf_l.o)[m
[32m+[m[32m    0x080001d4   0x080001d4   0x00000006   Code   RO         3987    .ARM.Collect$$_printf_percent$$00000013  c_w.l(_printf_c.o)[m
[32m+[m[32m    0x080001da   0x080001da   0x00000006   Code   RO         3988    .ARM.Collect$$_printf_percent$$00000014  c_w.l(_printf_s.o)[m
[32m+[m[32m    0x080001e0   0x080001e0   0x00000006   Code   RO         4005    .ARM.Collect$$_printf_percent$$00000015  c_w.l(_printf_lc.o)[m
[32m+[m[32m    0x080001e6   0x080001e6   0x00000006   Code   RO         4006    .ARM.Collect$$_printf_percent$$00000016  c_w.l(_printf_ls.o)[m
[32m+[m[32m    0x080001ec   0x080001ec   0x00000004   Code   RO         3999    .ARM.Collect$$_printf_percent$$00000017  c_w.l(_printf_percent_end.o)[m
[32m+[m[32m    0x080001f0   0x080001f0   0x00000002   Code   RO         4105    .ARM.Collect$$libinit$$00000000  c_w.l(libinit.o)[m
[32m+[m[32m    0x080001f2   0x080001f2   0x00000000   Code   RO         4107    .ARM.Collect$$libinit$$00000002  c_w.l(libinit2.o)[m
[32m+[m[32m    0x080001f2   0x080001f2   0x00000000   Code   RO         4109    .ARM.Collect$$libinit$$00000004  c_w.l(libinit2.o)[m
[32m+[m[32m    0x080001f2   0x080001f2   0x00000000   Code   RO         4112    .ARM.Collect$$libinit$$0000000A  c_w.l(libinit2.o)[m
[32m+[m[32m    0x080001f2   0x080001f2   0x00000000   Code   RO         4114    .ARM.Collect$$libinit$$0000000C  c_w.l(libinit2.o)[m
[32m+[m[32m    0x080001f2   0x080001f2   0x00000000   Code   RO         4116    .ARM.Collect$$libinit$$0000000E  c_w.l(libinit2.o)[m
[32m+[m[32m    0x080001f2   0x080001f2   0x00000006   Code   RO         4117    .ARM.Collect$$libinit$$0000000F  c_w.l(libinit2.o)[m
[32m+[m[32m    0x080001f8   0x080001f8   0x00000000   Code   RO         4119    .ARM.Collect$$libinit$$00000011  c_w.l(libinit2.o)[m
[32m+[m[32m    0x080001f8   0x080001f8   0x0000000c   Code   RO         4120    .ARM.Collect$$libinit$$00000012  c_w.l(libinit2.o)[m
[32m+[m[32m    0x08000204   0x08000204   0x00000000   Code   RO         4121    .ARM.Collect$$libinit$$00000013  c_w.l(libinit2.o)[m
[32m+[m[32m    0x08000204   0x08000204   0x00000000   Code   RO         4123    .ARM.Collect$$libinit$$00000015  c_w.l(libinit2.o)[m
[32m+[m[32m    0x08000204   0x08000204   0x0000000a   Code   RO         4124    .ARM.Collect$$libinit$$00000016  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4125    .ARM.Collect$$libinit$$00000017  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4127    .ARM.Collect$$libinit$$00000019  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4129    .ARM.Collect$$libinit$$0000001B  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4131    .ARM.Collect$$libinit$$0000001D  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4133    .ARM.Collect$$libinit$$0000001F  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4135    .ARM.Collect$$libinit$$00000021  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4137    .ARM.Collect$$libinit$$00000023  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4139    .ARM.Collect$$libinit$$00000025  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4143    .ARM.Collect$$libinit$$0000002C  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4145    .ARM.Collect$$libinit$$0000002E  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4147    .ARM.Collect$$libinit$$00000030  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000000   Code   RO         4149    .ARM.Collect$$libinit$$00000032  c_w.l(libinit2.o)[m
[32m+[m[32m    0x0800020e   0x0800020e   0x00000002   Code   RO         4150    .ARM.Collect$$libinit$$00000033  c_w.l(libinit2.o)[m
[32m+[m[32m    0x08000210   0x08000210   0x00000002   Code   RO         4183    .ARM.Collect$$libshutdown$$00000000  c_w.l(libshutdown.o)[m
[32m+[m[32m    0x08000212   0x08000212   0x00000000   Code   RO         4194    .ARM.Collect$$libshutdown$$00000002  c_w.l(libshutdown2.o)[m
[32m+[m[32m    0x08000212   0x08000212   0x00000000   Code   RO         4196    .ARM.Collect$$libshutdown$$00000004  c_w.l(libshutdown2.o)[m
[32m+[m[32m    0x08000212   0x08000212   0x00000000   Code   RO         4199    .ARM.Collect$$libshutdown$$00000007  c_w.l(libshutdown2.o)[m
[32m+[m[32m    0x08000212   0x08000212   0x00000000   Code   RO         4202    .ARM.Collect$$libshutdown$$0000000A  c_w.l(libshutdown2.o)[m
[32m+[m[32m    0x08000212   0x08000212   0x00000000   Code   RO         4204    .ARM.Collect$$libshutdown$$0000000C  c_w.l(libshutdown2.o)[m
[32m+[m[32m    0x08000212   0x08000212   0x00000000   Code   RO         4207    .ARM.Collect$$libshutdown$$0000000F  c_w.l(libshutdown2.o)[m
[32m+[m[32m    0x08000212   0x08000212   0x00000002   Code   RO         4208    .ARM.Collect$$libshutdown$$00000010  c_w.l(libshutdown2.o)[m
[32m+[m[32m    0x08000214   0x08000214   0x00000000   Code   RO         3938    .ARM.Collect$$rtentry$$00000000  c_w.l(__rtentry.o)[m
[32m+[m[32m    0x08000214   0x08000214   0x00000000   Code   RO         4023    .ARM.Collect$$rtentry$$00000002  c_w.l(__rtentry2.o)[m
[32m+[m[32m    0x08000214   0x08000214   0x00000006   Code   RO         4035    .ARM.Collect$$rtentry$$00000004  c_w.l(__rtentry4.o)[m
[32m+[m[32m    0x0800021a   0x0800021a   0x00000000   Code   RO         4025    .ARM.Collect$$rtentry$$00000009  c_w.l(__rtentry2.o)[m
[32m+[m[32m    0x0800021a   0x0800021a   0x00000004   Code   RO         4026    .ARM.Collect$$rtentry$$0000000A  c_w.l(__rtentry2.o)[m
[32m+[m[32m    0x0800021e   0x0800021e   0x00000000   Code   RO         4028    .ARM.Collect$$rtentry$$0000000C  c_w.l(__rtentry2.o)[m
[32m+[m[32m    0x0800021e   0x0800021e   0x00000008   Code   RO         4029    .ARM.Collect$$rtentry$$0000000D  c_w.l(__rtentry2.o)[m
[32m+[m[32m    0x08000226   0x08000226   0x00000002   Code   RO         4151    .ARM.Collect$$rtexit$$00000000  c_w.l(rtexit.o)[m
[32m+[m[32m    0x08000228   0x08000228   0x00000000   Code   RO         4161    .ARM.Collect$$rtexit$$00000002  c_w.l(rtexit2.o)[m
[32m+[m[32m    0x08000228   0x08000228   0x00000004   Code   RO         4162    .ARM.Collect$$rtexit$$00000003  c_w.l(rtexit2.o)[m
[32m+[m[32m    0x0800022c   0x0800022c   0x00000006   Code   RO         4163    .ARM.Collect$$rtexit$$00000004  c_w.l(rtexit2.o)[m
[32m+[m[32m    0x08000232   0x08000232   0x00000002   PAD[m
[32m+[m[32m    0x08000234   0x08000234   0x00000040   Code   RO            4    .text               startup_stm32f10x_md.o[m
[32m+[m[32m    0x08000274   0x08000274   0x00000024   Code   RO         3886    .text               c_w.l(vsprintf.o)[m
[32m+[m[32m    0x08000298   0x08000298   0x00000188   Code   RO         3911    .text               c_w.l(__printf_flags_ss_wp.o)[m
[32m+[m[32m    0x08000420   0x08000420   0x00000080   Code   RO         3918    .text               c_w.l(strcmpv7m.o)[m
[32m+[m[32m    0x080004a0   0x080004a0   0x00000006   Code   RO         3920    .text               c_w.l(heapauxi.o)[m
[32m+[m[32m    0x080004a6   0x080004a6   0x0000004e   Code   RO         3939    .text               c_w.l(_printf_pad.o)[m
[32m+[m[32m    0x080004f4   0x080004f4   0x00000024   Code   RO         3941    .text               c_w.l(_printf_truncate.o)[m
[32m+[m[32m    0x08000518   0x08000518   0x00000052   Code   RO         3943    .text               c_w.l(_printf_str.o)[m
[32m+[m[32m    0x0800056a   0x0800056a   0x00000002   PAD[m
[32m+[m[32m    0x0800056c   0x0800056c   0x00000078   Code   RO         3945    .text               c_w.l(_printf_dec.o)[m
[32m+[m[32m    0x080005e4   0x080005e4   0x00000028   Code   RO         3947    .text               c_w.l(_printf_charcount.o)[m
[32m+[m[32m    0x0800060c   0x0800060c   0x0000041e   Code   RO         3949    .text               c_w.l(_printf_fp_dec.o)[m
[32m+[m[32m    0x08000a2a   0x08000a2a   0x00000002   PAD[m
[32m+[m[32m    0x08000a2c   0x08000a2c   0x00000030   Code   RO         3951    .text               c_w.l(_printf_char_common.o)[m
[32m+[m[32m    0x08000a5c   0x08000a5c   0x0000000a   Code   RO         3953    .text               c_w.l(_sputc.o)[m
[32m+[m[32m    0x08000a66   0x08000a66   0x00000002   PAD[m
[32m+[m[32m    0x08000a68   0x08000a68   0x000000bc   Code   RO         3955    .text               c_w.l(_printf_wctomb.o)[m
[32m+[m[32m    0x08000b24   0x08000b24   0x0000007c   Code   RO         3958    .text               c_w.l(_printf_longlong_dec.o)[m
[32m+[m[32m    0x08000ba0   0x08000ba0   0x00000070   Code   RO         3964    .text               c_w.l(_printf_oct_int_ll.o)[m
[32m+[m[32m    0x08000c10   0x08000c10   0x00000094   Code   RO         3984    .text               c_w.l(_printf_hex_int_ll_ptr.o)[m
[32m+[m[32m    0x08000ca4   0x08000ca4   0x00000008   Code   RO         4040    .text               c_w.l(rt_locale_intlibspace.o)[m
[32m+[m[32m    0x08000cac   0x08000cac   0x0000008a   Code   RO         4042    .text               c_w.l(lludiv10.o)[m
[32m+[m[32m    0x08000d36   0x08000d36   0x000000b2   Code   RO         4044    .text               c_w.l(_printf_intcommon.o)[m
[32m+[m[32m    0x08000de8   0x08000de8   0x000002fc   Code   RO         4046    .text               c_w.l(_printf_fp_hex.o)[m
[32m+[m[32m    0x080010e4   0x080010e4   0x00000080   Code   RO         4049    .text               c_w.l(_printf_fp_infnan.o)[m
[32m+[m[32m    0x08001164   0x08001164   0x0000002c   Code   RO         4053    .text               c_w.l(_printf_char.o)[m
[32m+[m[32m    0x08001190   0x08001190   0x0000002c   Code   RO         4055    .text               c_w.l(_printf_wchar.o)[m
[32m+[m[32m    0x080011bc   0x080011bc   0x000000e4   Code   RO         4057    .text               c_w.l(bigflt0.o)[m
[32m+[m[32m    0x080012a0   0x080012a0   0x00000040   Code   RO         4082    .text               c_w.l(_wcrtomb.o)[m
[32m+[m[32m    0x080012e0   0x080012e0   0x00000008   Code   RO         4091    .text               c_w.l(libspace.o)[m
[32m+[m[32m    0x080012e8   0x080012e8   0x0000004a   Code   RO         4094    .text               c_w.l(sys_stackheap_outer.o)[m
[32m+[m[32m    0x08001332   0x08001332   0x00000002   PAD[m
[32m+[m[32m    0x08001334   0x08001334   0x00000010   Code   RO         4096    .text               c_w.l(rt_ctype_table.o)[m
[32m+[m[32m    0x08001344   0x08001344   0x00000012   Code   RO         4098    .text               c_w.l(exit.o)[m
[32m+[m[32m    0x08001356   0x08001356   0x00000002   PAD[m
[32m+[m[32m    0x08001358   0x08001358   0x0000000c   Code   RO         4173    .text               c_w.l(sys_exit.o)[m
[32m+[m[32m    0x08001364   0x08001364   0x00000002   Code   RO         4184    .text               c_w.l(use_no_semi.o)[m
[32m+[m[32m    0x08001366   0x08001366   0x00000000   Code   RO         4186    .text               c_w.l(indicate_semi.o)[m
[32m+[m[32m    0x08001366   0x08001366   0x0000003e   Code   RO         4060    CL$$btod_d2e        c_w.l(btod.o)[m
[32m+[m[32m    0x080013a4   0x080013a4   0x00000046   Code   RO         4062    CL$$btod_d2e_denorm_low  c_w.l(btod.o)[m
[32m+[m[32m    0x080013ea   0x080013ea   0x00000060   Code   RO         4061    CL$$btod_d2e_norm_op1  c_w.l(btod.o)[m
[32m+[m[32m    0x0800144a   0x0800144a   0x00000338   Code   RO         4070    CL$$btod_div_common  c_w.l(btod.o)[m
[32m+[m[32m    0x08001782   0x08001782   0x000000dc   Code   RO         4067    CL$$btod_e2e        c_w.l(btod.o)[m
[32m+[m[32m    0x0800185e   0x0800185e   0x0000002a   Code   RO         4064    CL$$btod_ediv       c_w.l(btod.o)[m
[32m+[m[32m    0x08001888   0x08001888   0x0000002a   Code   RO         4063    CL$$btod_emul       c_w.l(btod.o)[m
[32m+[m[32m    0x080018b2   0x080018b2   0x00000244   Code   RO         4069    CL$$btod_mult_common  c_w.l(btod.o)[m
[32m+[m[32m    0x08001af6   0x08001af6   0x00000004   Code   RO         3821    i.BusFault_Handler  stm32f10x_it.o[m
[32m+[m[32m    0x08001afa   0x08001afa   0x00000002   Code   RO         3822    i.DebugMon_Handler  stm32f10x_it.o[m
[32m+[m[32m    0x08001afc   0x08001afc   0x0000001c   Code   RO         3609    i.Encoder_Get       encoder.o[m
[32m+[m[32m    0x08001b18   0x08001b18   0x000000a0   Code   RO         3610    i.Encoder_Init      encoder.o[m
[32m+[m[32m    0x08001bb8   0x08001bb8   0x00000116   Code   RO         1904    i.GPIO_Init         stm32f10x_gpio.o[m
[32m+[m[32m    0x08001cce   0x08001cce   0x00000012   Code   RO         1908    i.GPIO_ReadInputDataBit  stm32f10x_gpio.o[m
[32m+[m[32m    0x08001ce0   0x08001ce0   0x0000000a   Code   RO         1915    i.GPIO_WriteBit     stm32f10x_gpio.o[m
[32m+[m[32m    0x08001cea   0x08001cea   0x00000004   Code   RO         3823    i.HardFault_Handler  stm32f10x_it.o[m
[32m+[m[32m    0x08001cee   0x08001cee   0x00000002   PAD[m
[32m+[m[32m    0x08001cf0   0x08001cf0   0x00000058   Code   RO         3363    i.Key_GetState      key.o[m
[32m+[m[32m    0x08001d48   0x08001d48   0x00000054   Code   RO         3364    i.Key_Init          key.o[m
[32m+[m[32m    0x08001d9c   0x08001d9c   0x000002b0   Code   RO         3365    i.Key_Tick          key.o[m
[32m+[m[32m    0x0800204c   0x0800204c   0x00000004   Code   RO         3824    i.MemManage_Handler  stm32f10x_it.o[m
[32m+[m[32m    0x08002050   0x08002050   0x00000002   Code   RO         3825    i.NMI_Handler       stm32f10x_it.o[m
[32m+[m[32m    0x08002052   0x08002052   0x00000002   PAD[m
[32m+[m[32m    0x08002054   0x08002054   0x00000070   Code   RO         3157    i.NVIC_Init         misc.o[m
[32m+[m[32m    0x080020c4   0x080020c4   0x00000014   Code   RO         3158    i.NVIC_PriorityGroupConfig  misc.o[m
[32m+[m[32m    0x080020d8   0x080020d8   0x0000002a   Code   RO         3403    i.OLED_Clear        oled.o[m
[32m+[m[32m    0x08002102   0x08002102   0x00000002   PAD[m
[32m+[m[32m    0x08002104   0x08002104   0x00000050   Code   RO         3404    i.OLED_I2C_Init     oled.o[m
[32m+[m[32m    0x08002154   0x08002154   0x0000005c   Code   RO         3405    i.OLED_I2C_SendByte  oled.o[m
[32m+[m[32m    0x080021b0   0x080021b0   0x00000034   Code   RO         3406    i.OLED_I2C_Start    oled.o[m
[32m+[m[32m    0x080021e4   0x080021e4   0x00000028   Code   RO         3407    i.OLED_I2C_Stop     oled.o[m
[32m+[m[32m    0x0800220c   0x0800220c   0x000000ae   Code   RO         3408    i.OLED_Init         oled.o[m
[32m+[m[32m    0x080022ba   0x080022ba   0x00000014   Code   RO         3409    i.OLED_Pow          oled.o[m
[32m+[m[32m    0x080022ce   0x080022ce   0x00000022   Code   RO         3410    i.OLED_SetCursor    oled.o[m
[32m+[m[32m    0x080022f0   0x080022f0   0x00000074   Code   RO         3412    i.OLED_ShowChar     oled.o[m
[32m+[m[32m    0x08002364   0x08002364   0x00000066   Code   RO         3416    i.OLED_ShowSignedNum  oled.o[m
[32m+[m[32m    0x080023ca   0x080023ca   0x00000020   Code   RO         3418    i.OLED_WriteCommand  oled.o[m
[32m+[m[32m    0x080023ea   0x080023ea   0x00000020   Code   RO         3419    i.OLED_WriteData    oled.o[m
[32m+[m[32m    0x0800240a   0x0800240a   0x00000002   Code   RO         3826    i.PendSV_Handler    stm32f10x_it.o[m
[32m+[m[32m    0x0800240c   0x0800240c   0x00000020   Code   RO         1382    i.RCC_APB1PeriphClockCmd  stm32f10x_rcc.o[m
[32m+[m[32m    0x0800242c   0x0800242c   0x00000020   Code   RO         1384    i.RCC_APB2PeriphClockCmd  stm32f10x_rcc.o[m
[32m+[m[32m    0x0800244c   0x0800244c   0x000000d4   Code   RO         1392    i.RCC_GetClocksFreq  stm32f10x_rcc.o[m
[32m+[m[32m    0x08002520   0x08002520   0x00000002   Code   RO         3827    i.SVC_Handler       stm32f10x_it.o[m
[32m+[m[32m    0x08002522   0x08002522   0x00000002   PAD[m
[32m+[m[32m    0x08002524   0x08002524   0x000000b8   Code   RO         3710    i.Serial_Init       serial.o[m
[32m+[m[32m    0x080025dc   0x080025dc   0x00000024   Code   RO         3712    i.Serial_Printf     serial.o[m
[32m+[m[32m    0x08002600   0x08002600   0x00000020   Code   RO         3714    i.Serial_SendByte   serial.o[m
[32m+[m[32m    0x08002620   0x08002620   0x0000001a   Code   RO         3716    i.Serial_SendString  serial.o[m
[32m+[m[32m    0x0800263a   0x0800263a   0x00000008   Code   RO           24    i.SetSysClock       system_stm32f10x.o[m
[32m+[m[32m    0x08002642   0x08002642   0x00000002   PAD[m
[32m+[m[32m    0x08002644   0x08002644   0x000000e0   Code   RO           25    i.SetSysClockTo72   system_stm32f10x.o[m
[32m+[m[32m    0x08002724   0x08002724   0x00000002   Code   RO         3828    i.SysTick_Handler   stm32f10x_it.o[m
[32m+[m[32m    0x08002726   0x08002726   0x00000002   PAD[m
[32m+[m[32m    0x08002728   0x08002728   0x00000060   Code   RO           27    i.SystemInit        system_stm32f10x.o[m
[32m+[m[32m    0x08002788   0x08002788   0x00000080   Code   RO          400    i.TI1_Config        stm32f10x_tim.o[m
[32m+[m[32m    0x08002808   0x08002808   0x00000098   Code   RO          401    i.TI2_Config        stm32f10x_tim.o[m
[32m+[m[32m    0x080028a0   0x080028a0   0x00000090   Code   RO          402    i.TI3_Config        stm32f10x_tim.o[m
[32m+[m[32m    0x08002930   0x08002930   0x00000098   Code   RO          403    i.TI4_Config        stm32f10x_tim.o[m
[32m+[m[32m    0x080029c8   0x080029c8   0x00000044   Code   RO         3781    i.TIM2_IRQHandler   main.o[m
[32m+[m[32m    0x08002a0c   0x08002a0c   0x00000006   Code   RO          410    i.TIM_ClearFlag     stm32f10x_tim.o[m
[32m+[m[32m    0x08002a12   0x08002a12   0x00000006   Code   RO          411    i.TIM_ClearITPendingBit  stm32f10x_tim.o[m
[32m+[m[32m    0x08002a18   0x08002a18   0x00000018   Code   RO          416    i.TIM_Cmd           stm32f10x_tim.o[m
[32m+[m[32m    0x08002a30   0x08002a30   0x00000042   Code   RO          425    i.TIM_EncoderInterfaceConfig  stm32f10x_tim.o[m
[32m+[m[32m    0x08002a72   0x08002a72   0x00000006   Code   RO          435    i.TIM_GetCounter    stm32f10x_tim.o[m
[32m+[m[32m    0x08002a78   0x08002a78   0x00000022   Code   RO          437    i.TIM_GetITStatus   stm32f10x_tim.o[m
[32m+[m[32m    0x08002a9a   0x08002a9a   0x00000002   PAD[m
[32m+[m[32m    0x08002a9c   0x08002a9c   0x000000ac   Code   RO          439    i.TIM_ICInit        stm32f10x_tim.o[m
[32m+[m[32m    0x08002b48   0x08002b48   0x00000012   Code   RO          440    i.TIM_ICStructInit  stm32f10x_tim.o[m
[32m+[m[32m    0x08002b5a   0x08002b5a   0x00000012   Code   RO          441    i.TIM_ITConfig      stm32f10x_tim.o[m
[32m+[m[32m    0x08002b6c   0x08002b6c   0x0000000c   Code   RO          443    i.TIM_InternalClockConfig  stm32f10x_tim.o[m
[32m+[m[32m    0x08002b78   0x08002b78   0x00000004   Code   RO          481    i.TIM_SetCounter    stm32f10x_tim.o[m
[32m+[m[32m    0x08002b7c   0x08002b7c   0x00000012   Code   RO          482    i.TIM_SetIC1Prescaler  stm32f10x_tim.o[m
[32m+[m[32m    0x08002b8e   0x08002b8e   0x0000001a   Code   RO          483    i.TIM_SetIC2Prescaler  stm32f10x_tim.o[m
[32m+[m[32m    0x08002ba8   0x08002ba8   0x00000012   Code   RO          484    i.TIM_SetIC3Prescaler  stm32f10x_tim.o[m
[32m+[m[32m    0x08002bba   0x08002bba   0x0000001a   Code   RO          485    i.TIM_SetIC4Prescaler  stm32f10x_tim.o[m
[32m+[m[32m    0x08002bd4   0x08002bd4   0x000000a4   Code   RO          487    i.TIM_TimeBaseInit  stm32f10x_tim.o[m
[32m+[m[32m    0x08002c78   0x08002c78   0x0000007c   Code   RO         3220    i.Timer_Init        timer.o[m
[32m+[m[32m    0x08002cf4   0x08002cf4   0x000000a0   Code   RO         3717    i.USART1_IRQHandler  serial.o[m
[32m+[m[32m    0x08002d94   0x08002d94   0x0000001e   Code   RO          218    i.USART_ClearITPendingBit  stm32f10x_usart.o[m
[32m+[m[32m    0x08002db2   0x08002db2   0x00000018   Code   RO          221    i.USART_Cmd         stm32f10x_usart.o[m
[32m+[m[32m    0x08002dca   0x08002dca   0x0000001a   Code   RO          224    i.USART_GetFlagStatus  stm32f10x_usart.o[m
[32m+[m[32m    0x08002de4   0x08002de4   0x00000054   Code   RO          225    i.USART_GetITStatus  stm32f10x_usart.o[m
[32m+[m[32m    0x08002e38   0x08002e38   0x0000004a   Code   RO          227    i.USART_ITConfig    stm32f10x_usart.o[m
[32m+[m[32m    0x08002e82   0x08002e82   0x00000002   PAD[m
[32m+[m[32m    0x08002e84   0x08002e84   0x000000d8   Code   RO          228    i.USART_Init        stm32f10x_usart.o[m
[32m+[m[32m    0x08002f5c   0x08002f5c   0x0000000a   Code   RO          235    i.USART_ReceiveData  stm32f10x_usart.o[m
[32m+[m[32m    0x08002f66   0x08002f66   0x00000008   Code   RO          238    i.USART_SendData    stm32f10x_usart.o[m
[32m+[m[32m    0x08002f6e   0x08002f6e   0x00000004   Code   RO         3829    i.UsageFault_Handler  stm32f10x_it.o[m
[32m+[m[32m    0x08002f72   0x08002f72   0x00000028   Code   RO         4089    i.__ARM_fpclassify  m_ws.l(fpclassify.o)[m
[32m+[m[32m    0x08002f9a   0x08002f9a   0x0000000e   Code   RO         3904    i._is_digit         c_w.l(__printf_wp.o)[m
[32m+[m[32m    0x08002fa8   0x08002fa8   0x0000003c   Code   RO         3782    i.main              main.o[m
[32m+[m[32m    0x08002fe4   0x08002fe4   0x0000002c   Code   RO         4085    locale$$code        c_w.l(lc_numeric_c.o)[m
[32m+[m[32m    0x08003010   0x08003010   0x0000002c   Code   RO         4154    locale$$code        c_w.l(lc_ctype_c.o)[m
[32m+[m[32m    0x0800303c   0x0800303c   0x00000004   Code   RO         3936    x$fpl$printf1       fz_ws.l(printf1.o)[m
[32m+[m[32m    0x08003040   0x08003040   0x00000004   Code   RO         4015    x$fpl$printf2       fz_ws.l(printf2.o)[m
[32m+[m[32m    0x08003044   0x08003044   0x00000000   Code   RO         4021    x$fpl$usenofp       fz_ws.l(usenofp.o)[m
[32m+[m[32m    0x08003044   0x08003044   0x000005f0   Data   RO         3420    .constdata          oled.o[m
[32m+[m[32m    0x08003634   0x08003634   0x00000011   Data   RO         3912    .constdata          c_w.l(__printf_flags_ss_wp.o)[m
[32m+[m[32m    0x08003645   0x08003645   0x00000003   PAD[m
[32m+[m[32m    0x08003648   0x08003648   0x00000008   Data   RO         3956    .constdata          c_w.l(_printf_wctomb.o)[m
[32m+[m[32m    0x08003650   0x08003650   0x00000028   Data   RO         3985    .constdata          c_w.l(_printf_hex_int_ll_ptr.o)[m
[32m+[m[32m    0x08003678   0x08003678   0x00000026   Data   RO         4047    .constdata          c_w.l(_printf_fp_hex.o)[m
[32m+[m[32m    0x0800369e   0x0800369e   0x00000002   PAD[m
[32m+[m[32m    0x080036a0   0x080036a0   0x00000094   Data   RO         4058    .constdata          c_w.l(bigflt0.o)[m
[32m+[m[32m    0x08003734   0x08003734   0x00000020   Data   RO         4233    Region$$Table       anon$$obj.o[m
[32m+[m[32m    0x08003754   0x08003754   0x0000001c   Data   RO         4084    locale$$data        c_w.l(lc_numeric_c.o)[m
[32m+[m[32m    0x08003770   0x08003770   0x00000110   Data   RO         4153    locale$$data        c_w.l(lc_ctype_c.o)[m
 [m
 [m
[31m-    Execution Region RW_IRAM1 (Exec base: 0x20000000, Load base: 0x080031e4, Size: 0x00000808, Max: 0x00005000, ABSOLUTE)[m
[32m+[m[32m    Execution Region RW_IRAM1 (Exec base: 0x20000000, Load base: 0x08003880, Size: 0x00000700, Max: 0x00005000, ABSOLUTE)[m
 [m
     Exec Addr    Load Addr    Size         Type   Attr      Idx    E Section Name        Object[m
 [m
[31m-    0x20000000   0x080031e4   0x0000000c   Data   RW         3322    .data               led.o[m
[31m-    0x2000000c   0x080031f0   0x0000001a   Data   RW         3366    .data               key.o[m
[31m-    0x20000026   0x0800320a   0x00000002   PAD[m
[31m-    0x20000028   0x0800320c   0x0000003c   Data   RW         3531    .data               menu.o[m
[31m-    0x20000064   0x08003248   0x00000002   Data   RW         3613    .data               encoder.o[m
[31m-    0x20000066   0x0800324a   0x00000002   PAD[m
[31m-    0x20000068        -       0x00000140   Zero   RW         3528    .bss                menu.o[m
[31m-    0x200001a8        -       0x00000060   Zero   RW         3867    .bss                c_w.l(libspace.o)[m
[31m-    0x20000208        -       0x00000200   Zero   RW            2    HEAP                startup_stm32f10x_md.o[m
[31m-    0x20000408        -       0x00000400   Zero   RW            1    STACK               startup_stm32f10x_md.o[m
[32m+[m[32m    0x20000000   0x08003880   0x00000014   Data   RW         1412    .data               stm32f10x_rcc.o[m
[32m+[m[32m    0x20000014   0x08003894   0x0000001a   Data   RW         3366    .data               key.o[m
[32m+[m[32m    0x2000002e   0x080038ae   0x00000003   Data   RW         3720    .data               serial.o[m
[32m+[m[32m    0x20000031   0x080038b1   0x00000001   PAD[m
[32m+[m[32m    0x20000032   0x080038b2   0x00000004   Data   RW         3783    .data               main.o[m
[32m+[m[32m    0x20000036   0x080038b6   0x00000002   PAD[m
[32m+[m[32m    0x20000038        -       0x00000064   Zero   RW         3719    .bss                serial.o[m
[32m+[m[32m    0x2000009c        -       0x00000060   Zero   RW         4092    .bss                c_w.l(libspace.o)[m
[32m+[m[32m    0x200000fc   0x080038b6   0x00000004   PAD[m
[32m+[m[32m    0x20000100        -       0x00000200   Zero   RW            2    HEAP                startup_stm32f10x_md.o[m
[32m+[m[32m    0x20000300        -       0x00000400   Zero   RW            1    STACK               startup_stm32f10x_md.o[m
 [m
 [m
 ==============================================================================[m
[36m@@ -1706,92 +2017,123 @@[m [mImage component sizes[m
       Code (inc. data)   RO Data    RW Data    ZI Data      Debug   Object Name[m
 [m
          0          0          0          0          0       4500   core_cm3.o[m
[31m-       296         28          0          2          0       2119   encoder.o[m
[31m-       958         50          0         26          0       3445   key.o[m
[31m-       340         52         18         12          0       3427   led.o[m
[31m-        98          0          0          0          0        909   main.o[m
[31m-      1040        106        402         60        320       4865   menu.o[m
[32m+[m[32m       188         14          0          0          0     238817   encoder.o[m
[32m+[m[32m       860         46          0         26          0       2424   key.o[m
[32m+[m[32m       128         16          0          4          0        993   main.o[m
        132         22          0          0          0       2971   misc.o[m
[31m-       882         32       1520          0          0       7909   oled.o[m
[32m+[m[32m       816         22       1520          0          0       6681   oled.o[m
[32m+[m[32m       438         34          0          3        100       3760   serial.o[m
         64         26        236          0       1536        792   startup_stm32f10x_md.o[m
[31m-       200         18          0          0          0       3976   stm32f10x_exti.o[m
[31m-       374          4          0          0          0      12257   stm32f10x_gpio.o[m
[32m+[m[32m       306          0          0          0          0      11220   stm32f10x_gpio.o[m
         26          0          0          0          0       3530   stm32f10x_it.o[m
[31m-        64         12          0          0          0       1026   stm32f10x_rcc.o[m
[31m-       264         42          0          0          0      24416   stm32f10x_tim.o[m
[32m+[m[32m       276         32          0         20          0      12830   stm32f10x_rcc.o[m
[32m+[m[32m      1194        150          0          0          0      32019   stm32f10x_tim.o[m
[32m+[m[32m       472          6          0          0          0      12255   stm32f10x_usart.o[m
          0          0          0          0          0     201660   stm32f10x_wwdg.o[m
[31m-       328         28          0          0          0       1733   system_stm32f10x.o[m
[31m-       124          0          0          0          0     234557   timer.o[m
[32m+[m[32m       328         28          0          0          0      10429   system_stm32f10x.o[m
[32m+[m[32m       124          0          0          0          0     232433   timer.o[m
 [m
     ----------------------------------------------------------------------[m
[31m-      5204        420       2210        104       1856     514092   Object Totals[m
[32m+[m[32m      5368        396       1788         56       1636     777314   Object Totals[m
          0          0         32          0          0          0   (incl. Generated)[m
[31m-        14          0          2          4          0          0   (incl. Padding)[m
[32m+[m[32m        16          0          0          3          0          0   (incl. Padding)[m
 [m
     ----------------------------------------------------------------------[m
 [m
       Code (inc. data)   RO Data    RW Data    ZI Data      Debug   Library Member Name[m
 [m
          8          0          0          0          0         68   __main.o[m
[31m-       284          0          0          0          0        156   __printf_wp.o[m
[32m+[m[32m       392          4         17          0          0         92   __printf_flags_ss_wp.o[m
[32m+[m[32m        14          0          0          0          0         68   __printf_wp.o[m
          0          0          0          0          0          0   __rtentry.o[m
         12          0          0          0          0          0   __rtentry2.o[m
          6          0          0          0          0          0   __rtentry4.o[m
         52          8          0          0          0          0   __scatter.o[m
         26          0          0          0          0          0   __scatter_copy.o[m
         28          0          0          0          0          0   __scatter_zi.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_a.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_c.o[m
[32m+[m[32m        44          0          0          0          0        108   _printf_char.o[m
         48          6          0          0          0         96   _printf_char_common.o[m
[32m+[m[32m        40          0          0          0          0         68   _printf_charcount.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_d.o[m
[32m+[m[32m       120         16          0          0          0         92   _printf_dec.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_e.o[m
          6          0          0          0          0          0   _printf_f.o[m
       1054          0          0          0          0        216   _printf_fp_dec.o[m
[32m+[m[32m       764          8         38          0          0        100   _printf_fp_hex.o[m
        128         16          0          0          0         84   _printf_fp_infnan.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_g.o[m
[32m+[m[32m       148          4         40          0          0        160   _printf_hex_int_ll_ptr.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_i.o[m
[32m+[m[32m       178          0          0          0          0         88   _printf_intcommon.o[m
[32m+[m[32m        10          0          0          0          0          0   _printf_l.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_lc.o[m
[32m+[m[32m        10          0          0          0          0          0   _printf_ll.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_lld.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_lli.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_llo.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_llu.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_llx.o[m
[32m+[m[32m       124         16          0          0          0         92   _printf_longlong_dec.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_ls.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_n.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_o.o[m
[32m+[m[32m       112         10          0          0          0        124   _printf_oct_int_ll.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_p.o[m
[32m+[m[32m        78          0          0          0          0        108   _printf_pad.o[m
          0          0          0          0          0          0   _printf_percent.o[m
          4          0          0          0          0          0   _printf_percent_end.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_s.o[m
[32m+[m[32m        82          0          0          0          0         80   _printf_str.o[m
[32m+[m[32m        36          0          0          0          0         84   _printf_truncate.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_u.o[m
[32m+[m[32m        44          0          0          0          0        108   _printf_wchar.o[m
[32m+[m[32m       188          6          8          0          0         92   _printf_wctomb.o[m
[32m+[m[32m         6          0          0          0          0          0   _printf_x.o[m
         10          0          0          0          0         68   _sputc.o[m
[32m+[m[32m        64          0          0          0          0         92   _wcrtomb.o[m
        228          4        148          0          0         96   bigflt0.o[m
       1936        128          0          0          0        672   btod.o[m
         18          0          0          0          0         80   exit.o[m
          6          0          0          0          0        152   heapauxi.o[m
          0          0          0          0          0          0   indicate_semi.o[m
[32m+[m[32m        44         10        272          0          0         76   lc_ctype_c.o[m
         44         10         28          0          0         76   lc_numeric_c.o[m
          2          0          0          0          0          0   libinit.o[m
[31m-        18          0          0          0          0          0   libinit2.o[m
[32m+[m[32m        30          0          0          0          0          0   libinit2.o[m
          2          0          0          0          0          0   libshutdown.o[m
          2          0          0          0          0          0   libshutdown2.o[m
          8          4          0          0         96         68   libspace.o[m
        138          0          0          0          0         80   lludiv10.o[m
[31m-        40          6          0          0          0         84   noretval__2sprintf.o[m
[32m+[m[32m        16          4          0          0          0         76   rt_ctype_table.o[m
          8          4          0          0          0         68   rt_locale_intlibspace.o[m
[31m-       100          0          0          0          0         80   rt_memcpy_w.o[m
          2          0          0          0          0          0   rtexit.o[m
         10          0          0          0          0          0   rtexit2.o[m
        128          0          0          0          0         68   strcmpv7m.o[m
         12          4          0          0          0         68   sys_exit.o[m
         74          0          0          0          0         80   sys_stackheap_outer.o[m
          2          0          0          0          0         68   use_no_semi.o[m
[31m-        12          0          0          0          0         68   dretinf.o[m
[31m-        86          4          0          0          0         84   f2d.o[m
[31m-       388         76          0          0          0         96   fdiv.o[m
[31m-        48          0          0          0          0         68   fflt_clz.o[m
[31m-       140          4          0          0          0         84   fnaninf.o[m
[31m-        10          0          0          0          0         68   fretinf.o[m
[32m+[m[32m        36          4          0          0          0         76   vsprintf.o[m
          4          0          0          0          0         68   printf1.o[m
[32m+[m[32m         4          0          0          0          0         68   printf2.o[m
          0          0          0          0          0          0   usenofp.o[m
         40          0          0          0          0         68   fpclassify.o[m
 [m
     ----------------------------------------------------------------------[m
[31m-      5182        274        176          0         96       3032   Library Totals[m
[31m-        10          0          0          0          0          0   (incl. Padding)[m
[32m+[m[32m      6752        266        556          0        100       4096   Library Totals[m
[32m+[m[32m        14          0          5          0          4          0   (incl. Padding)[m
 [m
     ----------------------------------------------------------------------[m
 [m
       Code (inc. data)   RO Data    RW Data    ZI Data      Debug   Library Name[m
 [m
[31m-      4444        190        176          0         96       2428   c_w.l[m
[31m-       688         84          0          0          0        536   fz_ws.l[m
[32m+[m[32m      6690        266        551          0         96       3892   c_w.l[m
[32m+[m[32m         8          0          0          0          0        136   fz_ws.l[m
         40          0          0          0          0         68   m_ws.l[m
 [m
     ----------------------------------------------------------------------[m
[31m-      5182        274        176          0         96       3032   Library Totals[m
[32m+[m[32m      6752        266        556          0        100       4096   Library Totals[m
 [m
     ----------------------------------------------------------------------[m
 [m
[36m@@ -1800,15 +2142,15 @@[m [mImage component sizes[m
 [m
       Code (inc. data)   RO Data    RW Data    ZI Data      Debug   [m
 [m
[31m-     10386        694       2386        104       1952     512024   Grand Totals[m
[31m-     10386        694       2386        104       1952     512024   ELF Image Totals[m
[31m-     10386        694       2386        104          0          0   ROM Totals[m
[32m+[m[32m     12120        662       2344         56       1736     775582   Grand Totals[m
[32m+[m[32m     12120        662       2344         56       1736     775582   ELF Image Totals[m
[32m+[m[32m     12120        662       2344         56          0          0   ROM Totals[m
 [m
 ==============================================================================[m
 [m
[31m-    Total RO  Size (Code + RO Data)                12772 (  12.47kB)[m
[31m-    Total RW  Size (RW Data + ZI Data)              2056 (   2.01kB)[m
[31m-    Total ROM Size (Code + RO Data + RW Data)      12876 (  12.57kB)[m
[32m+[m[32m    Total RO  Size (Code + RO Data)                14464 (  14.13kB)[m
[32m+[m[32m    Total RW  Size (RW Data + ZI Data)              1792 (   1.75kB)[m
[32m+[m[32m    Total ROM Size (Code + RO Data + RW Data)      14520 (  14.18kB)[m
 [m
 ==============================================================================[m
 [m
[1mdiff --git a/asc2/Objects/Project.axf b/asc2/Objects/Project.axf[m
[1mindex a6c5670..40d9aa4 100644[m
Binary files a/asc2/Objects/Project.axf and b/asc2/Objects/Project.axf differ
[1mdiff --git a/asc2/Objects/Project.build_log.htm b/asc2/Objects/Project.build_log.htm[m
[1mindex 89a2197..b797e9c 100644[m
[1m--- a/asc2/Objects/Project.build_log.htm[m
[1m+++ b/asc2/Objects/Project.build_log.htm[m
[36m@@ -21,12 +21,15 @@[m [mTarget DLL:      STLink\ST-LINKIII-KEIL_SWO.dll V3.0.1.0[m
 Dialog DLL:      TCM.DLL V1.32.0.0[m
  [m
 <h2>Project:</h2>[m
[31m-D:\STM32Project\µ⁄“ª¥Œ—È ’»ŒŒÒ\Project.uvprojx[m
[31m-Project File Date:  10/24/2025[m
[32m+[m[32mD:\github\asc2\Project.uvprojx[m
[32m+[m[32mProject File Date:  11/06/2025[m
 [m
 <h2>Output:</h2>[m
 *** Using Compiler 'V5.06 update 5 (build 528)', folder: 'D:\Keil5\ARM\ARMCC\Bin'[m
 Build target 'Target 1'[m
[32m+[m[32mcompiling main.c...[m
[32m+[m[32mlinking...[m
[32m+[m[32mProgram Size: Code=12120 RO-data=2344 RW-data=56 ZI-data=1736[m[41m  [m
 ".\Objects\Project.axf" - 0 Error(s), 0 Warning(s).[m
 [m
 <h2>Software Packages used:</h2>[m
[1mdiff --git a/asc2/Objects/Project.htm b/asc2/Objects/Project.htm[m
[1mindex 2d5d1f9..e01543e 100644[m
[1m--- a/asc2/Objects/Project.htm[m
[1m+++ b/asc2/Objects/Project.htm[m
[36m@@ -3,16 +3,16 @@[m
 <title>Static Call Graph - [.\Objects\Project.axf]</title></head>[m
 <body><HR>[m
 <H1>Static Call Graph for image .\Objects\Project.axf</H1><HR>[m
[31m-<BR><P>#&#060CALLGRAPH&#062# ARM Linker, 5060528: Last Updated: Sat Oct 25 17:26:26 2025[m
[32m+[m[32m<BR><P>#&#060CALLGRAPH&#062# ARM Linker, 5060528: Last Updated: Thu Nov 06 21:52:02 2025[m
 <BR><P>[m
[31m-<H3>Maximum Stack Usage =        336 bytes + Unknown(Functions without stacksize, Cycles, Untraceable Function Pointers)</H3><H3>[m
[32m+[m[32m<H3>Maximum Stack Usage =        324 bytes + Unknown(Functions without stacksize, Cycles, Untraceable Function Pointers)</H3><H3>[m
 Call chain for Maximum Stack Depth:</H3>[m
[31m-__rt_entry_main &rArr; main &rArr; Menu_Init &rArr; __aeabi_memcpy4[m
[32m+[m[32m_printf_f &rArr; _printf_fp_dec &rArr; _printf_fp_dec_real &rArr; _fp_digits &rArr; _btod_etento &rArr; _btod_emul &rArr; _e2e[m
 <P>[m
 <H3>[m
 Functions with no stack information[m
 </H3><UL>[m
[31m- <LI><a href="#[62]">__user_initial_stackheap</a>[m
[32m+[m[32m <LI><a href="#[92]">__user_initial_stackheap</a>[m
 </UL>[m
 </UL>[m
 <P>[m
[36m@@ -36,9 +36,9 @@[m [mFunction Pointers[m
  <LI><a href="#[1e]">DMA1_Channel6_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[1f]">DMA1_Channel7_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[b]">DebugMon_Handler</a> from stm32f10x_it.o(i.DebugMon_Handler) referenced from startup_stm32f10x_md.o(RESET)[m
[31m- <LI><a href="#[14]">EXTI0_IRQHandler</a> from encoder.o(i.EXTI0_IRQHandler) referenced from startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m <LI><a href="#[14]">EXTI0_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[36]">EXTI15_10_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
[31m- <LI><a href="#[15]">EXTI1_IRQHandler</a> from encoder.o(i.EXTI1_IRQHandler) referenced from startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m <LI><a href="#[15]">EXTI1_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[16]">EXTI2_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[17]">EXTI3_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[18]">EXTI4_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
[36m@@ -70,7 +70,7 @@[m [mFunction Pointers[m
  <LI><a href="#[2a]">TIM2_IRQHandler</a> from main.o(i.TIM2_IRQHandler) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[2b]">TIM3_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[2c]">TIM4_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
[31m- <LI><a href="#[33]">USART1_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m <LI><a href="#[33]">USART1_IRQHandler</a> from serial.o(i.USART1_IRQHandler) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[34]">USART2_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[35]">USART3_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[38]">USBWakeUp_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
[36m@@ -78,169 +78,311 @@[m [mFunction Pointers[m
  <LI><a href="#[22]">USB_LP_CAN1_RX0_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[9]">UsageFault_Handler</a> from stm32f10x_it.o(i.UsageFault_Handler) referenced from startup_stm32f10x_md.o(RESET)[m
  <LI><a href="#[e]">WWDG_IRQHandler</a> from startup_stm32f10x_md.o(.text) referenced from startup_stm32f10x_md.o(RESET)[m
[31m- <LI><a href="#[3d]">__main</a> from __main.o(!!!main) referenced from startup_stm32f10x_md.o(.text)[m
[32m+[m[32m <LI><a href="#[3e]">__main</a> from __main.o(!!!main) referenced from startup_stm32f10x_md.o(.text)[m
[32m+[m[32m <LI><a href="#[3d]">_get_lc_ctype</a> from lc_ctype_c.o(locale$$code) referenced from rt_ctype_table.o(.text)[m
  <LI><a href="#[3c]">_printf_input_char</a> from _printf_char_common.o(.text) referenced from _printf_char_common.o(.text)[m
[31m- <LI><a href="#[3b]">_sputc</a> from _sputc.o(.text) referenced from noretval__2sprintf.o(.text)[m
[32m+[m[32m <LI><a href="#[3b]">_sputc</a> from _sputc.o(.text) referenced from vsprintf.o(.text)[m
 </UL>[m
 <P>[m
 <H3>[m
 Global Symbols[m
 </H3>[m
[31m-<P><STRONG><a name="[3d]"></a>__main</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, __main.o(!!!main))[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[3f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry[m
[31m-<LI><a href="#[3e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload[m
[32m+[m[32m<P><STRONG><a name="[3e]"></a>__main</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, __main.o(!!!main))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[40]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry[m
[32m+[m[32m<LI><a href="#[3f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[3e]"></a>__scatterload</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __scatter.o(!!!scatter))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[3d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__main[m
[32m+[m[32m<P><STRONG><a name="[3f]"></a>__scatterload</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __scatter.o(!!!scatter))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[3e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[40]"></a>__scatterload_rt2</STRONG> (Thumb, 44 bytes, Stack size unknown bytes, __scatter.o(!!!scatter), UNUSED)[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[3f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry[m
[32m+[m[32m<P><STRONG><a name="[41]"></a>__scatterload_rt2</STRONG> (Thumb, 44 bytes, Stack size unknown bytes, __scatter.o(!!!scatter), UNUSED)[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[40]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[aa]"></a>__scatterload_rt2_thumb_only</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __scatter.o(!!!scatter), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[d6]"></a>__scatterload_rt2_thumb_only</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __scatter.o(!!!scatter), UNUSED)[m
 [m
[31m-<P><STRONG><a name="[ab]"></a>__scatterload_null</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __scatter.o(!!!scatter), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[d7]"></a>__scatterload_null</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __scatter.o(!!!scatter), UNUSED)[m
 [m
[31m-<P><STRONG><a name="[41]"></a>__scatterload_copy</STRONG> (Thumb, 26 bytes, Stack size unknown bytes, __scatter_copy.o(!!handler_copy), UNUSED)[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[41]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload_copy[m
[32m+[m[32m<P><STRONG><a name="[42]"></a>__scatterload_copy</STRONG> (Thumb, 26 bytes, Stack size unknown bytes, __scatter_copy.o(!!handler_copy), UNUSED)[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[42]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload_copy[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[41]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload_copy[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[42]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload_copy[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[ac]"></a>__scatterload_zeroinit</STRONG> (Thumb, 28 bytes, Stack size unknown bytes, __scatter_zi.o(!!handler_zi), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[d8]"></a>__scatterload_zeroinit</STRONG> (Thumb, 28 bytes, Stack size unknown bytes, __scatter_zi.o(!!handler_zi), UNUSED)[m
 [m
[31m-<P><STRONG><a name="[42]"></a>_printf_f</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_f.o(.ARM.Collect$$_printf_percent$$00000003))[m
[32m+[m[32m<P><STRONG><a name="[43]"></a>_printf_n</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_n.o(.ARM.Collect$$_printf_percent$$00000001))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[44]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_charcount[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[79]"></a>_printf_percent</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_percent.o(.ARM.Collect$$_printf_percent$$00000000))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[77]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__printf[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[45]"></a>_printf_p</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_p.o(.ARM.Collect$$_printf_percent$$00000002))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 64 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_p &rArr; _printf_hex_ptr &rArr; _printf_longlong_hex &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[46]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_hex_ptr[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[47]"></a>_printf_f</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_f.o(.ARM.Collect$$_printf_percent$$00000003))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 324 + Unknown Stack Size[m
 <LI>Call Chain = _printf_f &rArr; _printf_fp_dec &rArr; _printf_fp_dec_real &rArr; _fp_digits &rArr; _btod_etento &rArr; _btod_emul &rArr; _e2e[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[43]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[48]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[57]"></a>_printf_percent</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_percent.o(.ARM.Collect$$_printf_percent$$00000000))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[55]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__printf[m
[32m+[m[32m<P><STRONG><a name="[49]"></a>_printf_e</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_e.o(.ARM.Collect$$_printf_percent$$00000004))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 324 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_e &rArr; _printf_fp_dec &rArr; _printf_fp_dec_real &rArr; _fp_digits &rArr; _btod_etento &rArr; _btod_emul &rArr; _e2e[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[48]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[ad]"></a>_printf_percent_end</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_percent_end.o(.ARM.Collect$$_printf_percent$$00000017))[m
[32m+[m[32m<P><STRONG><a name="[4a]"></a>_printf_g</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_g.o(.ARM.Collect$$_printf_percent$$00000005))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 324 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_g &rArr; _printf_fp_dec &rArr; _printf_fp_dec_real &rArr; _fp_digits &rArr; _btod_etento &rArr; _btod_emul &rArr; _e2e[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[48]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[4b]"></a>__rt_lib_init</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit.o(.ARM.Collect$$libinit$$00000000))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[4a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_li[m
[32m+[m[32m<P><STRONG><a name="[4b]"></a>_printf_a</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_a.o(.ARM.Collect$$_printf_percent$$00000006))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 112 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_a &rArr; _printf_fp_hex &rArr; _printf_fp_hex_real &rArr; _printf_fp_infnan &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[4c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_hex[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[ae]"></a>__rt_lib_init_fp_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000002))[m
[32m+[m[32m<P><STRONG><a name="[d9]"></a>_printf_ll</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_ll.o(.ARM.Collect$$_printf_percent$$00000007))[m
 [m
[31m-<P><STRONG><a name="[af]"></a>__rt_lib_init_heap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000A))[m
[32m+[m[32m<P><STRONG><a name="[4d]"></a>_printf_i</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_i.o(.ARM.Collect$$_printf_percent$$00000008))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_i &rArr; _printf_int_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_dec[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[44]"></a>__rt_lib_init_lc_common</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000F))[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[45]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_locale[m
[32m+[m[32m<P><STRONG><a name="[4f]"></a>_printf_d</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_d.o(.ARM.Collect$$_printf_percent$$00000009))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_d &rArr; _printf_int_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_dec[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[b0]"></a>__rt_lib_init_preinit_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000004))[m
[32m+[m[32m<P><STRONG><a name="[50]"></a>_printf_u</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_u.o(.ARM.Collect$$_printf_percent$$0000000A))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_u &rArr; _printf_int_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_dec[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[51]"></a>_printf_o</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_o.o(.ARM.Collect$$_printf_percent$$0000000B))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 64 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_o &rArr; _printf_int_oct &rArr; _printf_longlong_oct &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[52]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_oct[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[b1]"></a>__rt_lib_init_rand_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000E))[m
[32m+[m[32m<P><STRONG><a name="[53]"></a>_printf_x</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_x.o(.ARM.Collect$$_printf_percent$$0000000C))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 80 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_x &rArr; _printf_int_hex &rArr; _printf_longlong_hex &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[54]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_hex[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[b2]"></a>__rt_lib_init_user_alloc_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000C))[m
[32m+[m[32m<P><STRONG><a name="[55]"></a>_printf_lli</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_lli.o(.ARM.Collect$$_printf_percent$$0000000D))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_lli &rArr; _printf_longlong_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[56]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_dec[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[b3]"></a>__rt_lib_init_lc_collate_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000011))[m
[32m+[m[32m<P><STRONG><a name="[57]"></a>_printf_lld</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_lld.o(.ARM.Collect$$_printf_percent$$0000000E))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_lld &rArr; _printf_longlong_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[56]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_dec[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[b4]"></a>__rt_lib_init_lc_ctype_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000013))[m
[32m+[m[32m<P><STRONG><a name="[58]"></a>_printf_llu</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_llu.o(.ARM.Collect$$_printf_percent$$0000000F))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_llu &rArr; _printf_longlong_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[56]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_dec[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[b5]"></a>__rt_lib_init_lc_monetary_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000015))[m
[32m+[m[32m<P><STRONG><a name="[59]"></a>_printf_llo</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_llo.o(.ARM.Collect$$_printf_percent$$00000010))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 56 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_llo &rArr; _printf_ll_oct &rArr; _printf_longlong_oct &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[5a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_ll_oct[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[46]"></a>__rt_lib_init_lc_numeric_2</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000016))[m
[32m+[m[32m<P><STRONG><a name="[5b]"></a>_printf_llx</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_llx.o(.ARM.Collect$$_printf_percent$$00000011))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 64 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_llx &rArr; _printf_ll_hex &rArr; _printf_longlong_hex &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[5c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_ll_hex[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[da]"></a>_printf_l</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_l.o(.ARM.Collect$$_printf_percent$$00000012))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[5d]"></a>_printf_c</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_c.o(.ARM.Collect$$_printf_percent$$00000013))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_c &rArr; _printf_char &rArr; _printf_cs_common &rArr; _printf_str &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[5e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_char[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[5f]"></a>_printf_s</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_s.o(.ARM.Collect$$_printf_percent$$00000014))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_s &rArr; _printf_string &rArr; _printf_cs_common &rArr; _printf_str &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[60]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_string[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[61]"></a>_printf_lc</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_lc.o(.ARM.Collect$$_printf_percent$$00000015))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 88 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_lc &rArr; _printf_wchar &rArr; _printf_lcs_common &rArr; _printf_wctomb &rArr; _wcrtomb &rArr; __rt_ctype_table[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[62]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wchar[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[63]"></a>_printf_ls</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_ls.o(.ARM.Collect$$_printf_percent$$00000016))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 88 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = _printf_ls &rArr; _printf_wstring &rArr; _printf_lcs_common &rArr; _printf_wctomb &rArr; _wcrtomb &rArr; __rt_ctype_table[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[64]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wstring[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[db]"></a>_printf_percent_end</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, _printf_percent_end.o(.ARM.Collect$$_printf_percent$$00000017))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[6d]"></a>__rt_lib_init</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit.o(.ARM.Collect$$libinit$$00000000))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_li[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[dc]"></a>__rt_lib_init_fp_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000002))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[dd]"></a>__rt_lib_init_heap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000A))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[65]"></a>__rt_lib_init_lc_common</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000F))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[66]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_locale[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[de]"></a>__rt_lib_init_preinit_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000004))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[df]"></a>__rt_lib_init_rand_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000E))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[e0]"></a>__rt_lib_init_user_alloc_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000000C))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[e1]"></a>__rt_lib_init_lc_collate_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000011))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[67]"></a>__rt_lib_init_lc_ctype_2</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000012))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = __rt_lib_init_lc_ctype_2 &rArr; _get_lc_ctype[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[3d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_get_lc_ctype[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[e2]"></a>__rt_lib_init_lc_ctype_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000013))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[e3]"></a>__rt_lib_init_lc_monetary_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000015))[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[68]"></a>__rt_lib_init_lc_numeric_2</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000016))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 8 + Unknown Stack Size[m
 <LI>Call Chain = __rt_lib_init_lc_numeric_2 &rArr; _get_lc_numeric[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[47]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_get_lc_numeric[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[69]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_get_lc_numeric[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[b6]"></a>__rt_lib_init_alloca_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000002E))[m
[32m+[m[32m<P><STRONG><a name="[e4]"></a>__rt_lib_init_alloca_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000002E))[m
 [m
[31m-<P><STRONG><a name="[b7]"></a>__rt_lib_init_argv_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000002C))[m
[32m+[m[32m<P><STRONG><a name="[e5]"></a>__rt_lib_init_argv_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000002C))[m
 [m
[31m-<P><STRONG><a name="[b8]"></a>__rt_lib_init_atexit_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000001B))[m
[32m+[m[32m<P><STRONG><a name="[e6]"></a>__rt_lib_init_atexit_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000001B))[m
 [m
[31m-<P><STRONG><a name="[b9]"></a>__rt_lib_init_clock_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000021))[m
[32m+[m[32m<P><STRONG><a name="[e7]"></a>__rt_lib_init_clock_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000021))[m
 [m
[31m-<P><STRONG><a name="[ba]"></a>__rt_lib_init_cpp_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000032))[m
[32m+[m[32m<P><STRONG><a name="[e8]"></a>__rt_lib_init_cpp_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000032))[m
 [m
[31m-<P><STRONG><a name="[bb]"></a>__rt_lib_init_exceptions_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000030))[m
[32m+[m[32m<P><STRONG><a name="[e9]"></a>__rt_lib_init_exceptions_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000030))[m
 [m
[31m-<P><STRONG><a name="[bc]"></a>__rt_lib_init_fp_trap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000001F))[m
[32m+[m[32m<P><STRONG><a name="[ea]"></a>__rt_lib_init_fp_trap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000001F))[m
 [m
[31m-<P><STRONG><a name="[bd]"></a>__rt_lib_init_getenv_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000023))[m
[32m+[m[32m<P><STRONG><a name="[eb]"></a>__rt_lib_init_getenv_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000023))[m
 [m
[31m-<P><STRONG><a name="[be]"></a>__rt_lib_init_lc_numeric_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000017))[m
[32m+[m[32m<P><STRONG><a name="[ec]"></a>__rt_lib_init_lc_numeric_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000017))[m
 [m
[31m-<P><STRONG><a name="[bf]"></a>__rt_lib_init_lc_time_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000019))[m
[32m+[m[32m<P><STRONG><a name="[ed]"></a>__rt_lib_init_lc_time_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000019))[m
 [m
[31m-<P><STRONG><a name="[c0]"></a>__rt_lib_init_return</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000033))[m
[32m+[m[32m<P><STRONG><a name="[ee]"></a>__rt_lib_init_return</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000033))[m
 [m
[31m-<P><STRONG><a name="[c1]"></a>__rt_lib_init_signal_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000001D))[m
[32m+[m[32m<P><STRONG><a name="[ef]"></a>__rt_lib_init_signal_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$0000001D))[m
 [m
[31m-<P><STRONG><a name="[c2]"></a>__rt_lib_init_stdio_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000025))[m
[32m+[m[32m<P><STRONG><a name="[f0]"></a>__rt_lib_init_stdio_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libinit2.o(.ARM.Collect$$libinit$$00000025))[m
 [m
[31m-<P><STRONG><a name="[50]"></a>__rt_lib_shutdown</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown.o(.ARM.Collect$$libshutdown$$00000000))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[4f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_exit_ls[m
[32m+[m[32m<P><STRONG><a name="[72]"></a>__rt_lib_shutdown</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown.o(.ARM.Collect$$libshutdown$$00000000))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[71]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_exit_ls[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[c3]"></a>__rt_lib_shutdown_cpp_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000002))[m
[32m+[m[32m<P><STRONG><a name="[f1]"></a>__rt_lib_shutdown_cpp_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000002))[m
 [m
[31m-<P><STRONG><a name="[c4]"></a>__rt_lib_shutdown_fp_trap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000007))[m
[32m+[m[32m<P><STRONG><a name="[f2]"></a>__rt_lib_shutdown_fp_trap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000007))[m
 [m
[31m-<P><STRONG><a name="[c5]"></a>__rt_lib_shutdown_heap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$0000000F))[m
[32m+[m[32m<P><STRONG><a name="[f3]"></a>__rt_lib_shutdown_heap_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$0000000F))[m
 [m
[31m-<P><STRONG><a name="[c6]"></a>__rt_lib_shutdown_return</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000010))[m
[32m+[m[32m<P><STRONG><a name="[f4]"></a>__rt_lib_shutdown_return</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000010))[m
 [m
[31m-<P><STRONG><a name="[c7]"></a>__rt_lib_shutdown_signal_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$0000000A))[m
[32m+[m[32m<P><STRONG><a name="[f5]"></a>__rt_lib_shutdown_signal_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$0000000A))[m
 [m
[31m-<P><STRONG><a name="[c8]"></a>__rt_lib_shutdown_stdio_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000004))[m
[32m+[m[32m<P><STRONG><a name="[f6]"></a>__rt_lib_shutdown_stdio_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$00000004))[m
 [m
[31m-<P><STRONG><a name="[c9]"></a>__rt_lib_shutdown_user_alloc_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$0000000C))[m
[32m+[m[32m<P><STRONG><a name="[f7]"></a>__rt_lib_shutdown_user_alloc_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, libshutdown2.o(.ARM.Collect$$libshutdown$$0000000C))[m
 [m
[31m-<P><STRONG><a name="[3f]"></a>__rt_entry</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry.o(.ARM.Collect$$rtentry$$00000000))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[3d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__main[m
[31m-<LI><a href="#[40]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload_rt2[m
[32m+[m[32m<P><STRONG><a name="[40]"></a>__rt_entry</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry.o(.ARM.Collect$$rtentry$$00000000))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[3e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__main[m
[32m+[m[32m<LI><a href="#[41]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__scatterload_rt2[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[ca]"></a>__rt_entry_presh_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$00000002))[m
[32m+[m[32m<P><STRONG><a name="[f8]"></a>__rt_entry_presh_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$00000002))[m
 [m
[31m-<P><STRONG><a name="[48]"></a>__rt_entry_sh</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry4.o(.ARM.Collect$$rtentry$$00000004))[m
[32m+[m[32m<P><STRONG><a name="[6a]"></a>__rt_entry_sh</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry4.o(.ARM.Collect$$rtentry$$00000004))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 8 + Unknown Stack Size[m
 <LI>Call Chain = __rt_entry_sh &rArr; __user_setup_stackheap[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[49]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_setup_stackheap[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[6b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_setup_stackheap[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[4a]"></a>__rt_entry_li</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$0000000A))[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[4b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_init[m
[32m+[m[32m<P><STRONG><a name="[6c]"></a>__rt_entry_li</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$0000000A))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[6d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[cb]"></a>__rt_entry_postsh_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$00000009))[m
[32m+[m[32m<P><STRONG><a name="[f9]"></a>__rt_entry_postsh_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$00000009))[m
 [m
[31m-<P><STRONG><a name="[4c]"></a>__rt_entry_main</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$0000000D))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 336 + Unknown Stack Size[m
[31m-<LI>Call Chain = __rt_entry_main &rArr; main &rArr; Menu_Init &rArr; __aeabi_memcpy4[m
[32m+[m[32m<P><STRONG><a name="[6e]"></a>__rt_entry_main</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$0000000D))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 248 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = __rt_entry_main &rArr; main &rArr; Serial_Printf &rArr; vsprintf &rArr; _printf_char_common &rArr; __printf[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;exit[m
[31m-<LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<LI><a href="#[70]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;exit[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[cc]"></a>__rt_entry_postli_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$0000000C))[m
[32m+[m[32m<P><STRONG><a name="[fa]"></a>__rt_entry_postli_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, __rtentry2.o(.ARM.Collect$$rtentry$$0000000C))[m
 [m
[31m-<P><STRONG><a name="[63]"></a>__rt_exit</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit.o(.ARM.Collect$$rtexit$$00000000))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;exit[m
[32m+[m[32m<P><STRONG><a name="[93]"></a>__rt_exit</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit.o(.ARM.Collect$$rtexit$$00000000))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[70]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;exit[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[4f]"></a>__rt_exit_ls</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit2.o(.ARM.Collect$$rtexit$$00000003))[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[50]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_shutdown[m
[32m+[m[32m<P><STRONG><a name="[71]"></a>__rt_exit_ls</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit2.o(.ARM.Collect$$rtexit$$00000003))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[72]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_shutdown[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[cd]"></a>__rt_exit_prels_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit2.o(.ARM.Collect$$rtexit$$00000002))[m
[32m+[m[32m<P><STRONG><a name="[fb]"></a>__rt_exit_prels_1</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit2.o(.ARM.Collect$$rtexit$$00000002))[m
 [m
[31m-<P><STRONG><a name="[51]"></a>__rt_exit_exit</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit2.o(.ARM.Collect$$rtexit$$00000004))[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[52]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_sys_exit[m
[32m+[m[32m<P><STRONG><a name="[73]"></a>__rt_exit_exit</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rtexit2.o(.ARM.Collect$$rtexit$$00000004))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[74]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_sys_exit[m
 </UL>[m
 [m
 <P><STRONG><a name="[4]"></a>Reset_Handler</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
[36m@@ -280,9 +422,15 @@[m [mGlobal Symbols[m
 <P><STRONG><a name="[1f]"></a>DMA1_Channel7_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[32m+[m[32m<P><STRONG><a name="[14]"></a>EXTI0_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m</UL>[m
 <P><STRONG><a name="[36]"></a>EXTI15_10_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[32m+[m[32m<P><STRONG><a name="[15]"></a>EXTI1_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m</UL>[m
 <P><STRONG><a name="[16]"></a>EXTI2_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[36m@@ -349,9 +497,6 @@[m [mGlobal Symbols[m
 <P><STRONG><a name="[2c]"></a>TIM4_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[31m-<P><STRONG><a name="[33]"></a>USART1_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[31m-</UL>[m
 <P><STRONG><a name="[34]"></a>USART2_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[36m@@ -370,776 +515,974 @@[m [mGlobal Symbols[m
 <P><STRONG><a name="[e]"></a>WWDG_IRQHandler</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, startup_stm32f10x_md.o(.text))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[31m-<P><STRONG><a name="[62]"></a>__user_initial_stackheap</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, startup_stm32f10x_md.o(.text))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[49]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_setup_stackheap[m
[32m+[m[32m<P><STRONG><a name="[92]"></a>__user_initial_stackheap</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, startup_stm32f10x_md.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[6b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_setup_stackheap[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[53]"></a>__2sprintf</STRONG> (Thumb, 34 bytes, Stack size 32 bytes, noretval__2sprintf.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 128 + Unknown Stack Size[m
[31m-<LI>Call Chain = __2sprintf &rArr; _printf_char_common &rArr; __printf[m
[32m+[m[32m<P><STRONG><a name="[75]"></a>vsprintf</STRONG> (Thumb, 32 bytes, Stack size 16 bytes, vsprintf.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 120 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = vsprintf &rArr; _printf_char_common &rArr; __printf[m
 </UL>[m
 <BR>[Calls]<UL><LI><a href="#[3b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_sputc[m
[31m-<LI><a href="#[54]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_char_common[m
[32m+[m[32m<LI><a href="#[76]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_char_common[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[8b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowFloatNum[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[bc]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Printf[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[55]"></a>__printf</STRONG> (Thumb, 270 bytes, Stack size 32 bytes, __printf_wp.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 32 + Unknown Stack Size[m
[32m+[m[32m<P><STRONG><a name="[77]"></a>__printf</STRONG> (Thumb, 388 bytes, Stack size 40 bytes, __printf_flags_ss_wp.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40 + Unknown Stack Size[m
 <LI>Call Chain = __printf[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[57]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_percent[m
[31m-<LI><a href="#[56]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_is_digit[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[79]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_percent[m
[32m+[m[32m<LI><a href="#[78]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_is_digit[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[54]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_char_common[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[76]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_char_common[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[7f]"></a>__aeabi_memcpy4</STRONG> (Thumb, 0 bytes, Stack size 8 bytes, rt_memcpy_w.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = __aeabi_memcpy4[m
[31m-</UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[7e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Init[m
[32m+[m[32m<P><STRONG><a name="[d5]"></a>strcmp</STRONG> (Thumb, 128 bytes, Stack size 0 bytes, strcmpv7m.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[3d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_get_lc_ctype[m
[32m+[m[32m<LI><a href="#[69]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_get_lc_numeric[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[ce]"></a>__aeabi_memcpy8</STRONG> (Thumb, 0 bytes, Stack size 8 bytes, rt_memcpy_w.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[fc]"></a>__use_two_region_memory</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, heapauxi.o(.text), UNUSED)[m
 [m
[31m-<P><STRONG><a name="[cf]"></a>__rt_memcpy_w</STRONG> (Thumb, 100 bytes, Stack size 8 bytes, rt_memcpy_w.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[fd]"></a>__rt_heap_escrow$2region</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, heapauxi.o(.text), UNUSED)[m
 [m
[31m-<P><STRONG><a name="[d0]"></a>_memcpy_lastbytes_aligned</STRONG> (Thumb, 0 bytes, Stack size unknown bytes, rt_memcpy_w.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[fe]"></a>__rt_heap_expand$2region</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, heapauxi.o(.text), UNUSED)[m
 [m
[31m-<P><STRONG><a name="[82]"></a>strcmp</STRONG> (Thumb, 128 bytes, Stack size 0 bytes, strcmpv7m.o(.text))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[47]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_get_lc_numeric[m
[31m-<LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[31m-<LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Option[m
[32m+[m[32m<P><STRONG><a name="[7b]"></a>_printf_pre_padding</STRONG> (Thumb, 44 bytes, Stack size 16 bytes, _printf_pad.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = _printf_pre_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[89]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wctomb[m
[32m+[m[32m<LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<LI><a href="#[7a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_str[m
[32m+[m[32m<LI><a href="#[88]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_infnan[m
[32m+[m[32m<LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_hex_real[m
[32m+[m[32m<LI><a href="#[7f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_common[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[d1]"></a>__use_two_region_memory</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, heapauxi.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[7c]"></a>_printf_post_padding</STRONG> (Thumb, 34 bytes, Stack size 16 bytes, _printf_pad.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[89]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wctomb[m
[32m+[m[32m<LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<LI><a href="#[7a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_str[m
[32m+[m[32m<LI><a href="#[88]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_infnan[m
[32m+[m[32m<LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_hex_real[m
[32m+[m[32m<LI><a href="#[7f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_common[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[d2]"></a>__rt_heap_escrow$2region</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, heapauxi.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[7d]"></a>_printf_truncate_signed</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, _printf_truncate.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_dec[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[d3]"></a>__rt_heap_expand$2region</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, heapauxi.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[7e]"></a>_printf_truncate_unsigned</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, _printf_truncate.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_dec[m
[32m+[m[32m<LI><a href="#[54]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_hex[m
[32m+[m[32m<LI><a href="#[52]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_oct[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[d4]"></a>__lib_sel_fp_printf</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, _printf_fp_dec.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[7a]"></a>_printf_str</STRONG> (Thumb, 82 bytes, Stack size 16 bytes, _printf_str.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 32<LI>Call Chain = _printf_str &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_post_padding[m
[32m+[m[32m<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_pre_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[8e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_cs_common[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[5e]"></a>_printf_fp_dec_real</STRONG> (Thumb, 620 bytes, Stack size 104 bytes, _printf_fp_dec.o(.text))[m
[32m+[m[32m<P><STRONG><a name="[4e]"></a>_printf_int_dec</STRONG> (Thumb, 104 bytes, Stack size 24 bytes, _printf_dec.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72<LI>Call Chain = _printf_int_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_truncate_unsigned[m
[32m+[m[32m<LI><a href="#[7d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_truncate_signed[m
[32m+[m[32m<LI><a href="#[7f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_common[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[50]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_u[m
[32m+[m[32m<LI><a href="#[4f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_d[m
[32m+[m[32m<LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_i[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[44]"></a>_printf_charcount</STRONG> (Thumb, 40 bytes, Stack size 0 bytes, _printf_charcount.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[43]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_n[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[ff]"></a>__lib_sel_fp_printf</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, _printf_fp_dec.o(.text), UNUSED)[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[86]"></a>_printf_fp_dec_real</STRONG> (Thumb, 620 bytes, Stack size 104 bytes, _printf_fp_dec.o(.text))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 324<LI>Call Chain = _printf_fp_dec_real &rArr; _fp_digits &rArr; _btod_etento &rArr; _btod_emul &rArr; _e2e[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[5f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__ARM_fpclassify[m
[31m-<LI><a href="#[60]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_infnan[m
[31m-<LI><a href="#[45]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_locale[m
[31m-<LI><a href="#[58]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<LI><a href="#[7c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_post_padding[m
[32m+[m[32m<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_pre_padding[m
[32m+[m[32m<LI><a href="#[87]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__ARM_fpclassify[m
[32m+[m[32m<LI><a href="#[88]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_infnan[m
[32m+[m[32m<LI><a href="#[66]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_locale[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[43]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[48]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[54]"></a>_printf_char_common</STRONG> (Thumb, 32 bytes, Stack size 64 bytes, _printf_char_common.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 96 + Unknown Stack Size[m
[32m+[m[32m<P><STRONG><a name="[76]"></a>_printf_char_common</STRONG> (Thumb, 32 bytes, Stack size 64 bytes, _printf_char_common.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 104 + Unknown Stack Size[m
 <LI>Call Chain = _printf_char_common &rArr; __printf[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[55]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__printf[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[77]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__printf[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[53]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__2sprintf[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[75]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;vsprintf[m
 </UL>[m
 [m
 <P><STRONG><a name="[3b]"></a>_sputc</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, _sputc.o(.text))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[53]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__2sprintf[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[75]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;vsprintf[m
 </UL>[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> noretval__2sprintf.o(.text)[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> vsprintf.o(.text)[m
 </UL>[m
[31m-<P><STRONG><a name="[45]"></a>__rt_locale</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, rt_locale_intlibspace.o(.text))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[44]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_init_lc_common[m
[31m-<LI><a href="#[5e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<P><STRONG><a name="[89]"></a>_printf_wctomb</STRONG> (Thumb, 182 bytes, Stack size 56 bytes, _printf_wctomb.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 80<LI>Call Chain = _printf_wctomb &rArr; _wcrtomb &rArr; __rt_ctype_table[m
 </UL>[m
[31m-[m
[31m-<P><STRONG><a name="[5d]"></a>_ll_udiv10</STRONG> (Thumb, 138 bytes, Stack size 12 bytes, lludiv10.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 12<LI>Call Chain = _ll_udiv10[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_post_padding[m
[32m+[m[32m<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_pre_padding[m
[32m+[m[32m<LI><a href="#[8a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_wcrtomb[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[58]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[8f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_lcs_common[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[60]"></a>_printf_fp_infnan</STRONG> (Thumb, 112 bytes, Stack size 24 bytes, _printf_fp_infnan.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = _printf_fp_infnan[m
[32m+[m[32m<P><STRONG><a name="[56]"></a>_printf_longlong_dec</STRONG> (Thumb, 108 bytes, Stack size 24 bytes, _printf_longlong_dec.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 72<LI>Call Chain = _printf_longlong_dec &rArr; _printf_int_common &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[5e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_common[m
[32m+[m[32m<LI><a href="#[85]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_ll_udiv10[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[58]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_llu[m
[32m+[m[32m<LI><a href="#[57]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_lld[m
[32m+[m[32m<LI><a href="#[55]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_lli[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[59]"></a>_btod_etento</STRONG> (Thumb, 224 bytes, Stack size 72 bytes, bigflt0.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 124<LI>Call Chain = _btod_etento &rArr; _btod_emul &rArr; _e2e[m
[32m+[m[32m<P><STRONG><a name="[8b]"></a>_printf_longlong_oct</STRONG> (Thumb, 66 bytes, Stack size 8 bytes, _printf_oct_int_ll.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 56<LI>Call Chain = _printf_longlong_oct &rArr; _printf_int_common &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[5c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
[31m-<LI><a href="#[5b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_common[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[58]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[5a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_ll_oct[m
[32m+[m[32m<LI><a href="#[52]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_oct[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[d5]"></a>__user_libspace</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, libspace.o(.text), UNUSED)[m
[31m-[m
[31m-<P><STRONG><a name="[61]"></a>__user_perproc_libspace</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, libspace.o(.text))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[49]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_setup_stackheap[m
[32m+[m[32m<P><STRONG><a name="[52]"></a>_printf_int_oct</STRONG> (Thumb, 24 bytes, Stack size 8 bytes, _printf_oct_int_ll.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 64<LI>Call Chain = _printf_int_oct &rArr; _printf_longlong_oct &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_truncate_unsigned[m
[32m+[m[32m<LI><a href="#[8b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_oct[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[51]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_o[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[d6]"></a>__user_perthread_libspace</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, libspace.o(.text), UNUSED)[m
[31m-[m
[31m-<P><STRONG><a name="[49]"></a>__user_setup_stackheap</STRONG> (Thumb, 74 bytes, Stack size 8 bytes, sys_stackheap_outer.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 8 + Unknown Stack Size[m
[31m-<LI>Call Chain = __user_setup_stackheap[m
[32m+[m[32m<P><STRONG><a name="[5a]"></a>_printf_ll_oct</STRONG> (Thumb, 12 bytes, Stack size 0 bytes, _printf_oct_int_ll.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 56<LI>Call Chain = _printf_ll_oct &rArr; _printf_longlong_oct &rArr; _printf_int_common &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[62]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_initial_stackheap[m
[31m-<LI><a href="#[61]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_perproc_libspace[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_oct[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[48]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_sh[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[59]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_llo[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[4e]"></a>exit</STRONG> (Thumb, 18 bytes, Stack size 8 bytes, exit.o(.text))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 8 + Unknown Stack Size[m
[31m-<LI>Call Chain = exit[m
[32m+[m[32m<P><STRONG><a name="[8c]"></a>_printf_longlong_hex</STRONG> (Thumb, 86 bytes, Stack size 16 bytes, _printf_hex_int_ll_ptr.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 64<LI>Call Chain = _printf_longlong_hex &rArr; _printf_int_common &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[63]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_exit[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_common[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[46]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_hex_ptr[m
[32m+[m[32m<LI><a href="#[5c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_ll_hex[m
[32m+[m[32m<LI><a href="#[54]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_hex[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[52]"></a>_sys_exit</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, sys_exit.o(.text))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[51]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_exit_exit[m
[32m+[m[32m<P><STRONG><a name="[54]"></a>_printf_int_hex</STRONG> (Thumb, 28 bytes, Stack size 16 bytes, _printf_hex_int_ll_ptr.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 80<LI>Call Chain = _printf_int_hex &rArr; _printf_longlong_hex &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_truncate_unsigned[m
[32m+[m[32m<LI><a href="#[8c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_hex[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[53]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_x[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[d7]"></a>__I$use$semihosting</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, use_no_semi.o(.text), UNUSED)[m
[31m-[m
[31m-<P><STRONG><a name="[d8]"></a>__use_no_semihosting_swi</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, use_no_semi.o(.text), UNUSED)[m
[31m-[m
[31m-<P><STRONG><a name="[d9]"></a>__semihosting_library_function</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, indicate_semi.o(.text), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[5c]"></a>_printf_ll_hex</STRONG> (Thumb, 12 bytes, Stack size 0 bytes, _printf_hex_int_ll_ptr.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 64<LI>Call Chain = _printf_ll_hex &rArr; _printf_longlong_hex &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_hex[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[5b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_llx[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[5a]"></a>_btod_d2e</STRONG> (Thumb, 62 bytes, Stack size 0 bytes, btod.o(CL$$btod_d2e))[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[64]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_d2e_norm_op1[m
[32m+[m[32m<P><STRONG><a name="[46]"></a>_printf_hex_ptr</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, _printf_hex_int_ll_ptr.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 64<LI>Call Chain = _printf_hex_ptr &rArr; _printf_longlong_hex &rArr; _printf_int_common &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_hex[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[58]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[45]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_p[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[65]"></a>_d2e_denorm_low</STRONG> (Thumb, 70 bytes, Stack size 0 bytes, btod.o(CL$$btod_d2e_denorm_low))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[64]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_d2e_norm_op1[m
[32m+[m[32m<P><STRONG><a name="[66]"></a>__rt_locale</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, rt_locale_intlibspace.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<LI><a href="#[65]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_init_lc_common[m
[32m+[m[32m<LI><a href="#[90]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_ctype_table[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[64]"></a>_d2e_norm_op1</STRONG> (Thumb, 96 bytes, Stack size 0 bytes, btod.o(CL$$btod_d2e_norm_op1))[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[65]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_d2e_denorm_low[m
[32m+[m[32m<P><STRONG><a name="[85]"></a>_ll_udiv10</STRONG> (Thumb, 138 bytes, Stack size 12 bytes, lludiv10.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 12<LI>Call Chain = _ll_udiv10[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[5a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_d2e[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[56]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_dec[m
[32m+[m[32m<LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[66]"></a>__btod_div_common</STRONG> (Thumb, 696 bytes, Stack size 24 bytes, btod.o(CL$$btod_div_common))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = __btod_div_common[m
[32m+[m[32m<P><STRONG><a name="[7f]"></a>_printf_int_common</STRONG> (Thumb, 178 bytes, Stack size 32 bytes, _printf_intcommon.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 48<LI>Call Chain = _printf_int_common &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[5b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_post_padding[m
[32m+[m[32m<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_pre_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[56]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_dec[m
[32m+[m[32m<LI><a href="#[4e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_int_dec[m
[32m+[m[32m<LI><a href="#[8c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_hex[m
[32m+[m[32m<LI><a href="#[8b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_longlong_oct[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[67]"></a>_e2e</STRONG> (Thumb, 220 bytes, Stack size 24 bytes, btod.o(CL$$btod_e2e))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = _e2e[m
[32m+[m[32m<P><STRONG><a name="[8d]"></a>_printf_fp_hex_real</STRONG> (Thumb, 756 bytes, Stack size 72 bytes, _printf_fp_hex.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 112<LI>Call Chain = _printf_fp_hex_real &rArr; _printf_fp_infnan &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[5c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
[31m-<LI><a href="#[5b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_post_padding[m
[32m+[m[32m<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_pre_padding[m
[32m+[m[32m<LI><a href="#[87]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__ARM_fpclassify[m
[32m+[m[32m<LI><a href="#[88]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_infnan[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[4c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_hex[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[5b]"></a>_btod_ediv</STRONG> (Thumb, 42 bytes, Stack size 28 bytes, btod.o(CL$$btod_ediv))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 52<LI>Call Chain = _btod_ediv &rArr; _e2e[m
[32m+[m[32m<P><STRONG><a name="[88]"></a>_printf_fp_infnan</STRONG> (Thumb, 112 bytes, Stack size 24 bytes, _printf_fp_infnan.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40<LI>Call Chain = _printf_fp_infnan &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[67]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_e2e[m
[31m-<LI><a href="#[66]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__btod_div_common[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_post_padding[m
[32m+[m[32m<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_pre_padding[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[59]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_etento[m
[31m-<LI><a href="#[58]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_hex_real[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[5c]"></a>_btod_emul</STRONG> (Thumb, 42 bytes, Stack size 28 bytes, btod.o(CL$$btod_emul))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 52<LI>Call Chain = _btod_emul &rArr; _e2e[m
[32m+[m[32m<P><STRONG><a name="[8e]"></a>_printf_cs_common</STRONG> (Thumb, 20 bytes, Stack size 8 bytes, _printf_char.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40<LI>Call Chain = _printf_cs_common &rArr; _printf_str &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[68]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__btod_mult_common[m
[31m-<LI><a href="#[67]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_e2e[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[7a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_str[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[59]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_etento[m
[31m-<LI><a href="#[58]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[60]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_string[m
[32m+[m[32m<LI><a href="#[5e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_char[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[68]"></a>__btod_mult_common</STRONG> (Thumb, 580 bytes, Stack size 16 bytes, btod.o(CL$$btod_mult_common))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = __btod_mult_common[m
[32m+[m[32m<P><STRONG><a name="[5e]"></a>_printf_char</STRONG> (Thumb, 16 bytes, Stack size 0 bytes, _printf_char.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40<LI>Call Chain = _printf_char &rArr; _printf_cs_common &rArr; _printf_str &rArr; _printf_post_padding[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[5c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_cs_common[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[5d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_c[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[8]"></a>BusFault_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.BusFault_Handler))[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m<P><STRONG><a name="[60]"></a>_printf_string</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, _printf_char.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40<LI>Call Chain = _printf_string &rArr; _printf_cs_common &rArr; _printf_str &rArr; _printf_post_padding[m
 </UL>[m
[31m-<P><STRONG><a name="[b]"></a>DebugMon_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.DebugMon_Handler))[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_cs_common[m
 </UL>[m
[31m-<P><STRONG><a name="[14]"></a>EXTI0_IRQHandler</STRONG> (Thumb, 52 bytes, Stack size 8 bytes, encoder.o(i.EXTI0_IRQHandler))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = EXTI0_IRQHandler[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[5f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_s[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[69]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI_GetITStatus[m
[31m-<LI><a href="#[6b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI_ClearITPendingBit[m
[31m-<LI><a href="#[6a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_ReadInputDataBit[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[8f]"></a>_printf_lcs_common</STRONG> (Thumb, 20 bytes, Stack size 8 bytes, _printf_wchar.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 88<LI>Call Chain = _printf_lcs_common &rArr; _printf_wctomb &rArr; _wcrtomb &rArr; __rt_ctype_table[m
 </UL>[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[89]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wctomb[m
 </UL>[m
[31m-<P><STRONG><a name="[15]"></a>EXTI1_IRQHandler</STRONG> (Thumb, 52 bytes, Stack size 8 bytes, encoder.o(i.EXTI1_IRQHandler))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = EXTI1_IRQHandler[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[64]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wstring[m
[32m+[m[32m<LI><a href="#[62]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wchar[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[69]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI_GetITStatus[m
[31m-<LI><a href="#[6b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI_ClearITPendingBit[m
[31m-<LI><a href="#[6a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_ReadInputDataBit[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[62]"></a>_printf_wchar</STRONG> (Thumb, 16 bytes, Stack size 0 bytes, _printf_wchar.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 88<LI>Call Chain = _printf_wchar &rArr; _printf_lcs_common &rArr; _printf_wctomb &rArr; _wcrtomb &rArr; __rt_ctype_table[m
 </UL>[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_lcs_common[m
 </UL>[m
[31m-<P><STRONG><a name="[6b]"></a>EXTI_ClearITPendingBit</STRONG> (Thumb, 6 bytes, Stack size 0 bytes, stm32f10x_exti.o(i.EXTI_ClearITPendingBit))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[15]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI1_IRQHandler[m
[31m-<LI><a href="#[14]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI0_IRQHandler[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[61]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_lc[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[69]"></a>EXTI_GetITStatus</STRONG> (Thumb, 34 bytes, Stack size 0 bytes, stm32f10x_exti.o(i.EXTI_GetITStatus))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[15]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI1_IRQHandler[m
[31m-<LI><a href="#[14]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI0_IRQHandler[m
[32m+[m[32m<P><STRONG><a name="[64]"></a>_printf_wstring</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, _printf_wchar.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 88<LI>Call Chain = _printf_wstring &rArr; _printf_lcs_common &rArr; _printf_wctomb &rArr; _wcrtomb &rArr; __rt_ctype_table[m
 </UL>[m
[31m-[m
[31m-<P><STRONG><a name="[70]"></a>EXTI_Init</STRONG> (Thumb, 142 bytes, Stack size 0 bytes, stm32f10x_exti.o(i.EXTI_Init))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_lcs_common[m
 </UL>[m
[31m-[m
[31m-<P><STRONG><a name="[83]"></a>Encoder_Get</STRONG> (Thumb, 14 bytes, Stack size 0 bytes, encoder.o(i.Encoder_Get))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Option[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[63]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_ls[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[6c]"></a>Encoder_Init</STRONG> (Thumb, 150 bytes, Stack size 24 bytes, encoder.o(i.Encoder_Init))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 48<LI>Call Chain = Encoder_Init &rArr; GPIO_Init[m
[32m+[m[32m<P><STRONG><a name="[81]"></a>_btod_etento</STRONG> (Thumb, 224 bytes, Stack size 72 bytes, bigflt0.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 124<LI>Call Chain = _btod_etento &rArr; _btod_emul &rArr; _e2e[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[70]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI_Init[m
[31m-<LI><a href="#[6e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[31m-<LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_EXTILineConfig[m
[31m-<LI><a href="#[6d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
[31m-<LI><a href="#[71]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_PriorityGroupConfig[m
[31m-<LI><a href="#[72]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_Init[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
[32m+[m[32m<LI><a href="#[83]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[73]"></a>FloodLED_LeftTurn</STRONG> (Thumb, 86 bytes, Stack size 16 bytes, led.o(i.FloodLED_LeftTurn))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = FloodLED_LeftTurn[m
[32m+[m[32m<P><STRONG><a name="[8a]"></a>_wcrtomb</STRONG> (Thumb, 64 bytes, Stack size 16 bytes, _wcrtomb.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = _wcrtomb &rArr; __rt_ctype_table[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[74]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
[31m-<LI><a href="#[6a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_ReadInputDataBit[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[90]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_ctype_table[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[7d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_Tick[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[89]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_wctomb[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[75]"></a>FloodLED_RightTurn</STRONG> (Thumb, 86 bytes, Stack size 16 bytes, led.o(i.FloodLED_RightTurn))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = FloodLED_RightTurn[m
[32m+[m[32m<P><STRONG><a name="[100]"></a>__user_libspace</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, libspace.o(.text), UNUSED)[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[91]"></a>__user_perproc_libspace</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, libspace.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[6b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_setup_stackheap[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[101]"></a>__user_perthread_libspace</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, libspace.o(.text), UNUSED)[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[6b]"></a>__user_setup_stackheap</STRONG> (Thumb, 74 bytes, Stack size 8 bytes, sys_stackheap_outer.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = __user_setup_stackheap[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[74]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
[31m-<LI><a href="#[6a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_ReadInputDataBit[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[92]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_initial_stackheap[m
[32m+[m[32m<LI><a href="#[91]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__user_perproc_libspace[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[7d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_Tick[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_sh[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[6f]"></a>GPIO_EXTILineConfig</STRONG> (Thumb, 60 bytes, Stack size 12 bytes, stm32f10x_gpio.o(i.GPIO_EXTILineConfig))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 12<LI>Call Chain = GPIO_EXTILineConfig[m
[32m+[m[32m<P><STRONG><a name="[90]"></a>__rt_ctype_table</STRONG> (Thumb, 16 bytes, Stack size 8 bytes, rt_ctype_table.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = __rt_ctype_table[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[66]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_locale[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[8a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_wcrtomb[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[6e]"></a>GPIO_Init</STRONG> (Thumb, 278 bytes, Stack size 24 bytes, stm32f10x_gpio.o(i.GPIO_Init))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = GPIO_Init[m
[32m+[m[32m<P><STRONG><a name="[70]"></a>exit</STRONG> (Thumb, 18 bytes, Stack size 8 bytes, exit.o(.text))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = exit[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[31m-<LI><a href="#[8e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
[31m-<LI><a href="#[79]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Init[m
[31m-<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_Init[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[93]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_exit[m
 </UL>[m
[31m-[m
[31m-<P><STRONG><a name="[6a]"></a>GPIO_ReadInputDataBit</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, stm32f10x_gpio.o(i.GPIO_ReadInputDataBit))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[15]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI1_IRQHandler[m
[31m-<LI><a href="#[14]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;EXTI0_IRQHandler[m
[31m-<LI><a href="#[78]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_GetState[m
[31m-<LI><a href="#[75]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;FloodLED_RightTurn[m
[31m-<LI><a href="#[73]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;FloodLED_LeftTurn[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[7c]"></a>GPIO_SetBits</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_gpio.o(i.GPIO_SetBits))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_Init[m
[32m+[m[32m<P><STRONG><a name="[74]"></a>_sys_exit</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, sys_exit.o(.text))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[73]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_exit_exit[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[74]"></a>GPIO_WriteBit</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, stm32f10x_gpio.o(i.GPIO_WriteBit))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[91]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Stop[m
[31m-<LI><a href="#[90]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Start[m
[31m-<LI><a href="#[8f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_SendByte[m
[31m-<LI><a href="#[8e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
[31m-<LI><a href="#[75]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;FloodLED_RightTurn[m
[31m-<LI><a href="#[73]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;FloodLED_LeftTurn[m
[31m-</UL>[m
[32m+[m[32m<P><STRONG><a name="[102]"></a>__I$use$semihosting</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, use_no_semi.o(.text), UNUSED)[m
 [m
[31m-<P><STRONG><a name="[6]"></a>HardFault_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.HardFault_Handler))[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m<P><STRONG><a name="[103]"></a>__use_no_semihosting_swi</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, use_no_semi.o(.text), UNUSED)[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[104]"></a>__semihosting_library_function</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, indicate_semi.o(.text), UNUSED)[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[82]"></a>_btod_d2e</STRONG> (Thumb, 62 bytes, Stack size 0 bytes, btod.o(CL$$btod_d2e))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[94]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_d2e_norm_op1[m
 </UL>[m
[31m-<P><STRONG><a name="[77]"></a>Key_Check</STRONG> (Thumb, 32 bytes, Stack size 0 bytes, key.o(i.Key_Check))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Option[m
[31m-<LI><a href="#[76]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_GetNum[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[76]"></a>Key_GetNum</STRONG> (Thumb, 62 bytes, Stack size 4 bytes, key.o(i.Key_GetNum))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 4<LI>Call Chain = Key_GetNum[m
[32m+[m[32m<P><STRONG><a name="[95]"></a>_d2e_denorm_low</STRONG> (Thumb, 70 bytes, Stack size 0 bytes, btod.o(CL$$btod_d2e_denorm_low))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[94]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_d2e_norm_op1[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[77]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Check[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[94]"></a>_d2e_norm_op1</STRONG> (Thumb, 96 bytes, Stack size 0 bytes, btod.o(CL$$btod_d2e_norm_op1))[m
[32m+[m[32m<BR><BR>[Calls]<UL><LI><a href="#[95]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_d2e_denorm_low[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[82]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_d2e[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[78]"></a>Key_GetState</STRONG> (Thumb, 80 bytes, Stack size 8 bytes, key.o(i.Key_GetState))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = Key_GetState[m
[32m+[m[32m<P><STRONG><a name="[96]"></a>__btod_div_common</STRONG> (Thumb, 696 bytes, Stack size 24 bytes, btod.o(CL$$btod_div_common))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = __btod_div_common[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[6a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_ReadInputDataBit[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[83]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[7a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Tick[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[97]"></a>_e2e</STRONG> (Thumb, 220 bytes, Stack size 24 bytes, btod.o(CL$$btod_e2e))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = _e2e[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
[32m+[m[32m<LI><a href="#[83]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[79]"></a>Key_Init</STRONG> (Thumb, 74 bytes, Stack size 8 bytes, key.o(i.Key_Init))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 32<LI>Call Chain = Key_Init &rArr; GPIO_Init[m
[32m+[m[32m<P><STRONG><a name="[83]"></a>_btod_ediv</STRONG> (Thumb, 42 bytes, Stack size 28 bytes, btod.o(CL$$btod_ediv))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 52<LI>Call Chain = _btod_ediv &rArr; _e2e[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[6e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[31m-<LI><a href="#[6d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[97]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_e2e[m
[32m+[m[32m<LI><a href="#[96]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__btod_div_common[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<LI><a href="#[81]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_etento[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[7a]"></a>Key_Tick</STRONG> (Thumb, 660 bytes, Stack size 8 bytes, key.o(i.Key_Tick))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = Key_Tick &rArr; Key_GetState[m
[32m+[m[32m<P><STRONG><a name="[84]"></a>_btod_emul</STRONG> (Thumb, 42 bytes, Stack size 28 bytes, btod.o(CL$$btod_emul))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 52<LI>Call Chain = _btod_emul &rArr; _e2e[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[78]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_GetState[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[98]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__btod_mult_common[m
[32m+[m[32m<LI><a href="#[97]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_e2e[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[2a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM2_IRQHandler[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fp_digits[m
[32m+[m[32m<LI><a href="#[81]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_etento[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a2]"></a>LED_DirSet</STRONG> (Thumb, 6 bytes, Stack size 0 bytes, led.o(i.LED_DirSet))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<P><STRONG><a name="[98]"></a>__btod_mult_common</STRONG> (Thumb, 580 bytes, Stack size 16 bytes, btod.o(CL$$btod_mult_common))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = __btod_mult_common[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[7b]"></a>LED_Init</STRONG> (Thumb, 50 bytes, Stack size 8 bytes, led.o(i.LED_Init))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 32<LI>Call Chain = LED_Init &rArr; GPIO_Init[m
[32m+[m[32m<P><STRONG><a name="[8]"></a>BusFault_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.BusFault_Handler))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m</UL>[m
[32m+[m[32m<P><STRONG><a name="[b]"></a>DebugMon_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.DebugMon_Handler))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[7c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_SetBits[m
[31m-<LI><a href="#[6e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[31m-<LI><a href="#[6d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
[32m+[m[32m<P><STRONG><a name="[99]"></a>Encoder_Get</STRONG> (Thumb, 22 bytes, Stack size 8 bytes, encoder.o(i.Encoder_Get))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = Encoder_Get[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[9b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_SetCounter[m
[32m+[m[32m<LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_GetCounter[m
 </UL>[m
[31m-[m
[31m-<P><STRONG><a name="[a4]"></a>LED_SpeedSet</STRONG> (Thumb, 12 bytes, Stack size 0 bytes, led.o(i.LED_SpeedSet))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[2a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM2_IRQHandler[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[7d]"></a>LED_Tick</STRONG> (Thumb, 48 bytes, Stack size 8 bytes, led.o(i.LED_Tick))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = LED_Tick &rArr; FloodLED_RightTurn[m
[32m+[m[32m<P><STRONG><a name="[9c]"></a>Encoder_Init</STRONG> (Thumb, 152 bytes, Stack size 32 bytes, encoder.o(i.Encoder_Init))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 68<LI>Call Chain = Encoder_Init &rArr; TIM_ICInit &rArr; TI4_Config[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[75]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;FloodLED_RightTurn[m
[31m-<LI><a href="#[73]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;FloodLED_LeftTurn[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[9f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[32m+[m[32m<LI><a href="#[9e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
[32m+[m[32m<LI><a href="#[9d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB1PeriphClockCmd[m
[32m+[m[32m<LI><a href="#[a0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_TimeBaseInit[m
[32m+[m[32m<LI><a href="#[a1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICStructInit[m
[32m+[m[32m<LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
[32m+[m[32m<LI><a href="#[a3]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_EncoderInterfaceConfig[m
[32m+[m[32m<LI><a href="#[a4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_Cmd[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[2a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM2_IRQHandler[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[7]"></a>MemManage_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.MemManage_Handler))[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[31m-</UL>[m
[31m-<P><STRONG><a name="[7e]"></a>Menu_Init</STRONG> (Thumb, 230 bytes, Stack size 328 bytes, menu.o(i.Menu_Init))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 336<LI>Call Chain = Menu_Init &rArr; __aeabi_memcpy4[m
[32m+[m[32m<P><STRONG><a name="[9f]"></a>GPIO_Init</STRONG> (Thumb, 278 bytes, Stack size 24 bytes, stm32f10x_gpio.o(i.GPIO_Init))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = GPIO_Init[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[7f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__aeabi_memcpy4[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
[32m+[m[32m<LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<LI><a href="#[ac]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
[32m+[m[32m<LI><a href="#[a7]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Init[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[a6]"></a>GPIO_ReadInputDataBit</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, stm32f10x_gpio.o(i.GPIO_ReadInputDataBit))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[a5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_GetState[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a1]"></a>Menu_LED_Direction</STRONG> (Thumb, 14 bytes, Stack size 0 bytes, menu.o(i.Menu_LED_Direction))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<P><STRONG><a name="[ad]"></a>GPIO_WriteBit</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, stm32f10x_gpio.o(i.GPIO_WriteBit))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[b0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Stop[m
[32m+[m[32m<LI><a href="#[af]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Start[m
[32m+[m[32m<LI><a href="#[ae]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_SendByte[m
[32m+[m[32m<LI><a href="#[ac]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a3]"></a>Menu_LED_Speed</STRONG> (Thumb, 14 bytes, Stack size 0 bytes, menu.o(i.Menu_LED_Speed))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<P><STRONG><a name="[6]"></a>HardFault_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.HardFault_Handler))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m</UL>[m
[32m+[m[32m<P><STRONG><a name="[a5]"></a>Key_GetState</STRONG> (Thumb, 80 bytes, Stack size 8 bytes, key.o(i.Key_GetState))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = Key_GetState[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[a6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_ReadInputDataBit[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[a8]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Tick[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[80]"></a>Menu_Option</STRONG> (Thumb, 382 bytes, Stack size 16 bytes, menu.o(i.Menu_Option))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 232 + Unknown Stack Size[m
[31m-<LI>Call Chain = Menu_Option &rArr; Menu_Show &rArr; OLED_ShowFloatNum &rArr; __2sprintf &rArr; _printf_char_common &rArr; __printf[m
[32m+[m[32m<P><STRONG><a name="[a7]"></a>Key_Init</STRONG> (Thumb, 74 bytes, Stack size 8 bytes, key.o(i.Key_Init))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 32<LI>Call Chain = Key_Init &rArr; GPIO_Init[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[82]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;strcmp[m
[31m-<LI><a href="#[83]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Get[m
[31m-<LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[31m-<LI><a href="#[81]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Clear[m
[31m-<LI><a href="#[77]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Check[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[9f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[32m+[m[32m<LI><a href="#[9e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[84]"></a>Menu_Show</STRONG> (Thumb, 266 bytes, Stack size 32 bytes, menu.o(i.Menu_Show))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 216 + Unknown Stack Size[m
[31m-<LI>Call Chain = Menu_Show &rArr; OLED_ShowFloatNum &rArr; __2sprintf &rArr; _printf_char_common &rArr; __printf[m
[32m+[m[32m<P><STRONG><a name="[a8]"></a>Key_Tick</STRONG> (Thumb, 660 bytes, Stack size 8 bytes, key.o(i.Key_Tick))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = Key_Tick &rArr; Key_GetState[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[89]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__aeabi_i2f[m
[31m-<LI><a href="#[8a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__aeabi_fdiv[m
[31m-<LI><a href="#[82]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;strcmp[m
[31m-<LI><a href="#[87]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Numlen[m
[31m-<LI><a href="#[85]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowString[m
[31m-<LI><a href="#[88]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowNum[m
[31m-<LI><a href="#[8b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowFloatNum[m
[31m-<LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[a5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_GetState[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[31m-<LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Option[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[2a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM2_IRQHandler[m
 </UL>[m
 [m
[32m+[m[32m<P><STRONG><a name="[7]"></a>MemManage_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.MemManage_Handler))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m</UL>[m
 <P><STRONG><a name="[5]"></a>NMI_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.NMI_Handler))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[31m-<P><STRONG><a name="[72]"></a>NVIC_Init</STRONG> (Thumb, 100 bytes, Stack size 16 bytes, misc.o(i.NVIC_Init))[m
[32m+[m[32m<P><STRONG><a name="[ba]"></a>NVIC_Init</STRONG> (Thumb, 100 bytes, Stack size 16 bytes, misc.o(i.NVIC_Init))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = NVIC_Init[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[31m-<LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[31m-</UL>[m
[31m-[m
[31m-<P><STRONG><a name="[71]"></a>NVIC_PriorityGroupConfig</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, misc.o(i.NVIC_PriorityGroupConfig))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[31m-<LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
[32m+[m[32m<LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[87]"></a>Numlen</STRONG> (Thumb, 28 bytes, Stack size 0 bytes, menu.o(i.Numlen))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[32m+[m[32m<P><STRONG><a name="[b9]"></a>NVIC_PriorityGroupConfig</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, misc.o(i.NVIC_PriorityGroupConfig))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
[32m+[m[32m<LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[81]"></a>OLED_Clear</STRONG> (Thumb, 42 bytes, Stack size 16 bytes, oled.o(i.OLED_Clear))[m
[32m+[m[32m<P><STRONG><a name="[a9]"></a>OLED_Clear</STRONG> (Thumb, 42 bytes, Stack size 16 bytes, oled.o(i.OLED_Clear))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 56<LI>Call Chain = OLED_Clear &rArr; OLED_SetCursor &rArr; OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[31m-<LI><a href="#[8c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_SetCursor[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[ab]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[32m+[m[32m<LI><a href="#[aa]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_SetCursor[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Option[m
[31m-<LI><a href="#[92]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[8e]"></a>OLED_I2C_Init</STRONG> (Thumb, 76 bytes, Stack size 8 bytes, oled.o(i.OLED_I2C_Init))[m
[32m+[m[32m<P><STRONG><a name="[ac]"></a>OLED_I2C_Init</STRONG> (Thumb, 76 bytes, Stack size 8 bytes, oled.o(i.OLED_I2C_Init))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 32<LI>Call Chain = OLED_I2C_Init &rArr; GPIO_Init[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[74]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
[31m-<LI><a href="#[6e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[31m-<LI><a href="#[6d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[ad]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
[32m+[m[32m<LI><a href="#[9f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[32m+[m[32m<LI><a href="#[9e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[92]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[8f]"></a>OLED_I2C_SendByte</STRONG> (Thumb, 88 bytes, Stack size 16 bytes, oled.o(i.OLED_I2C_SendByte))[m
[32m+[m[32m<P><STRONG><a name="[ae]"></a>OLED_I2C_SendByte</STRONG> (Thumb, 88 bytes, Stack size 16 bytes, oled.o(i.OLED_I2C_SendByte))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[74]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[ad]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[31m-<LI><a href="#[93]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[ab]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[32m+[m[32m<LI><a href="#[b2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[90]"></a>OLED_I2C_Start</STRONG> (Thumb, 48 bytes, Stack size 8 bytes, oled.o(i.OLED_I2C_Start))[m
[32m+[m[32m<P><STRONG><a name="[af]"></a>OLED_I2C_Start</STRONG> (Thumb, 48 bytes, Stack size 8 bytes, oled.o(i.OLED_I2C_Start))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = OLED_I2C_Start[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[74]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[ad]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[31m-<LI><a href="#[93]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[ab]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[32m+[m[32m<LI><a href="#[b2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[91]"></a>OLED_I2C_Stop</STRONG> (Thumb, 36 bytes, Stack size 8 bytes, oled.o(i.OLED_I2C_Stop))[m
[32m+[m[32m<P><STRONG><a name="[b0]"></a>OLED_I2C_Stop</STRONG> (Thumb, 36 bytes, Stack size 8 bytes, oled.o(i.OLED_I2C_Stop))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = OLED_I2C_Stop[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[74]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[ad]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_WriteBit[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[31m-<LI><a href="#[93]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[ab]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[32m+[m[32m<LI><a href="#[b2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[92]"></a>OLED_Init</STRONG> (Thumb, 174 bytes, Stack size 16 bytes, oled.o(i.OLED_Init))[m
[32m+[m[32m<P><STRONG><a name="[b1]"></a>OLED_Init</STRONG> (Thumb, 174 bytes, Stack size 16 bytes, oled.o(i.OLED_Init))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 72<LI>Call Chain = OLED_Init &rArr; OLED_Clear &rArr; OLED_SetCursor &rArr; OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[93]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
[31m-<LI><a href="#[8e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
[31m-<LI><a href="#[81]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Clear[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[b2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
[32m+[m[32m<LI><a href="#[ac]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
[32m+[m[32m<LI><a href="#[a9]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Clear[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[95]"></a>OLED_Pow</STRONG> (Thumb, 20 bytes, Stack size 8 bytes, oled.o(i.OLED_Pow))[m
[32m+[m[32m<P><STRONG><a name="[b5]"></a>OLED_Pow</STRONG> (Thumb, 20 bytes, Stack size 8 bytes, oled.o(i.OLED_Pow))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = OLED_Pow[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[88]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowNum[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowSignedNum[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[8c]"></a>OLED_SetCursor</STRONG> (Thumb, 34 bytes, Stack size 16 bytes, oled.o(i.OLED_SetCursor))[m
[32m+[m[32m<P><STRONG><a name="[aa]"></a>OLED_SetCursor</STRONG> (Thumb, 34 bytes, Stack size 16 bytes, oled.o(i.OLED_SetCursor))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 40<LI>Call Chain = OLED_SetCursor &rArr; OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[93]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[b2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteCommand[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[31m-<LI><a href="#[81]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Clear[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b3]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[32m+[m[32m<LI><a href="#[a9]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Clear[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[86]"></a>OLED_ShowChar</STRONG> (Thumb, 110 bytes, Stack size 24 bytes, oled.o(i.OLED_ShowChar))[m
[32m+[m[32m<P><STRONG><a name="[b3]"></a>OLED_ShowChar</STRONG> (Thumb, 110 bytes, Stack size 24 bytes, oled.o(i.OLED_ShowChar))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 64<LI>Call Chain = OLED_ShowChar &rArr; OLED_SetCursor &rArr; OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[31m-<LI><a href="#[8c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_SetCursor[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[ab]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_WriteData[m
[32m+[m[32m<LI><a href="#[aa]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_SetCursor[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[31m-<LI><a href="#[85]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowString[m
[31m-<LI><a href="#[88]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowNum[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowSignedNum[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[8b]"></a>OLED_ShowFloatNum</STRONG> (Thumb, 50 bytes, Stack size 56 bytes, oled.o(i.OLED_ShowFloatNum))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 184 + Unknown Stack Size[m
[31m-<LI>Call Chain = OLED_ShowFloatNum &rArr; __2sprintf &rArr; _printf_char_common &rArr; __printf[m
[32m+[m[32m<P><STRONG><a name="[b4]"></a>OLED_ShowSignedNum</STRONG> (Thumb, 102 bytes, Stack size 32 bytes, oled.o(i.OLED_ShowSignedNum))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 96<LI>Call Chain = OLED_ShowSignedNum &rArr; OLED_ShowChar &rArr; OLED_SetCursor &rArr; OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[94]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__aeabi_f2d[m
[31m-<LI><a href="#[53]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__2sprintf[m
[31m-<LI><a href="#[85]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowString[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[b3]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[32m+[m[32m<LI><a href="#[b5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Pow[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[88]"></a>OLED_ShowNum</STRONG> (Thumb, 68 bytes, Stack size 32 bytes, oled.o(i.OLED_ShowNum))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 96<LI>Call Chain = OLED_ShowNum &rArr; OLED_ShowChar &rArr; OLED_SetCursor &rArr; OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
[32m+[m[32m<P><STRONG><a name="[b2]"></a>OLED_WriteCommand</STRONG> (Thumb, 32 bytes, Stack size 8 bytes, oled.o(i.OLED_WriteCommand))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[31m-<LI><a href="#[95]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Pow[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[b0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Stop[m
[32m+[m[32m<LI><a href="#[af]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Start[m
[32m+[m[32m<LI><a href="#[ae]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[aa]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_SetCursor[m
[32m+[m[32m<LI><a href="#[b1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[85]"></a>OLED_ShowString</STRONG> (Thumb, 40 bytes, Stack size 24 bytes, oled.o(i.OLED_ShowString))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 88<LI>Call Chain = OLED_ShowString &rArr; OLED_ShowChar &rArr; OLED_SetCursor &rArr; OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
[32m+[m[32m<P><STRONG><a name="[ab]"></a>OLED_WriteData</STRONG> (Thumb, 32 bytes, Stack size 8 bytes, oled.o(i.OLED_WriteData))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = OLED_WriteData &rArr; OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[b0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Stop[m
[32m+[m[32m<LI><a href="#[af]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Start[m
[32m+[m[32m<LI><a href="#[ae]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_SendByte[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[31m-<LI><a href="#[8b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowFloatNum[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b3]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[32m+[m[32m<LI><a href="#[a9]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Clear[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[93]"></a>OLED_WriteCommand</STRONG> (Thumb, 32 bytes, Stack size 8 bytes, oled.o(i.OLED_WriteCommand))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = OLED_WriteCommand &rArr; OLED_I2C_SendByte[m
[31m-</UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[91]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Stop[m
[31m-<LI><a href="#[90]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Start[m
[31m-<LI><a href="#[8f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_SendByte[m
[32m+[m[32m<P><STRONG><a name="[c]"></a>PendSV_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.PendSV_Handler))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[8c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_SetCursor[m
[31m-<LI><a href="#[92]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
[32m+[m[32m<P><STRONG><a name="[9d]"></a>RCC_APB1PeriphClockCmd</STRONG> (Thumb, 26 bytes, Stack size 0 bytes, stm32f10x_rcc.o(i.RCC_APB1PeriphClockCmd))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[8d]"></a>OLED_WriteData</STRONG> (Thumb, 32 bytes, Stack size 8 bytes, oled.o(i.OLED_WriteData))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = OLED_WriteData &rArr; OLED_I2C_SendByte[m
[32m+[m[32m<P><STRONG><a name="[9e]"></a>RCC_APB2PeriphClockCmd</STRONG> (Thumb, 26 bytes, Stack size 0 bytes, stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
[32m+[m[32m<LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<LI><a href="#[ac]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
[32m+[m[32m<LI><a href="#[a7]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Init[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[91]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Stop[m
[31m-<LI><a href="#[90]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Start[m
[31m-<LI><a href="#[8f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_SendByte[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[d4]"></a>RCC_GetClocksFreq</STRONG> (Thumb, 192 bytes, Stack size 12 bytes, stm32f10x_rcc.o(i.RCC_GetClocksFreq))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 12<LI>Call Chain = RCC_GetClocksFreq[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowChar[m
[31m-<LI><a href="#[81]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Clear[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b7]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[c]"></a>PendSV_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.PendSV_Handler))[m
[32m+[m[32m<P><STRONG><a name="[a]"></a>SVC_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.SVC_Handler))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[31m-<P><STRONG><a name="[9b]"></a>RCC_APB1PeriphClockCmd</STRONG> (Thumb, 26 bytes, Stack size 0 bytes, stm32f10x_rcc.o(i.RCC_APB1PeriphClockCmd))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<P><STRONG><a name="[b6]"></a>Serial_Init</STRONG> (Thumb, 174 bytes, Stack size 32 bytes, serial.o(i.Serial_Init))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 100<LI>Call Chain = Serial_Init &rArr; USART_Init &rArr; RCC_GetClocksFreq[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[b7]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_Init[m
[32m+[m[32m<LI><a href="#[b8]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_ITConfig[m
[32m+[m[32m<LI><a href="#[bb]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_Cmd[m
[32m+[m[32m<LI><a href="#[9f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;GPIO_Init[m
[32m+[m[32m<LI><a href="#[9e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB2PeriphClockCmd[m
[32m+[m[32m<LI><a href="#[b9]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_PriorityGroupConfig[m
[32m+[m[32m<LI><a href="#[ba]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_Init[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[bc]"></a>Serial_Printf</STRONG> (Thumb, 36 bytes, Stack size 128 bytes, serial.o(i.Serial_Printf))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 248 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = Serial_Printf &rArr; vsprintf &rArr; _printf_char_common &rArr; __printf[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[75]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;vsprintf[m
[32m+[m[32m<LI><a href="#[bd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_SendString[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[6d]"></a>RCC_APB2PeriphClockCmd</STRONG> (Thumb, 26 bytes, Stack size 0 bytes, stm32f10x_rcc.o(i.RCC_APB2PeriphClockCmd))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[31m-<LI><a href="#[8e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_I2C_Init[m
[31m-<LI><a href="#[79]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Init[m
[31m-<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_Init[m
[32m+[m[32m<P><STRONG><a name="[be]"></a>Serial_SendByte</STRONG> (Thumb, 28 bytes, Stack size 8 bytes, serial.o(i.Serial_SendByte))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = Serial_SendByte[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[bf]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_SendData[m
[32m+[m[32m<LI><a href="#[c0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_GetFlagStatus[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[bd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_SendString[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a]"></a>SVC_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.SVC_Handler))[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m<P><STRONG><a name="[bd]"></a>Serial_SendString</STRONG> (Thumb, 26 bytes, Stack size 16 bytes, serial.o(i.Serial_SendString))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = Serial_SendString &rArr; Serial_SendByte[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[be]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_SendByte[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[bc]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Printf[m
 </UL>[m
[32m+[m
 <P><STRONG><a name="[d]"></a>SysTick_Handler</STRONG> (Thumb, 2 bytes, Stack size 0 bytes, stm32f10x_it.o(i.SysTick_Handler))[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
 <P><STRONG><a name="[39]"></a>SystemInit</STRONG> (Thumb, 78 bytes, Stack size 8 bytes, system_stm32f10x.o(i.SystemInit))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 28<LI>Call Chain = SystemInit &rArr; SetSysClock &rArr; SetSysClockTo72[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[96]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;SetSysClock[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[c1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;SetSysClock[m
 </UL>[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(.text)[m
 </UL>[m
[31m-<P><STRONG><a name="[2a]"></a>TIM2_IRQHandler</STRONG> (Thumb, 32 bytes, Stack size 8 bytes, main.o(i.TIM2_IRQHandler))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 32<LI>Call Chain = TIM2_IRQHandler &rArr; LED_Tick &rArr; FloodLED_RightTurn[m
[32m+[m[32m<P><STRONG><a name="[2a]"></a>TIM2_IRQHandler</STRONG> (Thumb, 60 bytes, Stack size 8 bytes, main.o(i.TIM2_IRQHandler))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = TIM2_IRQHandler &rArr; Key_Tick &rArr; Key_GetState[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[98]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_GetITStatus[m
[31m-<LI><a href="#[99]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ClearITPendingBit[m
[31m-<LI><a href="#[7a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Tick[m
[31m-<LI><a href="#[7d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_Tick[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[c3]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_GetITStatus[m
[32m+[m[32m<LI><a href="#[c4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ClearITPendingBit[m
[32m+[m[32m<LI><a href="#[99]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Get[m
[32m+[m[32m<LI><a href="#[a8]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Tick[m
 </UL>[m
 <BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
 </UL>[m
[31m-<P><STRONG><a name="[9e]"></a>TIM_ClearFlag</STRONG> (Thumb, 6 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_ClearFlag))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<P><STRONG><a name="[cf]"></a>TIM_ClearFlag</STRONG> (Thumb, 6 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_ClearFlag))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[99]"></a>TIM_ClearITPendingBit</STRONG> (Thumb, 6 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_ClearITPendingBit))[m
[32m+[m[32m<P><STRONG><a name="[c4]"></a>TIM_ClearITPendingBit</STRONG> (Thumb, 6 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_ClearITPendingBit))[m
 <BR><BR>[Called By]<UL><LI><a href="#[2a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM2_IRQHandler[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a0]"></a>TIM_Cmd</STRONG> (Thumb, 24 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_Cmd))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<P><STRONG><a name="[a4]"></a>TIM_Cmd</STRONG> (Thumb, 24 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_Cmd))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[a3]"></a>TIM_EncoderInterfaceConfig</STRONG> (Thumb, 66 bytes, Stack size 20 bytes, stm32f10x_tim.o(i.TIM_EncoderInterfaceConfig))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 20<LI>Call Chain = TIM_EncoderInterfaceConfig[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[9a]"></a>TIM_GetCounter</STRONG> (Thumb, 6 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_GetCounter))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[99]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Get[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[98]"></a>TIM_GetITStatus</STRONG> (Thumb, 34 bytes, Stack size 12 bytes, stm32f10x_tim.o(i.TIM_GetITStatus))[m
[32m+[m[32m<P><STRONG><a name="[c3]"></a>TIM_GetITStatus</STRONG> (Thumb, 34 bytes, Stack size 12 bytes, stm32f10x_tim.o(i.TIM_GetITStatus))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 12<LI>Call Chain = TIM_GetITStatus[m
 </UL>[m
 <BR>[Called By]<UL><LI><a href="#[2a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM2_IRQHandler[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[9f]"></a>TIM_ITConfig</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_ITConfig))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<P><STRONG><a name="[a2]"></a>TIM_ICInit</STRONG> (Thumb, 150 bytes, Stack size 16 bytes, stm32f10x_tim.o(i.TIM_ICInit))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 36<LI>Call Chain = TIM_ICInit &rArr; TI4_Config[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[cc]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_SetIC4Prescaler[m
[32m+[m[32m<LI><a href="#[ca]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_SetIC3Prescaler[m
[32m+[m[32m<LI><a href="#[c8]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_SetIC2Prescaler[m
[32m+[m[32m<LI><a href="#[c6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_SetIC1Prescaler[m
[32m+[m[32m<LI><a href="#[cb]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TI4_Config[m
[32m+[m[32m<LI><a href="#[c9]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TI3_Config[m
[32m+[m[32m<LI><a href="#[c7]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TI2_Config[m
[32m+[m[32m<LI><a href="#[c5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TI1_Config[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[9c]"></a>TIM_InternalClockConfig</STRONG> (Thumb, 12 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_InternalClockConfig))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<P><STRONG><a name="[a1]"></a>TIM_ICStructInit</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_ICStructInit))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[9d]"></a>TIM_TimeBaseInit</STRONG> (Thumb, 122 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_TimeBaseInit))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<P><STRONG><a name="[d0]"></a>TIM_ITConfig</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_ITConfig))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[9a]"></a>Timer_Init</STRONG> (Thumb, 124 bytes, Stack size 24 bytes, timer.o(i.Timer_Init))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 40<LI>Call Chain = Timer_Init &rArr; NVIC_Init[m
[32m+[m[32m<P><STRONG><a name="[ce]"></a>TIM_InternalClockConfig</STRONG> (Thumb, 12 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_InternalClockConfig))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[9b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB1PeriphClockCmd[m
[31m-<LI><a href="#[9d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_TimeBaseInit[m
[31m-<LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_InternalClockConfig[m
[31m-<LI><a href="#[9f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ITConfig[m
[31m-<LI><a href="#[a0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_Cmd[m
[31m-<LI><a href="#[9e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ClearFlag[m
[31m-<LI><a href="#[71]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_PriorityGroupConfig[m
[31m-<LI><a href="#[72]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_Init[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[9b]"></a>TIM_SetCounter</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_SetCounter))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[99]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Get[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[c6]"></a>TIM_SetIC1Prescaler</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_SetIC1Prescaler))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[9]"></a>UsageFault_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.UsageFault_Handler))[m
[31m-<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m<P><STRONG><a name="[c8]"></a>TIM_SetIC2Prescaler</STRONG> (Thumb, 26 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_SetIC2Prescaler))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[ca]"></a>TIM_SetIC3Prescaler</STRONG> (Thumb, 18 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_SetIC3Prescaler))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
 </UL>[m
[31m-<P><STRONG><a name="[5f]"></a>__ARM_fpclassify</STRONG> (Thumb, 40 bytes, Stack size 0 bytes, fpclassify.o(i.__ARM_fpclassify))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[5e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[cc]"></a>TIM_SetIC4Prescaler</STRONG> (Thumb, 26 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_SetIC4Prescaler))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[56]"></a>_is_digit</STRONG> (Thumb, 14 bytes, Stack size 0 bytes, __printf_wp.o(i._is_digit))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[55]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__printf[m
[32m+[m[32m<P><STRONG><a name="[a0]"></a>TIM_TimeBaseInit</STRONG> (Thumb, 122 bytes, Stack size 0 bytes, stm32f10x_tim.o(i.TIM_TimeBaseInit))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[4d]"></a>main</STRONG> (Thumb, 66 bytes, Stack size 0 bytes, main.o(i.main))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 336 + Unknown Stack Size[m
[31m-<LI>Call Chain = main &rArr; Menu_Init &rArr; __aeabi_memcpy4[m
[32m+[m[32m<P><STRONG><a name="[cd]"></a>Timer_Init</STRONG> (Thumb, 124 bytes, Stack size 24 bytes, timer.o(i.Timer_Init))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 40<LI>Call Chain = Timer_Init &rArr; NVIC_Init[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[6c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[31m-<LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[31m-<LI><a href="#[80]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Option[m
[31m-<LI><a href="#[a3]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_LED_Speed[m
[31m-<LI><a href="#[a1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_LED_Direction[m
[31m-<LI><a href="#[7e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Init[m
[31m-<LI><a href="#[92]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
[31m-<LI><a href="#[79]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Init[m
[31m-<LI><a href="#[76]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_GetNum[m
[31m-<LI><a href="#[a4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_SpeedSet[m
[31m-<LI><a href="#[7b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_Init[m
[31m-<LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;LED_DirSet[m
[31m-<LI><a href="#[9a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[9d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_APB1PeriphClockCmd[m
[32m+[m[32m<LI><a href="#[a0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_TimeBaseInit[m
[32m+[m[32m<LI><a href="#[ce]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_InternalClockConfig[m
[32m+[m[32m<LI><a href="#[d0]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ITConfig[m
[32m+[m[32m<LI><a href="#[a4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_Cmd[m
[32m+[m[32m<LI><a href="#[cf]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ClearFlag[m
[32m+[m[32m<LI><a href="#[b9]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_PriorityGroupConfig[m
[32m+[m[32m<LI><a href="#[ba]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;NVIC_Init[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[4c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_main[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6f]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[47]"></a>_get_lc_numeric</STRONG> (Thumb, 44 bytes, Stack size 8 bytes, lc_numeric_c.o(locale$$code))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = _get_lc_numeric[m
[32m+[m[32m<P><STRONG><a name="[33]"></a>USART1_IRQHandler</STRONG> (Thumb, 140 bytes, Stack size 8 bytes, serial.o(i.USART1_IRQHandler))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 24<LI>Call Chain = USART1_IRQHandler &rArr; USART_GetITStatus[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[d2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_ReceiveData[m
[32m+[m[32m<LI><a href="#[d1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_GetITStatus[m
[32m+[m[32m<LI><a href="#[d3]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART_ClearITPendingBit[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m</UL>[m
[32m+[m[32m<P><STRONG><a name="[d3]"></a>USART_ClearITPendingBit</STRONG> (Thumb, 30 bytes, Stack size 8 bytes, stm32f10x_usart.o(i.USART_ClearITPendingBit))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = USART_ClearITPendingBit[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[82]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;strcmp[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[33]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART1_IRQHandler[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[46]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_init_lc_numeric_2[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[bb]"></a>USART_Cmd</STRONG> (Thumb, 24 bytes, Stack size 0 bytes, stm32f10x_usart.o(i.USART_Cmd))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a7]"></a>__fpl_dretinf</STRONG> (Thumb, 12 bytes, Stack size 0 bytes, dretinf.o(x$fpl$dretinf), UNUSED)[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[a5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_f2d[m
[32m+[m[32m<P><STRONG><a name="[c0]"></a>USART_GetFlagStatus</STRONG> (Thumb, 26 bytes, Stack size 0 bytes, stm32f10x_usart.o(i.USART_GetFlagStatus))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[be]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_SendByte[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[94]"></a>__aeabi_f2d</STRONG> (Thumb, 0 bytes, Stack size 16 bytes, f2d.o(x$fpl$f2d))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = __aeabi_f2d[m
[32m+[m[32m<P><STRONG><a name="[d1]"></a>USART_GetITStatus</STRONG> (Thumb, 84 bytes, Stack size 16 bytes, stm32f10x_usart.o(i.USART_GetITStatus))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = USART_GetITStatus[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[8b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowFloatNum[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[33]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART1_IRQHandler[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a5]"></a>_f2d</STRONG> (Thumb, 86 bytes, Stack size 16 bytes, f2d.o(x$fpl$f2d), UNUSED)[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[a6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__fpl_fnaninf[m
[31m-<LI><a href="#[a7]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__fpl_dretinf[m
[32m+[m[32m<P><STRONG><a name="[b8]"></a>USART_ITConfig</STRONG> (Thumb, 74 bytes, Stack size 20 bytes, stm32f10x_usart.o(i.USART_ITConfig))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 20<LI>Call Chain = USART_ITConfig[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[8a]"></a>__aeabi_fdiv</STRONG> (Thumb, 0 bytes, Stack size 16 bytes, fdiv.o(x$fpl$fdiv))[m
[31m-<BR><BR>[Stack]<UL><LI>Max Depth = 16<LI>Call Chain = __aeabi_fdiv[m
[32m+[m[32m<P><STRONG><a name="[b7]"></a>USART_Init</STRONG> (Thumb, 210 bytes, Stack size 56 bytes, stm32f10x_usart.o(i.USART_Init))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 68<LI>Call Chain = USART_Init &rArr; RCC_GetClocksFreq[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[d4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;RCC_GetClocksFreq[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[d2]"></a>USART_ReceiveData</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, stm32f10x_usart.o(i.USART_ReceiveData))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[33]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;USART1_IRQHandler[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a8]"></a>_fdiv</STRONG> (Thumb, 384 bytes, Stack size 16 bytes, fdiv.o(x$fpl$fdiv), UNUSED)[m
[31m-<BR><BR>[Calls]<UL><LI><a href="#[a9]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__fpl_fretinf[m
[31m-<LI><a href="#[a6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__fpl_fnaninf[m
[32m+[m[32m<P><STRONG><a name="[bf]"></a>USART_SendData</STRONG> (Thumb, 8 bytes, Stack size 0 bytes, stm32f10x_usart.o(i.USART_SendData))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[be]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_SendByte[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[89]"></a>__aeabi_i2f</STRONG> (Thumb, 0 bytes, Stack size 0 bytes, fflt_clz.o(x$fpl$fflt))[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Menu_Show[m
[32m+[m[32m<P><STRONG><a name="[9]"></a>UsageFault_Handler</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, stm32f10x_it.o(i.UsageFault_Handler))[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> startup_stm32f10x_md.o(RESET)[m
[32m+[m[32m</UL>[m
[32m+[m[32m<P><STRONG><a name="[87]"></a>__ARM_fpclassify</STRONG> (Thumb, 40 bytes, Stack size 0 bytes, fpclassify.o(i.__ARM_fpclassify))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_hex_real[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[da]"></a>_fflt</STRONG> (Thumb, 48 bytes, Stack size 0 bytes, fflt_clz.o(x$fpl$fflt), UNUSED)[m
[32m+[m[32m<P><STRONG><a name="[78]"></a>_is_digit</STRONG> (Thumb, 14 bytes, Stack size 0 bytes, __printf_wp.o(i._is_digit))[m
[32m+[m[32m<BR><BR>[Called By]<UL><LI><a href="#[77]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__printf[m
[32m+[m[32m</UL>[m
 [m
[31m-<P><STRONG><a name="[a6]"></a>__fpl_fnaninf</STRONG> (Thumb, 140 bytes, Stack size 8 bytes, fnaninf.o(x$fpl$fnaninf), UNUSED)[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[a8]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fdiv[m
[31m-<LI><a href="#[a5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_f2d[m
[32m+[m[32m<P><STRONG><a name="[6f]"></a>main</STRONG> (Thumb, 52 bytes, Stack size 0 bytes, main.o(i.main))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 248 + Unknown Stack Size[m
[32m+[m[32m<LI>Call Chain = main &rArr; Serial_Printf &rArr; vsprintf &rArr; _printf_char_common &rArr; __printf[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[bc]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Printf[m
[32m+[m[32m<LI><a href="#[b6]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Serial_Init[m
[32m+[m[32m<LI><a href="#[9c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Encoder_Init[m
[32m+[m[32m<LI><a href="#[b4]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_ShowSignedNum[m
[32m+[m[32m<LI><a href="#[b1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;OLED_Init[m
[32m+[m[32m<LI><a href="#[a7]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Key_Init[m
[32m+[m[32m<LI><a href="#[cd]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;Timer_Init[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[6e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_entry_main[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[a9]"></a>__fpl_fretinf</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, fretinf.o(x$fpl$fretinf), UNUSED)[m
[31m-<BR><BR>[Called By]<UL><LI><a href="#[a8]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_fdiv[m
[32m+[m[32m<P><STRONG><a name="[69]"></a>_get_lc_numeric</STRONG> (Thumb, 44 bytes, Stack size 8 bytes, lc_numeric_c.o(locale$$code))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = _get_lc_numeric[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[d5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;strcmp[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[68]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_init_lc_numeric_2[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[43]"></a>_printf_fp_dec</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, printf1.o(x$fpl$printf1))[m
[32m+[m[32m<P><STRONG><a name="[3d]"></a>_get_lc_ctype</STRONG> (Thumb, 44 bytes, Stack size 8 bytes, lc_ctype_c.o(locale$$code))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 8<LI>Call Chain = _get_lc_ctype[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[d5]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;strcmp[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[67]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;__rt_lib_init_lc_ctype_2[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Address Reference Count : 1]<UL><LI> rt_ctype_table.o(.text)[m
[32m+[m[32m</UL>[m
[32m+[m[32m<P><STRONG><a name="[48]"></a>_printf_fp_dec</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, printf1.o(x$fpl$printf1))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 324<LI>Call Chain = _printf_fp_dec &rArr; _printf_fp_dec_real &rArr; _fp_digits &rArr; _btod_etento &rArr; _btod_emul &rArr; _e2e[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[5e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[42]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_f[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[47]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_f[m
[32m+[m[32m<LI><a href="#[4a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_g[m
[32m+[m[32m<LI><a href="#[49]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_e[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[4c]"></a>_printf_fp_hex</STRONG> (Thumb, 4 bytes, Stack size 0 bytes, printf2.o(x$fpl$printf2))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 112<LI>Call Chain = _printf_fp_hex &rArr; _printf_fp_hex_real &rArr; _printf_fp_infnan &rArr; _printf_post_padding[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[8d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_hex_real[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[4b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_a[m
 </UL>[m
 <P>[m
 <H3>[m
 Local Symbols[m
 </H3>[m
[31m-<P><STRONG><a name="[96]"></a>SetSysClock</STRONG> (Thumb, 8 bytes, Stack size 8 bytes, system_stm32f10x.o(i.SetSysClock))[m
[32m+[m[32m<P><STRONG><a name="[c1]"></a>SetSysClock</STRONG> (Thumb, 8 bytes, Stack size 8 bytes, system_stm32f10x.o(i.SetSysClock))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 20<LI>Call Chain = SetSysClock &rArr; SetSysClockTo72[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[97]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;SetSysClockTo72[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[c2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;SetSysClockTo72[m
 </UL>[m
 <BR>[Called By]<UL><LI><a href="#[39]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;SystemInit[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[97]"></a>SetSysClockTo72</STRONG> (Thumb, 214 bytes, Stack size 12 bytes, system_stm32f10x.o(i.SetSysClockTo72))[m
[32m+[m[32m<P><STRONG><a name="[c2]"></a>SetSysClockTo72</STRONG> (Thumb, 214 bytes, Stack size 12 bytes, system_stm32f10x.o(i.SetSysClockTo72))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 12<LI>Call Chain = SetSysClockTo72[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[96]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;SetSysClock[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[c1]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;SetSysClock[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[c5]"></a>TI1_Config</STRONG> (Thumb, 108 bytes, Stack size 20 bytes, stm32f10x_tim.o(i.TI1_Config))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 20<LI>Call Chain = TI1_Config[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[c7]"></a>TI2_Config</STRONG> (Thumb, 130 bytes, Stack size 20 bytes, stm32f10x_tim.o(i.TI2_Config))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 20<LI>Call Chain = TI2_Config[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[c9]"></a>TI3_Config</STRONG> (Thumb, 122 bytes, Stack size 20 bytes, stm32f10x_tim.o(i.TI3_Config))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 20<LI>Call Chain = TI3_Config[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
[32m+[m[32m</UL>[m
[32m+[m
[32m+[m[32m<P><STRONG><a name="[cb]"></a>TI4_Config</STRONG> (Thumb, 130 bytes, Stack size 20 bytes, stm32f10x_tim.o(i.TI4_Config))[m
[32m+[m[32m<BR><BR>[Stack]<UL><LI>Max Depth = 20<LI>Call Chain = TI4_Config[m
[32m+[m[32m</UL>[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[a2]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;TIM_ICInit[m
 </UL>[m
 [m
[31m-<P><STRONG><a name="[58]"></a>_fp_digits</STRONG> (Thumb, 432 bytes, Stack size 96 bytes, _printf_fp_dec.o(.text))[m
[32m+[m[32m<P><STRONG><a name="[80]"></a>_fp_digits</STRONG> (Thumb, 432 bytes, Stack size 96 bytes, _printf_fp_dec.o(.text))[m
 <BR><BR>[Stack]<UL><LI>Max Depth = 220<LI>Call Chain = _fp_digits &rArr; _btod_etento &rArr; _btod_emul &rArr; _e2e[m
 </UL>[m
[31m-<BR>[Calls]<UL><LI><a href="#[5c]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
[31m-<LI><a href="#[5b]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
[31m-<LI><a href="#[5a]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_d2e[m
[31m-<LI><a href="#[59]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_etento[m
[31m-<LI><a href="#[5d]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_ll_udiv10[m
[32m+[m[32m<BR>[Calls]<UL><LI><a href="#[84]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_emul[m
[32m+[m[32m<LI><a href="#[83]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_ediv[m
[32m+[m[32m<LI><a href="#[82]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_d2e[m
[32m+[m[32m<LI><a href="#[81]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_btod_etento[m
[32m+[m[32m<LI><a href="#[85]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_ll_udiv10[m
 </UL>[m
[31m-<BR>[Called By]<UL><LI><a href="#[5e]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
[32m+[m[32m<BR>[Called By]<UL><LI><a href="#[86]">&gt;&gt;</a>&nbsp;&nbsp;&nbsp;_printf_fp_dec_real[m
 </UL>[m
 [m
 <P><STRONG><a name="[3c]"></a>_printf_input_char</STRONG> (Thumb, 10 bytes, Stack size 0 bytes, _printf_char_common.o(.text))[m
[1mdiff --git a/asc2/Objects/Project.lnp b/asc2/Objects/Project.lnp[m
[1mindex 11ad186..de0c702 100644[m
[1m--- a/asc2/Objects/Project.lnp[m
[1m+++ b/asc2/Objects/Project.lnp[m
[36m@@ -33,6 +33,7 @@[m
 ".\objects\menu.o"[m
 ".\objects\printer.o"[m
 ".\objects\encoder.o"[m
[32m+[m[32m".\objects\serial.o"[m
 ".\objects\main.o"[m
 ".\objects\stm32f10x_it.o"[m
 --strict --scatter ".\Objects\Project.sct"[m
[1mdiff --git a/asc2/Objects/Project_Target 1.dep b/asc2/Objects/Project_Target 1.dep[m
[1mindex 610487e..9c7bdc0 100644[m
[1m--- a/asc2/Objects/Project_Target 1.dep[m	
[1m+++ b/asc2/Objects/Project_Target 1.dep[m	
[36m@@ -968,7 +968,7 @@[m [mI (.\Library\stm32f10x_wwdg.h)(0x4D783BB4)[m
 I (.\Library\misc.h)(0x4D783BB4)[m
 I (Hardware\OLED.h)(0x68FB75E9)[m
 F (.\hardware\Printer.h)(0x68FB0AE5)()[m
[31m-F (.\hardware\Encoder.c)(0x68FB7EFB)(--c99 -c --cpu Cortex-M3 -g -O0 --apcs=interwork --split_sections -I .\Start -I .\Library -I .\User -I .\System -I .\Hardware-I.\RTE\_Target_1-ID:\Keil5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include-ID:\Keil5\ARM\CMSIS\Include-D__UVISION_VERSION="524" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER-o .\objects\encoder.o --omf_browse .\objects\encoder.crf --depend .\objects\encoder.d)[m
[32m+[m[32mF (.\hardware\Encoder.c)(0x690C7207)(--c99 -c --cpu Cortex-M3 -g -O0 --apcs=interwork --split_sections -I .\Start -I .\Library -I .\User -I .\System -I .\Hardware-I.\RTE\_Target_1-ID:\Keil5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include-ID:\Keil5\ARM\CMSIS\Include-D__UVISION_VERSION="524" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER-o .\objects\encoder.o --omf_browse .\objects\encoder.crf --depend .\objects\encoder.d)[m[41m[m
 I (.\Start\stm32f10x.h)(0x4D783CB4)[m
 I (.\Start\core_cm3.h)(0x4D523B58)[m
 I (D:\Keil5\ARM\ARMCC\include\stdint.h)(0x588B8344)[m
[36m@@ -998,7 +998,39 @@[m [mI (.\Library\stm32f10x_usart.h)(0x4D783BB4)[m
 I (.\Library\stm32f10x_wwdg.h)(0x4D783BB4)[m
 I (.\Library\misc.h)(0x4D783BB4)[m
 F (.\hardware\Encoder.h)(0x68FB8004)()[m
[31m-F (.\User\main.c)(0x68FB8B0D)(--c99 -c --cpu Cortex-M3 -g -O0 --apcs=interwork --split_sections -I .\Start -I .\Library -I .\User -I .\System -I .\Hardware-I.\RTE\_Target_1-ID:\Keil5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include-ID:\Keil5\ARM\CMSIS\Include-D__UVISION_VERSION="524" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER-o .\objects\main.o --omf_browse .\objects\main.crf --depend .\objects\main.d)[m
[32m+[m[32mF (.\Hardware\Serial.c)(0x65323DA2)(--c99 -c --cpu Cortex-M3 -g -O0 --apcs=interwork --split_sections -I .\Start -I .\Library -I .\User -I .\System -I .\Hardware-I.\RTE\_Target_1-ID:\Keil5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include-ID:\Keil5\ARM\CMSIS\Include-D__UVISION_VERSION="524" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER-o .\objects\serial.o --omf_browse .\objects\serial.crf --depend .\objects\serial.d)[m[41m[m
[32m+[m[32mI (.\Start\stm32f10x.h)(0x4D783CB4)[m[41m[m
[32m+[m[32mI (.\Start\core_cm3.h)(0x4D523B58)[m[41m[m
[32m+[m[32mI (D:\Keil5\ARM\ARMCC\include\stdint.h)(0x588B8344)[m[41m[m
[32m+[m[32mI (.\Start\system_stm32f10x.h)(0x4D783CAA)[m[41m[m
[32m+[m[32mI (.\User\stm32f10x_conf.h)(0x4D99A59E)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_adc.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_bkp.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_can.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_cec.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_crc.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_dac.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_dbgmcu.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_dma.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_exti.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_flash.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_fsmc.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_gpio.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_i2c.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_iwdg.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_pwr.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_rcc.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_rtc.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_sdio.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_spi.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_tim.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_usart.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\stm32f10x_wwdg.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (.\Library\misc.h)(0x4D783BB4)[m[41m[m
[32m+[m[32mI (D:\Keil5\ARM\ARMCC\include\stdio.h)(0x588B8344)[m[41m[m
[32m+[m[32mI (D:\Keil5\ARM\ARMCC\include\stdarg.h)(0x588B8344)[m[41m[m
[32m+[m[32mF (.\Hardware\Serial.h)(0x62E10822)()[m[41m[m
[32m+[m[32mF (.\User\main.c)(0x690CA7FC)(--c99 -c --cpu Cortex-M3 -g -O0 --apcs=interwork --split_sections -I .\Start -I .\Library -I .\User -I .\System -I .\Hardware-I.\RTE\_Target_1-ID:\Keil5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include-ID:\Keil5\ARM\CMSIS\Include-D__UVISION_VERSION="524" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER-o .\objects\main.o --omf_browse .\objects\main.crf --depend .\objects\main.d)[m[41m[m
 I (.\Start\stm32f10x.h)(0x4D783CB4)[m
 I (.\Start\core_cm3.h)(0x4D523B58)[m
 I (D:\Keil5\ARM\ARMCC\include\stdint.h)(0x588B8344)[m
[36m@@ -1031,8 +1063,9 @@[m [mI (.\Hardware\OLED.h)(0x68FB75E9)[m
 I (.\Hardware\LED.h)(0x68FB89F7)[m
 I (.\Hardware\Key.h)(0x68FC32C4)[m
 I (.\System\Timer.h)(0x68FACC45)[m
[31m-I (.\Hardware\Menu.h)(0x68FB8893)[m
 I (.\Hardware\Encoder.h)(0x68FB8004)[m
[32m+[m[32mI (.\Hardware\Serial.h)(0x62E10822)[m[41m[m
[32m+[m[32mI (D:\Keil5\ARM\ARMCC\include\stdio.h)(0x588B8344)[m[41m[m
 F (.\User\stm32f10x_conf.h)(0x4D99A59E)()[m
 F (.\User\stm32f10x_it.c)(0x4D99A59E)(--c99 -c --cpu Cortex-M3 -g -O0 --apcs=interwork --split_sections -I .\Start -I .\Library -I .\User -I .\System -I .\Hardware-I.\RTE\_Target_1-ID:\Keil5\ARM\PACK\Keil\STM32F1xx_DFP\2.2.0\Device\Include-ID:\Keil5\ARM\CMSIS\Include-D__UVISION_VERSION="524" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER-o .\objects\stm32f10x_it.o --omf_browse .\objects\stm32f10x_it.crf --depend .\objects\stm32f10x_it.d)[m
 I (User\stm32f10x_it.h)(0x4D99A59E)[m
[1mdiff --git a/asc2/Objects/encoder.crf b/asc2/Objects/encoder.crf[m
[1mindex 3c0367e..6692e78 100644[m
Binary files a/asc2/Objects/encoder.crf and b/asc2/Objects/encoder.crf differ
[1mdiff --git a/asc2/Objects/encoder.o b/asc2/Objects/encoder.o[m
[1mindex 5c5c42d..bf79331 100644[m
Binary files a/asc2/Objects/encoder.o and b/asc2/Objects/encoder.o differ
[1mdiff --git a/asc2/Objects/main.crf b/asc2/Objects/main.crf[m
[1mindex aee62e0..3a89d9b 100644[m
Binary files a/asc2/Objects/main.crf and b/asc2/Objects/main.crf differ
[1mdiff --git a/asc2/Objects/main.d b/asc2/Objects/main.d[m
[1mindex 1c66af0..18ea5d2 100644[m
[1m--- a/asc2/Objects/main.d[m
[1m+++ b/asc2/Objects/main.d[m
[36m@@ -32,5 +32,6 @@[m
 .\objects\main.o: .\Hardware\LED.h[m
 .\objects\main.o: .\Hardware\Key.h[m
 .\objects\main.o: .\System\Timer.h[m
[31m-.\objects\main.o: .\Hardware\Menu.h[m
 .\objects\main.o: .\Hardware\Encoder.h[m
[32m+[m[32m.\objects\main.o: .\Hardware\Serial.h[m
[32m+[m[32m.\objects\main.o: D:\Keil5\ARM\ARMCC\Bin\..\include\stdio.h[m
[1mdiff --git a/asc2/Objects/main.o b/asc2/Objects/main.o[m
[1mindex 6d310c9..af65909 100644[m
Binary files a/asc2/Objects/main.o and b/asc2/Objects/main.o differ
[1mdiff --git a/asc2/Project.uvoptx b/asc2/Project.uvoptx[m
[1mindex f22d434..fa12eff 100644[m
[1m--- a/asc2/Project.uvoptx[m
[1m+++ b/asc2/Project.uvoptx[m
[36m@@ -1036,6 +1036,30 @@[m
       <RteFlg>0</RteFlg>[m
       <bShared>0</bShared>[m
     </File>[m
[32m+[m[32m    <File>[m
[32m+[m[32m      <GroupNumber>4</GroupNumber>[m
[32m+[m[32m      <FileNumber>70</FileNumber>[m
[32m+[m[32m      <FileType>1</FileType>[m
[32m+[m[32m      <tvExp>0</tvExp>[m
[32m+[m[32m      <tvExpOptDlg>0</tvExpOptDlg>[m
[32m+[m[32m      <bDave2>0</bDave2>[m
[32m+[m[32m      <PathWithFileName>.\Hardware\Serial.c</PathWithFileName>[m
[32m+[m[32m      <FilenameWithoutPath>Serial.c</FilenameWithoutPath>[m
[32m+[m[32m      <RteFlg>0</RteFlg>[m
[32m+[m[32m      <bShared>0</bShared>[m
[32m+[m[32m    </File>[m
[32m+[m[32m    <File>[m
[32m+[m[32m      <GroupNumber>4</GroupNumber>[m
[32m+[m[32m      <FileNumber>71</FileNumber>[m
[32m+[m[32m      <FileType>5</FileType>[m
[32m+[m[32m      <tvExp>0</tvExp>[m
[32m+[m[32m      <tvExpOptDlg>0</tvExpOptDlg>[m
[32m+[m[32m      <bDave2>0</bDave2>[m
[32m+[m[32m      <PathWithFileName>.\Hardware\Serial.h</PathWithFileName>[m
[32m+[m[32m      <FilenameWithoutPath>Serial.h</FilenameWithoutPath>[m
[32m+[m[32m      <RteFlg>0</RteFlg>[m
[32m+[m[32m      <bShared>0</bShared>[m
[32m+[m[32m    </File>[m
   </Group>[m
 [m
   <Group>[m
[36m@@ -1046,7 +1070,7 @@[m
     <RteFlg>0</RteFlg>[m
     <File>[m
       <GroupNumber>5</GroupNumber>[m
[31m-      <FileNumber>70</FileNumber>[m
[32m+[m[32m      <FileNumber>72</FileNumber>[m
       <FileType>1</FileType>[m
       <tvExp>0</tvExp>[m
       <tvExpOptDlg>0</tvExpOptDlg>[m
[36m@@ -1058,7 +1082,7 @@[m
     </File>[m
     <File>[m
       <GroupNumber>5</GroupNumber>[m
[31m-      <FileNumber>71</FileNumber>[m
[32m+[m[32m      <FileNumber>73</FileNumber>[m
       <FileType>5</FileType>[m
       <tvExp>0</tvExp>[m
       <tvExpOptDlg>0</tvExpOptDlg>[m
[36m@@ -1070,7 +1094,7 @@[m
     </File>[m
     <File>[m
       <GroupNumber>5</GroupNumber>[m
[31m-      <FileNumber>72</FileNumber>[m
[32m+[m[32m      <FileNumber>74</FileNumber>[m
       <FileType>1</FileType>[m
       <tvExp>0</tvExp>[m
       <tvExpOptDlg>0</tvExpOptDlg>[m
[36m@@ -1082,7 +1106,7 @@[m
     </File>[m
     <File>[m
       <GroupNumber>5</GroupNumber>[m
[31m-      <FileNumber>73</FileNumber>[m
[32m+[m[32m      <FileNumber>75</FileNumber>[m
       <FileType>5</FileType>[m
       <tvExp>0</tvExp>[m
       <tvExpOptDlg>0</tvExpOptDlg>[m
[1mdiff --git a/asc2/Project.uvprojx b/asc2/Project.uvprojx[m
[1mindex a855775..51d0c3a 100644[m
[1m--- a/asc2/Project.uvprojx[m
[1m+++ b/asc2/Project.uvprojx[m
[36m@@ -740,6 +740,16 @@[m
               <FileType>5</FileType>[m
               <FilePath>.\hardware\Encoder.h</FilePath>[m
             </File>[m
[32m+[m[32m            <File>[m
[32m+[m[32m              <FileName>Serial.c</FileName>[m
[32m+[m[32m              <FileType>1</FileType>[m
[32m+[m[32m              <FilePath>.\Hardware\Serial.c</FilePath>[m
[32m+[m[32m            </File>[m
[32m+[m[32m            <File>[m
[32m+[m[32m              <FileName>Serial.h</FileName>[m
[32m+[m[32m              <FileType>5</FileType>[m
[32m+[m[32m              <FilePath>.\Hardware\Serial.h</FilePath>[m
[32m+[m[32m            </File>[m
           </Files>[m
         </Group>[m
         <Group>[m
[1mdiff --git a/asc2/User/main.c b/asc2/User/main.c[m
[1mindex cd9203f..cdb254f 100644[m
[1m--- a/asc2/User/main.c[m
[1m+++ b/asc2/User/main.c[m
[36m@@ -3,35 +3,38 @@[m
 #include "LED.h"[m
 #include "Key.h"[m
 #include "Timer.h"[m
[31m-#include "Menu.h"[m
 #include "Encoder.h"[m
[32m+[m[32m#include "Serial.h"[m
 [m
[31m-// uint16_t LEDStatus;[m
[32m+[m
[32m+[m[32mint16_t Speed;[m
 [m
 int main(void)[m
 {[m
 	Key_Init();[m
[31m-	LED_Init();[m
 	OLED_Init();[m
 	Timer_Init();[m
 	Encoder_Init();[m
[31m-	Menu_Init();[m
[32m+[m	[32mSerial_Init();[m
 	while (1)[m
 	{[m
[31m-		Menu_Show();[m
[31m-		uint8_t Key = Key_GetNum();[m
[31m-		Menu_Option(Key);[m
[31m-		LED_DirSet(Menu_LED_Direction());[m
[31m-		LED_SpeedSet(Menu_LED_Speed());[m
[32m+[m		[32mOLED_ShowSignedNum(1, 7, Speed, 4);[m
[32m+[m		[32mSerial_Printf("%d\n", Speed);[m
 	}[m
 }[m
 [m
 void TIM2_IRQHandler(void)[m
 {[m
[32m+[m	[32mstatic uint16_t Speed_Tim = 0;[m
 	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)[m
 	{[m
 		Key_Tick();[m
[31m-		LED_Tick();[m
[32m+[m		[32mSpeed_Tim++;[m
[32m+[m		[32mif (Speed_Tim == 100 - 1)[m
[32m+[m		[32m{[m
[32m+[m			[32mSpeed = Encoder_Get();[m
[32m+[m			[32mSpeed_Tim = 0;[m
[32m+[m		[32m}[m
 		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);[m
 	}[m
 }[m
