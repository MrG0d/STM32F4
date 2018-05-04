#include "vstm.h"

void v_init(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  //Включаем тактирование контроллера конфигурации системы

	v_configTimerPWM(TIM1, 1);
	v_configTimerPWM(TIM2, 2);
	v_configTimerPWM(TIM3, 3);
	v_configTimerPWM(TIM4, 4);
	v_configTimerPWM(TIM9, 9);
	v_configTimerPWM(TIM12, 12);


	v_usart1_rd = v_new_RingBuffer();
	v_kernel.usart[0].receive_data = &v_usart1_rd;
	v_usart2_rd = v_new_RingBuffer();
	v_kernel.usart[1].receive_data = &v_usart2_rd;
	v_usart3_rd = v_new_RingBuffer();
	v_kernel.usart[2].receive_data = &v_usart3_rd;
	v_usart4_rd = v_new_RingBuffer();
	v_kernel.usart[3].receive_data = &v_usart4_rd;
	v_usart5_rd = v_new_RingBuffer();
	v_kernel.usart[4].receive_data = &v_usart5_rd;
	v_usart6_rd = v_new_RingBuffer();
	v_kernel.usart[5].receive_data = &v_usart6_rd;


	ADC_InitTypeDef adc_setup;
	adc_setup.ADC_ContinuousConvMode = DISABLE;
	adc_setup.ADC_Resolution = ADC_Resolution_12b;
	adc_setup.ADC_ScanConvMode = DISABLE;
	adc_setup.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	adc_setup.ADC_DataAlign = ADC_DataAlign_Right;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_Init(ADC1, &adc_setup);
	ADC_Cmd(ADC1,ENABLE);
}

void pinMode(unsigned short int _pin, mode m){
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Pin = v_getPin(_pin);
	switch(m){
		case Analog:
			PORT.GPIO_Speed = GPIO_Speed_2MHz;  // Скорость 2 МГц
			PORT.GPIO_OType = GPIO_OType_PP;  // Двухтактный выход
			PORT.GPIO_Mode = GPIO_Mode_AN; // Альтернативный режим
			PORT.GPIO_PuPd = GPIO_PuPd_DOWN; //Подтягивающий к земле резистор
			break;
		case Output: //Выход, для генерирования 1 или 0
			PORT.GPIO_Mode = GPIO_Mode_OUT;     // Режим "Выхода"
			PORT.GPIO_Speed = GPIO_Speed_2MHz;  // Скорость 2 МГц
			PORT.GPIO_OType = GPIO_OType_PP;    // Двухтактный выход
			PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;  // Без подтяжки
			break;
		case Input: //Вход, для чтения сигнала
		case Exti: //Прерывания
			PORT.GPIO_Mode = GPIO_Mode_IN; 	   // Режим "Входа"
			PORT.GPIO_Speed = GPIO_Speed_2MHz; // Скорость 2 МГц
			PORT.GPIO_OType = GPIO_OType_PP;   //
			PORT.GPIO_PuPd = GPIO_PuPd_DOWN;   //Подтягивающий к земле резистор
			if(m == Exti){
				SYSCFG_EXTILineConfig(v_getExtiPortSource(_pin), v_getExtiPinSource(_pin));

				EXTI_InitTypeDef exti;
				exti.EXTI_Line = v_getExtiLine(_pin);                   //Выбираем линию
				exti.EXTI_Mode = EXTI_Mode_Interrupt;                   //Устанавливаем, что событие будет источником прерывания
																		//Запрос на прерывание генерируется по фронту импульса, т.е. при
				exti.EXTI_Trigger = EXTI_Trigger_Rising;                //переходе из 0 в 1
				exti.EXTI_LineCmd = ENABLE;                             //Указываем новое состояние для линии прерывания
				EXTI_Init(&exti);                                       //Инициализируем настройки

				NVIC_InitTypeDef nvic;
				nvic.NVIC_IRQChannel = v_getExtiIRQ(_pin);                      //Устанавливаем источник запроса на прерывание
				NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //присваиваем данному прерыванию первую группу
				nvic.NVIC_IRQChannelPreemptionPriority = 0;             //Устанавливаем приоритет для обработчика прерывания
				nvic.NVIC_IRQChannelSubPriority = 0;                    //Устанавливаем субприоритет
				nvic.NVIC_IRQChannelCmd = ENABLE;                       //Указываем, что выбранный канал будет источником вектора прерывания
				NVIC_Init(&nvic);
			}
			break;
		case Alt: //Конфигурация под альтернативный режим
		case PWM: //Конфигурация пина под выход ШИМ-а
			PORT.GPIO_Mode = GPIO_Mode_AF;
			PORT.GPIO_Speed = GPIO_Speed_100MHz;
			PORT.GPIO_OType = GPIO_OType_PP;
			PORT.GPIO_PuPd = GPIO_PuPd_DOWN;
			if(m == PWM){
				v_pinAFconfig(_pin, v_getAF_Timer(_pin));
			}
			break;
	}

	switch(v_getPort(_pin)){
		case A: GPIO_Init(GPIOA, &PORT); break;
		case B: GPIO_Init(GPIOB, &PORT); break;
		case C: GPIO_Init(GPIOC, &PORT); break;
		case D: GPIO_Init(GPIOD, &PORT); break;
		case E: GPIO_Init(GPIOE, &PORT); break;
		case H: GPIO_Init(GPIOH, &PORT); break;
	}
}

void digitalWrite(uint16_t _pin, bool status){
	uint16_t pin = v_getPin(_pin);
	switch(v_getPort(_pin)){
		case A:
			if(status == true)
				GPIO_WriteBit(GPIOA, pin, Bit_SET);
			else
				GPIO_WriteBit(GPIOA, pin, Bit_RESET);
			break;
		case B:
			if(status == true)
				GPIO_WriteBit(GPIOB, pin, Bit_SET);
			else
				GPIO_WriteBit(GPIOB, pin, Bit_RESET);
			break;
		case C:
			if(status == true)
				GPIO_WriteBit(GPIOC, pin, Bit_SET);
			else
				GPIO_WriteBit(GPIOC, pin, Bit_RESET);
			break;
		case D:
			if(status == true)
				GPIO_WriteBit(GPIOD, pin, Bit_SET);
			else
				GPIO_WriteBit(GPIOD, pin, Bit_RESET);
			break;
		case E:
			if(status == true)
				GPIO_WriteBit(GPIOE, pin, Bit_SET);
			else
				GPIO_WriteBit(GPIOE, pin, Bit_RESET);
			break;
		case H:
			if(status == true)
				GPIO_WriteBit(GPIOH, pin, Bit_SET);
			else
				GPIO_WriteBit(GPIOH, pin, Bit_RESET);
			break;
	}
}

uint16_t analogRead(uint16_t _pin){
	ADC_TypeDef* ADCx;
	uint8_t channel;
	switch(_pin){
		case A0: ADCx = ADC1; channel = ADC_Channel_0; break;
		case A1: ADCx = ADC1; channel = ADC_Channel_1; break;
		case A2: ADCx = ADC1; channel = ADC_Channel_2; break;
		case A3: ADCx = ADC1; channel = ADC_Channel_3; break;
		case A4: ADCx = ADC1; channel = ADC_Channel_4; break;
		case A5: ADCx = ADC1; channel = ADC_Channel_5; break;
		case A6: ADCx = ADC1; channel = ADC_Channel_6; break;
		case A7: ADCx = ADC1; channel = ADC_Channel_7; break;
		case B0: ADCx = ADC1; channel = ADC_Channel_8; break;
		case B1: ADCx = ADC1; channel = ADC_Channel_9; break;
		case C0: ADCx = ADC1; channel = ADC_Channel_10; break;
		case C1: ADCx = ADC1; channel = ADC_Channel_11; break;
		case C2: ADCx = ADC1; channel = ADC_Channel_12; break;
		case C3: ADCx = ADC1; channel = ADC_Channel_13; break;
		case C4: ADCx = ADC1; channel = ADC_Channel_14; break;
		case C5: ADCx = ADC1; channel = ADC_Channel_15; break;
	}

	//настройка - канал, время преобразования - 3 цикла
	ADC_RegularChannelConfig(ADCx, channel, 1, ADC_SampleTime_3Cycles);
	//начать преобразование
	ADC_SoftwareStartConv(ADCx);
	// ждем окончания преобразования
	while(ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
	// возвращаем полученное значение
	return ADC_GetConversionValue(ADCx);
 }

void analogWrite(uint16_t _pin, uint16_t val){ //maxVal 1024
	switch(_pin){
//		case A8:
//		case E9: TIM1->CCR1 = val; break;
//		case A9:
//		case E10: TIM1->CCR2 = val; break;
//		case A10:
//		case E13: TIM1->CCR3 = val; break;
//		case A11:
//		case E14: TIM1->CCR4 = val; break;

		case A0:
		case A5:
		case A15: TIM2->CCR1 = val; break;
		case A1:
		case B3: TIM2->CCR2 = val; break;
		case A2:
		case B10: TIM2->CCR3 = val; break;
		case A3:
		case B11: TIM2->CCR4 = val; break;

		case A6:
		case B4:
		case C6: TIM3->CCR1 = val; break;
		case A7:
		case B5:
		case C7: TIM3->CCR2 = val; break;
		case B0:
		case C8: TIM3->CCR3 = val; break;
		case B1:
		case C9: TIM3->CCR4 = val; break;


		case B6:
		case D12: TIM4->CCR1 = val; break;
		case B7:
		case D13: TIM4->CCR2 = val; break;
		case B8:
		case D14: TIM4->CCR3 = val; break;
		case B9:
		case D15: TIM4->CCR4 = val; break;

//		case A2: //TIM2!!!
		case E5: TIM9->CCR1 = val; break;
//		case A3: //TIM2!!!
		case E6: TIM9->CCR2 = val; break;

		case B14: TIM12->CCR1 = val; break;
		case B15: TIM12->CCR2 = val; break;
	}
}

uint8_t v_getAF_Timer(uint16_t _pin){
	uint8_t af;
	if(_pin == A8 || _pin == E9 || _pin == A9 || _pin == E10 || _pin == A10 || _pin == E13 || _pin == A11 || _pin == E14){
		af = GPIO_AF_TIM1;
	}else if(_pin == A0 || _pin == A5 || _pin == A15 || _pin == A1 || _pin == B3 || _pin == A2 || _pin == B10 || _pin == A3 || _pin == B11){
		af = GPIO_AF_TIM2;
	}else if(_pin == A6 || _pin == B4 || _pin == C6 || _pin == A7 || _pin == B5 ||_pin == C7 || _pin == B0 || _pin == C8 || _pin == B1 || _pin == C9){
		af = GPIO_AF_TIM3;
	}else if(_pin == B6 || _pin == B7 || _pin == B8 || _pin == B9 || _pin == D12 ||  _pin == D13 ||  _pin == D14 ||  _pin == D15){
		af = GPIO_AF_TIM4;
	}else if(_pin == A2 || _pin == A3  || _pin == E5 || _pin == E6){
		af = GPIO_AF_TIM9;
	}else if(_pin == B14 || _pin == B15){
		af = GPIO_AF_TIM12;
	}else
		af = 0;
	return af;
}

uint8_t v_getAF_USART(uint16_t _pin){
	uint8_t af;
	if(_pin == A9 || _pin == A10 || _pin == B6 || _pin == B7){
		af = GPIO_AF_USART1;
	}else if(_pin == A2 || _pin == A3 || _pin == D5 || _pin == D6){
		af = GPIO_AF_USART2;
	}else if(_pin == B10 || _pin == B11 || _pin == D8 || _pin == D9){
		af = GPIO_AF_USART3;
	}else if(_pin == A0 || _pin == A1 || _pin == C10 || _pin == C11){
		af = GPIO_AF_UART4;
	}else if(_pin == C12 || _pin == D2 ){
		af = GPIO_AF_UART5;
	}else if(_pin == C6 || _pin == C7 ){
		af = GPIO_AF_USART6;
	}else
		af = 0;
	return af;
}

void v_pinAFconfig(uint16_t _pin, uint8_t af){
	if(af == 0) return;
	uint8_t pin_source = v_getPinSource(_pin);
	switch(v_getPort(_pin)){
		case A: GPIO_PinAFConfig(GPIOA, pin_source, af); break;
		case B: GPIO_PinAFConfig(GPIOB, pin_source, af); break;
		case C: GPIO_PinAFConfig(GPIOC, pin_source, af); break;
		case D: GPIO_PinAFConfig(GPIOD, pin_source, af); break;
		case E: GPIO_PinAFConfig(GPIOE, pin_source, af); break;
		case H: GPIO_PinAFConfig(GPIOH, pin_source, af); break;
	}
}
bool digitalRead(uint16_t _pin){
	uint16_t pin = v_getPin(_pin);
	switch (v_getPort(_pin)){
		case A:
			return GPIO_ReadInputDataBit(GPIOA, pin);
		case B:
			return GPIO_ReadInputDataBit(GPIOB, pin);
		case C:
			return GPIO_ReadInputDataBit(GPIOC, pin);
		case D:
			return GPIO_ReadInputDataBit(GPIOD, pin);
		case E:
			return GPIO_ReadInputDataBit(GPIOE, pin);
		case H:
			return GPIO_ReadInputDataBit(GPIOH, pin);
	}
}

v_port v_getPort(uint16_t _pin){
	if(_pin >= 100 && _pin <= 115)
		return A;
	if(_pin >= 200 && _pin <= 215)
		return B;
	if(_pin >= 300 && _pin <= 315)
		return C;
	if(_pin >= 400 && _pin <= 415)
		return D;
	if(_pin >= 500 && _pin <= 515)
		return E;
	if(_pin == 600 || _pin == 601)
		return H;
}

uint16_t v_getPin(uint16_t _pin){
	unsigned char pin = _pin%100;
	switch(pin){
		case 0: return GPIO_Pin_0;
		case 1: return GPIO_Pin_1;
		case 2: return GPIO_Pin_2;
		case 3: return GPIO_Pin_3;
		case 4: return GPIO_Pin_4;
		case 5: return GPIO_Pin_5;
		case 6: return GPIO_Pin_6;
		case 7: return GPIO_Pin_7;
		case 8: return GPIO_Pin_8;
		case 9: return GPIO_Pin_9;
		case 10: return GPIO_Pin_10;
		case 11: return GPIO_Pin_11;
		case 12: return GPIO_Pin_12;
		case 13: return GPIO_Pin_13;
		case 14: return GPIO_Pin_14;
		case 15: return GPIO_Pin_15;
	}
}

uint8_t v_getPinSource(uint16_t _pin){
	unsigned char pin = _pin%100;
	switch(pin){
		case 0: return GPIO_PinSource0;
		case 1: return GPIO_PinSource1;
		case 2: return GPIO_PinSource2;
		case 3: return GPIO_PinSource3;
		case 4: return GPIO_PinSource4;
		case 5: return GPIO_PinSource5;
		case 6: return GPIO_PinSource6;
		case 7: return GPIO_PinSource7;
		case 8: return GPIO_PinSource8;
		case 9: return GPIO_PinSource9;
		case 10: return GPIO_PinSource10;
		case 11: return GPIO_PinSource11;
		case 12: return GPIO_PinSource12;
		case 13: return GPIO_PinSource13;
		case 14: return GPIO_PinSource14;
		case 15: return GPIO_PinSource15;
	}
}

uint8_t v_getPinNumber(uint16_t _pin){
	return _pin%100;
}


void v_configTimerPWM(TIM_TypeDef *timer, uint8_t number){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	/* Time base configuration */
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 1024;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(timer, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);
	/* PWM1 Mode configuration: Channel2 */
	TIM_OC2Init(timer, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
	if(number == 2 || number == 1 || number == 3 || number == 4){
		/* PWM1 Mode configuration: Channel3 */
		TIM_OC3Init(timer, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
		/* PWM1 Mode configuration: Channel4 */
		TIM_OC4Init(timer, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
	}
//	/* TIM enable counter */
	TIM_ARRPreloadConfig(timer, ENABLE);
	TIM_Cmd(timer, ENABLE);
}


uint32_t v_getExtiLine(uint16_t _pin){
	unsigned char pin = _pin%100;
	switch(pin){
		case 0: return EXTI_Line0;
		case 1: return EXTI_Line1;
		case 2: return EXTI_Line2;
		case 3: return EXTI_Line3;
		case 4: return EXTI_Line4;
		case 5: return EXTI_Line5;
		case 6: return EXTI_Line6;
		case 7: return EXTI_Line7;
		case 8: return EXTI_Line8;
		case 9: return EXTI_Line9;
		case 10: return EXTI_Line10;
		case 11: return EXTI_Line11;
		case 12: return EXTI_Line12;
		case 13: return EXTI_Line13;
		case 14: return EXTI_Line14;
		case 15: return EXTI_Line15;
	}
}

uint32_t v_getExtiPinSource(uint16_t _pin){
	unsigned char pin = _pin%100;
	switch(pin){
		case 0: return EXTI_PinSource0;
		case 1: return EXTI_PinSource1;
		case 2: return EXTI_PinSource2;
		case 3: return EXTI_PinSource3;
		case 4: return EXTI_PinSource4;
		case 5: return EXTI_PinSource5;
		case 6: return EXTI_PinSource6;
		case 7: return EXTI_PinSource7;
		case 8: return EXTI_PinSource8;
		case 9: return EXTI_PinSource9;
		case 10: return EXTI_PinSource10;
		case 11: return EXTI_PinSource11;
		case 12: return EXTI_PinSource12;
		case 13: return EXTI_PinSource13;
		case 14: return EXTI_PinSource14;
		case 15: return EXTI_PinSource15;
	}
}


uint32_t v_getExtiIRQ(uint16_t _pin){
	/**
	 * Каналы 0 - 4 имеют собственный обработчик прерывания, 5 - 9 и 10 - 15 - один на группу
	 */
	unsigned char pin = _pin%100;
	switch(pin){
		case 0: return EXTI0_IRQn;
		case 1: return EXTI1_IRQn;
		case 2: return EXTI2_IRQn;
		case 3: return EXTI3_IRQn;
		case 4: return EXTI4_IRQn;
		case 5:
		case 6:
		case 7:
		case 8:
		case 9: return EXTI9_5_IRQn;
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15: return EXTI15_10_IRQn;
	}
//	return void;
}

uint8_t v_getExtiPortSource(uint16_t _pin){
	switch(v_getPort(_pin)){
		case A: return EXTI_PortSourceGPIOA;
		case B: return EXTI_PortSourceGPIOB;
		case C: return EXTI_PortSourceGPIOC;
		case D: return EXTI_PortSourceGPIOD;
		case E: return EXTI_PortSourceGPIOE;
		case H: return EXTI_PortSourceGPIOH;
		default: return 0;
	}
}

void v_USART_init(USART_TypeDef* USARTx, uint32_t speed, uint8_t contact_group){
	uint16_t tx, rx;
	IRQn_Type irq;
	if(USARTx == USART1){
		irq = USART1_IRQn;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		if(contact_group == 2){
			tx = B6;
			rx = B7;
		}else{
			tx = A9;
			rx = A10;
		}
	}else if(USARTx == USART2){
		irq = USART2_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		if(contact_group == 2){
			tx = D5;
			rx = D6;
		}else{
			tx = A2;
			rx = A3;
		}
	}else if(USARTx == USART3){
		irq = USART3_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		switch(contact_group){
		case 2:
			tx = C10; //!!!!!! Конфликтуют с UART4
			rx = C11; //!!!!!!
			break;
		case 3:
			tx = D8;
			rx = D9;
			break;
		default:
			tx = B10;
			rx = B11;
		}
	}else if(USARTx == UART4){
		irq = UART4_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		if(contact_group == 2){
			tx = C10; // Tx !!!!!!  Конфликтуют с UART3
			rx = C11; //!!!!!!
		}else{
			tx = A0;
			rx = A1;
		}
	}else if(USARTx == UART5){
		irq = UART5_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		tx = C12;
		rx = D2;
	}else if(USARTx == USART6){
		irq = USART6_IRQn;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		tx = C6;
		rx = C7;
	}/*else if(USARTx == UART7){
		tx = E8;
		rx = E7;
	}else if(USARTx == UART8){
		tx = E1;
		rx = E0;
	}*/
	pinMode(tx, Alt);
	pinMode(rx, Alt);
	v_pinAFconfig(tx, v_getAF_USART(tx));
	v_pinAFconfig(rx, v_getAF_USART(rx));

//	USART_OverSampling8Cmd(USARTx, ENABLE);
	
	USART_InitTypeDef uart_setup;
	uart_setup.USART_BaudRate = speed; //скорость 9600
	uart_setup.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //нет контроля потока
	uart_setup.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;
	uart_setup.USART_Parity = USART_Parity_No; //без контроля четности
	uart_setup.USART_StopBits = USART_StopBits_1; //1 стоп бит
	uart_setup.USART_WordLength = USART_WordLength_8b; //8 бит данных
	USART_Init(USARTx, &uart_setup); //запись структуры

	USART_Cmd(USARTx, ENABLE); //включить юарт
	
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

	// Включаем прерывания. Теперь передаваемые данные необходимо ловить в прерываниях
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = irq;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;            // Разрешить прерывания
	NVIC_Init(&nvic);

}


void v_USART_sendChar(USART_TypeDef* USARTx, uint8_t ch)
{
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	USART_SendData(USARTx, ch);
}

void v_USART_sendString(USART_TypeDef* USARTx, uint8_t *str)
{
  while(*str != 0){
	  v_USART_sendChar(USARTx, *str);
	  str++;
  }
}

void v_USART_sendNumber(USART_TypeDef* USARTx, uint32_t x)
{
  char value[10]; //a temp array to hold results of conversion
  uint8_t i = 0; //loop index
  do{
    value[i++] = (char)(x % 10) + '0'; //convert integer to character
    x /= 10;
  } while(x);

  while(i) //send data{
	  v_USART_sendChar(USARTx, value[--i]);
//  v_USART_sendChar(USARTx, '|');
//  v_USART_sendChar(USARTx, '\n');
}

bool v_USART_available(USART_TypeDef* USARTx){
	uint8_t usart_number = v_USARTx_number(USARTx);
	if(usart_number == 0) return 0;
	return v_kernel.usart[usart_number - 1].receive_data->length;
}
uint16_t v_USART_read(USART_TypeDef* USARTx){
	uint8_t usart_number = v_USARTx_number(USARTx);
	if(usart_number == 0) return 0;
	return v_RingBuffer_get(v_kernel.usart[usart_number - 1].receive_data);
}
uint16_t v_USART_readLast(USART_TypeDef* USARTx){
	uint8_t usart_number = v_USARTx_number(USARTx);
	if(usart_number == 0) return 0;
	return v_RingBuffer_getLast(v_kernel.usart[usart_number - 1].receive_data);
}
void v_USART_clear(USART_TypeDef* USARTx){
	uint8_t usart_number = v_USARTx_number(USARTx);
	if(usart_number == 0) return 0;
	v_RingBuffer_clear(v_kernel.usart[usart_number - 1].receive_data);
}

RingBuffer v_new_RingBuffer(){
	RingBuffer result;
	result.end = 0;
	result.length = 0;
	result.start = 0;
	return result;
}


void v_RingBuffer_put(RingBuffer *buffer, uint16_t ch){
	if(buffer->length < v_RING_BUFFER_LENGTH){
		buffer->data[buffer->end] = ch;
		++buffer->end;
		++buffer->length;
		if(buffer->end == v_RING_BUFFER_LENGTH)
			buffer->end = 0;
	}
}

uint16_t v_RingBuffer_get(RingBuffer *buffer){
	if(buffer->length == 0) return 0;
	char result = buffer->data[buffer->start];
	++buffer->start;
	--buffer->length;
	if(buffer->start == v_RING_BUFFER_LENGTH)
		buffer->start = 0;
	return result;
}

uint16_t v_RingBuffer_getLast(RingBuffer *buffer){
	if(buffer->length == 0) return 0;
	char result = buffer->data[buffer->end - 1];
	--buffer->end;
	--buffer->length;
	if(buffer->start == 0)
		buffer->start = v_RING_BUFFER_LENGTH;
	return result;
}
void v_RingBuffer_clear(RingBuffer *buffer){
	buffer->end = 0;
	buffer->start = 0;
	buffer->length = 0;
}

uint8_t v_USARTx_number(USART_TypeDef* USARTx){
	if(USARTx == USART1)
		return 1;
	else if(USARTx == USART2)
		return 2;
	else if(USARTx == USART3)
		return 3;
	else if(USARTx == UART4)
		return 4;
	else if(USARTx == UART5)
		return 5;
	else if(USARTx == USART6)
		return 6;
	else
		return 0;
}


///---------------------------Обработчики прерываний для USART
void v_USARTx_IRQHandlerRead(USART_TypeDef* USARTx){
	uint8_t usart_number = v_USARTx_number(USARTx);
	if(usart_number == 0) return;

	if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET){
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		v_RingBuffer_put(v_kernel.usart[usart_number - 1].receive_data, USART_ReceiveData(USARTx));
	}
}
void USART1_IRQHandler(void){ v_USARTx_IRQHandlerRead(USART1);}

void USART2_IRQHandler(void){ v_USARTx_IRQHandlerRead(USART2);}

void USART3_IRQHandler(void){ v_USARTx_IRQHandlerRead(USART3);}

void UART4_IRQHandler(void){ v_USARTx_IRQHandlerRead(UART4);}

void UART5_IRQHandler(void){ v_USARTx_IRQHandlerRead(UART5);}

void USART6_IRQHandler(void){ v_USARTx_IRQHandlerRead(USART6);}
