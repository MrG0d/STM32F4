/**
 * VSTM
 * Данная библиотека предназначена для упрощения работы с контроллером STM32F4
 * В ней приведены функции подобные функциям Arduino
 * @author Ilya Verem'ev [https://vk.com/id67735631, veiv.veremyev@gmail.com]
 * @author Ivan Kudryavsky [spam@mr-god.net]
 *
 * @site http://master.virmandy.net/
 * @site https://master.virmandy.net/category/stm32f4/
 */

#include "stm32f4xx_usart.h"
#include "VSTM/vstm.h"

int main(void)
{
	v_init();

	/*
	 *
	 *
	 * If the blue (custom) button is pressed, the PWM (D13) is increased by 2 units
	 * and the indicator on D14 lights up.
	 * PWM level information is transmitted via USART2
	 *
	 * Если нажата синяя (custom) кнопка, то увеличивается ШИМ (D13) на 2 ед.
	 * и загается индикатор на D14.
	 * Информация о уровне ШИМ передается по USART2
	 *
	 *
	 *
	 */


	pinMode(A0, Input); //UserButton
	pinMode(D13, PWM);
	pinMode(D14, Output);
	uint16_t pwm_level = 0;

	v_USART_init(USART2, 9600, 1); // tx - A2, rx - A3
	while(1)
	{
		if(digitalRead(A0)){ //Button press
			digitalWrite(D14, true);
			pwm_level += 2;

			if(pwm_level > 20) pwm_level = 0;
			analogWrite(D13, pwm_level);

			v_USART_sendString(USART2, "PWM Level: ");
			v_USART_sendNumber(USART2, pwm_level);
			v_USART_sendString(USART2, "\n");
		}else{
			digitalWrite(D14, false);
		}

		//Delay
		for(uint16_t delay = 10000; delay > 0; --delay) digitalRead(A0);
	}
}
