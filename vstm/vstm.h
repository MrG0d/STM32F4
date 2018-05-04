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
#ifndef VSTM_H
#define VSTM_H

#include "vdefine.h"

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_adc.h>
#include <misc.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>

#define v_RING_BUFFER_LENGTH 100

typedef struct v_RingBuffer{
	uint16_t data[v_RING_BUFFER_LENGTH];
	uint8_t start;
	uint8_t end ;
	uint8_t length ;

} RingBuffer;
typedef struct V_USART{
	RingBuffer *receive_data;

} V_USART;

RingBuffer v_usart1_rd, v_usart2_rd, v_usart3_rd, v_usart4_rd, v_usart5_rd, v_usart6_rd;
typedef struct _v_kernel{
	V_USART usart[6];
} _v_kernel;

_v_kernel v_kernel;


/**
 * Initial setting of the controller
 * Начальная настройка контроллера
 */
void v_init();

/**
 * Returns the port of the specified pin (A, B, C, D, E)
 * Возвращает порт указанного пина (A, B, C, D, E)
 * @param uint16_t _pin - pin number | номер пина
 * @return port
 */
v_port v_getPort(uint16_t pin);

/**
 * Returns the STM-pin of the specified pin
 * Возвращает STM-овский пин указанного пина (GPIO_Pin_xx)
 * @param uint16_t _pin - pin number | номер пина
 * @return uint16_t - STM-pin | STM-пин
 */
uint16_t v_getPin(uint16_t _pin);

/**
 * Function for obtaining the numeric part of the pin number
 * Функция для получения числовой части номера пина
 * @param uint16_t _pin - pin number | номер пина
 * @return uint8_t - number of pin number | число номера пина
 */
uint8_t v_getPinNumber(uint16_t _pin);

/**
 * Returns the STM source pin
 * Возвращает STM-овский source пина (GPIO_PinSourceXX)
 * @param uint16_t _pin - pin number | номер пина
 * @return uint16_t - source pin | source пина
 */
uint8_t v_getPinSource(uint16_t _pin);

/**
 * Pin configuration for the corresponding operation mode
 * Конфигурация пина на соответствующий режим работы
 * @param uint16_t _pin - pin number | номер пина
 * @param mode m - operation mode | режим работы
 */
void pinMode(uint16_t _pin, mode m);

/**
 * Reading a digital signal from the specified pin
 * Чтение цифрового сигнала с указанного пина
 * @param uint16_t _pin - pin number | номер пина
 * @return bool - signal value | значение сигнала
 */
uint8_t digitalRead(uint16_t _pin);

/**
 * Record HIGH (true) level or LOW (false) on the specified pin
 * Запись HIGH (true) уровня или LOW (false) на указанный pin
 * @param uint16_t _pin - pin number | номер пина
 * @param bool status - signal value | значение сигнала
 */
void digitalWrite(uint16_t _pin, bool status);

/**
 * Reading an analog signal
 * Считывание аналогового сигнала
 * @param uint16_t _pin - Pin number from which to read the signal | Номер пина, с которого считывается сигнал
 * @return uint16_t - signal value | значение сигнала
 */
u16 analogRead(uint16_t _pin);

/**
 * The PWM recording function on the specified pin
 * Функция записи ШИМа на указанном пине
 * @param uint16_t _pin - pin number | номер пина
 * @param uint16_t _pin - pulse width (up to 1024) | ширина импульса (до 1024)
 */
void analogWrite(uint16_t _pin, uint16_t val);

/**
 * Timer setting function for PWM
 * Функция настройки таймера под ШИМ
 * @param IM_TypeDef *time - timer for setting | таймер для настройки
 * @param uint8_t number - timer number | номер таймера
 */
void v_configTimerPWM(TIM_TypeDef *timer, uint8_t number);


/**
 * Enabling an alternative function for pin
 * Включение альтернативной функции у пина
 * @param uint16_t _pin - pin number | номер пина
 * @param uint8_t af - alternative function | альтернативная функция (GPIO_AF_TIM3, GPIO_AF_USART1)
 */
void v_pinAFconfig(uint16_t _pin, uint8_t af);


/**
 * Alternate function for pin - Timers
 * Альтернативная функция для пина - Таймеры
 * @param uint16_t _pin - pin number | номер пина
 */
uint8_t v_getAF_Timer(uint16_t _pin);

/**
 * Alternate function for pin - USART
 * Альтернативная функция для пина - USART
 * @param uint16_t _pin - pin number | номер пина
 */
uint8_t v_getAF_USART(uint16_t _pin);

/**
 * Interrupt line for pin
 * Линия прерывания для пина
 * @param uint16_t _pin - pin number | номер пина
 */
uint32_t v_getExtiLine(uint16_t _pin);


/**
 * Source-pin for interruption
 * Source-пин для прерывания
 * @param uint16_t _pin - pin number | номер пина
 */
uint32_t v_getExtiPinSource(uint16_t _pin);


/**
 * Interrupt Handler Channel
 * Канал обработчика прерывания
 * @param uint16_t _pin - pin number | номер пина
 */
uint32_t v_getExtiIRQ(uint16_t _pin);

/**
 * Port source for pin
 * Источник порта для пина
 * @param uint16_t _pin - pin number | номер пина
 */
uint8_t v_getExtiPortSource(uint16_t _pin);


/**
 * USART Initialization
 * Инициализация USART
 * @param USART_TypeDef* USARTx - USART, which will be initialized | USART, который будет инициализироваться
 * @param uint32_t speed - the speed of data exchange | скорость обмена данными
 * @param uint8_t contact_group - group of pins: 1, 2, or 3 (some USART have alternate pins)
 * @param uint8_t contact_group - группа пинов: 1, 2 или 3 (некоторые USART имеют альтернативные пины)
 */
void v_USART_init(USART_TypeDef* USARTx, uint32_t speed, uint8_t contact_group);

/**
 * Sending a character using USART
 * Отправка символа с помощью USART
 * @param USART_TypeDef* USARTx - USART, which will be used to send data | USART, который будет использован для отправки данных
 * @param uint8_t ch - sent character (the character must be in single quotes) | отправляемый символ (символ должен быть в одинарных кавычках)
 */
void v_USART_sendChar(USART_TypeDef* USARTx, uint8_t ch);

/**
 * Sending a string using USART
 * Отправка строки с помощью USART
 * @param USART_TypeDef* USARTx - - USART, which will be used to send data | USART, который будет использован для отправки данных
 * @param uint8_t *str - send a string (must be in DOUBLE quotes) | отправляемая строка (должна быть в ДВОЙНЫХ кавычках)
 */
void v_USART_sendString(USART_TypeDef* USARTx, uint8_t *str);

/**
 * Sending an integer using USART
 * Отправка целого числа с помощью USARTs
 * @param USART_TypeDef* USARTx - USART, который будет использован для отправки данных
 * @param uint32_t num - отправляемое ЦЕЛОЕ БЕЗЗНАКОВОЕ число
 */
void v_USART_sendNumber(USART_TypeDef* USARTx, uint32_t num);

/**
 * Checking for data in USARTx
 * Проверка на наличие данных в USARTx
 * @param USART_TypeDef* USARTx - checked USART
 * @return bool
 */
bool v_USART_available(USART_TypeDef* USARTx);

/**
 * Read from USART
 * Читаем из USART
 * @param USART_TypeDef* USARTx - USART, with which we work | USART, с которым работаем
 * @return bool
 */
uint16_t v_USART_read(USART_TypeDef* USARTx);

/**
 * Reading the latest (new) data from USART
 * Чтение последних (новых) данных из USART
 * @param USART_TypeDef* USARTx - USART, with which we work | USART, с которым работаем
 * @return bool
 */
uint16_t v_USART_readLast(USART_TypeDef* USARTx);


/**
 * A handler for all interrupts to read data
 * Обработчик всех прерываний для чтения данных
 * @param USART_TypeDef* USARTx - USART, with which we work | USART, с которым работаем
 */
void v_USARTx_IRQHandlerRead(USART_TypeDef* USARTx);

/**
 * Returns the USART number (USART1 - 1, USART2 - 2)
 * Возвращает номер USART (USART1 - 1, USART2 - 2)
 * @param USART_TypeDef* USARTx - USART
 */
uint8_t v_USARTx_number(USART_TypeDef* USARTx);

/**
 * Creates a circular buffer
 * Создает кольцевой буфер
 */
RingBuffer v_new_RingBuffer();

/**
 * Adding a character to the circular buffer
 * Добавление символа в кольцевой буфер
 * @param RingBuffer *buffer - pointer to buffer | указатель на буфер
 * @param uint16_t ch - added character | добавляемый символ
 */
void v_RingBuffer_put(RingBuffer *buffer, uint16_t ch);

/**
 * Getting the first character from the circular buffer
 * Получение первого символа из кольцевого буфера
 * @param RingBuffer *buffer - указатель на буфер
 * @return uint16_t ch - char | символ
 */
uint16_t v_RingBuffer_get(RingBuffer *buffer);


/**
 * Getting the last (new) character from the circular buffer
 * Получение последнего (нового) символа из кольцевого буфера
 * @param RingBuffer *buffer - pointer to buffer | указатель на буфер
 * @return uint16_t ch - char | символ
 */
uint16_t v_RingBuffer_getLast(RingBuffer *buffer);


/**
 * Cleaning the ring buffer
 * Очистка кольцевого буфера
 * @param RingBuffer *buffer - pointer to buffer | указатель на буфер
 */
void v_RingBuffer_clear(RingBuffer *buffer);
#endif
