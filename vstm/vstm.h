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
	uint8_t start;      //"указатель" хвоста кольцевого буфера
	uint8_t end ;   // "указатель" головы буфера (указывает на следующий, еще не сущ. символ)
	uint8_t length ;  //счетчик символов

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
 * Начальная настройка контроллера
 * @param uint16_t _pin - Номер пина, с которого считывается сигнал
 * @return uint16_t - значение сигнала
 */
void v_init();

/**
 * Возвращает порт указанного пина (A, B, C, D, E)
 * @param uint16_t _pin - номер пина
 * @return port - значение сигнала
 */
v_port v_getPort(uint16_t pin);

/**
 * Возвращает STM-овский пин указанного пина (GPIO_Pin_xx)
 * @param uint16_t _pin - номер пина
 * @return uint16_t - пин
 */
uint16_t v_getPin(uint16_t _pin);

/**
 * Функция для получения числовой части номера пина
 * @param uint16_t _pin - номер пина
 * @return uint8_t - число номера пина
 */
uint8_t v_getPinNumber(uint16_t _pin);

/**
 * Возвращает STM-овский источник пина (GPIO_PinSourceXX)
 * @param uint16_t _pin - номер пина
 * @return uint16_t - источник пина
 */
uint8_t v_getPinSource(uint16_t _pin);

/**
 * Конфигурация пина на соответвтвущий режим работы
 * @param uint16_t _pin - номер пина
 * @param mode m - режим работы
 */
void pinMode(uint16_t _pin, mode m);

/**
 * Чтение цифрового сигнала с указанного пина
 * @param uint16_t _pin - номер пина
 * @return bool - значение сигнала
 */
uint8_t digitalRead(uint16_t _pin);

/**
 * Запись HIGH (true) уровня или LOW (false) на указанный pin
 * @param uint16_t _pin - номер пина
 * @param bool status - значение сигнала
 */
void digitalWrite(uint16_t _pin, bool status);

/**
 * Считывание аналогового сигнала
 * @param uint16_t _pin - Номер пина, с которого считывается сигнал
 * @return uint16_t - значение сигнала
 */
u16 analogRead(uint16_t _pin);

/**
 * Функция записи ШИМа на указанном пине
 * @param uint16_t _pin - номер пина
 * @param uint16_t _pin - ширина импульса (до 1024)
 */
void analogWrite(uint16_t _pin, uint16_t val);

/**
 * Функция настройки таймера под ШИМ
 * @param IM_TypeDef *time - таймер для настройки
 * @param uint8_t number - номер таймера
 */
void v_configTimerPWM(TIM_TypeDef *timer, uint8_t number);


/**
 * Включение альтернативной функции у пина
 * @param uint16_t _pin - номер пина
 * @param uint8_t af - альтернативная функция (GPIO_AF_TIM3, GPIO_AF_USART1)
 */
void v_pinAFconfig(uint16_t _pin, uint8_t af);


/**
 * Альтернативная функция для пина - Таймеры
 * @param uint16_t _pin - номер пина
 */
uint8_t v_getAF_Timer(uint16_t _pin);

/**
 * Альтернативная функция для пина - USART
 * @param uint16_t _pin - номер пина
 */
uint8_t v_getAF_USART(uint16_t _pin);

/**
 * Линия прерывания для пина
 * @param uint16_t _pin - номер пина
 */
uint32_t v_getExtiLine(uint16_t _pin);


/**
 * Source-пин для прерывания
 * @param uint16_t _pin - номер пина
 */
uint32_t v_getExtiPinSource(uint16_t _pin);


/**
 * Канал обработчика прерывания
 * @param uint16_t _pin - номер пина
 */
uint32_t v_getExtiIRQ(uint16_t _pin);

/**
 * Источние порта для пина
 * @param uint16_t _pin - номер пина
 */
uint8_t v_getExtiPortSource(uint16_t _pin);


/**
 * Инициализация USART
 * @param USART_TypeDef* USARTx - USART, который будет инициализироваться
 * @param uint32_t speed - скорость обмена данными
 * @param uint8_t contact_group - группа пинов: 1, 2 или 3 (некоторые USART имеют альтернативные пины)
 */
void v_USART_init(USART_TypeDef* USARTx, uint32_t speed, uint8_t contact_group);

/**
 * Отправка символа с помощью USART
 * @param USART_TypeDef* USARTx - USART, который будет использован для отправки данных
 * @param uint8_t ch - отправляемый символ (символ должен быть в одинарных кавычках)
 */
void v_USART_sendChar(USART_TypeDef* USARTx, uint8_t ch);

/**
 * Инициализация USART
 * @param USART_TypeDef* USARTx - USART, который будет использован для отправки данных
 * @param uint8_t *str - отправляемая строка (должна быть в ДВОЙНЫХ кавычках
 */
void v_USART_sendString(USART_TypeDef* USARTx, uint8_t *str);

/**
 * Инициализация USART
 * @param USART_TypeDef* USARTx - USART, который будет использован для отправки данных
 * @param uint32_t num - отправляемое ЦЕЛОЕ БЕЗЗНАКОВОЕ число
 */
void v_USART_sendNumber(USART_TypeDef* USARTx, uint32_t num);

/**
 * Проверяем, есть ли данные в USARTx
 * @param USART_TypeDef* USARTx - проверяемый USART
 * @return bool
 */
bool v_USART_available(USART_TypeDef* USARTx);

/**
 * Читаем из USART
 * @param USART_TypeDef* USARTx - USART, с которым работаем
 * @return bool
 */
uint16_t v_USART_read(USART_TypeDef* USARTx);

/**
 * Читаем из USART
 * @param USART_TypeDef* USARTx - USART, с которым работаем
 * @return bool
 */
uint16_t v_USART_readLast(USART_TypeDef* USARTx);


/**
 * Обработчик всех прерываний для чтения данных
 * @param USART_TypeDef* USARTx - USART, с которым работаем
 */
void v_USARTx_IRQHandlerRead(USART_TypeDef* USARTx);

/**
 * Возвращает номер USART (USART1 - 1, USART2 - 2)
 * @param USART_TypeDef* USARTx - USART, с которым работаем
 */
uint8_t v_USARTx_number(USART_TypeDef* USARTx);

/**
 * Создает кольевой буфер
 */
RingBuffer v_new_RingBuffer();

/**
 * Добавление символа в кольцевой буффер
 * @param RingBuffer *buffer - указатель на буфер
 * @param uint16_t ch - добавляемый символ
 */
void v_RingBuffer_put(RingBuffer *buffer, uint16_t ch);

/**
 * Получение первого символа из кольцевого буфера
 * @param RingBuffer *buffer - указатель на буфер
 * @return uint16_t ch - символ
 */
uint16_t v_RingBuffer_get(RingBuffer *buffer);


/**
 * Получение последнего символа из кольцевого буфера
 * @param RingBuffer *buffer - указатель на буфер
 * @return uint16_t ch - символ
 */
uint16_t v_RingBuffer_getLast(RingBuffer *buffer);


/**
 * Очистка кольцевого буфера
 * @param RingBuffer *buffer - указатель на буфер
 */
void v_RingBuffer_clear(RingBuffer *buffer);
#endif
