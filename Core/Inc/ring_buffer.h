/*
 * ring_buffer.h
 *
 *  Created on: Mar 30, 2021
 *      Author: Yevhenii Syrotenko
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_


#include "main.h"

// Возможные состояния заполнения
typedef struct
{
	unsigned MessageReady 	: 1;
	unsigned BufferOverrun 	: 1;
}uart_flag_t;

// Тип описывающий кольцовой буфер
typedef struct
{
    uint8_t 	*buffer;		//< Указатель на буфер-хранилище
    uint16_t 	idxIn;          //< Номер ячейки для записи
    uint16_t 	idxOut;         //< Номер ячейки для чтения
    uint16_t 	size;           //< Размер Буфера-хранилища
    uart_flag_t flag;			//< Флажок состояния
} RING_buffer_t;


// Возможные состояния выполнения функции инициализации
typedef enum
{
    RING_ERROR = 0,             //< Значение если произошла ошибка
    RING_SUCCESS = !RING_ERROR  //< Значение если всё прошло удачно
} RING_ErrorStatus_t;



// @function RING_Init − Инициализация буфера
RING_ErrorStatus_t RING_Init(RING_buffer_t *ring, uint8_t *buf, uint16_t size);

// @function RING_Put − Загружает элемент в буфер.
void RING_Put(uint8_t symbol, RING_buffer_t* buf);


// @function RING_Pop − Получает из буфера байт.
uint8_t RING_Pop(RING_buffer_t *buf);


// @function RING_GetCount − Количество полезных данных в буфере.
uint16_t RING_GetCount(RING_buffer_t *buf);


// @function RING_ShowSymbol − Показывает содержимое элемента без его удаления из буфера.
int32_t RING_ShowSymbol(uint16_t symbolNumber ,RING_buffer_t *buf);


// @function RING_Clear − Очищает буфер.
void RING_Clear(RING_buffer_t* buf);


// @function RING_PutBuffr − Загружает элементы из массива-источника в кольцевой буфер.
void RING_PutBuffr(RING_buffer_t *ringbuf, uint8_t *src, uint16_t len);


// @function RING_PopBuffr − Заполняет элементами кольцевого буфера массив.
void RING_PopBuffr(RING_buffer_t *ringbuf, uint8_t *destination, uint16_t len);


// @function Ring_GetMessage − Reads full message from the ring buffer and clears appropriate flags.
uint8_t Ring_GetMessage(RING_buffer_t * ringbuf, uint8_t * string);


#endif /* INC_RING_BUFFER_H_ */
