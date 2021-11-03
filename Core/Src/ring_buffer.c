/*
 * ring_buffer.c
 *
 *  Created on: Mar 30, 2021
 *      Author: Yevhenii Syrotenko
 */

#include "ring_buffer.h"


/**
@function RING_Init − Инициализация буфера.
@param RING_buffer_t *ring − Указатель на кольцевой буфер.
@param uint8_t *buf − Указатель на буфер хранения.
@param uint16_t size − Сколько элементов в буфере.
@return RING_ErrorStatus_t Результат инициализации @ref RING_ErrorStatus_t
 */
RING_ErrorStatus_t RING_Init(RING_buffer_t *ring, uint8_t *buf, uint16_t size)
{
    ring->size = size;
    ring->buffer = buf;
    RING_Clear( ring );

    return ( ring->buffer ? RING_SUCCESS : RING_ERROR ) ;
}

/**
@function RING_Clear − Очищает буфер.
@param RING_buffer_t *buf − Указатель на кольцевой буфер.
 */
void RING_Clear(RING_buffer_t* buf)
{
    buf->idxIn = 0;
    buf->idxOut = 0;
	buf->flag.BufferOverrun = 0;
	buf->flag.MessageReady = 0;
}

/**
@function RING_Put − Загружает элемент в буфер.
@param uint8_t symbol − Элемент для загрузки в буфер.
@param RING_buffer_t *buf − Указатель на кольцевой буфер.
 */
void RING_Put(uint8_t symbol, RING_buffer_t* buf)
{
    buf->buffer[buf->idxOut++] = symbol;
    if (buf->idxOut >= buf->size) buf->idxOut = 0;
}

/**
@function RING_Pop − Получает из буфера байт.
@param RING_buffer_t *buf − Указатель на кольцевой буфер.
@return uint8_t Значение полученого элемента.
 */
uint8_t RING_Pop(RING_buffer_t *buf)
{
    uint8_t retval = buf->buffer[buf->idxIn++];
    if (buf->idxIn >= buf->size) buf->idxIn = 0;
    return retval;
}

/**
@function RING_GetCount − Количество полезных данных в буфере.
@param RING_buffer_t *buf − Указатель на кольцевой буфер.
@return uint16_t Количество полезных данных в буфере.
 */
uint16_t RING_GetCount(RING_buffer_t *buf)
{
    uint16_t retval = 0;
    if (buf->idxIn < buf->idxOut) retval = buf->size + buf->idxIn - buf->idxOut;
    else retval = buf->idxIn - buf->idxOut;
    return retval;
}

/**
@function RING_ShowSymbol − Показывает содержимое элемента без его удаления из буфера.
@param RING_buffer_t *ringbuf − Указатель на кольцевой буфер.
@param uint16_t symbolNumbe − Номер элемента.
@return int32_t Значение полученого элемента. -1 если ошибка.
 */
int32_t RING_ShowSymbol(uint16_t symbolNumber ,RING_buffer_t *buf)
{
    uint32_t pointer = buf->idxOut + symbolNumber;
    int32_t  retval = -1;
    if (symbolNumber < RING_GetCount(buf))
    {
        if (pointer > buf->size) pointer -= buf->size;
        retval = buf->buffer[ pointer ] ;
    }
    return retval;
}

/**
@function RING_PutBuffr − Загружает элементы из массива-источника в кольцевой буфер.
@param RING_buffer_t *ringbuf − Указатель на кольцевой буфер.
@param uint8_t *src − Указатель на массив-источник.
@param uint16_t len − количество загружаемых элементов.
 */
void RING_PutBuffr(RING_buffer_t *ringbuf, uint8_t *src, uint16_t len)
{
    while(len--) RING_Put(*(src++), ringbuf);
}

/**
@function RING_PopBuffr − Заполняет элементами кольцевого буфера массив.
@param RING_buffer_t *ringbuf − Указатель на кольцевой буфер.
@param uint8_t *destination − Указатель на массив.
@param uint16_t len − количество получаемых элементов.
 */
void RING_PopBuffr(RING_buffer_t *ringbuf, uint8_t *destination, uint16_t len)
{
    while(len--) *(destination++) = RING_Pop(ringbuf);
}


/**
@function Ring_GetMessage − Reads full message from the ring buffer and clears appropriate flags.
@param RING_buffer_t *ringbuf − Указатель на кольцевой буфер.
@param uint8_t *string − Указатель на строчку.
 */
uint8_t Ring_GetMessage(RING_buffer_t *ringbuf, uint8_t * string)
{
	uint16_t char_count = 0;
	// Check if the message has been received
	if (ringbuf->flag.MessageReady)
		{
			if (ringbuf->flag.BufferOverrun)
				{
				ringbuf->idxIn = ringbuf->idxOut;
				ringbuf->flag.BufferOverrun = 0;
				}
			while ((ringbuf->buffer[ringbuf->idxIn] != '\r') &&
						 (ringbuf->buffer[ringbuf->idxIn] != '\n') &&
						 (ringbuf->size != char_count - 1))
				{
					*string =  RING_Pop(ringbuf);
					string++;
					char_count++;
				}
			*string =  RING_Pop(ringbuf);
			string++;
			char_count++;
			*string = '\0';
			ringbuf->flag.MessageReady = 0;
		}
	return char_count;
}
