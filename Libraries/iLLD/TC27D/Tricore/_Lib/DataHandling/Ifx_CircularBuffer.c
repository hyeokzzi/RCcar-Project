/**
 * \file Ifx_CircularBuffer.c
 * \brief Circular buffer functions in C language.
 *
 * \version iLLD_1_0_1_16_1
 * \copyright Copyright (c) 2013 Infineon Technologies AG. All rights reserved.
 *
 *
 *                                 IMPORTANT NOTICE
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees. If and as long as no such terms
 * of use are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer, must
 * be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are
 * solely in the form of machine-executable object code generated by a source
 * language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include "Ifx_CircularBuffer.h"

#if (IFX_CFG_CIRCULARBUFFER_C)

uint32 Ifx_CircularBuffer_get32(Ifx_CircularBuffer *buffer)
{
    uint32 data = ((uint32 *)buffer->base)[buffer->index];

    buffer->index += 4;

    if (buffer->index >= buffer->length)
    {
        buffer->index = 0;
    }

    return data;
}


uint16 Ifx_CircularBuffer_get16(Ifx_CircularBuffer *buffer)
{
    uint16 data = ((uint16 *)buffer->base)[buffer->index];

    buffer->index += 2;

    if (buffer->index >= buffer->length)
    {
        buffer->index = 0;
    }

    return data;
}


/** \brief Add a 32 bit value to the circular buffer, and post-increment the circular buffer pointer
 *
 * \param buffer Specifies circular buffer.
 * \param data Specifies value to be added to the buffer.
 *
 * \return None.
 */
void Ifx_CircularBuffer_addDataIncr(Ifx_CircularBuffer *buffer, uint32 data)
{
    ((uint32 *)buffer->base)[buffer->index] = data;
    buffer->index                          += 4;

    if (buffer->index >= buffer->length)
    {
        buffer->index = 0;
    }
}


void *Ifx_CircularBuffer_read8(Ifx_CircularBuffer *buffer, void *data, Ifx_SizeT count)
{
    uint8 *Dest = (uint8 *)data;

    do
    {
        count--;
        *Dest = ((uint8 *)buffer->base)[buffer->index];
        Dest  = &Dest[1];
        buffer->index++;

        if (buffer->index >= buffer->length)
        {
            buffer->index = 0;
        }
    } while (count > 0);

    return Dest;
}


void *Ifx_CircularBuffer_read32(Ifx_CircularBuffer *buffer, void *data, Ifx_SizeT count)
{
    uint32 *Dest = (uint32 *)data;
    uint8  *base = buffer->base;

    do
    {
        *Dest         = *((uint32 *)(&base[buffer->index]));
        Dest          = &Dest[1];
        buffer->index = buffer->index + 4;

        if (buffer->index >= buffer->length)
        {
            buffer->index = 0;
        }

        count--;
    } while (count > 0);

    return Dest;
}


const void *Ifx_CircularBuffer_write8(Ifx_CircularBuffer *buffer, const void *data, Ifx_SizeT count)
{
    const uint8 *source = (const uint8 *)data;

    do
    {
        count--;
        ((uint8 *)buffer->base)[buffer->index] = *source;
        source                                 = &source[1];
        buffer->index++;

        if (buffer->index >= buffer->length)
        {
            buffer->index = 0;
        }
    } while (count > 0);

    return source;
}


const void *Ifx_CircularBuffer_write32(Ifx_CircularBuffer *buffer, const void *data, Ifx_SizeT count)
{
    const uint32 *source = (const uint32 *)data;
    uint8        *base   = buffer->base;

    do
    {
        *((uint32 *)(&base[buffer->index])) = *source;
        source                              = &source[1];
        buffer->index                       = buffer->index + 4;

        if (buffer->index >= buffer->length)
        {
            buffer->index = 0;
        }

        count--;
    } while (count > 0);

    return source;
}


#endif
