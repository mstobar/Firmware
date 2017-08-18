/* Copyright 2014, Your Name <youremail@domain.com>
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Short description of this file
 **
 ** Long description of this file
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "ciaaDnp_ringBuffer.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

ciaaDnp_headerBlock_type header;
ciaaDnp_ringBuffer_type inputFrameQueue;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void ciaaDnp_initRingBuffer(ciaaDnp_ringBuffer_type* rb)
{
	rb->size = sizeof(rb->data)/sizeof(ciaaDnp_linkFrame_type);
	rb->rPosition = 0;
	rb->wPosition = 0;
	rb->elements = 0;
}

uint8_t ciaaDnp_enqueueRingBuffer(ciaaDnp_ringBuffer_type* rb, ciaaDnp_linkFrame_type* frame)
{
	rb->data[rb->wPosition] = *frame;
	rb->wPosition++;
	if (++rb->elements > rb->size) rb->elements = rb->size;
	rb->wPosition %= rb->size;

	return ciaaDnp_isFullRingBuffer(rb);
}
uint8_t ciaaDnp_dequeueRingBuffer(ciaaDnp_ringBuffer_type* rb, ciaaDnp_linkFrame_type* frame)
{
	*frame = rb->data[rb->rPosition];
	rb->rPosition++;
	if (--rb->elements < 0) rb->elements = 0;
	rb->rPosition %= rb->size;

	return ciaaDnp_isEmptyRingBuffer(rb);
}
uint8_t ciaaDnp_isFullRingBuffer(ciaaDnp_ringBuffer_type* rb)
{
	return (rb->elements >= rb->size);
}
uint8_t ciaaDnp_isEmptyRingBuffer(ciaaDnp_ringBuffer_type* rb)
{
	return (!rb->size);
}
/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/





