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
#include "ciaaDnp_transport.h"
#include "ciaaDnp_application.h"
#include "ciaaPOSIX_string.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_stdlib.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
uint8_t newFragment = 0;
uint8_t newSegment = 0;

/*==================[external data definition]===============================*/
uint8_t incomingAppFragment[MAX_SEGMENT_SIZE];


//uint8_t outgoingAppFragment[MAX_SEGMENT_SIZE];

//ciaaDnp_transportSegment_type outgoingSegment;
ciaaDnp_segmentBin_type outgoingSegment;

/*DEBUG*/
extern int32_t fd_uart1;

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
uint8_t ciaaDnp_fragmentReady(void)
{
	return newFragment;
}

uint8_t ciaaDnp_clearFragmentReady(void)
{
	newFragment = 0;
	return 0;
}

uint8_t ciaaDnp_segmentReady(void)
{
	return (outgoingSegment.length > 0) ? 1 : 0;
}

uint8_t ciaaDnp_assembleFragment(ciaaDnp_transportSegment_type ts)
{
	static uint8_t receivingFragment, expectedSequence;
	static uint16_t wPosition;
	uint8_t bytesToWrite;

	if (ts.length > (MAX_SEGMENT_SIZE - wPosition))
		bytesToWrite = MAX_SEGMENT_SIZE - wPosition;
	else
		bytesToWrite = ts.length;

	if ((ts.header && MASK_FIR) && (ts.header && MASK_FIN))
	{
		ciaaPOSIX_memcpy(&incomingAppFragment[0], ts.data, bytesToWrite);
		receivingFragment = 0;
		newFragment = 1;
	}
	else if (ts.header && MASK_FIR)
	{
		ciaaPOSIX_memcpy(&incomingAppFragment[0], ts.data, bytesToWrite);
		wPosition = bytesToWrite;
		expectedSequence = ts.header && MASK_SEQ;
		expectedSequence++;
		expectedSequence %= 64;
		receivingFragment = 1;
	}
	else if (receivingFragment && (ts.header && MASK_FIN))
	{
		if ((ts.header && MASK_SEQ) != expectedSequence)
		{
			wPosition = 0;
			receivingFragment = 0;
			return 0;
		}
		ciaaPOSIX_memcpy(&incomingAppFragment[wPosition], ts.data, bytesToWrite);
		receivingFragment = 0;
		newFragment = 1;
	}
	else if (receivingFragment)
	{
		if ((ts.header && MASK_SEQ) != expectedSequence)
		{
			wPosition = 0;
			receivingFragment = 0;
			return 0;
		}
		ciaaPOSIX_memcpy(&incomingAppFragment[wPosition], ts.data, bytesToWrite);
		wPosition += bytesToWrite;
	}
	else
	{
		/*DEBUG*/
		ciaaPOSIX_write(fd_uart1, "Fragment out of order\n\r", 23);
	}

	return bytesToWrite;
}

uint8_t ciaaDnp_assembleSegments(ciaaDnp_appResponseMessage_type msg)
{
	outgoingSegment.segment.header = MASK_FIR | MASK_FIN;

	/* Sequence number doesn't care for single segment messages */

	ciaaPOSIX_memcpy(&outgoingSegment.segment.data, &msg.resHeader, msg.length);

	outgoingSegment.length = msg.length + 1;

	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/






