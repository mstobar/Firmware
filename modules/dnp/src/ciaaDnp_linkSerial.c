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
#include "ciaaPOSIX_stdint.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_string.h"
#include "ciaaDnp_link.h"
#include "ciaaDnp_linkSerial.h"
#include "ciaaDnp_ringBuffer.h"
#include "ciaaDnp_crc.h"
#include "ciaaDnp.h"
#include "ciaaDnp_transport.h"
/*==================[macros and definitions]=================================*/
typedef enum
{
	SYNC,
	HEADER,
	DATA,
	DONE
}ciaaDnp_linkParserStateEnum;

typedef struct
{
	uint8_t data[292];
	uint32_t rPosition;
	uint32_t wPosition;
}ciaaDnp_shiftBuffer_type;

/*==================[internal data declaration]==============================*/

ciaaDnp_linkParserStateEnum parserState;
ciaaDnp_shiftBuffer_type buffer;
ciaaDnp_headerBlock_type header;
ciaaDnp_linkFrame_type lastInputFrame;

/*==================[internal functions declaration]=========================*/

uint32_t ciaaDnp_bytesRead(ciaaDnp_shiftBuffer_type* buf);
void ciaaDnp_shiftBuffer(ciaaDnp_shiftBuffer_type* buf);
void ciaaDnp_resetBuffer(ciaaDnp_shiftBuffer_type* buf);
_Bool ciaaDnp_parseSync(ciaaDnp_shiftBuffer_type* buf);
_Bool ciaaDnp_parseHeader(ciaaDnp_shiftBuffer_type* buf);
_Bool ciaaDnp_parseData(ciaaDnp_shiftBuffer_type* buf);
uint32_t ciaaDnp_frameSize(ciaaDnp_headerBlock_type* hdr);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/
extern ciaaDnp_ringBuffer_type inputFrameQueue;
extern connectionVariables_type connection;

extern ciaaDnp_segmentBin_type outgoingSegment;

/*DEBUG*/
extern int32_t fd_uart1;

/*==================[internal functions definition]==========================*/

int8_t ciaaDnp_linkParser(int32_t fd)
{

	//char buffer[1465];
	// char temp_char;
	int32_t byte_count;

	ciaaDnp_shiftBuffer(&buffer);

	if (buffer.wPosition < 292)
	{
		byte_count = ciaaPOSIX_read(fd, buffer.data, 292 - buffer.wPosition);
		if (byte_count > 0) buffer.wPosition += byte_count;
	}

	//ciaaDnp_enqueueRingBuffer(&inputFrameQueue, &lastInputFrame);

	switch (parserState)
	{
		case SYNC:
			if (ciaaDnp_bytesRead(&buffer) >= 10 && ciaaDnp_parseSync(&buffer))
			{
				parserState = HEADER;
			}
			else
			{
				/*DEBUG*/
				ciaaPOSIX_write(fd_uart1, "Fail frame sync\n\r", 17);
				break;
			}
		case HEADER:
			if (ciaaDnp_bytesRead(&buffer) >= 10 && ciaaDnp_parseHeader(&buffer))
			{
				parserState = DATA;
			}
			else
			{
				/*DEBUG*/
				ciaaPOSIX_write(fd_uart1, "Fail parse header\n\r", 19);
				ciaaDnp_resetBuffer(&buffer);
				parserState = SYNC;
				break;
			}
		case DATA:
			if (ciaaDnp_parseData(&buffer))
			{
				parserState = DONE;
			}
			else
			{
				/*DEBUG*/
				ciaaPOSIX_write(fd_uart1, "Fail parse data\n\r", 17);
				ciaaDnp_resetBuffer(&buffer);
				parserState = SYNC;
				break;
			}
		case DONE:
			/* Data to upper layer?*/

			ciaaPOSIX_memcpy(&lastInputFrame, &buffer.data[buffer.rPosition], ciaaDnp_frameSize(&header));

			ciaaDnp_enqueueRingBuffer(&inputFrameQueue, &lastInputFrame);

			ciaaDnp_resetBuffer(&buffer);
			parserState = SYNC;
			break;
		default:
			break;
	}
	return 0;
}

void ciaaDnp_initLinkParser(ciaaDnp_shiftBuffer_type* buf)
{
	parserState = SYNC;
	ciaaDnp_initRingBuffer(&inputFrameQueue);
	ciaaDnp_resetBuffer(&buffer);
}

/* Shift buffer */
uint32_t ciaaDnp_bytesRead(ciaaDnp_shiftBuffer_type* buf)
{
	return (buf->wPosition - buf->rPosition);
}

void ciaaDnp_advanceRead(ciaaDnp_shiftBuffer_type* buf, uint32_t numBytes)
{
	if (numBytes > ciaaDnp_bytesRead(buf)) numBytes = ciaaDnp_bytesRead(buf);

	buf->rPosition += numBytes;
}

void ciaaDnp_shiftBuffer(ciaaDnp_shiftBuffer_type* buf)
{
	uint32_t i;

	for (i=buf->rPosition; i < buf->wPosition; i++)
	{
		buf->data[i-buf->rPosition] = buf->data[i];
	}
	buf->rPosition = 0;
	buf->wPosition = i;
}

void ciaaDnp_resetBuffer(ciaaDnp_shiftBuffer_type* buf)
{
	buf->rPosition = 0;
	buf->wPosition = 0;
}

_Bool ciaaDnp_parseSync(ciaaDnp_shiftBuffer_type* buf)
{
	while (ciaaDnp_bytesRead(buf) > 1)
	{
		if (buf->data[buf->rPosition] == 0x05 && buf->data[buf->rPosition+1] == 0x64)
			return 1;
		else
			ciaaDnp_advanceRead(buf, 1);
	}
	return 0;
}

_Bool ciaaDnp_parseHeader(ciaaDnp_shiftBuffer_type* buf)
{
	header.length = buf->data[buf->rPosition + 2];
	header.control = buf->data[buf->rPosition + 3];
	header.destination = (uint16_t) buf->data[buf->rPosition + 4];
	header.source = (uint16_t) buf->data[buf->rPosition + 6];

	if (ciaaDnp_bytesRead(buf) >= 10)
	{
		/* CRC */
		if (!verifyCRC(&buf->data[buf->rPosition], 8))
			return 0;

		/* Length */
		if (!(header.length > 4))
			return 0;

		/* Function code */
		if (header.control & MASK_PRM) /* Primary to Secondary */
		{
			switch (header.control & MASK_FC)
			{
				case CIAA_DNP_PRI_FC_TEST_LINK_STATES:
				case CIAA_DNP_PRI_FC_CONFIRMED_USER_DATA:
				case CIAA_DNP_PRI_FC_RESET_LINK_STATES:
				case CIAA_DNP_PRI_FC_UNCONFIRMED_USER_DATA:
				case CIAA_DNP_PRI_FC_REQUEST_LINK_STATUS:
					break;
				default:
					return 0;
			}
		}
		else /* Secondary to Primary */
		{
			switch (header.control)
			{
				case CIAA_DNP_SEC_FC_ACK:
				case CIAA_DNP_SEC_FC_NACK:
				case CIAA_DNP_SEC_FC_LINK_STATUS:
				case CIAA_DNP_SEC_FC_NOT_SUPPORTED:
					break;
				default:
					return 0;
			}
		}
		/* Destination address */
		if ((header.destination != connection.slaveAddress))
			return 0;
		return 1;
	}
	return 0;
}

_Bool ciaaDnp_parseData(ciaaDnp_shiftBuffer_type* buf)
{
	uint32_t i;
	uint32_t modulo, dataBlocks;

	if (ciaaDnp_bytesRead(buf) < ciaaDnp_frameSize(&header))
		return 0;

	if (header.length < 6)
		return 1;

	modulo = (header.length - 5) % 16;
	dataBlocks = (header.length - 5) / 16;

	for (i=0 ; i<dataBlocks ; i++)
	{
		if (!verifyCRC(&buf->data[buf->rPosition+10+(18*i)], 16))
			return 0;
	}
	if (!verifyCRC(&buf->data[buf->rPosition+10+(18*dataBlocks)], modulo))
		return 0;

	return 1;
}

uint32_t ciaaDnp_frameSize(ciaaDnp_headerBlock_type* hdr)
{
	uint32_t dataSize, modulo, dataBlocks;

	dataSize = hdr->length - 5;

	if (!dataSize) return 10;

	modulo = dataSize % 16;
	dataBlocks = dataSize / 16;

	return (modulo > 0) ? (10 + dataBlocks * 18 + modulo + 2) : (10 + dataBlocks * 18);
}

uint32_t ciaaDnp_framePayloadSize(ciaaDnp_headerBlock_type* hdr)
{
	uint32_t dataSize, modulo, dataBlocks;

	dataSize = hdr->length - 5;

	if (!dataSize) return 10;

	modulo = dataSize % 16;
	dataBlocks = dataSize / 16;

	return (modulo > 0) ? (10 + dataBlocks * 18 + modulo + 2) : (10 + dataBlocks * 18);
}

/*==================[external functions definition]==========================*/
int8_t ciaaDnp_queueAck(int32_t fd)
{
	ciaaDnp_headerBlock_type ackBlock;

	ackBlock.sync = 0x6405;
	ackBlock.length = 0x05;
	ackBlock.control = 0x00;
	ackBlock.destination = 0x0008;
	ackBlock.source = 0x0004;
	ackBlock.crc = 0xA952;

	ciaaPOSIX_write(fd, &ackBlock, 10);

	return 0;
}

int8_t ciaaDnp_queueLinkStatus(int32_t fd)
{
	ciaaDnp_headerBlock_type ackBlock;

	ackBlock.sync = 0x6405;
	ackBlock.length = 0x05;
	ackBlock.control = 0x0B;
	ackBlock.destination = 0x0008;
	ackBlock.source = 0x0004;
	ackBlock.crc = 0x9911;

	ciaaPOSIX_write(fd, &ackBlock, 10);

	return 0;
}

int8_t ciaaDnp_queueLastXAck(void)
{
	return 0;
}

int8_t ciaaDnp_queueResetLinkStates(void)
{
	return 0;
}

int8_t ciaaDnp_queueConfirmedUserData(void)
{
	return 0;
}

int8_t ciaaDnp_queueUnconfirmedUserData(int32_t fd)
{
	ciaaDnp_linkFrame_type frame;
	uint16_t i;
	uint8_t* ptr;

	uint32_t modulo, dataBlocks;

	frame.headerBlock.sync = 0x6405;
	frame.headerBlock.length = outgoingSegment.length + 5;
	frame.headerBlock.control = MASK_PRM | CIAA_DNP_PRI_FC_UNCONFIRMED_USER_DATA;
	frame.headerBlock.destination = connection.masterAddress;
	frame.headerBlock.source = connection.slaveAddress;
	generateCRC((uint8_t*)&frame.headerBlock.sync, 8);

	modulo = outgoingSegment.length % 16;
	dataBlocks = outgoingSegment.length / 16;

	ptr = &outgoingSegment.segment.header;
	for (i=0; i<dataBlocks;i++)
	{
		ciaaPOSIX_memcpy(frame.dataBlock[i].data, &ptr[i*16], 16);
		generateCRC(frame.dataBlock[i].data, 16);
	}

	ciaaPOSIX_memcpy(frame.dataBlock[i].data, &ptr[i*16], modulo);
	generateCRC(frame.dataBlock[i].data, modulo);

	ciaaPOSIX_write(fd, (uint8_t*) &frame, ciaaDnp_frameSize(&frame.headerBlock));

	outgoingSegment.length = 0;

	return 0;
}

int8_t ciaaDnp_queueRequestLinkStatus(void)
{
	return 0;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/





