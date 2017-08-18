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
#include "ciaaDnp_application.h"
#include "ciaaDnp.h"
#include "ciaaDnp_transport.h"
#include "ciaaPOSIX_stdio.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
int8_t ciaaDnp_appParser(void);
int8_t ciaaDnp_parseObjectHeader(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_read(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_select(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_operate(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_directOperate(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_directOperateNR(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_coldRestart(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_enqueueResponseMessage(uint8_t group, uint8_t variation);
int8_t ciaaDnp_enqueueG12V1ResponseMessage(ciaaDnp_objectHeader_type* firstObjHeader);
int8_t ciaaDnp_enqueueG60V1ResponseMessage(void);

ciaaDnp_groupVariationEnum ciaaDnp_groupVariation(uint8_t group, uint8_t variation);
void ciaaDnp_executeColdRestart (void);
/*==================[internal data definition]===============================*/
uint8_t select_do;
uint8_t coldRestartReq;
/*==================[external data definition]===============================*/
extern connectionVariables_type connection;

extern uint8_t incomingAppFragment[MAX_SEGMENT_SIZE];

ciaaDnp_appResponseMessage_type outgoingAppFragment;

extern int32_t fd_in, fd_out;

/*DEBUG*/
extern int32_t fd_uart1;

/*==================[internal functions definition]==========================*/
static uint8_t lastSequence;

/*==================[external functions definition]==========================*/

int8_t ciaaDnp_appTask(int32_t fd)
{
	if (coldRestartReq)
		ciaaDnp_executeColdRestart ();

	/* Outstation State Machine */
	switch(connection.outstationState)
    {
    	case CIAA_DNP_OUT_S_IDLE:
    		if (ciaaDnp_fragmentReady())
    		{
    			ciaaDnp_clearFragmentReady();
    			ciaaDnp_appParser();
    		}
    		break;
    	case CIAA_DNP_OUT_S_SOLCONFWAIT:
    	case CIAA_DNP_PRI_S_UNSOLCONFWAIT:

    	default:
    		break;
    }
	return 0;
}

int8_t ciaaDnp_appParser(void)
{
	ciaaDnp_appRequestMessage_type* appReqMessage;

	appReqMessage = (ciaaDnp_appRequestMessage_type*) &incomingAppFragment;

	lastSequence = appReqMessage->reqHeader.control & AH_MASK_SEQ;

	/*TODO Application control*/

	switch (appReqMessage->reqHeader.functionCode)
	{
		case READ:
			ciaaDnp_read(appReqMessage->firstObjectHeader);
			break;
		case SELECT:
			ciaaDnp_select(appReqMessage->firstObjectHeader);
			break;
		case OPERATE:
			ciaaDnp_operate(appReqMessage->firstObjectHeader);
			break;
		case DIRECT_OPERATE:
			ciaaDnp_directOperate(appReqMessage->firstObjectHeader);
			break;
		case DIRECT_OPERATE_NR:
			ciaaDnp_directOperateNR(appReqMessage->firstObjectHeader);
			break;
		case COLD_RESTART:
			ciaaDnp_coldRestart(appReqMessage->firstObjectHeader);
			break;

		default:
			break;
	}

	return 0;
}

int8_t ciaaDnp_select(ciaaDnp_objectHeader_type* firstObjHeader)
{
	ciaaDnp_ciaaG12V1ResMessage_type g12v1;
	ciaaDnp_ciaaDOCommand_type* cmd;
	uint32_t i, count;

	if (ciaaDnp_groupVariation(firstObjHeader->group, firstObjHeader->variation) == GROUP12VAR1)
	{
		if (firstObjHeader->qualifier == ONE_OCTET_IDX_ONE_OCTET_CNT)
		{
			count = firstObjHeader->range[0];
			cmd = (ciaaDnp_ciaaDOCommand_type*) ((uint8_t* )firstObjHeader + 4);
			for (i=0;i<count;i++)
			{
				if (cmd[i*12].count > 1)
					return -1;
				if (cmd[i*12].controlCode == LATCH_ON || cmd[i*12].controlCode == LATCH_OFF)
				{
					select_do |= (0x01 << cmd->index);
				}
				else
					return -1;

				g12v1.doCmd.index = cmd[i*12].index;
				g12v1.doCmd.controlCode = cmd[i*12].controlCode;
				g12v1.doCmd.count = cmd[i*12].count;
				g12v1.doCmd.onTime[0] = cmd[i*12].onTime[0];
				g12v1.doCmd.onTime[1] = cmd[i*12].onTime[1];
				g12v1.doCmd.onTime[2] = cmd[i*12].onTime[2];
				g12v1.doCmd.onTime[3] = cmd[i*12].onTime[3];
				g12v1.doCmd.offTime[0] = cmd[i*12].offTime[0];
				g12v1.doCmd.offTime[1] = cmd[i*12].offTime[1];
				g12v1.doCmd.offTime[3] = cmd[i*12].offTime[2];
				g12v1.doCmd.offTime[4] = cmd[i*12].offTime[3];
				g12v1.doCmd.statusCode = cmd[i*12].statusCode;
			}
		}

		g12v1.resHeader.control = AH_MASK_FIR | AH_MASK_FIN;
		g12v1.resHeader.control |= lastSequence & AH_MASK_SEQ;
		g12v1.resHeader.functionCode = RESPONSE;
		g12v1.resHeader.intIndications1 = 0x00;
		g12v1.resHeader.intIndications2 = 0x00;
		g12v1.group = 12;
		g12v1.variation = 1;
		g12v1.qualifier = firstObjHeader->qualifier;
		g12v1.range[0] = 0x01;

		ciaaPOSIX_memcpy (&outgoingAppFragment, &g12v1, 8+i*12);
		outgoingAppFragment.length = 8+i*12;

		ciaaDnp_assembleSegments(outgoingAppFragment);
	}
return 0;
}

int8_t ciaaDnp_operate(ciaaDnp_objectHeader_type* firstObjHeader)
{
	ciaaDnp_ciaaG12V1ResMessage_type g12v1;
	ciaaDnp_ciaaDOCommand_type* cmd;
	uint32_t i, count;
	uint8_t outputs;

	if (ciaaDnp_groupVariation(firstObjHeader->group, firstObjHeader->variation) == GROUP12VAR1)
	{
		if (firstObjHeader->qualifier == ONE_OCTET_IDX_ONE_OCTET_CNT)
		{
			count = firstObjHeader->range[0];
			cmd = (ciaaDnp_ciaaDOCommand_type*) ((uint8_t* )firstObjHeader + 4);
			for (i=0;i<count;i++)
			{
				if (cmd[i*12].count > 1)
					return -1;

				if (select_do & (0x01 << cmd->index))
				{
					if (cmd[i*12].controlCode == LATCH_ON)
					{
						ciaaPOSIX_read(fd_out, &outputs, 1);
						outputs |= (0x01 << cmd->index);
						ciaaPOSIX_write(fd_out, &outputs, 1);

						select_do &= ~(0x01 << cmd->index);

						g12v1.doCmd.statusCode = cmd[i*12].statusCode;
					}
					else if (cmd[i*12].controlCode == LATCH_OFF)
					{
						ciaaPOSIX_read(fd_out, &outputs, 1);
						outputs &= ~(0x01 << cmd->index);
						ciaaPOSIX_write(fd_out, &outputs, 1);

						select_do &= ~(0x01 << cmd->index);

						g12v1.doCmd.statusCode = cmd[i*12].statusCode;
					}
					else
					{
						return -1;
						/* Fail */
					}
				}
				else
				{
					g12v1.doCmd.statusCode = NO_SELECT;
				}

				g12v1.doCmd.index = cmd[i*12].index;
				g12v1.doCmd.controlCode = cmd[i*12].controlCode;
				g12v1.doCmd.count = cmd[i*12].count;
				g12v1.doCmd.onTime[0] = cmd[i*12].onTime[0];
				g12v1.doCmd.onTime[1] = cmd[i*12].onTime[1];
				g12v1.doCmd.onTime[2] = cmd[i*12].onTime[2];
				g12v1.doCmd.onTime[3] = cmd[i*12].onTime[3];
				g12v1.doCmd.offTime[0] = cmd[i*12].offTime[0];
				g12v1.doCmd.offTime[1] = cmd[i*12].offTime[1];
				g12v1.doCmd.offTime[2] = cmd[i*12].offTime[2];
				g12v1.doCmd.offTime[3] = cmd[i*12].offTime[3];
			}
			g12v1.resHeader.control = AH_MASK_FIR | AH_MASK_FIN;
			g12v1.resHeader.control |= lastSequence & AH_MASK_SEQ;
			g12v1.resHeader.functionCode = RESPONSE;
			g12v1.resHeader.intIndications1 = 0x00;
			g12v1.resHeader.intIndications2 = 0x00;
			g12v1.group = 12;
			g12v1.variation = 1;
			g12v1.qualifier = firstObjHeader->qualifier;
			g12v1.range[0] = 0x01;

			ciaaPOSIX_memcpy (&outgoingAppFragment, &g12v1, 8+i*12);
			outgoingAppFragment.length = 8+i*12;

			ciaaDnp_assembleSegments(outgoingAppFragment);
		}
	}
	return 0;
}


int8_t ciaaDnp_directOperate(ciaaDnp_objectHeader_type* firstObjHeader)
{

	ciaaDnp_directOperateNR(firstObjHeader);
	ciaaDnp_enqueueG12V1ResponseMessage(firstObjHeader);

	return 0;
}

int8_t ciaaDnp_directOperateNR(ciaaDnp_objectHeader_type* firstObjHeader)
{
	uint8_t i, count;
	ciaaDnp_ciaaDOCommand_type* cmd;
	uint8_t outputs;

	if (ciaaDnp_groupVariation(firstObjHeader->group, firstObjHeader->variation) == GROUP12VAR1)
		if (firstObjHeader->qualifier == ONE_OCTET_IDX_ONE_OCTET_CNT)
		{
			count = firstObjHeader->range[0];
			cmd = (ciaaDnp_ciaaDOCommand_type*) ((uint8_t* )firstObjHeader + 4);

			for (i=0;i<count;i++)
			{
				if (cmd[i*11].count > 1)
					return -1;
				if (cmd[i*11].controlCode == LATCH_ON)
				{
					ciaaPOSIX_read(fd_out, &outputs, 1);
					outputs |= (0x01 << cmd->index);
					ciaaPOSIX_write(fd_out, &outputs, 1);
				}
				else if (cmd[i*11].controlCode == LATCH_OFF)
				{
					ciaaPOSIX_read(fd_out, &outputs, 1);
					outputs &= ~(0x01 << cmd->index);
					ciaaPOSIX_write(fd_out, &outputs, 1);
				}
				else
					return -1;
			}
		}
	return 0;
}

int8_t ciaaDnp_coldRestart(ciaaDnp_objectHeader_type* firstObjHeader)
{
	ciaaDnp_ciaaG52V1ResMessage_type g52v1;

	g52v1.resHeader.control = AH_MASK_FIR | AH_MASK_FIN;
	g52v1.resHeader.control |= lastSequence & AH_MASK_SEQ;
	g52v1.resHeader.functionCode = RESPONSE;
	g52v1.resHeader.intIndications1 = 0x00;
	g52v1.resHeader.intIndications2 = 0x00;
	g52v1.group = 52;
	g52v1.variation = 1;
	g52v1.qualifier = ONE_OCTET_CNT;
	g52v1.range[0] = 0x01;

	g52v1.dnp3Time[0] = 0x05;
	g52v1.dnp3Time[1] = 0x00;

	ciaaPOSIX_memcpy (&outgoingAppFragment, &g52v1, 10);
	outgoingAppFragment.length = 10;

	ciaaDnp_assembleSegments(outgoingAppFragment);

	coldRestartReq = 1;

	return 0;
}

int8_t ciaaDnp_read(ciaaDnp_objectHeader_type* firstObjHeader)
{
	if (ciaaDnp_groupVariation(firstObjHeader->group, firstObjHeader->variation) == GROUP10VAR0)
			if (firstObjHeader->qualifier == ALL_VALUES)
				ciaaDnp_enqueueResponseMessage(10, 2);

	if (ciaaDnp_groupVariation(firstObjHeader->group, firstObjHeader->variation) == GROUP60VAR1)
				if (firstObjHeader->qualifier == ALL_VALUES)
					ciaaDnp_enqueueG60V1ResponseMessage();
	return 0;
}

int8_t ciaaDnp_parseObjectHeader(ciaaDnp_objectHeader_type* firstObjHeader)
{
	switch(firstObjHeader->qualifier)
	{
		case ONE_OCTET_START_STOP:
			break;
		case TWO_OCTET_START_STOP:
			break;
		case ALL_VALUES:
			break;
		case ONE_OCTET_CNT:
			break;
		case TWO_OCTET_CNT:
			break;
		case ONE_OCTET_IDX_ONE_OCTET_CNT:
			break;
		case TWO_OCTET_IDX_TWO_OCTET_CNT:
			break;
		case TWO_OCTET_SZE_ONE_OCTET_CNT:
			break;
	}
	/*
	switch(firstObjHeader->group)
	{
		case 1:
		case 2:
			break;

		case 10:
			switch(firstObjHeader->variation)
			{
				case 0:
					if (firstObjHeader->qualifier == 0x06)
						ciaaDnp_enqueueResponseMessage(10, 2);
					break;
				case 2:
					break;
			}
			break;
		case 12:
			switch(firstObjHeader->variation)
			{
				case 1:
					if (firstObjHeader->qualifier == 0x17)
						ciaaDnp_enqueueResponseMessage(12, 1);

			}

	}*/
	return 0;
}

int8_t ciaaDnp_enqueueG60V1ResponseMessage(void)
{
	ciaaDnp_ciaaG60V1ResMessage_type g60v1;
	uint8_t inputs, outputs;

	ciaaPOSIX_read(fd_in, &inputs, 1);
	ciaaPOSIX_read(fd_out, &outputs, 1);

	g60v1.resHeader.control = AH_MASK_FIR | AH_MASK_FIN;
	g60v1.resHeader.control |= lastSequence & AH_MASK_SEQ;
	g60v1.resHeader.functionCode = RESPONSE;
	g60v1.resHeader.intIndications1 = 0x00;
	g60v1.resHeader.intIndications2 = 0x00;

	g60v1.group_di = 1;
	g60v1.variation_di = 2;
	g60v1.qualifier_di = 0x00;
	g60v1.range_di[0] = 0;
	g60v1.range_di[1] = 7;
	g60v1.digitalInputStatus[0] = ONLINE | ((inputs << 7) & 0x80);
	g60v1.digitalInputStatus[1] = ONLINE | ((inputs << 6) & 0x80);
	g60v1.digitalInputStatus[2] = ONLINE | ((inputs << 5) & 0x80);
	g60v1.digitalInputStatus[3] = ONLINE | ((inputs << 4) & 0x80);
	g60v1.digitalInputStatus[4] = ONLINE | ((inputs << 3) & 0x80);
	g60v1.digitalInputStatus[5] = ONLINE | ((inputs << 2) & 0x80);
	g60v1.digitalInputStatus[6] = ONLINE | ((inputs << 1) & 0x80);
	g60v1.digitalInputStatus[7] = ONLINE | ((inputs << 0) & 0x80);

	g60v1.group_do = 10;
	g60v1.variation_do = 2;
	g60v1.qualifier_do = 0x00;
	g60v1.range_do[0] = 0;
	g60v1.range_do[1] = 7;
	g60v1.digitalOutputStatus[0] = ONLINE | ((outputs << 7) & 0x80);
	g60v1.digitalOutputStatus[1] = ONLINE | ((outputs << 6) & 0x80);
	g60v1.digitalOutputStatus[2] = ONLINE | ((outputs << 5) & 0x80);
	g60v1.digitalOutputStatus[3] = ONLINE | ((outputs << 4) & 0x80);
	g60v1.digitalOutputStatus[4] = ONLINE | ((outputs << 3) & 0x80);
	g60v1.digitalOutputStatus[5] = ONLINE | ((outputs << 2) & 0x80);
	g60v1.digitalOutputStatus[6] = ONLINE | ((outputs << 1) & 0x80);
	g60v1.digitalOutputStatus[7] = ONLINE | ((outputs << 0) & 0x80);

	ciaaPOSIX_memcpy (&outgoingAppFragment, &g60v1, 30);
	outgoingAppFragment.length = 30;

	ciaaDnp_assembleSegments(outgoingAppFragment);

	return 0;
}

int8_t ciaaDnp_enqueueResponseMessage(uint8_t group, uint8_t variation)
{
	ciaaDnp_ciaaG10V2ResMessage_type g10v2;
	uint8_t outputs;

	ciaaPOSIX_read(fd_out, &outputs, 1);

	g10v2.resHeader.control = AH_MASK_FIR | AH_MASK_FIN;
	g10v2.resHeader.control |= lastSequence & AH_MASK_SEQ;
	g10v2.resHeader.functionCode = RESPONSE;
	g10v2.resHeader.intIndications1 = 0x00;
	g10v2.resHeader.intIndications2 = 0x00;
	g10v2.group = 10;
	g10v2.variation = 2;
	g10v2.qualifier = 0x00;
	g10v2.range[0] = 0x00;
	g10v2.range[1] = 0x07;
	g10v2.digitalOutputStatus[0] = ONLINE | ((outputs << 7) & 0x80);
	g10v2.digitalOutputStatus[1] = ONLINE | ((outputs << 6) & 0x80);
	g10v2.digitalOutputStatus[2] = ONLINE | ((outputs << 5) & 0x80);
	g10v2.digitalOutputStatus[3] = ONLINE | ((outputs << 4) & 0x80);
	g10v2.digitalOutputStatus[4] = ONLINE | ((outputs << 3) & 0x80);
	g10v2.digitalOutputStatus[5] = ONLINE | ((outputs << 2) & 0x80);
	g10v2.digitalOutputStatus[6] = ONLINE | ((outputs << 1) & 0x80);
	g10v2.digitalOutputStatus[7] = ONLINE | ((outputs << 0) & 0x80);

	ciaaPOSIX_memcpy (&outgoingAppFragment, &g10v2, 17);
	outgoingAppFragment.length = 17;

	ciaaDnp_assembleSegments(outgoingAppFragment);

	return 0;
}

int8_t ciaaDnp_enqueueG12V1ResponseMessage(ciaaDnp_objectHeader_type* firstObjHeader)
{
	ciaaDnp_ciaaG12V1ResMessage_type g12v1;

	g12v1.resHeader.control = AH_MASK_FIR | AH_MASK_FIN;
	g12v1.resHeader.control |= lastSequence & AH_MASK_SEQ;
	g12v1.resHeader.functionCode = RESPONSE;
	g12v1.resHeader.intIndications1 = 0x00;
	g12v1.resHeader.intIndications2 = 0x00;
	g12v1.group = 12;
	g12v1.variation = 1;
	g12v1.qualifier = firstObjHeader->qualifier;
	g12v1.range[0] = 0x01;
	g12v1.doCmd.index = 0x01;
	g12v1.doCmd.controlCode = 0x00;
	g12v1.doCmd.count = 0x00;
	g12v1.doCmd.onTime[0] = 0x00;
	g12v1.doCmd.onTime[1] = 0x00;
	g12v1.doCmd.onTime[2] = 0x00;
	g12v1.doCmd.onTime[3] = 0x00;
	g12v1.doCmd.offTime[0] = 0x00;
	g12v1.doCmd.offTime[1] = 0x00;
	g12v1.doCmd.offTime[2] = 0x00;
	g12v1.doCmd.offTime[3] = 0x00;
	g12v1.doCmd.statusCode = SUCCESS;

	ciaaPOSIX_memcpy (&outgoingAppFragment, &g12v1, 20);
	outgoingAppFragment.length = 20;

	ciaaDnp_assembleSegments(outgoingAppFragment);

	return 0;
}

ciaaDnp_groupVariationEnum ciaaDnp_groupVariation(uint8_t group, uint8_t variation)
{
	switch(group)
	{
		case 10:
			switch (variation)
			{
				case 0:
					return GROUP10VAR0;
					break;
				case 2:
					return GROUP10VAR2;
					break;
			}
			break;
		case 12:
			switch (variation)
			{
				case 1:
					return GROUP12VAR1;
					break;
			}
			break;
		case 52:
			switch (variation)
			{
				case 2:
					return GROUP52VAR2;
					break;
			}
			break;
		case 60:
			switch (variation)
			{
				case 1:
					return GROUP60VAR1;
					break;
			}
	}
	return GROUPVARUNK;
}

void ciaaDnp_executeColdRestart (void)
{
	uint32_t *reset_ctrl0 = (uint32_t *) 0x40053100;
	*reset_ctrl0 = 0xFFFFFFFF;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/








