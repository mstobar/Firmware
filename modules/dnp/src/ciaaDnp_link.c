/* Copyright 2014, Your Name <youremail@domain.com>
 * All rights reserved.
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
#include "ciaaDnp_link.h"
#include "ciaaDnp.h"
#include "ciaaDnp_linkSerial.h"
#include "ciaaPOSIX_stdint.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaDnp_ringBuffer.h"
#include "ciaaDnp_transport.h"
#include "ciaaDnp_application.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[external data declaration]==============================*/
extern ciaaDnp_segmentBin_type outgoingSegment;

/*==================[internal functions declaration]=========================*/
int8_t ciaaDnp_getSegmentFromFrame(ciaaDnp_linkFrame_type* linkFrame, ciaaDnp_transportSegment_type* ts);
/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/
extern ciaaDnp_ringBuffer_type inputFrameQueue;
extern connectionVariables_type connection;

/*DEBUG*/
extern int32_t fd_uart1;

/* Primary station variables*/
//_Bool SecondaryStationIsReset = 0;
//uint16_t NFCB; /** <- Next Frame Count Bit */
uint8_t PSSoftwareOperatingState;

/* Secondary station variables*/
//_Bool LinkIsReset;
//uint8_t SSSoftwareOperatingState;
//uint16_t EFCB; /** <- Expected Frame Count Bit */

//ciaaDnp_PrimaryStationStatesEnum PrimaryStationState;
//ciaaDnp_SecondaryStationStatesEnum SecondaryStationState;

ciaaDnp_headerBlock_type newFrameHeader;

_Bool testLinkRequest;

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
int8_t ciaaDnp_getDIR(ciaaDnp_linkFrame_type *linkFrame)
{
    return (linkFrame->headerBlock.control & MASK_DIR ? 1 : 0);
}

int8_t ciaaDnp_getPRM(ciaaDnp_linkFrame_type *linkFrame)
{
    return (linkFrame->headerBlock.control & MASK_PRM ? 1 : 0);
}

int8_t ciaaDnp_getFCB(ciaaDnp_linkFrame_type *linkFrame)
{
    return (linkFrame->headerBlock.control & MASK_FCB ? 1 : 0);
}

int8_t ciaaDnp_getFCV(ciaaDnp_linkFrame_type *linkFrame)
{
    return (linkFrame->headerBlock.control & MASK_FCV ? 1 : 0);
}

/** \brief Get Data Flow Control bit */
int8_t ciaaDnp_getDFC(ciaaDnp_linkFrame_type *linkFrame)
{
    return ((*linkFrame).headerBlock.control & MASK_DFC);
}

int8_t ciaaDnp_getFC(ciaaDnp_linkFrame_type *linkFrame)
{
    return ((*linkFrame).headerBlock.control & MASK_FC);
}

int16_t ciaaDnp_linkInit(void)
{
    connection.masterAddress = 8;
    connection.slaveAddress = 4;
    connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    connection.SecondaryStationState = CIAA_DNP_SEC_S_UNRESET;

    ciaaDnp_initRingBuffer(&inputFrameQueue);

    return 0;
}

int8_t ciaaDnp_getSegmentFromFrame(ciaaDnp_linkFrame_type* linkFrame, ciaaDnp_transportSegment_type* ts)
{
	uint16_t dataSize, modulo, dataBlocks;
	uint16_t i,j;

	dataSize = linkFrame->headerBlock.length - 5;

	dataBlocks = dataSize / 16;
	modulo = dataSize % 16;

	for (i=0; i < dataBlocks; i++)
		for (j=0;j<16;j++)
			(&ts->header)[j+i*16] = linkFrame->dataBlock[i].data[j];

	for (j=0;j<modulo;j++)
		(&ts->header)[j+dataBlocks*16] = linkFrame->dataBlock[dataBlocks].data[j];

	ts->length = dataSize;

    return 0;
}

int8_t ciaaDnp_linkPrimaryTask(int32_t fd)
{
	static int32_t resetLinkWaitOneTimer, resetLinkWaitTwoTimer, linkStatusWaitTimer, testWaitTimer;
	static int32_t urLinkStatusWaitTimer;
	static int32_t cfmdDataWaitTimer;
	static int32_t cfmdDataWaitRetry;

	ciaaDnp_linkFrame_type frameToPri;
	//ciaaDnp_transportSegment_type transportSegment;

	//ciaaDnp_linkParser(fd);


	/* Primary Station State Machine */
	switch(connection.PrimaryStationState)
    {
    	case CIAA_DNP_PRI_S_SECUNRESETIDLE:
    		if (ciaaDnp_ConfirmedDataReady())
    		{
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_RESETLINKWAIT_2;
    			ciaaDnp_queueResetLinkStates();
    		}
    		else if (ciaaDnp_segmentReady())
    		{
    			ciaaDnp_queueUnconfirmedUserData(fd);
    		}
    		break;
    	case CIAA_DNP_PRI_S_SECRESETIDLE:
    		if (testLinkRequest)
    		{
    			testLinkRequest = 0;
    			ciaaDnp_queueRequestLinkStatus();
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_TESTWAIT;
    		}
    		else if (ciaaDnp_ConfirmedDataReady())
    		{
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_CFMDDATAWAIT;
    		    ciaaDnp_queueConfirmedUserData();
    		}
    		else if (ciaaDnp_segmentReady())
    		{
    			ciaaDnp_queueUnconfirmedUserData(fd);
    		}
    		break;
    	case CIAA_DNP_PRI_S_RESETLINKWAIT_1:
    		if (ciaaDnp_getFC(&frameToPri) == CIAA_DNP_SEC_FC_ACK)
    		{
    			connection.SecondaryStationIsReset = 1;
    			connection.NFCB = 1;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECRESETIDLE;
    			break;
    		}
    		else
    		{
    			connection.SecondaryStationIsReset = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    			break;
    		}
    		if (++resetLinkWaitOneTimer > RESET_LINK_WAIT_ONE_TIMEOUT)
    		{
    			connection.SecondaryStationIsReset = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    			break;
    		}
    		break;
    	case CIAA_DNP_PRI_S_RESETLINKWAIT_2:
    		if (ciaaDnp_getFC(&frameToPri) == CIAA_DNP_SEC_FC_ACK)
    		{
    			connection.SecondaryStationIsReset = 1;
    			connection.NFCB = 1;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_CFMDDATAWAIT;
    		    //ciaaDnp_sendConfimedData();
    		    break;
    		}
    		else
    		{
    			connection.SecondaryStationIsReset = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    		    break;
    		}
    		if (++resetLinkWaitTwoTimer > RESET_LINK_WAIT_TWO_TIMEOUT)
    		{
    			connection.SecondaryStationIsReset = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    		    break;
    		}
    		break;
    	case CIAA_DNP_PRI_S_UR_LINKSTATUSWAIT:
    		if (ciaaDnp_getFC(&frameToPri) == CIAA_DNP_SEC_FC_LINK_STATUS)
    		{
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    		    //ciaaDnp_fillLinkStatusData();
    		    break;
    		}
    		else
    		{
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    			//ciaaDnp_fillLinkStatusData();
    			break;
    		}
    		if (++urLinkStatusWaitTimer > UR_LINK_STATUS_WAIT_TIMEOUT)
    		{
    			urLinkStatusWaitTimer = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    		    break;
    		}
    		break;
    	case CIAA_DNP_PRI_S_TESTWAIT:
    		if (ciaaDnp_getFC(&frameToPri) == CIAA_DNP_SEC_FC_ACK)
    		{
    			connection.NFCB = !connection.NFCB;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECRESETIDLE;
    			break;
    		}
    		else
    		{
    			connection.SecondaryStationIsReset = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    		    break;
    		}
    		if (++testWaitTimer > TEST_WAIT_TIMEOUT)
    		{
    			linkStatusWaitTimer = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    		    break;
    		}
    		break;
    	case CIAA_DNP_PRI_S_CFMDDATAWAIT:
    		if (ciaaDnp_getFC(&frameToPri) == CIAA_DNP_SEC_FC_ACK)
    		{
    			connection.NFCB = !connection.NFCB;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECRESETIDLE;
    			break;
    		}
    		else if (ciaaDnp_getFC(&frameToPri) == CIAA_DNP_SEC_FC_NACK)
    		{
    			if (!ciaaDnp_getDFC(&frameToPri))
    			{
    				connection.SecondaryStationIsReset = 0;
    				cfmdDataWaitRetry = 0;
    				ciaaDnp_queueResetLinkStates();
    				connection.PrimaryStationState = CIAA_DNP_PRI_S_RESETLINKWAIT_2;
    			}
    			else
    			{
    				connection.SecondaryStationIsReset = 0;
    				connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    			}
    			break;
    		}
    		else
    		{
    			connection.SecondaryStationIsReset = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    			break;
    		}
    		if (++cfmdDataWaitTimer > CFMD_DATA_WAIT_TIMEOUT)
    		{
    			cfmdDataWaitTimer = 0;
    			if (cfmdDataWaitRetry < CFMD_DATA_WAIT_MAXCOUNT)
    			{
    				cfmdDataWaitRetry++;
    				ciaaDnp_queueConfirmedUserData();
    				connection.PrimaryStationState = CIAA_DNP_PRI_S_CFMDDATAWAIT;
    				break;
    			}
    			else
    			{
    				connection.SecondaryStationIsReset = 0;
    				connection.PrimaryStationState = CIAA_DNP_PRI_S_SECUNRESETIDLE;
    				break;
    			}
    		}
    		break;
    	case CIAA_DNP_PRI_S_R_LINKSTATUSWAIT:
    		if (ciaaDnp_getFC(&frameToPri) == CIAA_DNP_SEC_FC_LINK_STATUS)
    		{
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECRESETIDLE;
    		    //ciaaDnp_fillLinkStatusData();
    		    break;
    		}
    		else
    		{
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECRESETIDLE;
    		    break;
    		}
    		if (++linkStatusWaitTimer > LINK_STATUS_WAIT_TIMEOUT)
    		{
    			linkStatusWaitTimer = 0;
    			connection.PrimaryStationState = CIAA_DNP_PRI_S_SECRESETIDLE;
    		    break;
    		}
    		break;

    }
    return 0;
}

int8_t ciaaDnp_linkSecondaryTask(int32_t fd)
{
	//static int32_t resetLinkWaitOneTimer, resetLinkWaitTwoTimer, linkStatusWaitTimer, testWaitTimer;
	//static int32_t urLinkStatusWaitTimer;
	//static int32_t cfmdDataWaitTimer;
	//static int32_t cfmdDataWaitRetry;

	ciaaDnp_linkFrame_type frameToSec;
	ciaaDnp_transportSegment_type transportSegment;

	ciaaDnp_linkParser(fd);

	if (inputFrameQueue.elements == 0)
		return 0;

	if (!inputFrameQueue.data[inputFrameQueue.rPosition].headerBlock.control && MASK_PRM)
		return 0;

	ciaaDnp_dequeueRingBuffer(&inputFrameQueue, &frameToSec);

	/* Secondary Station State Machine */
    switch(connection.SecondaryStationState)
    {
    	case CIAA_DNP_SEC_S_UNRESET:
    		if (!ciaaDnp_getFCV(&frameToSec))
    		{
    			if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_RESET_LINK_STATES)
    			{
    				connection.EFCB = 1;
    				connection.LinkIsReset = 1;
    				ciaaDnp_queueAck(fd);
    				connection.SecondaryStationState = CIAA_DNP_SEC_S_IDLE;
    				/*DEBUG*/
    				ciaaPOSIX_write(fd_uart1, "To Idle\n\r", 9);
    			}
    			else if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_UNCONFIRMED_USER_DATA)
    			{
    				ciaaDnp_getSegmentFromFrame(&frameToSec, &transportSegment);
    				ciaaDnp_assembleFragment(transportSegment);
    			}
    			else if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_REQUEST_LINK_STATUS)
    			{
    				ciaaDnp_queueLinkStatus(fd);
    			}
    		}
    		break;

    	case CIAA_DNP_SEC_S_IDLE:
    		if (!ciaaDnp_getFCV(&frameToSec))
    		{
    			if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_RESET_LINK_STATES)
    			{
    				connection.EFCB = 1;
    				ciaaDnp_queueAck(fd);
    			}
    			else if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_UNCONFIRMED_USER_DATA)
    			{
    				ciaaDnp_getSegmentFromFrame(&frameToSec, &transportSegment);
    				ciaaDnp_assembleFragment(transportSegment);
    			}
    			else if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_REQUEST_LINK_STATUS)
    			{
    				ciaaDnp_queueLinkStatus(fd);
    			}
    		}
    		else /* FCV == 1 */
    		{
    			if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_TEST_LINK_STATES)
    			{
    				if (connection.EFCB == ciaaDnp_getFCB(&frameToSec))
    				{
    					connection.EFCB = !connection.EFCB;
    					ciaaDnp_queueAck(fd);
    				}
    				else
    				{
    					ciaaDnp_queueLastXAck();
    				}
    			}
    			else if (ciaaDnp_getFC(&frameToSec) == CIAA_DNP_PRI_FC_CONFIRMED_USER_DATA)
    			{
    				if (connection.EFCB == ciaaDnp_getFCB(&frameToSec))
    				{
    					ciaaDnp_getSegmentFromFrame(&frameToSec, &transportSegment);
    					ciaaDnp_assembleFragment(transportSegment);
    					connection.EFCB = !connection.EFCB;
    				}
    				ciaaDnp_queueAck(fd);
    			}
    		}
    		break;
    }
    return 0;
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/




