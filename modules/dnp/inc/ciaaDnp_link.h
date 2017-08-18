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

#ifndef _CIAADNP_LINK_H
#define _CIAADNP_LINK_H
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
#include "ciaaDnp_linkBuffers.h"
//#include "ciaaDnp.h"
/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/


/*==================[typedef]================================================*/

#define RESET_LINK_WAIT_ONE_TIMEOUT					0x0A
#define RESET_LINK_WAIT_TWO_TIMEOUT					0x0A
#define LINK_STATUS_WAIT_TIMEOUT					0x0A
#define TEST_WAIT_TIMEOUT							0x0A
#define CFMD_DATA_WAIT_TIMEOUT						0x0A
#define UR_LINK_STATUS_WAIT_TIMEOUT					0x0A

#define CFMD_DATA_WAIT_MAXCOUNT						0x03

/** \brief Frame header block*/
typedef struct
{
	uint16_t sync;
	uint8_t length;
	uint8_t control;
	uint16_t destination;
	uint16_t source;
	uint16_t crc;
}ciaaDnp_headerBlock_type;

/** \brief User data block*/
typedef struct
{
   uint8_t data[16];
   uint16_t crc;
}ciaaDnp_userBlock_type;

typedef struct
{
	ciaaDnp_headerBlock_type headerBlock;
	ciaaDnp_userBlock_type dataBlock[16];
}ciaaDnp_linkFrame_type;

enum controlMask
{
	MASK_DIR = 0x80,
	MASK_PRM = 0x40,
	MASK_FCB = 0x20,
	MASK_FCV = 0x10,
	MASK_DFC = 0x10,
	MASK_FC = 0x0F,
};

/** \brief Primary to Secondary Function Codes */
typedef enum
{
   CIAA_DNP_PRI_FC_RESET_LINK_STATES = 0,
   CIAA_DNP_PRI_FC_TEST_LINK_STATES = 2,
   CIAA_DNP_PRI_FC_CONFIRMED_USER_DATA = 3,
   CIAA_DNP_PRI_FC_UNCONFIRMED_USER_DATA = 4,
   CIAA_DNP_PRI_FC_REQUEST_LINK_STATUS = 9
}ciaaDnp_PrimaryFCEnum;

/** \brief Secondary to Primary Function Codes */
typedef enum
{
   CIAA_DNP_SEC_FC_ACK = 0,
   CIAA_DNP_SEC_FC_NACK = 1,
   CIAA_DNP_SEC_FC_LINK_STATUS = 11,
   CIAA_DNP_SEC_FC_NOT_SUPPORTED = 15
}ciaaDnp_SecondaryFCEnum;

/** \brief Primary Station states */
typedef enum
{
   CIAA_DNP_PRI_S_SECUNRESETIDLE = 0,
   CIAA_DNP_PRI_S_SECRESETIDLE = 1,
   CIAA_DNP_PRI_S_RESETLINKWAIT_1 = 2,
   CIAA_DNP_PRI_S_RESETLINKWAIT_2 = 3,
   CIAA_DNP_PRI_S_UR_LINKSTATUSWAIT = 4,
   CIAA_DNP_PRI_S_TESTWAIT = 5,
   CIAA_DNP_PRI_S_CFMDDATAWAIT = 6,
   CIAA_DNP_PRI_S_R_LINKSTATUSWAIT = 7

}ciaaDnp_PrimaryStationStatesEnum;

/** \brief Secondary Station states */
typedef enum
{
   CIAA_DNP_SEC_S_UNRESET = 0,
   CIAA_DNP_SEC_S_IDLE = 1
}ciaaDnp_SecondaryStationStatesEnum;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** \brief Adds 2 16 bits signed and return a 16 bits signed value
 **
 ** If the result is bigger than the limit in 15 bits returns
 ** INT16_MAX is lower than the limit in 15 bits returns
 ** INT16_MIN
 **
 ** \param[in]    a first parameter to be added
 ** \param[in]    b second parameter ot be added
 ** \return E_OK  returns addition of a+b
 **/

int16_t ciaaDnp_linkInit(void);
int8_t ciaaDnp_linkPrimaryTask(int32_t fd);
int8_t ciaaDnp_linkSecondaryTask(int32_t fd);

int8_t ciaaDnp_getDIR(ciaaDnp_linkFrame_type *linkFrame);
int8_t ciaaDnp_getPRM(ciaaDnp_linkFrame_type *linkFrame);
int8_t ciaaDnp_getFCB(ciaaDnp_linkFrame_type *linkFrame);
int8_t ciaaDnp_getFCV(ciaaDnp_linkFrame_type *linkFrame);
int8_t ciaaDnp_getFC(ciaaDnp_linkFrame_type *linkFrame);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef TEMPLATE_FILE_H */

