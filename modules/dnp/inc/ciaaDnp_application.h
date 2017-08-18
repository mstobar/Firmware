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

#ifndef _CIAADNP_APPLICATION_H_
#define _CIAADNP_APPLICATION_H_
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

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
#define AH_MASK_FIN 0x80
#define AH_MASK_FIR 0x40
#define AH_MASK_CON 0x20
#define	AH_MASK_UNS 0x10
#define	AH_MASK_SEQ 0x0F

#define	ONLINE			0x01
#define	RESTART 		0x02
#define	COMM_LOST 		0x04
#define	REMOTE_FORCED 	0x08
#define	LOCAL_FORCED 	0x10
#define	RESERVED1 		0x20
#define	RESERVED2 		0x40
#define	STATE 			0x80

/*==================[typedef]================================================*/
typedef enum
{
	CONFIRM = 0x00,
	READ = 0x01,
	WRITE = 0x02,
	SELECT = 0x03,
	OPERATE = 0x04,
	DIRECT_OPERATE = 0x05,
	DIRECT_OPERATE_NR = 0x06,
	IMMED_FREEZE = 0x07,
	IMMED_FREEZE_NR = 0x08,
	FREEZE_CLEAR = 0x09,
	FREEZE_CLEAR_NR = 0x0A,
	FREEZE_AT_TIME = 0x0B,
	FREEZE_AT_TIME_NR = 0x0C,
	COLD_RESTART = 0x0D,
	WARM_RESTART = 0x0E,
	INITIALIZE_DATA = 0x0F,
	INITIALIZE_APPL = 0x10,
	START_APPL = 0x11,
	STOP_APPL = 0x12,
	SAVE_CONFIG = 0x13,
	ENABLE_UNSOLICITED = 0x14,
	DISABLE_UNSOLICITED = 0x15,
	ASSIGN_CLASS = 0x16,
	DELAY_MEASURE = 0x17,
	RECORD_CURRENT_TIME = 0x18,
	OPEN_FILE = 0x19,
	CLOSE_FILE = 0x1A,
	DELETE_FILE = 0x1B,
	GET_FILE_INFO = 0x1C,
	AUTHENTICATE_FILE = 0x1D,
	ABORT_FILE = 0x1E,
	AUTHENTICATE_REQ = 0x20,
	AUTH_REQ_NO_ACK = 0x21,
	RESPONSE = 0x81,
	UNSOLICITED_RESPONSE = 0x82,
	AUTHENTICATE_RESPONSE = 0x83
}ciaaDnp_appFunctionCodeEnum;

typedef enum
{
    BROADCAST = 0x01,
    CLASS1_EVENTS = 0x02,
    CLASS2_EVENTS = 0x04,
    CLASS3_EVENTS = 0x08,
    NEED_TIME = 0x10,
    LOCAL_CONTROL = 0x20,
    DEVICE_TROUBLE = 0x40,
    DEVICE_RESTART = 0x80,
    NO_FUNC_CODE_SUPPORT = 0x01,
    OBJECT_UNKNOWN = 0x02,
    PARAMETER_ERROR = 0x04,
    EVENT_BUFFER_OVERFLOW = 0x08,
    ALREADY_EXECUTING = 0x10,
    CONFIG_CORRUPT = 0x20,
    RESERVED_2 = 0x40,
    RESERVED_1 = 0x80,
}ciaaDnp_iinBitsEnum;

typedef enum
{
  ONE_OCTET_START_STOP = 0x00,
  TWO_OCTET_START_STOP = 0x01,
  ALL_VALUES = 0x06,
  ONE_OCTET_CNT = 0x07,
  TWO_OCTET_CNT = 0x08,
  ONE_OCTET_IDX_ONE_OCTET_CNT = 0x17,
  TWO_OCTET_IDX_TWO_OCTET_CNT = 0x28,
  TWO_OCTET_SZE_ONE_OCTET_CNT = 0x5B
}ciaaDnp_qualifierEnum;

typedef enum
{
	GROUPVARUNK,
	GROUP10VAR0,
	GROUP10VAR2,
	GROUP12VAR1,
	GROUP52VAR2,
	GROUP60VAR1
}ciaaDnp_groupVariationEnum;

typedef enum
{
	NUL_NUL = 0x00,
	NUL_NUL_CR = 0x20,
	PULSE_ON = 0x01,
	PULSE_ON_CR = 0x21,
	LATCH_ON = 0x03,
	LATCH_ON_CR = 0x23,
	LATCH_OFF = 0x04,
	LATCH_OFF_CR = 0x24,
	CLOSE_PULSE_ON = 0x41,
	CLOSE_PULSE_ON_CR = 0x61,
	TRIP_PULSE_ON = 0x81,
	TRIP_PULSE_ON_CANCEL = 0xA1,
}ciaaDnp_controlCodesEnum;

typedef enum
{
	SUCCESS = 0x00,
	TIMEOUT = 0x01,
	NO_SELECT = 0x02,
	FORMAT_ERROR = 0x03,
	NOT_SUPPORTED = 0x04,
	ALREADY_ACTIVE = 0x05,
	HARDWARE_ERROR = 0x06,
	LOCAL = 0x07,
	TOO_MANY_OBJS = 0x08,
	NOT_AUTHORIZED = 0x09,
	AUTOMATION_INHIBIT = 0x10,
	PROCESSING_LIMITED = 0x11,
	OUT_OF_RANGE = 0x12,
	NON_PARTICIPATING = 0x7e,
	UNDEFINED = 0x7f
}ciaaDnp_statusCodesEnum;

typedef struct
{
	uint8_t control;
	ciaaDnp_appFunctionCodeEnum functionCode;
}ciaaDnp_appRequestHeader_type;

typedef struct
{
	uint8_t control;
	ciaaDnp_appFunctionCodeEnum functionCode;
	uint8_t intIndications1;
	uint8_t intIndications2;
}ciaaDnp_appResponseHeader_type;

typedef struct
{
	uint8_t group;
	uint8_t variation;
	uint8_t qualifier;
	uint8_t range[];
}ciaaDnp_objectHeader_type;

typedef struct
{
	uint8_t group;
	uint8_t variation;
	uint8_t qualifier;
	uint8_t range[0];
}ciaaDnp_objectHeaderRange0_type;

typedef struct
{
	uint8_t group;
	uint8_t variation;
	uint8_t qualifier;
	uint8_t range[1];
}ciaaDnp_objectHeaderRange1_type;

typedef struct
{
	uint8_t group;
	uint8_t variation;
	uint8_t qualifier;
	uint8_t range[2];
}ciaaDnp_objectHeaderRange2_type;

typedef struct
{
	ciaaDnp_appRequestHeader_type reqHeader;
	ciaaDnp_objectHeader_type firstObjectHeader[];
}ciaaDnp_appRequestMessage_type;

typedef struct
{
	ciaaDnp_appResponseHeader_type resHeader;
	uint8_t objects[245];
	uint8_t length;
}ciaaDnp_appResponseMessage_type;

/** \brief Outstation states */
typedef enum
{
   CIAA_DNP_OUT_S_IDLE = 0,
   CIAA_DNP_OUT_S_SOLCONFWAIT = 1,
   CIAA_DNP_PRI_S_UNSOLCONFWAIT = 2
}ciaaDnp_outsationStatesEnum;

typedef struct
{
	uint8_t digitalOutputStatus[8];
}ciaaDnp_ciaaDOStatus_type;

typedef struct
{
	uint8_t index;
	uint8_t controlCode;
	uint8_t count;
	uint8_t onTime[4];
	uint8_t offTime[4];
	uint8_t statusCode;
}ciaaDnp_ciaaDOCommand_type;

typedef struct
{
	ciaaDnp_appResponseHeader_type resHeader;
	uint8_t group;
	uint8_t variation;
	uint8_t qualifier;
	uint8_t range[2];
	uint8_t digitalOutputStatus[8];
}ciaaDnp_ciaaG10V2ResMessage_type;

typedef struct
{
	ciaaDnp_appResponseHeader_type resHeader;
	uint8_t group;
	uint8_t variation;
	uint8_t qualifier;
	uint8_t range[1];
	ciaaDnp_ciaaDOCommand_type doCmd;
}ciaaDnp_ciaaG12V1ResMessage_type;

typedef struct
{
	ciaaDnp_appResponseHeader_type resHeader;
	uint8_t group;
	uint8_t variation;
	uint8_t qualifier;
	uint8_t range[1];
	uint8_t dnp3Time[2];
}ciaaDnp_ciaaG52V1ResMessage_type;

typedef struct
{
	ciaaDnp_appResponseHeader_type resHeader;
	uint8_t group_di;
	uint8_t variation_di;
	uint8_t qualifier_di;
	uint8_t range_di[2];
	uint8_t digitalInputStatus[8];
	uint8_t group_do;
	uint8_t variation_do;
	uint8_t qualifier_do;
	uint8_t range_do[2];
	uint8_t digitalOutputStatus[8];
}ciaaDnp_ciaaG60V1ResMessage_type;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef TEMPLATE_FILE_H */
