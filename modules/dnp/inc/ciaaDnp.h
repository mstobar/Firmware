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

#ifndef _CIAADNP_H
#define _CIAADNP_H
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
#include "ciaaDnp_link.h"
#include "ciaaDnp_application.h"
/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/** \brief maximum value for a int16_t type */
//#define INT16_MAX   32767

/** \brief minimum value for a int16_t type */
//#define INT16_MIN   -32768

/*==================[typedef]================================================*/
typedef struct
{
	uint16_t slaveAddress;
	uint16_t masterAddress;
	uint8_t SecondaryStationIsReset;
	uint16_t NFCB; /** <- Next Frame Count Bit */
	ciaaDnp_PrimaryStationStatesEnum PrimaryStationState;
	uint8_t LinkIsReset;
	uint16_t EFCB; /** <- Expected Frame Count Bit */
	ciaaDnp_SecondaryStationStatesEnum SecondaryStationState;
	ciaaDnp_outsationStatesEnum outstationState;
}connectionVariables_type;

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

void ciaaDnp_init(void);
int16_t ciaaDnp_task(int32_t fd);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef TEMPLATE_FILE_H */

