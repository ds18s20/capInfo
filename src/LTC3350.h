/*
LTC3350: High Current Supercapacitor Backup Controller and System Monitor

@verbatim
The  LTC®3350 is a backup power controller that can charge and monitor a series
stack  of  one  to  four  super-capacitors.  The LTC3350's synchronous step-down
controller  drives  N-channel  MOSFETs  for  constant  current/constant  voltage
charging  with  programmable  input  current  limit.  In addition, the step-down
converter  can  run  in reverse as a step-up converter to deliver power from the
supercapacitor stack to the backup supply rail. Internal balancers eliminate the
need for external balance resistors and each capacitor has a shunt regulator for
overvoltage  protection.  The  LTC3350 monitors system voltages, currents, stack
capacitance  and  stack  ESR which can all be read over the I²C/SMBus. The dual
ideal  diode controller uses N-channel MOSFETs for low loss power paths from the
input and super-capacitors to the backup system supply. The LTC3350 is available
in a low profile 38-lead 5mm × 7mm × 0.75mm QFN surface mount package.
@endverbatim

http://www.linear.com/product/LTC3350

http://www.linear.com/product/LTC3350#demoboards

REVISION HISTORY
$Revision: $
$Date: $

Copyright (c) 2016, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.  Redistributions  of source code must retain the above copyright notice, this
    list  of conditions and the following disclaimer.

2.  Redistributions  in  binary  form must reproduce the above copyright notice,
    this  list of conditions and  the following disclaimer in the  documentation
    and/or other materials provided with the distribution.

THIS  SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY  EXPRESS  OR  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES   OF  MERCHANTABILITY  AND  FITNESS  FOR  A  PARTICULAR  PURPOSE  ARE
DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY  DIRECT,  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING,  BUT  NOT  LIMITED  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS  OF  USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY  THEORY  OF  LIABILITY,  WHETHER  IN  CONTRACT,  STRICT  LIABILITY,  OR TORT
(INCLUDING  NEGLIGENCE  OR  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The  views and conclusions contained in the software and documentation are those
of  the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However,  the Linduino is only possible because of the Arduino team's commitment
to   the   open-source   community.   Please,  visit  http://www.arduino.cc  and
http://store.arduino.cc  ,  and  consider  a  purchase that will help fund their
ongoing work.

Generated on: 2016-01-08
*/


/*! @file
 *  @ingroup LTC3350
 *  @brief LTC3350 communication library core header file defining
 *  prototypes, data structures and constants used by LTC3350.c
 *
 *  Functions  matching  the  prototypes  of  @ref  smbus_write_register and @ref
 *  smbus_read_register  must  be  provided  to this API. They will implement the
 *  SMBus  read  and write transactions on your hardware. If the register size of
 *  the  LTC3350  is 8 bits, functions should be provided that implement SMBus
 *  read  byte and write byte. If the register size of the LTC3350 is 16 bits,
 *  functions  should  be provided that implement SMBus read word and write word.
 *  smbus_read_register  needs  to  store the value read from the LTC3350 into
 *  the  variable  data.  Both  functions  should return 0 on success and a non-0
 *  error  code  on  failure.  The  API functions will return your error codes on
 *  failure and a 0 on success.
 */

#ifndef LTC3350_H_
#define LTC3350_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LTC3350_reg_defs.h"
#include "LTC3350_formats.h"
#include <stdint.h>
#include <stddef.h>

// Type declarations
  /*! Hardware port information. Modify as needed for local hardware
  requirements. Available to user supplied read and write functions. */
  typedef struct
  {
    int file_descriptor; //!< Linux SMBus file handle
  } port_configuration_t;
  /*! Prototype of user supplied SMBus write_byte or write_word function. Should return 0 on success and a non-0 error code on failure. */
  typedef int (*smbus_write_register)(uint8_t addr, //!< Target IC's SMBus address
                                      uint8_t command_code, //!< Command code to be written to
                                      uint16_t data, //!< Data to be written
                                      port_configuration_t *port_configuration //!< Pointer to a user supplied port_configuration struct
                                     );
  /*! Prototype of user supplied SMBus read_byte or read_word function. Should return 0 on success and a non-0 error code on failure. */
  typedef int (*smbus_read_register)(uint8_t addr, //!< Target IC's SMBus address
                                     uint8_t command_code, //!< command code to be read from
                                     uint16_t *data, //!< Pointer to data destination
                                     port_configuration_t *port_configuration //!< Pointer to a user supplied port_configuration struct
                                    );
  typedef void *LTC3350;
  /*! Information required to access hardware SMBus port */
  typedef struct
  {
    uint8_t addr; //!< Target IC's SMBus address
    smbus_read_register read_register; //!< Pointer to a user supplied smbus_read_register function
    smbus_write_register write_register; //!< Pointer to a user supplied smbus_write_register function
    port_configuration_t *port_configuration; //!< Pointer to a user supplied port_configuration struct
  } LTC3350_chip_cfg_t;

  // function declarations
  /*! Returns a pointer to a LTC3350 structure used by LTC3350_write_register and LTC3350_read_register */
  LTC3350 LTC3350_init(LTC3350_chip_cfg_t *cfg //!< Information required to access hardware SMBus port
                      );
  /*! Function to modify a bit field within a register while preserving the unaddressed bit fields */
  int LTC3350_write_register(LTC3350 chip_handle, //!< Struct returned by LTC3350_init
                             uint16_t registerinfo, //!< Bit field name from LTC3350_regdefs.h
                             uint16_t data //!< Data to be written
                            );
  /*! Retrieves a bit field data into *data. Right shifts the addressed portion down to the 0 position */
  int LTC3350_read_register(LTC3350 chip_handle, //!< Struct returned by LTC3350_init
                            uint16_t registerinfo, //!< Register name from LTC3350_regdefs.h
                            uint16_t *data //!< Pointer to the data destination
                           );
  /*! Multiple LTC3350 use.
    Multiple LTC3350s can be used with this API. Each one must be initialized.
    The LTC3350_init requires some memory for each LTC3350. This memory can
    be  statically  allocated  by  defining  MAX_NUM_LTC3350_INSTANCES  to the
    number of LTC3350s required. Alternatively it can be dynamically allocated
    by  defining  LTC3350_USE_MALLOC.  The  default  is  to not use malloc and
    statically allocate one LTC3350.
  */
#ifndef MAX_NUM_LTC3350_INSTANCES
#define MAX_NUM_LTC3350_INSTANCES 1
#endif
#ifdef __cplusplus
}
#endif
#endif /* LTC3350_H_ */
