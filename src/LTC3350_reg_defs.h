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
 *  @brief LTC3350 Register Map Definition Header
 *
 *
 *  This file contains LTC3350 definitions for each command_code as well as
 *  each individual bit field for the case when a register contains multiple
 *  bit-packed fields smaller than the register width.
 *  Each bit field name is prepended with LTC3350_.
 *  Each bit field has individual definitions for its _SIZE, _OFFSET (LSB) and _MASK,
 *  as well as the three fields stored in a single 16-bit word for use with the access
 *  functions provided by LTC3350.c and LTC3350.h.
 *  In the case that the bit field contents represent an enumeration, _PRESET
 *  definitions exists to translate from human readable format to the encoded value.
 *  See @ref LTC3350_register_map for detailed descriptions of each bit field.
 */

/*! @defgroup LTC3350_register_map LTC3350 Register Map Definitions
 *  @ingroup LTC3350
 */

#ifndef LTC3350_REG_DEFS_H_
#define LTC3350_REG_DEFS_H_

#define LTC3350_ADDR_09 0x9 //!<LTC3350 I2C address in 7-bit format
/*! @defgroup LTC3350_CLR_ALARMS CLR_ALARMS
 *  @ingroup LTC3350_register_map
 *  @brief CLR_ALARMS Register
 *
 * |         15 |         14 |            13 |             12 |          11 |         10 |           9 |           8 |           7 |           6 |          5 |          4 |          3 |          2 |          1 |          0 |
 * |:----------:|:----------:|:-------------:|:--------------:|:-----------:|:----------:|:-----------:|:-----------:|:-----------:|:-----------:|:----------:|:----------:|:----------:|:----------:|:----------:|:----------:|
 * | CLR_CAP_LO | CLR_ESR_HI | CLR_DTEMP_HOT | CLR_DTEMP_COLD | CLR_ICHG_UC | CLR_IIN_OC | CLR_VOUT_OV | CLR_VOUT_UV | CLR_VCAP_OV | CLR_VCAP_UV | CLR_VIN_OV | CLR_VIN_UV | CLR_GPI_OV | CLR_GPI_UV | CLR_CAP_OV | CLR_CAP_UV |
 *
 * Clear Alarms Register: This register is used to clear alarms caused by exceeding a programmed limit. Writing a one to any bit in this register will cause its respective alarm to be cleared. The one written to this register is automatically cleared when its respective alarm is cleared.
 *   - CommandCode: 0
 *   - Contains Bit Fields:
 *     + @ref LTC3350_CLR_CAP_LO "CLR_CAP_LO" : Clear capacitance low alarm
 *     + @ref LTC3350_CLR_ESR_HI "CLR_ESR_HI" : Clear ESR high alarm
 *     + @ref LTC3350_CLR_DTEMP_HOT "CLR_DTEMP_HOT" : Clear die temperature hot alarm
 *     + @ref LTC3350_CLR_DTEMP_COLD "CLR_DTEMP_COLD" : Clear die temperature cold alarm
 *     + @ref LTC3350_CLR_ICHG_UC "CLR_ICHG_UC" : Clear charge undercurrent alarm
 *     + @ref LTC3350_CLR_IIN_OC "CLR_IIN_OC" : Clear input overcurrent alarm
 *     + @ref LTC3350_CLR_VOUT_OV "CLR_VOUT_OV" : Clear VOUT overvoltage alarm
 *     + @ref LTC3350_CLR_VOUT_UV "CLR_VOUT_UV" : Clear VOUT undervoltage alarm
 *     + @ref LTC3350_CLR_VCAP_OV "CLR_VCAP_OV" : Clear VCAP overvoltage alarm
 *     + @ref LTC3350_CLR_VCAP_UV "CLR_VCAP_UV" : Clear VCAP undervoltage alarm
 *     + @ref LTC3350_CLR_VIN_OV "CLR_VIN_OV" : Clear VIN overvoltage alarm
 *     + @ref LTC3350_CLR_VIN_UV "CLR_VIN_UV" : Clear VIN undervoltage alarm
 *     + @ref LTC3350_CLR_GPI_OV "CLR_GPI_OV" : Clear GPI overvoltage alarm
 *     + @ref LTC3350_CLR_GPI_UV "CLR_GPI_UV" : Clear GPI undervoltage alarm
 *     + @ref LTC3350_CLR_CAP_OV "CLR_CAP_OV" : Clear capacitor overvoltage alarm
 *     + @ref LTC3350_CLR_CAP_UV "CLR_CAP_UV" : Clear capacitor undervoltage alarm
*/

//!@{
#define LTC3350_CLR_ALARMS_SUBADDR 0
#define LTC3350_CLR_ALARMS (0 << 12 | (16 - 1) << 8 | LTC3350_CLR_ALARMS_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_CAP_LO CLR_CAP_LO
 *  @ingroup LTC3350_register_map
 *  @brief CLR_CAP_LO Bit Field
 *
 *  Clear capacitance low alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 15
 *   - MSB: 15
 *   - MASK: 0x8000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_CAP_LO_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_CAP_LO "CLR_CAP_LO"
#define LTC3350_CLR_CAP_LO_SIZE 1
#define LTC3350_CLR_CAP_LO_OFFSET 15
#define LTC3350_CLR_CAP_LO_MASK 0x8000
#define LTC3350_CLR_CAP_LO (LTC3350_CLR_CAP_LO_OFFSET << 12 | (LTC3350_CLR_CAP_LO_SIZE - 1) << 8 | LTC3350_CLR_CAP_LO_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_ESR_HI CLR_ESR_HI
 *  @ingroup LTC3350_register_map
 *  @brief CLR_ESR_HI Bit Field
 *
 *  Clear ESR high alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 14
 *   - MSB: 14
 *   - MASK: 0x4000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_ESR_HI_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_ESR_HI "CLR_ESR_HI"
#define LTC3350_CLR_ESR_HI_SIZE 1
#define LTC3350_CLR_ESR_HI_OFFSET 14
#define LTC3350_CLR_ESR_HI_MASK 0x4000
#define LTC3350_CLR_ESR_HI (LTC3350_CLR_ESR_HI_OFFSET << 12 | (LTC3350_CLR_ESR_HI_SIZE - 1) << 8 | LTC3350_CLR_ESR_HI_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_DTEMP_HOT CLR_DTEMP_HOT
 *  @ingroup LTC3350_register_map
 *  @brief CLR_DTEMP_HOT Bit Field
 *
 *  Clear die temperature hot alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 13
 *   - MSB: 13
 *   - MASK: 0x2000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_DTEMP_HOT_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_DTEMP_HOT "CLR_DTEMP_HOT"
#define LTC3350_CLR_DTEMP_HOT_SIZE 1
#define LTC3350_CLR_DTEMP_HOT_OFFSET 13
#define LTC3350_CLR_DTEMP_HOT_MASK 0x2000
#define LTC3350_CLR_DTEMP_HOT (LTC3350_CLR_DTEMP_HOT_OFFSET << 12 | (LTC3350_CLR_DTEMP_HOT_SIZE - 1) << 8 | LTC3350_CLR_DTEMP_HOT_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_DTEMP_COLD CLR_DTEMP_COLD
 *  @ingroup LTC3350_register_map
 *  @brief CLR_DTEMP_COLD Bit Field
 *
 *  Clear die temperature cold alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 12
 *   - MSB: 12
 *   - MASK: 0x1000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_DTEMP_COLD_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_DTEMP_COLD "CLR_DTEMP_COLD"
#define LTC3350_CLR_DTEMP_COLD_SIZE 1
#define LTC3350_CLR_DTEMP_COLD_OFFSET 12
#define LTC3350_CLR_DTEMP_COLD_MASK 0x1000
#define LTC3350_CLR_DTEMP_COLD (LTC3350_CLR_DTEMP_COLD_OFFSET << 12 | (LTC3350_CLR_DTEMP_COLD_SIZE - 1) << 8 | LTC3350_CLR_DTEMP_COLD_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_ICHG_UC CLR_ICHG_UC
 *  @ingroup LTC3350_register_map
 *  @brief CLR_ICHG_UC Bit Field
 *
 *  Clear charge undercurrent alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 11
 *   - MSB: 11
 *   - MASK: 0x0800
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_ICHG_UC_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_ICHG_UC "CLR_ICHG_UC"
#define LTC3350_CLR_ICHG_UC_SIZE 1
#define LTC3350_CLR_ICHG_UC_OFFSET 11
#define LTC3350_CLR_ICHG_UC_MASK 0x0800
#define LTC3350_CLR_ICHG_UC (LTC3350_CLR_ICHG_UC_OFFSET << 12 | (LTC3350_CLR_ICHG_UC_SIZE - 1) << 8 | LTC3350_CLR_ICHG_UC_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_IIN_OC CLR_IIN_OC
 *  @ingroup LTC3350_register_map
 *  @brief CLR_IIN_OC Bit Field
 *
 *  Clear input overcurrent alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 10
 *   - MSB: 10
 *   - MASK: 0x0400
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_IIN_OC_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_IIN_OC "CLR_IIN_OC"
#define LTC3350_CLR_IIN_OC_SIZE 1
#define LTC3350_CLR_IIN_OC_OFFSET 10
#define LTC3350_CLR_IIN_OC_MASK 0x0400
#define LTC3350_CLR_IIN_OC (LTC3350_CLR_IIN_OC_OFFSET << 12 | (LTC3350_CLR_IIN_OC_SIZE - 1) << 8 | LTC3350_CLR_IIN_OC_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_VOUT_OV CLR_VOUT_OV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_VOUT_OV Bit Field
 *
 *  Clear VOUT overvoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 9
 *   - MSB: 9
 *   - MASK: 0x0200
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_VOUT_OV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_VOUT_OV "CLR_VOUT_OV"
#define LTC3350_CLR_VOUT_OV_SIZE 1
#define LTC3350_CLR_VOUT_OV_OFFSET 9
#define LTC3350_CLR_VOUT_OV_MASK 0x0200
#define LTC3350_CLR_VOUT_OV (LTC3350_CLR_VOUT_OV_OFFSET << 12 | (LTC3350_CLR_VOUT_OV_SIZE - 1) << 8 | LTC3350_CLR_VOUT_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_VOUT_UV CLR_VOUT_UV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_VOUT_UV Bit Field
 *
 *  Clear VOUT undervoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 8
 *   - MSB: 8
 *   - MASK: 0x0100
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_VOUT_UV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_VOUT_UV "CLR_VOUT_UV"
#define LTC3350_CLR_VOUT_UV_SIZE 1
#define LTC3350_CLR_VOUT_UV_OFFSET 8
#define LTC3350_CLR_VOUT_UV_MASK 0x0100
#define LTC3350_CLR_VOUT_UV (LTC3350_CLR_VOUT_UV_OFFSET << 12 | (LTC3350_CLR_VOUT_UV_SIZE - 1) << 8 | LTC3350_CLR_VOUT_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_VCAP_OV CLR_VCAP_OV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_VCAP_OV Bit Field
 *
 *  Clear VCAP overvoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 7
 *   - MSB: 7
 *   - MASK: 0x0080
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_VCAP_OV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_VCAP_OV "CLR_VCAP_OV"
#define LTC3350_CLR_VCAP_OV_SIZE 1
#define LTC3350_CLR_VCAP_OV_OFFSET 7
#define LTC3350_CLR_VCAP_OV_MASK 0x0080
#define LTC3350_CLR_VCAP_OV (LTC3350_CLR_VCAP_OV_OFFSET << 12 | (LTC3350_CLR_VCAP_OV_SIZE - 1) << 8 | LTC3350_CLR_VCAP_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_VCAP_UV CLR_VCAP_UV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_VCAP_UV Bit Field
 *
 *  Clear VCAP undervoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 6
 *   - MSB: 6
 *   - MASK: 0x0040
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_VCAP_UV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_VCAP_UV "CLR_VCAP_UV"
#define LTC3350_CLR_VCAP_UV_SIZE 1
#define LTC3350_CLR_VCAP_UV_OFFSET 6
#define LTC3350_CLR_VCAP_UV_MASK 0x0040
#define LTC3350_CLR_VCAP_UV (LTC3350_CLR_VCAP_UV_OFFSET << 12 | (LTC3350_CLR_VCAP_UV_SIZE - 1) << 8 | LTC3350_CLR_VCAP_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_VIN_OV CLR_VIN_OV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_VIN_OV Bit Field
 *
 *  Clear VIN overvoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 5
 *   - MSB: 5
 *   - MASK: 0x0020
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_VIN_OV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_VIN_OV "CLR_VIN_OV"
#define LTC3350_CLR_VIN_OV_SIZE 1
#define LTC3350_CLR_VIN_OV_OFFSET 5
#define LTC3350_CLR_VIN_OV_MASK 0x0020
#define LTC3350_CLR_VIN_OV (LTC3350_CLR_VIN_OV_OFFSET << 12 | (LTC3350_CLR_VIN_OV_SIZE - 1) << 8 | LTC3350_CLR_VIN_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_VIN_UV CLR_VIN_UV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_VIN_UV Bit Field
 *
 *  Clear VIN undervoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 4
 *   - MSB: 4
 *   - MASK: 0x0010
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_VIN_UV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_VIN_UV "CLR_VIN_UV"
#define LTC3350_CLR_VIN_UV_SIZE 1
#define LTC3350_CLR_VIN_UV_OFFSET 4
#define LTC3350_CLR_VIN_UV_MASK 0x0010
#define LTC3350_CLR_VIN_UV (LTC3350_CLR_VIN_UV_OFFSET << 12 | (LTC3350_CLR_VIN_UV_SIZE - 1) << 8 | LTC3350_CLR_VIN_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_GPI_OV CLR_GPI_OV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_GPI_OV Bit Field
 *
 *  Clear GPI overvoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 3
 *   - MSB: 3
 *   - MASK: 0x0008
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_GPI_OV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_GPI_OV "CLR_GPI_OV"
#define LTC3350_CLR_GPI_OV_SIZE 1
#define LTC3350_CLR_GPI_OV_OFFSET 3
#define LTC3350_CLR_GPI_OV_MASK 0x0008
#define LTC3350_CLR_GPI_OV (LTC3350_CLR_GPI_OV_OFFSET << 12 | (LTC3350_CLR_GPI_OV_SIZE - 1) << 8 | LTC3350_CLR_GPI_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_GPI_UV CLR_GPI_UV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_GPI_UV Bit Field
 *
 *  Clear GPI undervoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 2
 *   - MSB: 2
 *   - MASK: 0x0004
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_GPI_UV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_GPI_UV "CLR_GPI_UV"
#define LTC3350_CLR_GPI_UV_SIZE 1
#define LTC3350_CLR_GPI_UV_OFFSET 2
#define LTC3350_CLR_GPI_UV_MASK 0x0004
#define LTC3350_CLR_GPI_UV (LTC3350_CLR_GPI_UV_OFFSET << 12 | (LTC3350_CLR_GPI_UV_SIZE - 1) << 8 | LTC3350_CLR_GPI_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_CAP_OV CLR_CAP_OV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_CAP_OV Bit Field
 *
 *  Clear capacitor overvoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 1
 *   - MSB: 1
 *   - MASK: 0x0002
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_CAP_OV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_CAP_OV "CLR_CAP_OV"
#define LTC3350_CLR_CAP_OV_SIZE 1
#define LTC3350_CLR_CAP_OV_OFFSET 1
#define LTC3350_CLR_CAP_OV_MASK 0x0002
#define LTC3350_CLR_CAP_OV (LTC3350_CLR_CAP_OV_OFFSET << 12 | (LTC3350_CLR_CAP_OV_SIZE - 1) << 8 | LTC3350_CLR_CAP_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CLR_CAP_UV CLR_CAP_UV
 *  @ingroup LTC3350_register_map
 *  @brief CLR_CAP_UV Bit Field
 *
 *  Clear capacitor undervoltage alarm
 *   - Register: @ref LTC3350_CLR_ALARMS "CLR_ALARMS"
 *   - CommandCode: 0
 *   - Size: 1
 *   - Offset: 0
 *   - MSB: 0
 *   - MASK: 0x0001
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CLR_CAP_UV_SUBADDR LTC3350_CLR_ALARMS_SUBADDR //!< @ref LTC3350_CLR_CAP_UV "CLR_CAP_UV"
#define LTC3350_CLR_CAP_UV_SIZE 1
#define LTC3350_CLR_CAP_UV_OFFSET 0
#define LTC3350_CLR_CAP_UV_MASK 0x0001
#define LTC3350_CLR_CAP_UV (LTC3350_CLR_CAP_UV_OFFSET << 12 | (LTC3350_CLR_CAP_UV_SIZE - 1) << 8 | LTC3350_CLR_CAP_UV_SUBADDR)
//!@}

/*! @defgroup LTC3350_MSK_ALARMS MSK_ALARMS
 *  @ingroup LTC3350_register_map
 *  @brief MSK_ALARMS Register
 *
 * |         15 |         14 |            13 |             12 |          11 |         10 |           9 |           8 |           7 |           6 |          5 |          4 |          3 |          2 |          1 |          0 |
 * |:----------:|:----------:|:-------------:|:--------------:|:-----------:|:----------:|:-----------:|:-----------:|:-----------:|:-----------:|:----------:|:----------:|:----------:|:----------:|:----------:|:----------:|
 * | MSK_CAP_LO | MSK_ESR_HI | MSK_DTEMP_HOT | MSK_DTEMP_COLD | MSK_ICHG_UC | MSK_IIN_OC | MSK_VOUT_OV | MSK_VOUT_UV | MSK_VCAP_OV | MSK_VCAP_UV | MSK_VIN_OV | MSK_VIN_UV | MSK_GPI_OV | MSK_GPI_UV | MSK_CAP_OV | MSK_CAP_UV |
 *
 * Mask Alarms Register: Writing a one to any bit in the Mask Alarms Register enables its respective alarm to trigger an SMBALERT.
 *   - CommandCode: 1
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MSK_CAP_UV "MSK_CAP_UV" : Enable capacitor undervoltage alarm
 *     + @ref LTC3350_MSK_CAP_OV "MSK_CAP_OV" : Enable capacitor over voltage alarm
 *     + @ref LTC3350_MSK_GPI_UV "MSK_GPI_UV" : Enable GPI undervoltage alarm
 *     + @ref LTC3350_MSK_GPI_OV "MSK_GPI_OV" : Enable GPI overvoltage alarm
 *     + @ref LTC3350_MSK_VIN_UV "MSK_VIN_UV" : Enable VIN undervoltage alarm
 *     + @ref LTC3350_MSK_VIN_OV "MSK_VIN_OV" : Enable VIN overvoltage alarm
 *     + @ref LTC3350_MSK_VCAP_UV "MSK_VCAP_UV" : Enable VCAP undervoltage alarm
 *     + @ref LTC3350_MSK_VCAP_OV "MSK_VCAP_OV" : Enable VCAP overvoltage alarm
 *     + @ref LTC3350_MSK_VOUT_UV "MSK_VOUT_UV" : Enable VOUT undervoltage alarm
 *     + @ref LTC3350_MSK_VOUT_OV "MSK_VOUT_OV" : Enable VOUT overvoltage alarm
 *     + @ref LTC3350_MSK_IIN_OC "MSK_IIN_OC" : Enable input overcurrent alarm
 *     + @ref LTC3350_MSK_ICHG_UC "MSK_ICHG_UC" : Enable charge undercurrent alarm
 *     + @ref LTC3350_MSK_DTEMP_COLD "MSK_DTEMP_COLD" : Enable die temperature cold alarm
 *     + @ref LTC3350_MSK_DTEMP_HOT "MSK_DTEMP_HOT" : Enable die temperature hot alarm
 *     + @ref LTC3350_MSK_ESR_HI "MSK_ESR_HI" : Enable ESR high alarm
 *     + @ref LTC3350_MSK_CAP_LO "MSK_CAP_LO" : Enable capacitance low alarm
*/

//!@{
#define LTC3350_MSK_ALARMS_SUBADDR 1
#define LTC3350_MSK_ALARMS (0 << 12 | (16 - 1) << 8 | LTC3350_MSK_ALARMS_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_CAP_UV MSK_CAP_UV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_CAP_UV Bit Field
 *
 *  Enable capacitor undervoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 0
 *   - MSB: 0
 *   - MASK: 0x0001
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_CAP_UV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_CAP_UV "MSK_CAP_UV"
#define LTC3350_MSK_CAP_UV_SIZE 1
#define LTC3350_MSK_CAP_UV_OFFSET 0
#define LTC3350_MSK_CAP_UV_MASK 0x0001
#define LTC3350_MSK_CAP_UV (LTC3350_MSK_CAP_UV_OFFSET << 12 | (LTC3350_MSK_CAP_UV_SIZE - 1) << 8 | LTC3350_MSK_CAP_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_CAP_OV MSK_CAP_OV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_CAP_OV Bit Field
 *
 *  Enable capacitor over voltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 1
 *   - MSB: 1
 *   - MASK: 0x0002
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_CAP_OV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_CAP_OV "MSK_CAP_OV"
#define LTC3350_MSK_CAP_OV_SIZE 1
#define LTC3350_MSK_CAP_OV_OFFSET 1
#define LTC3350_MSK_CAP_OV_MASK 0x0002
#define LTC3350_MSK_CAP_OV (LTC3350_MSK_CAP_OV_OFFSET << 12 | (LTC3350_MSK_CAP_OV_SIZE - 1) << 8 | LTC3350_MSK_CAP_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_GPI_UV MSK_GPI_UV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_GPI_UV Bit Field
 *
 *  Enable GPI undervoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 2
 *   - MSB: 2
 *   - MASK: 0x0004
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_GPI_UV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_GPI_UV "MSK_GPI_UV"
#define LTC3350_MSK_GPI_UV_SIZE 1
#define LTC3350_MSK_GPI_UV_OFFSET 2
#define LTC3350_MSK_GPI_UV_MASK 0x0004
#define LTC3350_MSK_GPI_UV (LTC3350_MSK_GPI_UV_OFFSET << 12 | (LTC3350_MSK_GPI_UV_SIZE - 1) << 8 | LTC3350_MSK_GPI_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_GPI_OV MSK_GPI_OV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_GPI_OV Bit Field
 *
 *  Enable GPI overvoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 3
 *   - MSB: 3
 *   - MASK: 0x0008
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_GPI_OV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_GPI_OV "MSK_GPI_OV"
#define LTC3350_MSK_GPI_OV_SIZE 1
#define LTC3350_MSK_GPI_OV_OFFSET 3
#define LTC3350_MSK_GPI_OV_MASK 0x0008
#define LTC3350_MSK_GPI_OV (LTC3350_MSK_GPI_OV_OFFSET << 12 | (LTC3350_MSK_GPI_OV_SIZE - 1) << 8 | LTC3350_MSK_GPI_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_VIN_UV MSK_VIN_UV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_VIN_UV Bit Field
 *
 *  Enable VIN undervoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 4
 *   - MSB: 4
 *   - MASK: 0x0010
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_VIN_UV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_VIN_UV "MSK_VIN_UV"
#define LTC3350_MSK_VIN_UV_SIZE 1
#define LTC3350_MSK_VIN_UV_OFFSET 4
#define LTC3350_MSK_VIN_UV_MASK 0x0010
#define LTC3350_MSK_VIN_UV (LTC3350_MSK_VIN_UV_OFFSET << 12 | (LTC3350_MSK_VIN_UV_SIZE - 1) << 8 | LTC3350_MSK_VIN_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_VIN_OV MSK_VIN_OV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_VIN_OV Bit Field
 *
 *  Enable VIN overvoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 5
 *   - MSB: 5
 *   - MASK: 0x0020
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_VIN_OV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_VIN_OV "MSK_VIN_OV"
#define LTC3350_MSK_VIN_OV_SIZE 1
#define LTC3350_MSK_VIN_OV_OFFSET 5
#define LTC3350_MSK_VIN_OV_MASK 0x0020
#define LTC3350_MSK_VIN_OV (LTC3350_MSK_VIN_OV_OFFSET << 12 | (LTC3350_MSK_VIN_OV_SIZE - 1) << 8 | LTC3350_MSK_VIN_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_VCAP_UV MSK_VCAP_UV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_VCAP_UV Bit Field
 *
 *  Enable VCAP undervoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 6
 *   - MSB: 6
 *   - MASK: 0x0040
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_VCAP_UV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_VCAP_UV "MSK_VCAP_UV"
#define LTC3350_MSK_VCAP_UV_SIZE 1
#define LTC3350_MSK_VCAP_UV_OFFSET 6
#define LTC3350_MSK_VCAP_UV_MASK 0x0040
#define LTC3350_MSK_VCAP_UV (LTC3350_MSK_VCAP_UV_OFFSET << 12 | (LTC3350_MSK_VCAP_UV_SIZE - 1) << 8 | LTC3350_MSK_VCAP_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_VCAP_OV MSK_VCAP_OV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_VCAP_OV Bit Field
 *
 *  Enable VCAP overvoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 7
 *   - MSB: 7
 *   - MASK: 0x0080
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_VCAP_OV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_VCAP_OV "MSK_VCAP_OV"
#define LTC3350_MSK_VCAP_OV_SIZE 1
#define LTC3350_MSK_VCAP_OV_OFFSET 7
#define LTC3350_MSK_VCAP_OV_MASK 0x0080
#define LTC3350_MSK_VCAP_OV (LTC3350_MSK_VCAP_OV_OFFSET << 12 | (LTC3350_MSK_VCAP_OV_SIZE - 1) << 8 | LTC3350_MSK_VCAP_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_VOUT_UV MSK_VOUT_UV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_VOUT_UV Bit Field
 *
 *  Enable VOUT undervoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 8
 *   - MSB: 8
 *   - MASK: 0x0100
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_VOUT_UV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_VOUT_UV "MSK_VOUT_UV"
#define LTC3350_MSK_VOUT_UV_SIZE 1
#define LTC3350_MSK_VOUT_UV_OFFSET 8
#define LTC3350_MSK_VOUT_UV_MASK 0x0100
#define LTC3350_MSK_VOUT_UV (LTC3350_MSK_VOUT_UV_OFFSET << 12 | (LTC3350_MSK_VOUT_UV_SIZE - 1) << 8 | LTC3350_MSK_VOUT_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_VOUT_OV MSK_VOUT_OV
 *  @ingroup LTC3350_register_map
 *  @brief MSK_VOUT_OV Bit Field
 *
 *  Enable VOUT overvoltage alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 9
 *   - MSB: 9
 *   - MASK: 0x0200
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_VOUT_OV_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_VOUT_OV "MSK_VOUT_OV"
#define LTC3350_MSK_VOUT_OV_SIZE 1
#define LTC3350_MSK_VOUT_OV_OFFSET 9
#define LTC3350_MSK_VOUT_OV_MASK 0x0200
#define LTC3350_MSK_VOUT_OV (LTC3350_MSK_VOUT_OV_OFFSET << 12 | (LTC3350_MSK_VOUT_OV_SIZE - 1) << 8 | LTC3350_MSK_VOUT_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_IIN_OC MSK_IIN_OC
 *  @ingroup LTC3350_register_map
 *  @brief MSK_IIN_OC Bit Field
 *
 *  Enable input overcurrent alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 10
 *   - MSB: 10
 *   - MASK: 0x0400
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_IIN_OC_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_IIN_OC "MSK_IIN_OC"
#define LTC3350_MSK_IIN_OC_SIZE 1
#define LTC3350_MSK_IIN_OC_OFFSET 10
#define LTC3350_MSK_IIN_OC_MASK 0x0400
#define LTC3350_MSK_IIN_OC (LTC3350_MSK_IIN_OC_OFFSET << 12 | (LTC3350_MSK_IIN_OC_SIZE - 1) << 8 | LTC3350_MSK_IIN_OC_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_ICHG_UC MSK_ICHG_UC
 *  @ingroup LTC3350_register_map
 *  @brief MSK_ICHG_UC Bit Field
 *
 *  Enable charge undercurrent alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 11
 *   - MSB: 11
 *   - MASK: 0x0800
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_ICHG_UC_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_ICHG_UC "MSK_ICHG_UC"
#define LTC3350_MSK_ICHG_UC_SIZE 1
#define LTC3350_MSK_ICHG_UC_OFFSET 11
#define LTC3350_MSK_ICHG_UC_MASK 0x0800
#define LTC3350_MSK_ICHG_UC (LTC3350_MSK_ICHG_UC_OFFSET << 12 | (LTC3350_MSK_ICHG_UC_SIZE - 1) << 8 | LTC3350_MSK_ICHG_UC_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_DTEMP_COLD MSK_DTEMP_COLD
 *  @ingroup LTC3350_register_map
 *  @brief MSK_DTEMP_COLD Bit Field
 *
 *  Enable die temperature cold alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 12
 *   - MSB: 12
 *   - MASK: 0x1000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_DTEMP_COLD_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_DTEMP_COLD "MSK_DTEMP_COLD"
#define LTC3350_MSK_DTEMP_COLD_SIZE 1
#define LTC3350_MSK_DTEMP_COLD_OFFSET 12
#define LTC3350_MSK_DTEMP_COLD_MASK 0x1000
#define LTC3350_MSK_DTEMP_COLD (LTC3350_MSK_DTEMP_COLD_OFFSET << 12 | (LTC3350_MSK_DTEMP_COLD_SIZE - 1) << 8 | LTC3350_MSK_DTEMP_COLD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_DTEMP_HOT MSK_DTEMP_HOT
 *  @ingroup LTC3350_register_map
 *  @brief MSK_DTEMP_HOT Bit Field
 *
 *  Enable die temperature hot alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 13
 *   - MSB: 13
 *   - MASK: 0x2000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_DTEMP_HOT_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_DTEMP_HOT "MSK_DTEMP_HOT"
#define LTC3350_MSK_DTEMP_HOT_SIZE 1
#define LTC3350_MSK_DTEMP_HOT_OFFSET 13
#define LTC3350_MSK_DTEMP_HOT_MASK 0x2000
#define LTC3350_MSK_DTEMP_HOT (LTC3350_MSK_DTEMP_HOT_OFFSET << 12 | (LTC3350_MSK_DTEMP_HOT_SIZE - 1) << 8 | LTC3350_MSK_DTEMP_HOT_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_ESR_HI MSK_ESR_HI
 *  @ingroup LTC3350_register_map
 *  @brief MSK_ESR_HI Bit Field
 *
 *  Enable ESR high alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 14
 *   - MSB: 14
 *   - MASK: 0x4000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_ESR_HI_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_ESR_HI "MSK_ESR_HI"
#define LTC3350_MSK_ESR_HI_SIZE 1
#define LTC3350_MSK_ESR_HI_OFFSET 14
#define LTC3350_MSK_ESR_HI_MASK 0x4000
#define LTC3350_MSK_ESR_HI (LTC3350_MSK_ESR_HI_OFFSET << 12 | (LTC3350_MSK_ESR_HI_SIZE - 1) << 8 | LTC3350_MSK_ESR_HI_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_CAP_LO MSK_CAP_LO
 *  @ingroup LTC3350_register_map
 *  @brief MSK_CAP_LO Bit Field
 *
 *  Enable capacitance low alarm
 *   - Register: @ref LTC3350_MSK_ALARMS "MSK_ALARMS"
 *   - CommandCode: 1
 *   - Size: 1
 *   - Offset: 15
 *   - MSB: 15
 *   - MASK: 0x8000
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_CAP_LO_SUBADDR LTC3350_MSK_ALARMS_SUBADDR //!< @ref LTC3350_MSK_CAP_LO "MSK_CAP_LO"
#define LTC3350_MSK_CAP_LO_SIZE 1
#define LTC3350_MSK_CAP_LO_OFFSET 15
#define LTC3350_MSK_CAP_LO_MASK 0x8000
#define LTC3350_MSK_CAP_LO (LTC3350_MSK_CAP_LO_OFFSET << 12 | (LTC3350_MSK_CAP_LO_SIZE - 1) << 8 | LTC3350_MSK_CAP_LO_SUBADDR)
//!@}

/*! @defgroup LTC3350_MSK_MON_STATUS MSK_MON_STATUS
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_STATUS Register
 *
 * | 15:10 |                      9 |                    8 |   7 |                  6 |                  5 |                4 |                3 |                      2 |                        1 |                     0 |
 * |:-----:|:----------------------:|:--------------------:|:---:|:------------------:|:------------------:|:----------------:|:----------------:|:----------------------:|:------------------------:|:---------------------:|
 * |   n/a | MSK_MON_POWER_RETURNED | MSK_MON_POWER_FAILED | n/a | MSK_MON_ESR_FAILED | MSK_MON_CAP_FAILED | MSK_MON_ESR_DONE | MSK_MON_CAP_DONE | MSK_MON_CAPESR_PENDING | MSK_MON_CAPESR_SCHEDULED | MSK_MON_CAPESR_ACTIVE |
 *
 * Mask Monitor Status Register: Writing a one to any bit in this register enables a rising edge of its respective bit in the mon_status register to trigger an SMBALERT.
 *   - CommandCode: 2
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MSK_MON_CAPESR_ACTIVE "MSK_MON_CAPESR_ACTIVE" : Set the SMBALERT when there is a rising edge on mon_capesr_active
 *     + @ref LTC3350_MSK_MON_CAPESR_SCHEDULED "MSK_MON_CAPESR_SCHEDULED" : Set the SMBALERT when there is a rising edge on mon_capesr_scheduled
 *     + @ref LTC3350_MSK_MON_CAPESR_PENDING "MSK_MON_CAPESR_PENDING" : Set the SMBALERT when there is a rising edge on mon_capesr_pending
 *     + @ref LTC3350_MSK_MON_CAP_DONE "MSK_MON_CAP_DONE" : Set the SMBALERT when there is a rising edge on mon_cap_done
 *     + @ref LTC3350_MSK_MON_ESR_DONE "MSK_MON_ESR_DONE" : Set the SMBALERT when there is a rising edge on mon_esr_done
 *     + @ref LTC3350_MSK_MON_CAP_FAILED "MSK_MON_CAP_FAILED" : Set the SMBALERT when there is a rising edge on mon_cap_failed
 *     + @ref LTC3350_MSK_MON_ESR_FAILED "MSK_MON_ESR_FAILED" : Set the SMBALERT when there is a rising edge on mon_esr_failed
 *     + @ref LTC3350_MSK_MON_POWER_FAILED "MSK_MON_POWER_FAILED" : Set the SMBALERT when there is a rising edge on mon_power_failed
 *     + @ref LTC3350_MSK_MON_POWER_RETURNED "MSK_MON_POWER_RETURNED" : Set the SMBALERT when there is a rising edge on mon_power_returned
*/

//!@{
#define LTC3350_MSK_MON_STATUS_SUBADDR 2
#define LTC3350_MSK_MON_STATUS (0 << 12 | (16 - 1) << 8 | LTC3350_MSK_MON_STATUS_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_CAPESR_ACTIVE MSK_MON_CAPESR_ACTIVE
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_CAPESR_ACTIVE Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_capesr_active
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 0
 *   - MSB: 0
 *   - MASK: 0x0001
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_CAPESR_ACTIVE_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_CAPESR_ACTIVE "MSK_MON_CAPESR_ACTIVE"
#define LTC3350_MSK_MON_CAPESR_ACTIVE_SIZE 1
#define LTC3350_MSK_MON_CAPESR_ACTIVE_OFFSET 0
#define LTC3350_MSK_MON_CAPESR_ACTIVE_MASK 0x0001
#define LTC3350_MSK_MON_CAPESR_ACTIVE (LTC3350_MSK_MON_CAPESR_ACTIVE_OFFSET << 12 | (LTC3350_MSK_MON_CAPESR_ACTIVE_SIZE - 1) << 8 | LTC3350_MSK_MON_CAPESR_ACTIVE_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_CAPESR_SCHEDULED MSK_MON_CAPESR_SCHEDULED
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_CAPESR_SCHEDULED Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_capesr_scheduled
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 1
 *   - MSB: 1
 *   - MASK: 0x0002
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_CAPESR_SCHEDULED_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_CAPESR_SCHEDULED "MSK_MON_CAPESR_SCHEDULED"
#define LTC3350_MSK_MON_CAPESR_SCHEDULED_SIZE 1
#define LTC3350_MSK_MON_CAPESR_SCHEDULED_OFFSET 1
#define LTC3350_MSK_MON_CAPESR_SCHEDULED_MASK 0x0002
#define LTC3350_MSK_MON_CAPESR_SCHEDULED (LTC3350_MSK_MON_CAPESR_SCHEDULED_OFFSET << 12 | (LTC3350_MSK_MON_CAPESR_SCHEDULED_SIZE - 1) << 8 | LTC3350_MSK_MON_CAPESR_SCHEDULED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_CAPESR_PENDING MSK_MON_CAPESR_PENDING
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_CAPESR_PENDING Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_capesr_pending
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 2
 *   - MSB: 2
 *   - MASK: 0x0004
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_CAPESR_PENDING_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_CAPESR_PENDING "MSK_MON_CAPESR_PENDING"
#define LTC3350_MSK_MON_CAPESR_PENDING_SIZE 1
#define LTC3350_MSK_MON_CAPESR_PENDING_OFFSET 2
#define LTC3350_MSK_MON_CAPESR_PENDING_MASK 0x0004
#define LTC3350_MSK_MON_CAPESR_PENDING (LTC3350_MSK_MON_CAPESR_PENDING_OFFSET << 12 | (LTC3350_MSK_MON_CAPESR_PENDING_SIZE - 1) << 8 | LTC3350_MSK_MON_CAPESR_PENDING_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_CAP_DONE MSK_MON_CAP_DONE
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_CAP_DONE Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_cap_done
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 3
 *   - MSB: 3
 *   - MASK: 0x0008
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_CAP_DONE_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_CAP_DONE "MSK_MON_CAP_DONE"
#define LTC3350_MSK_MON_CAP_DONE_SIZE 1
#define LTC3350_MSK_MON_CAP_DONE_OFFSET 3
#define LTC3350_MSK_MON_CAP_DONE_MASK 0x0008
#define LTC3350_MSK_MON_CAP_DONE (LTC3350_MSK_MON_CAP_DONE_OFFSET << 12 | (LTC3350_MSK_MON_CAP_DONE_SIZE - 1) << 8 | LTC3350_MSK_MON_CAP_DONE_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_ESR_DONE MSK_MON_ESR_DONE
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_ESR_DONE Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_esr_done
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 4
 *   - MSB: 4
 *   - MASK: 0x0010
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_ESR_DONE_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_ESR_DONE "MSK_MON_ESR_DONE"
#define LTC3350_MSK_MON_ESR_DONE_SIZE 1
#define LTC3350_MSK_MON_ESR_DONE_OFFSET 4
#define LTC3350_MSK_MON_ESR_DONE_MASK 0x0010
#define LTC3350_MSK_MON_ESR_DONE (LTC3350_MSK_MON_ESR_DONE_OFFSET << 12 | (LTC3350_MSK_MON_ESR_DONE_SIZE - 1) << 8 | LTC3350_MSK_MON_ESR_DONE_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_CAP_FAILED MSK_MON_CAP_FAILED
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_CAP_FAILED Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_cap_failed
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 5
 *   - MSB: 5
 *   - MASK: 0x0020
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_CAP_FAILED_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_CAP_FAILED "MSK_MON_CAP_FAILED"
#define LTC3350_MSK_MON_CAP_FAILED_SIZE 1
#define LTC3350_MSK_MON_CAP_FAILED_OFFSET 5
#define LTC3350_MSK_MON_CAP_FAILED_MASK 0x0020
#define LTC3350_MSK_MON_CAP_FAILED (LTC3350_MSK_MON_CAP_FAILED_OFFSET << 12 | (LTC3350_MSK_MON_CAP_FAILED_SIZE - 1) << 8 | LTC3350_MSK_MON_CAP_FAILED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_ESR_FAILED MSK_MON_ESR_FAILED
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_ESR_FAILED Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_esr_failed
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 6
 *   - MSB: 6
 *   - MASK: 0x0040
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_ESR_FAILED_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_ESR_FAILED "MSK_MON_ESR_FAILED"
#define LTC3350_MSK_MON_ESR_FAILED_SIZE 1
#define LTC3350_MSK_MON_ESR_FAILED_OFFSET 6
#define LTC3350_MSK_MON_ESR_FAILED_MASK 0x0040
#define LTC3350_MSK_MON_ESR_FAILED (LTC3350_MSK_MON_ESR_FAILED_OFFSET << 12 | (LTC3350_MSK_MON_ESR_FAILED_SIZE - 1) << 8 | LTC3350_MSK_MON_ESR_FAILED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_POWER_FAILED MSK_MON_POWER_FAILED
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_POWER_FAILED Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_power_failed
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 8
 *   - MSB: 8
 *   - MASK: 0x0100
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_POWER_FAILED_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_POWER_FAILED "MSK_MON_POWER_FAILED"
#define LTC3350_MSK_MON_POWER_FAILED_SIZE 1
#define LTC3350_MSK_MON_POWER_FAILED_OFFSET 8
#define LTC3350_MSK_MON_POWER_FAILED_MASK 0x0100
#define LTC3350_MSK_MON_POWER_FAILED (LTC3350_MSK_MON_POWER_FAILED_OFFSET << 12 | (LTC3350_MSK_MON_POWER_FAILED_SIZE - 1) << 8 | LTC3350_MSK_MON_POWER_FAILED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MSK_MON_POWER_RETURNED MSK_MON_POWER_RETURNED
 *  @ingroup LTC3350_register_map
 *  @brief MSK_MON_POWER_RETURNED Bit Field
 *
 *  Set the SMBALERT when there is a rising edge on mon_power_returned
 *   - Register: @ref LTC3350_MSK_MON_STATUS "MSK_MON_STATUS"
 *   - CommandCode: 2
 *   - Size: 1
 *   - Offset: 9
 *   - MSB: 9
 *   - MASK: 0x0200
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_MSK_MON_POWER_RETURNED_SUBADDR LTC3350_MSK_MON_STATUS_SUBADDR //!< @ref LTC3350_MSK_MON_POWER_RETURNED "MSK_MON_POWER_RETURNED"
#define LTC3350_MSK_MON_POWER_RETURNED_SIZE 1
#define LTC3350_MSK_MON_POWER_RETURNED_OFFSET 9
#define LTC3350_MSK_MON_POWER_RETURNED_MASK 0x0200
#define LTC3350_MSK_MON_POWER_RETURNED (LTC3350_MSK_MON_POWER_RETURNED_OFFSET << 12 | (LTC3350_MSK_MON_POWER_RETURNED_SIZE - 1) << 8 | LTC3350_MSK_MON_POWER_RETURNED_SUBADDR)
//!@}

/*! @defgroup LTC3350_CAP_ESR_PER_CMD CAP_ESR_PER_CMD
 *  @ingroup LTC3350_register_map
 *  @brief CAP_ESR_PER_CMD Register
 *
 * |        15:0 |
 * |:-----------:|
 * | CAP_ESR_PER |
 *
 *   - CommandCode: 4
 *   - Contains Bit Fields:
 *     + @ref LTC3350_CAP_ESR_PER "CAP_ESR_PER" : Capacitance and ESR Measurement Period: This register sets the period of repeated capacitance and ESR measurements. Each LSB represents 10 seconds. Capacitance and ESR measurements will not repeat if this register is zero.
*/

//!@{
#define LTC3350_CAP_ESR_PER_CMD_SUBADDR 4
#define LTC3350_CAP_ESR_PER_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_CAP_ESR_PER_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_CAP_ESR_PER CAP_ESR_PER
 *  @ingroup LTC3350_register_map
 *  @brief CAP_ESR_PER Bit Field
 *
 *  Capacitance and ESR Measurement Period: This register sets the period of repeated capacitance and ESR measurements. Each LSB represents 10 seconds. Capacitance and ESR measurements will not repeat if this register is zero.
 *   - Register: @ref LTC3350_CAP_ESR_PER_CMD "CAP_ESR_PER_CMD"
 *   - CommandCode: 4
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CAP_ESR_PER_SUBADDR LTC3350_CAP_ESR_PER_CMD_SUBADDR //!< @ref LTC3350_CAP_ESR_PER "CAP_ESR_PER"
#define LTC3350_CAP_ESR_PER_SIZE 16
#define LTC3350_CAP_ESR_PER_OFFSET 0
#define LTC3350_CAP_ESR_PER_MASK 0xFFFF
#define LTC3350_CAP_ESR_PER (LTC3350_CAP_ESR_PER_OFFSET << 12 | (LTC3350_CAP_ESR_PER_SIZE - 1) << 8 | LTC3350_CAP_ESR_PER_SUBADDR)
//!@}

/*! @defgroup LTC3350_VCAPFB_DAC_CMD VCAPFB_DAC_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VCAPFB_DAC_CMD Register
 *
 * | 15:4 |        3:0 |
 * |:----:|:----------:|
 * |  n/a | VCAPFB_DAC |
 *
 *   - CommandCode: 5
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VCAPFB_DAC "VCAPFB_DAC" : VCAP Regulation Reference: This register is used to program the capacitor voltage feedback loop's reference voltage. Only bits 3:0 are active. CAPFBREF = 37.5mV * vcapfb_dac + 637.5mV
*/

//!@{
#define LTC3350_VCAPFB_DAC_CMD_SUBADDR 5
#define LTC3350_VCAPFB_DAC_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VCAPFB_DAC_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VCAPFB_DAC VCAPFB_DAC
 *  @ingroup LTC3350_register_map
 *  @brief VCAPFB_DAC Bit Field
 *
 *  VCAP Regulation Reference: This register is used to program the capacitor voltage feedback loop's reference voltage. Only bits 3:0 are active. CAPFBREF = 37.5mV * vcapfb_dac + 637.5mV
 *   - Register: @ref LTC3350_VCAPFB_DAC_CMD "VCAPFB_DAC_CMD"
 *   - CommandCode: 5
 *   - Size: 4
 *   - Offset: 0
 *   - MSB: 3
 *   - MASK: 0x000F
 *   - Access: R/W
 *   - Default: 0xF
 */
//!@{
#define LTC3350_VCAPFB_DAC_SUBADDR LTC3350_VCAPFB_DAC_CMD_SUBADDR //!< @ref LTC3350_VCAPFB_DAC "VCAPFB_DAC"
#define LTC3350_VCAPFB_DAC_SIZE 4
#define LTC3350_VCAPFB_DAC_OFFSET 0
#define LTC3350_VCAPFB_DAC_MASK 0x000F
#define LTC3350_VCAPFB_DAC (LTC3350_VCAPFB_DAC_OFFSET << 12 | (LTC3350_VCAPFB_DAC_SIZE - 1) << 8 | LTC3350_VCAPFB_DAC_SUBADDR)
//!@}

/*! @defgroup LTC3350_VSHUNT_CMD VSHUNT_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VSHUNT_CMD Register
 *
 * |   15:0 |
 * |:------:|
 * | VSHUNT |
 *
 *   - CommandCode: 6
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VSHUNT "VSHUNT" : Shunt Voltage Register: This register programs the shunt voltage for each capacitor in the stack. The charger will limit current and the active shunts will shunt current to prevent this voltage from being exceeded. As a capacitor voltage nears this level, the charge current will be reduced. This should be programmed higher than the intended final balanced individual capacitor voltage. Setting this register to 0x0000 disables the shunt. 183.5uV per LSB.
*/

//!@{
#define LTC3350_VSHUNT_CMD_SUBADDR 6
#define LTC3350_VSHUNT_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VSHUNT_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VSHUNT VSHUNT
 *  @ingroup LTC3350_register_map
 *  @brief VSHUNT Bit Field
 *
 *  Shunt Voltage Register: This register programs the shunt voltage for each capacitor in the stack. The charger will limit current and the active shunts will shunt current to prevent this voltage from being exceeded. As a capacitor voltage nears this level, the charge current will be reduced. This should be programmed higher than the intended final balanced individual capacitor voltage. Setting this register to 0x0000 disables the shunt. 183.5uV per LSB.
 *   - Register: @ref LTC3350_VSHUNT_CMD "VSHUNT_CMD"
 *   - CommandCode: 6
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 14745
 */
//!@{
#define LTC3350_VSHUNT_SUBADDR LTC3350_VSHUNT_CMD_SUBADDR //!< @ref LTC3350_VSHUNT "VSHUNT"
#define LTC3350_VSHUNT_SIZE 16
#define LTC3350_VSHUNT_OFFSET 0
#define LTC3350_VSHUNT_MASK 0xFFFF
#define LTC3350_VSHUNT (LTC3350_VSHUNT_OFFSET << 12 | (LTC3350_VSHUNT_SIZE - 1) << 8 | LTC3350_VSHUNT_SUBADDR)
//!@}

/*! @defgroup LTC3350_CAP_UV_LVL_CMD CAP_UV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief CAP_UV_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | CAP_UV_LVL |
 *
 *   - CommandCode: 7
 *   - Contains Bit Fields:
 *     + @ref LTC3350_CAP_UV_LVL "CAP_UV_LVL" : Capacitor Undervoltage Level: This is an alarm threshold for each individual capacitor voltage in the stack. If enabled, any capacitor voltage falling below this level will trigger an alarm and an SMBALERT. 183.5uV per LSB.
*/

//!@{
#define LTC3350_CAP_UV_LVL_CMD_SUBADDR 7
#define LTC3350_CAP_UV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_CAP_UV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_CAP_UV_LVL CAP_UV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief CAP_UV_LVL Bit Field
 *
 *  Capacitor Undervoltage Level: This is an alarm threshold for each individual capacitor voltage in the stack. If enabled, any capacitor voltage falling below this level will trigger an alarm and an SMBALERT. 183.5uV per LSB.
 *   - Register: @ref LTC3350_CAP_UV_LVL_CMD "CAP_UV_LVL_CMD"
 *   - CommandCode: 7
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CAP_UV_LVL_SUBADDR LTC3350_CAP_UV_LVL_CMD_SUBADDR //!< @ref LTC3350_CAP_UV_LVL "CAP_UV_LVL"
#define LTC3350_CAP_UV_LVL_SIZE 16
#define LTC3350_CAP_UV_LVL_OFFSET 0
#define LTC3350_CAP_UV_LVL_MASK 0xFFFF
#define LTC3350_CAP_UV_LVL (LTC3350_CAP_UV_LVL_OFFSET << 12 | (LTC3350_CAP_UV_LVL_SIZE - 1) << 8 | LTC3350_CAP_UV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_CAP_OV_LVL_CMD CAP_OV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief CAP_OV_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | CAP_OV_LVL |
 *
 *   - CommandCode: 8
 *   - Contains Bit Fields:
 *     + @ref LTC3350_CAP_OV_LVL "CAP_OV_LVL" : Capacitor Overvoltage Level: This is an alarm threshold for each individual capacitor in the stack. If enabled, any capacitor voltage rising above this level will trigger an alarm and an SMBALERT. 183.5uV per LSB
*/

//!@{
#define LTC3350_CAP_OV_LVL_CMD_SUBADDR 8
#define LTC3350_CAP_OV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_CAP_OV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_CAP_OV_LVL CAP_OV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief CAP_OV_LVL Bit Field
 *
 *  Capacitor Overvoltage Level: This is an alarm threshold for each individual capacitor in the stack. If enabled, any capacitor voltage rising above this level will trigger an alarm and an SMBALERT. 183.5uV per LSB
 *   - Register: @ref LTC3350_CAP_OV_LVL_CMD "CAP_OV_LVL_CMD"
 *   - CommandCode: 8
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CAP_OV_LVL_SUBADDR LTC3350_CAP_OV_LVL_CMD_SUBADDR //!< @ref LTC3350_CAP_OV_LVL "CAP_OV_LVL"
#define LTC3350_CAP_OV_LVL_SIZE 16
#define LTC3350_CAP_OV_LVL_OFFSET 0
#define LTC3350_CAP_OV_LVL_MASK 0xFFFF
#define LTC3350_CAP_OV_LVL (LTC3350_CAP_OV_LVL_OFFSET << 12 | (LTC3350_CAP_OV_LVL_SIZE - 1) << 8 | LTC3350_CAP_OV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_GPI_UV_LVL_CMD GPI_UV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief GPI_UV_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | GPI_UV_LVL |
 *
 *   - CommandCode: 9
 *   - Contains Bit Fields:
 *     + @ref LTC3350_GPI_UV_LVL "GPI_UV_LVL" : General Purpose Input Undervoltage Level: This is an alarm threshold for the GPI pin. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 183.5uV per LSB
*/

//!@{
#define LTC3350_GPI_UV_LVL_CMD_SUBADDR 9
#define LTC3350_GPI_UV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_GPI_UV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_GPI_UV_LVL GPI_UV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief GPI_UV_LVL Bit Field
 *
 *  General Purpose Input Undervoltage Level: This is an alarm threshold for the GPI pin. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 183.5uV per LSB
 *   - Register: @ref LTC3350_GPI_UV_LVL_CMD "GPI_UV_LVL_CMD"
 *   - CommandCode: 9
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_GPI_UV_LVL_SUBADDR LTC3350_GPI_UV_LVL_CMD_SUBADDR //!< @ref LTC3350_GPI_UV_LVL "GPI_UV_LVL"
#define LTC3350_GPI_UV_LVL_SIZE 16
#define LTC3350_GPI_UV_LVL_OFFSET 0
#define LTC3350_GPI_UV_LVL_MASK 0xFFFF
#define LTC3350_GPI_UV_LVL (LTC3350_GPI_UV_LVL_OFFSET << 12 | (LTC3350_GPI_UV_LVL_SIZE - 1) << 8 | LTC3350_GPI_UV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_GPI_OV_LVL_CMD GPI_OV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief GPI_OV_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | GPI_OV_LVL |
 *
 *   - CommandCode: 10
 *   - Contains Bit Fields:
 *     + @ref LTC3350_GPI_OV_LVL "GPI_OV_LVL" : General Purpose Input Overvoltage Level: This is an alarm threshold for the GPI pin. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 183.5uV per LSB
*/

//!@{
#define LTC3350_GPI_OV_LVL_CMD_SUBADDR 10
#define LTC3350_GPI_OV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_GPI_OV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_GPI_OV_LVL GPI_OV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief GPI_OV_LVL Bit Field
 *
 *  General Purpose Input Overvoltage Level: This is an alarm threshold for the GPI pin. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 183.5uV per LSB
 *   - Register: @ref LTC3350_GPI_OV_LVL_CMD "GPI_OV_LVL_CMD"
 *   - CommandCode: 10
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_GPI_OV_LVL_SUBADDR LTC3350_GPI_OV_LVL_CMD_SUBADDR //!< @ref LTC3350_GPI_OV_LVL "GPI_OV_LVL"
#define LTC3350_GPI_OV_LVL_SIZE 16
#define LTC3350_GPI_OV_LVL_OFFSET 0
#define LTC3350_GPI_OV_LVL_MASK 0xFFFF
#define LTC3350_GPI_OV_LVL (LTC3350_GPI_OV_LVL_OFFSET << 12 | (LTC3350_GPI_OV_LVL_SIZE - 1) << 8 | LTC3350_GPI_OV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_VIN_UV_LVL_CMD VIN_UV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VIN_UV_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | VIN_UV_LVL |
 *
 *   - CommandCode: 11
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VIN_UV_LVL "VIN_UV_LVL" : VIN Undervoltage Level: This is an alarm threshold for the input voltage. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
*/

//!@{
#define LTC3350_VIN_UV_LVL_CMD_SUBADDR 11
#define LTC3350_VIN_UV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VIN_UV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VIN_UV_LVL VIN_UV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief VIN_UV_LVL Bit Field
 *
 *  VIN Undervoltage Level: This is an alarm threshold for the input voltage. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
 *   - Register: @ref LTC3350_VIN_UV_LVL_CMD "VIN_UV_LVL_CMD"
 *   - CommandCode: 11
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_VIN_UV_LVL_SUBADDR LTC3350_VIN_UV_LVL_CMD_SUBADDR //!< @ref LTC3350_VIN_UV_LVL "VIN_UV_LVL"
#define LTC3350_VIN_UV_LVL_SIZE 16
#define LTC3350_VIN_UV_LVL_OFFSET 0
#define LTC3350_VIN_UV_LVL_MASK 0xFFFF
#define LTC3350_VIN_UV_LVL (LTC3350_VIN_UV_LVL_OFFSET << 12 | (LTC3350_VIN_UV_LVL_SIZE - 1) << 8 | LTC3350_VIN_UV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_VIN_OV_LVL_CMD VIN_OV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VIN_OV_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | VIN_OV_LVL |
 *
 *   - CommandCode: 12
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VIN_OV_LVL "VIN_OV_LVL" : VIN Overvoltage Level: This is an alarm threshold for the input voltage. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
*/

//!@{
#define LTC3350_VIN_OV_LVL_CMD_SUBADDR 12
#define LTC3350_VIN_OV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VIN_OV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VIN_OV_LVL VIN_OV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief VIN_OV_LVL Bit Field
 *
 *  VIN Overvoltage Level: This is an alarm threshold for the input voltage. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
 *   - Register: @ref LTC3350_VIN_OV_LVL_CMD "VIN_OV_LVL_CMD"
 *   - CommandCode: 12
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_VIN_OV_LVL_SUBADDR LTC3350_VIN_OV_LVL_CMD_SUBADDR //!< @ref LTC3350_VIN_OV_LVL "VIN_OV_LVL"
#define LTC3350_VIN_OV_LVL_SIZE 16
#define LTC3350_VIN_OV_LVL_OFFSET 0
#define LTC3350_VIN_OV_LVL_MASK 0xFFFF
#define LTC3350_VIN_OV_LVL (LTC3350_VIN_OV_LVL_OFFSET << 12 | (LTC3350_VIN_OV_LVL_SIZE - 1) << 8 | LTC3350_VIN_OV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_VCAP_UV_LVL_CMD VCAP_UV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VCAP_UV_LVL_CMD Register
 *
 * |        15:0 |
 * |:-----------:|
 * | VCAP_UV_LVL |
 *
 *   - CommandCode: 13
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VCAP_UV_LVL "VCAP_UV_LVL" : VCAP Undervoltage Level: This is an alarm threshold for the capacitor stack voltage. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 1.476mV per LSB
*/

//!@{
#define LTC3350_VCAP_UV_LVL_CMD_SUBADDR 13
#define LTC3350_VCAP_UV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VCAP_UV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VCAP_UV_LVL VCAP_UV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief VCAP_UV_LVL Bit Field
 *
 *  VCAP Undervoltage Level: This is an alarm threshold for the capacitor stack voltage. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 1.476mV per LSB
 *   - Register: @ref LTC3350_VCAP_UV_LVL_CMD "VCAP_UV_LVL_CMD"
 *   - CommandCode: 13
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_VCAP_UV_LVL_SUBADDR LTC3350_VCAP_UV_LVL_CMD_SUBADDR //!< @ref LTC3350_VCAP_UV_LVL "VCAP_UV_LVL"
#define LTC3350_VCAP_UV_LVL_SIZE 16
#define LTC3350_VCAP_UV_LVL_OFFSET 0
#define LTC3350_VCAP_UV_LVL_MASK 0xFFFF
#define LTC3350_VCAP_UV_LVL (LTC3350_VCAP_UV_LVL_OFFSET << 12 | (LTC3350_VCAP_UV_LVL_SIZE - 1) << 8 | LTC3350_VCAP_UV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_VCAP_OV_LVL_CMD VCAP_OV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VCAP_OV_LVL_CMD Register
 *
 * |        15:0 |
 * |:-----------:|
 * | VCAP_OV_LVL |
 *
 *   - CommandCode: 14
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VCAP_OV_LVL "VCAP_OV_LVL" : VCAP Overvoltage Level: This is an alarm threshold for the capacitor stack voltage. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 1.476mV per LSB
*/

//!@{
#define LTC3350_VCAP_OV_LVL_CMD_SUBADDR 14
#define LTC3350_VCAP_OV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VCAP_OV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VCAP_OV_LVL VCAP_OV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief VCAP_OV_LVL Bit Field
 *
 *  VCAP Overvoltage Level: This is an alarm threshold for the capacitor stack voltage. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 1.476mV per LSB
 *   - Register: @ref LTC3350_VCAP_OV_LVL_CMD "VCAP_OV_LVL_CMD"
 *   - CommandCode: 14
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_VCAP_OV_LVL_SUBADDR LTC3350_VCAP_OV_LVL_CMD_SUBADDR //!< @ref LTC3350_VCAP_OV_LVL "VCAP_OV_LVL"
#define LTC3350_VCAP_OV_LVL_SIZE 16
#define LTC3350_VCAP_OV_LVL_OFFSET 0
#define LTC3350_VCAP_OV_LVL_MASK 0xFFFF
#define LTC3350_VCAP_OV_LVL (LTC3350_VCAP_OV_LVL_OFFSET << 12 | (LTC3350_VCAP_OV_LVL_SIZE - 1) << 8 | LTC3350_VCAP_OV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_VOUT_UV_LVL_CMD VOUT_UV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VOUT_UV_LVL_CMD Register
 *
 * |        15:0 |
 * |:-----------:|
 * | VOUT_UV_LVL |
 *
 *   - CommandCode: 15
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VOUT_UV_LVL "VOUT_UV_LVL" : VOUT Undervoltage Level: This is an alarm threshold for the output voltage. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
*/

//!@{
#define LTC3350_VOUT_UV_LVL_CMD_SUBADDR 15
#define LTC3350_VOUT_UV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VOUT_UV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VOUT_UV_LVL VOUT_UV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief VOUT_UV_LVL Bit Field
 *
 *  VOUT Undervoltage Level: This is an alarm threshold for the output voltage. If enabled, the voltage falling below this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
 *   - Register: @ref LTC3350_VOUT_UV_LVL_CMD "VOUT_UV_LVL_CMD"
 *   - CommandCode: 15
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_VOUT_UV_LVL_SUBADDR LTC3350_VOUT_UV_LVL_CMD_SUBADDR //!< @ref LTC3350_VOUT_UV_LVL "VOUT_UV_LVL"
#define LTC3350_VOUT_UV_LVL_SIZE 16
#define LTC3350_VOUT_UV_LVL_OFFSET 0
#define LTC3350_VOUT_UV_LVL_MASK 0xFFFF
#define LTC3350_VOUT_UV_LVL (LTC3350_VOUT_UV_LVL_OFFSET << 12 | (LTC3350_VOUT_UV_LVL_SIZE - 1) << 8 | LTC3350_VOUT_UV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_VOUT_OV_LVL_CMD VOUT_OV_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief VOUT_OV_LVL_CMD Register
 *
 * |        15:0 |
 * |:-----------:|
 * | VOUT_OV_LVL |
 *
 *   - CommandCode: 16
 *   - Contains Bit Fields:
 *     + @ref LTC3350_VOUT_OV_LVL "VOUT_OV_LVL" : VOUT Overvoltage Level: This is an alarm threshold for the output voltage. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
*/

//!@{
#define LTC3350_VOUT_OV_LVL_CMD_SUBADDR 16
#define LTC3350_VOUT_OV_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_VOUT_OV_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_VOUT_OV_LVL VOUT_OV_LVL
 *  @ingroup LTC3350_register_map
 *  @brief VOUT_OV_LVL Bit Field
 *
 *  VOUT Overvoltage Level: This is an alarm threshold for the output voltage. If enabled, the voltage rising above this level will trigger an alarm and an SMBALERT. 2.21mV per LSB
 *   - Register: @ref LTC3350_VOUT_OV_LVL_CMD "VOUT_OV_LVL_CMD"
 *   - CommandCode: 16
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_VOUT_OV_LVL_SUBADDR LTC3350_VOUT_OV_LVL_CMD_SUBADDR //!< @ref LTC3350_VOUT_OV_LVL "VOUT_OV_LVL"
#define LTC3350_VOUT_OV_LVL_SIZE 16
#define LTC3350_VOUT_OV_LVL_OFFSET 0
#define LTC3350_VOUT_OV_LVL_MASK 0xFFFF
#define LTC3350_VOUT_OV_LVL (LTC3350_VOUT_OV_LVL_OFFSET << 12 | (LTC3350_VOUT_OV_LVL_SIZE - 1) << 8 | LTC3350_VOUT_OV_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_IIN_OC_LVL_CMD IIN_OC_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief IIN_OC_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | IIN_OC_LVL |
 *
 *   - CommandCode: 17
 *   - Contains Bit Fields:
 *     + @ref LTC3350_IIN_OC_LVL "IIN_OC_LVL" : Input Overcurrent Level: This is an alarm threshold for the input current. If enabled, the current rising above this level will trigger an alarm and an SMBALERT. 1.983uV/RSNSI per LSB
*/

//!@{
#define LTC3350_IIN_OC_LVL_CMD_SUBADDR 17
#define LTC3350_IIN_OC_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_IIN_OC_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_IIN_OC_LVL IIN_OC_LVL
 *  @ingroup LTC3350_register_map
 *  @brief IIN_OC_LVL Bit Field
 *
 *  Input Overcurrent Level: This is an alarm threshold for the input current. If enabled, the current rising above this level will trigger an alarm and an SMBALERT. 1.983uV/RSNSI per LSB
 *   - Register: @ref LTC3350_IIN_OC_LVL_CMD "IIN_OC_LVL_CMD"
 *   - CommandCode: 17
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_IIN_OC_LVL_SUBADDR LTC3350_IIN_OC_LVL_CMD_SUBADDR //!< @ref LTC3350_IIN_OC_LVL "IIN_OC_LVL"
#define LTC3350_IIN_OC_LVL_SIZE 16
#define LTC3350_IIN_OC_LVL_OFFSET 0
#define LTC3350_IIN_OC_LVL_MASK 0xFFFF
#define LTC3350_IIN_OC_LVL (LTC3350_IIN_OC_LVL_OFFSET << 12 | (LTC3350_IIN_OC_LVL_SIZE - 1) << 8 | LTC3350_IIN_OC_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_ICHG_UC_LVL_CMD ICHG_UC_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief ICHG_UC_LVL_CMD Register
 *
 * |        15:0 |
 * |:-----------:|
 * | ICHG_UC_LVL |
 *
 *   - CommandCode: 18
 *   - Contains Bit Fields:
 *     + @ref LTC3350_ICHG_UC_LVL "ICHG_UC_LVL" : Charge Undercurrent Level: This is an alarm threshold for the charge current. If enabled, the current falling below this level will trigger an alarm and an SMBALERT. 1.983uV/RSNSC per LSB
*/

//!@{
#define LTC3350_ICHG_UC_LVL_CMD_SUBADDR 18
#define LTC3350_ICHG_UC_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_ICHG_UC_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_ICHG_UC_LVL ICHG_UC_LVL
 *  @ingroup LTC3350_register_map
 *  @brief ICHG_UC_LVL Bit Field
 *
 *  Charge Undercurrent Level: This is an alarm threshold for the charge current. If enabled, the current falling below this level will trigger an alarm and an SMBALERT. 1.983uV/RSNSC per LSB
 *   - Register: @ref LTC3350_ICHG_UC_LVL_CMD "ICHG_UC_LVL_CMD"
 *   - CommandCode: 18
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_ICHG_UC_LVL_SUBADDR LTC3350_ICHG_UC_LVL_CMD_SUBADDR //!< @ref LTC3350_ICHG_UC_LVL "ICHG_UC_LVL"
#define LTC3350_ICHG_UC_LVL_SIZE 16
#define LTC3350_ICHG_UC_LVL_OFFSET 0
#define LTC3350_ICHG_UC_LVL_MASK 0xFFFF
#define LTC3350_ICHG_UC_LVL (LTC3350_ICHG_UC_LVL_OFFSET << 12 | (LTC3350_ICHG_UC_LVL_SIZE - 1) << 8 | LTC3350_ICHG_UC_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_DTEMP_COLD_LVL_CMD DTEMP_COLD_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief DTEMP_COLD_LVL_CMD Register
 *
 * |           15:0 |
 * |:--------------:|
 * | DTEMP_COLD_LVL |
 *
 *   - CommandCode: 19
 *   - Contains Bit Fields:
 *     + @ref LTC3350_DTEMP_COLD_LVL "DTEMP_COLD_LVL" : Die Temperature Cold Level: This is an alarm threshold for the die temperature. If enabled, the die temperature falling below this level will trigger an alarm and an SMBALERT. Temperature = 0.028C per LSB - 251.4C
*/

//!@{
#define LTC3350_DTEMP_COLD_LVL_CMD_SUBADDR 19
#define LTC3350_DTEMP_COLD_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_DTEMP_COLD_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_DTEMP_COLD_LVL DTEMP_COLD_LVL
 *  @ingroup LTC3350_register_map
 *  @brief DTEMP_COLD_LVL Bit Field
 *
 *  Die Temperature Cold Level: This is an alarm threshold for the die temperature. If enabled, the die temperature falling below this level will trigger an alarm and an SMBALERT. Temperature = 0.028C per LSB - 251.4C
 *   - Register: @ref LTC3350_DTEMP_COLD_LVL_CMD "DTEMP_COLD_LVL_CMD"
 *   - CommandCode: 19
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_DTEMP_COLD_LVL_SUBADDR LTC3350_DTEMP_COLD_LVL_CMD_SUBADDR //!< @ref LTC3350_DTEMP_COLD_LVL "DTEMP_COLD_LVL"
#define LTC3350_DTEMP_COLD_LVL_SIZE 16
#define LTC3350_DTEMP_COLD_LVL_OFFSET 0
#define LTC3350_DTEMP_COLD_LVL_MASK 0xFFFF
#define LTC3350_DTEMP_COLD_LVL (LTC3350_DTEMP_COLD_LVL_OFFSET << 12 | (LTC3350_DTEMP_COLD_LVL_SIZE - 1) << 8 | LTC3350_DTEMP_COLD_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_DTEMP_HOT_LVL_CMD DTEMP_HOT_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief DTEMP_HOT_LVL_CMD Register
 *
 * |          15:0 |
 * |:-------------:|
 * | DTEMP_HOT_LVL |
 *
 *   - CommandCode: 20
 *   - Contains Bit Fields:
 *     + @ref LTC3350_DTEMP_HOT_LVL "DTEMP_HOT_LVL" : Die Temperature Hot Level: This is an alarm threshold for the die temperature. If enabled, the die temperature rising above this level will trigger an alarm and an SMBALERT. Temperature = 0.028C per LSB - 251.4C
*/

//!@{
#define LTC3350_DTEMP_HOT_LVL_CMD_SUBADDR 20
#define LTC3350_DTEMP_HOT_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_DTEMP_HOT_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_DTEMP_HOT_LVL DTEMP_HOT_LVL
 *  @ingroup LTC3350_register_map
 *  @brief DTEMP_HOT_LVL Bit Field
 *
 *  Die Temperature Hot Level: This is an alarm threshold for the die temperature. If enabled, the die temperature rising above this level will trigger an alarm and an SMBALERT. Temperature = 0.028C per LSB - 251.4C
 *   - Register: @ref LTC3350_DTEMP_HOT_LVL_CMD "DTEMP_HOT_LVL_CMD"
 *   - CommandCode: 20
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_DTEMP_HOT_LVL_SUBADDR LTC3350_DTEMP_HOT_LVL_CMD_SUBADDR //!< @ref LTC3350_DTEMP_HOT_LVL "DTEMP_HOT_LVL"
#define LTC3350_DTEMP_HOT_LVL_SIZE 16
#define LTC3350_DTEMP_HOT_LVL_OFFSET 0
#define LTC3350_DTEMP_HOT_LVL_MASK 0xFFFF
#define LTC3350_DTEMP_HOT_LVL (LTC3350_DTEMP_HOT_LVL_OFFSET << 12 | (LTC3350_DTEMP_HOT_LVL_SIZE - 1) << 8 | LTC3350_DTEMP_HOT_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_ESR_HI_LVL_CMD ESR_HI_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief ESR_HI_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | ESR_HI_LVL |
 *
 *   - CommandCode: 21
 *   - Contains Bit Fields:
 *     + @ref LTC3350_ESR_HI_LVL "ESR_HI_LVL" : ESR High Level: This is an alarm threshold for the measured stack ESR. If enabled, a measurement of stack ESR exceeding this level will trigger an alarm and an SMBALERT. RSNSC/64 per LSB.
*/

//!@{
#define LTC3350_ESR_HI_LVL_CMD_SUBADDR 21
#define LTC3350_ESR_HI_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_ESR_HI_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_ESR_HI_LVL ESR_HI_LVL
 *  @ingroup LTC3350_register_map
 *  @brief ESR_HI_LVL Bit Field
 *
 *  ESR High Level: This is an alarm threshold for the measured stack ESR. If enabled, a measurement of stack ESR exceeding this level will trigger an alarm and an SMBALERT. RSNSC/64 per LSB.
 *   - Register: @ref LTC3350_ESR_HI_LVL_CMD "ESR_HI_LVL_CMD"
 *   - CommandCode: 21
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_ESR_HI_LVL_SUBADDR LTC3350_ESR_HI_LVL_CMD_SUBADDR //!< @ref LTC3350_ESR_HI_LVL "ESR_HI_LVL"
#define LTC3350_ESR_HI_LVL_SIZE 16
#define LTC3350_ESR_HI_LVL_OFFSET 0
#define LTC3350_ESR_HI_LVL_MASK 0xFFFF
#define LTC3350_ESR_HI_LVL (LTC3350_ESR_HI_LVL_OFFSET << 12 | (LTC3350_ESR_HI_LVL_SIZE - 1) << 8 | LTC3350_ESR_HI_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_CAP_LO_LVL_CMD CAP_LO_LVL_CMD
 *  @ingroup LTC3350_register_map
 *  @brief CAP_LO_LVL_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | CAP_LO_LVL |
 *
 *   - CommandCode: 22
 *   - Contains Bit Fields:
 *     + @ref LTC3350_CAP_LO_LVL "CAP_LO_LVL" : Capacitance Low Level: This is an alarm threshold for the measured stack capacitance. If enabled, if the measured stack capacitance is less than this level it will trigger an alarm and an SMBALERT. When ctl_cap_scale is set to 1, capacitance is 3.36uF * RT/RTST per LSB. When ctl_cap_scale is set to 0 it is 336uF * RT/RTST per LSB.
*/

//!@{
#define LTC3350_CAP_LO_LVL_CMD_SUBADDR 22
#define LTC3350_CAP_LO_LVL_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_CAP_LO_LVL_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_CAP_LO_LVL CAP_LO_LVL
 *  @ingroup LTC3350_register_map
 *  @brief CAP_LO_LVL Bit Field
 *
 *  Capacitance Low Level: This is an alarm threshold for the measured stack capacitance. If enabled, if the measured stack capacitance is less than this level it will trigger an alarm and an SMBALERT. When ctl_cap_scale is set to 1, capacitance is 3.36uF * RT/RTST per LSB. When ctl_cap_scale is set to 0 it is 336uF * RT/RTST per LSB.
 *   - Register: @ref LTC3350_CAP_LO_LVL_CMD "CAP_LO_LVL_CMD"
 *   - CommandCode: 22
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CAP_LO_LVL_SUBADDR LTC3350_CAP_LO_LVL_CMD_SUBADDR //!< @ref LTC3350_CAP_LO_LVL "CAP_LO_LVL"
#define LTC3350_CAP_LO_LVL_SIZE 16
#define LTC3350_CAP_LO_LVL_OFFSET 0
#define LTC3350_CAP_LO_LVL_MASK 0xFFFF
#define LTC3350_CAP_LO_LVL (LTC3350_CAP_LO_LVL_OFFSET << 12 | (LTC3350_CAP_LO_LVL_SIZE - 1) << 8 | LTC3350_CAP_LO_LVL_SUBADDR)
//!@}

/*! @defgroup LTC3350_CTL_REG CTL_REG
 *  @ingroup LTC3350_register_map
 *  @brief CTL_REG Register
 *
 * | 15:4 |             3 |               2 |                 1 |               0 |
 * |:----:|:-------------:|:---------------:|:-----------------:|:---------------:|
 * |  n/a | CTL_CAP_SCALE | CTL_STOP_CAPESR | CTL_GPI_BUFFER_EN | CTL_STRT_CAPESR |
 *
 * Control Register: Several Control Functions are grouped into this register.
 *   - CommandCode: 23
 *   - Contains Bit Fields:
 *     + @ref LTC3350_CTL_STRT_CAPESR "CTL_STRT_CAPESR" : Begin a capacitance and ESR measurement when possible; this bit clears itself once a cycle begins.
 *     + @ref LTC3350_CTL_GPI_BUFFER_EN "CTL_GPI_BUFFER_EN" : A one in this bit location enables the input buffer on the GPI pin. With a zero in this location the GPI pin is measured without the buffer.
 *     + @ref LTC3350_CTL_STOP_CAPESR "CTL_STOP_CAPESR" : Stops an active capacitance/ESR measurement.
 *     + @ref LTC3350_CTL_CAP_SCALE "CTL_CAP_SCALE" : Increases capacitor measurement resolution by 100x, this is used when measuring smaller capacitors.
*/

//!@{
#define LTC3350_CTL_REG_SUBADDR 23
#define LTC3350_CTL_REG (0 << 12 | (16 - 1) << 8 | LTC3350_CTL_REG_SUBADDR)
//!@}
/*! @defgroup LTC3350_CTL_STRT_CAPESR CTL_STRT_CAPESR
 *  @ingroup LTC3350_register_map
 *  @brief CTL_STRT_CAPESR Bit Field
 *
 *  Begin a capacitance and ESR measurement when possible; this bit clears itself once a cycle begins.
 *   - Register: @ref LTC3350_CTL_REG "CTL_REG"
 *   - CommandCode: 23
 *   - Size: 1
 *   - Offset: 0
 *   - MSB: 0
 *   - MASK: 0x0001
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CTL_STRT_CAPESR_SUBADDR LTC3350_CTL_REG_SUBADDR //!< @ref LTC3350_CTL_STRT_CAPESR "CTL_STRT_CAPESR"
#define LTC3350_CTL_STRT_CAPESR_SIZE 1
#define LTC3350_CTL_STRT_CAPESR_OFFSET 0
#define LTC3350_CTL_STRT_CAPESR_MASK 0x0001
#define LTC3350_CTL_STRT_CAPESR (LTC3350_CTL_STRT_CAPESR_OFFSET << 12 | (LTC3350_CTL_STRT_CAPESR_SIZE - 1) << 8 | LTC3350_CTL_STRT_CAPESR_SUBADDR)
#define LTC3350_CTL_STRT_CAPESR_PRESET_START 1
//!@}
/*! @defgroup LTC3350_CTL_GPI_BUFFER_EN CTL_GPI_BUFFER_EN
 *  @ingroup LTC3350_register_map
 *  @brief CTL_GPI_BUFFER_EN Bit Field
 *
 *  A one in this bit location enables the input buffer on the GPI pin. With a zero in this location the GPI pin is measured without the buffer.
 *   - Register: @ref LTC3350_CTL_REG "CTL_REG"
 *   - CommandCode: 23
 *   - Size: 1
 *   - Offset: 1
 *   - MSB: 1
 *   - MASK: 0x0002
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CTL_GPI_BUFFER_EN_SUBADDR LTC3350_CTL_REG_SUBADDR //!< @ref LTC3350_CTL_GPI_BUFFER_EN "CTL_GPI_BUFFER_EN"
#define LTC3350_CTL_GPI_BUFFER_EN_SIZE 1
#define LTC3350_CTL_GPI_BUFFER_EN_OFFSET 1
#define LTC3350_CTL_GPI_BUFFER_EN_MASK 0x0002
#define LTC3350_CTL_GPI_BUFFER_EN (LTC3350_CTL_GPI_BUFFER_EN_OFFSET << 12 | (LTC3350_CTL_GPI_BUFFER_EN_SIZE - 1) << 8 | LTC3350_CTL_GPI_BUFFER_EN_SUBADDR)
//!@}
/*! @defgroup LTC3350_CTL_STOP_CAPESR CTL_STOP_CAPESR
 *  @ingroup LTC3350_register_map
 *  @brief CTL_STOP_CAPESR Bit Field
 *
 *  Stops an active capacitance/ESR measurement.
 *   - Register: @ref LTC3350_CTL_REG "CTL_REG"
 *   - CommandCode: 23
 *   - Size: 1
 *   - Offset: 2
 *   - MSB: 2
 *   - MASK: 0x0004
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CTL_STOP_CAPESR_SUBADDR LTC3350_CTL_REG_SUBADDR //!< @ref LTC3350_CTL_STOP_CAPESR "CTL_STOP_CAPESR"
#define LTC3350_CTL_STOP_CAPESR_SIZE 1
#define LTC3350_CTL_STOP_CAPESR_OFFSET 2
#define LTC3350_CTL_STOP_CAPESR_MASK 0x0004
#define LTC3350_CTL_STOP_CAPESR (LTC3350_CTL_STOP_CAPESR_OFFSET << 12 | (LTC3350_CTL_STOP_CAPESR_SIZE - 1) << 8 | LTC3350_CTL_STOP_CAPESR_SUBADDR)
#define LTC3350_CTL_STOP_CAPESR_PRESET_STOP 1
//!@}
/*! @defgroup LTC3350_CTL_CAP_SCALE CTL_CAP_SCALE
 *  @ingroup LTC3350_register_map
 *  @brief CTL_CAP_SCALE Bit Field
 *
 *  Increases capacitor measurement resolution by 100x, this is used when measuring smaller capacitors.
 *   - Register: @ref LTC3350_CTL_REG "CTL_REG"
 *   - CommandCode: 23
 *   - Size: 1
 *   - Offset: 3
 *   - MSB: 3
 *   - MASK: 0x0008
 *   - Access: R/W
 *   - Default: 0
 */
//!@{
#define LTC3350_CTL_CAP_SCALE_SUBADDR LTC3350_CTL_REG_SUBADDR //!< @ref LTC3350_CTL_CAP_SCALE "CTL_CAP_SCALE"
#define LTC3350_CTL_CAP_SCALE_SIZE 1
#define LTC3350_CTL_CAP_SCALE_OFFSET 3
#define LTC3350_CTL_CAP_SCALE_MASK 0x0008
#define LTC3350_CTL_CAP_SCALE (LTC3350_CTL_CAP_SCALE_OFFSET << 12 | (LTC3350_CTL_CAP_SCALE_SIZE - 1) << 8 | LTC3350_CTL_CAP_SCALE_SUBADDR)
#define LTC3350_CTL_CAP_SCALE_PRESET_SMALL 1
#define LTC3350_CTL_CAP_SCALE_PRESET_LARGE 0
//!@}

/*! @defgroup LTC3350_NUM_CAPS_CMD NUM_CAPS_CMD
 *  @ingroup LTC3350_register_map
 *  @brief NUM_CAPS_CMD Register
 *
 * | 15:2 |      1:0 |
 * |:----:|:--------:|
 * |  n/a | NUM_CAPS |
 *
 *   - CommandCode: 26
 *   - Contains Bit Fields:
 *     + @ref LTC3350_NUM_CAPS "NUM_CAPS" : Number of Capacitors. This register shows the state of the CAP_SLCT1, CAP_SLCT0 pins. The value read in this register is the number of capacitors programmed minus one.
*/

//!@{
#define LTC3350_NUM_CAPS_CMD_SUBADDR 26
#define LTC3350_NUM_CAPS_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_NUM_CAPS_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_NUM_CAPS NUM_CAPS
 *  @ingroup LTC3350_register_map
 *  @brief NUM_CAPS Bit Field
 *
 *  Number of Capacitors. This register shows the state of the CAP_SLCT1, CAP_SLCT0 pins. The value read in this register is the number of capacitors programmed minus one.
 *   - Register: @ref LTC3350_NUM_CAPS_CMD "NUM_CAPS_CMD"
 *   - CommandCode: 26
 *   - Size: 2
 *   - Offset: 0
 *   - MSB: 1
 *   - MASK: 0x0003
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_NUM_CAPS_SUBADDR LTC3350_NUM_CAPS_CMD_SUBADDR //!< @ref LTC3350_NUM_CAPS "NUM_CAPS"
#define LTC3350_NUM_CAPS_SIZE 2
#define LTC3350_NUM_CAPS_OFFSET 0
#define LTC3350_NUM_CAPS_MASK 0x0003
#define LTC3350_NUM_CAPS (LTC3350_NUM_CAPS_OFFSET << 12 | (LTC3350_NUM_CAPS_SIZE - 1) << 8 | LTC3350_NUM_CAPS_SUBADDR)
//!@}

/*! @defgroup LTC3350_CHRG_STATUS CHRG_STATUS
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_STATUS Register
 *
 * | 15:12 |       11 |  10 |       9 |        8 |        7 |         6 |          5 |               4 |         3 |       2 |           1 |             0 |
 * |:-----:|:--------:|:---:|:-------:|:--------:|:--------:|:---------:|:----------:|:---------------:|:---------:|:-------:|:-----------:|:-------------:|
 * |   n/a | CHRG_PFO | n/a | CHRG_CI | CHRG_DIS | CHRG_BAL | CHRG_SHNT | CHRG_CAPPG | CHRG_INPUT_ILIM | CHRG_UVLO | CHRG_CV | CHRG_STEPUP | CHRG_STEPDOWN |
 *
 * Charger Status Register: This register provides real time status information about the state of the charger system. Each bit is active high.
 *   - CommandCode: 27
 *   - Contains Bit Fields:
 *     + @ref LTC3350_CHRG_STEPDOWN "CHRG_STEPDOWN" : The synchronous controller is in step-down mode (charging)
 *     + @ref LTC3350_CHRG_STEPUP "CHRG_STEPUP" : The synchronous controller is in step-up mode (backup)
 *     + @ref LTC3350_CHRG_CV "CHRG_CV" : The charger is in constant voltage mode
 *     + @ref LTC3350_CHRG_UVLO "CHRG_UVLO" : The charger is in under voltage lockout
 *     + @ref LTC3350_CHRG_INPUT_ILIM "CHRG_INPUT_ILIM" : The charger is in input current limit
 *     + @ref LTC3350_CHRG_CAPPG "CHRG_CAPPG" : The capacitor voltage is above power good threshold
 *     + @ref LTC3350_CHRG_SHNT "CHRG_SHNT" : The capacitor manager is shunting
 *     + @ref LTC3350_CHRG_BAL "CHRG_BAL" : The capacitor manager is balancing
 *     + @ref LTC3350_CHRG_DIS "CHRG_DIS" : The charger is temporarily disabled for capacitance measurement
 *     + @ref LTC3350_CHRG_CI "CHRG_CI" : The charger is in constant current mode
 *     + @ref LTC3350_CHRG_PFO "CHRG_PFO" : Input voltage is below PFI threshold
*/

//!@{
#define LTC3350_CHRG_STATUS_SUBADDR 27
#define LTC3350_CHRG_STATUS (0 << 12 | (16 - 1) << 8 | LTC3350_CHRG_STATUS_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_STEPDOWN CHRG_STEPDOWN
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_STEPDOWN Bit Field
 *
 *  The synchronous controller is in step-down mode (charging)
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 0
 *   - MSB: 0
 *   - MASK: 0x0001
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_STEPDOWN_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_STEPDOWN "CHRG_STEPDOWN"
#define LTC3350_CHRG_STEPDOWN_SIZE 1
#define LTC3350_CHRG_STEPDOWN_OFFSET 0
#define LTC3350_CHRG_STEPDOWN_MASK 0x0001
#define LTC3350_CHRG_STEPDOWN (LTC3350_CHRG_STEPDOWN_OFFSET << 12 | (LTC3350_CHRG_STEPDOWN_SIZE - 1) << 8 | LTC3350_CHRG_STEPDOWN_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_STEPUP CHRG_STEPUP
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_STEPUP Bit Field
 *
 *  The synchronous controller is in step-up mode (backup)
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 1
 *   - MSB: 1
 *   - MASK: 0x0002
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_STEPUP_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_STEPUP "CHRG_STEPUP"
#define LTC3350_CHRG_STEPUP_SIZE 1
#define LTC3350_CHRG_STEPUP_OFFSET 1
#define LTC3350_CHRG_STEPUP_MASK 0x0002
#define LTC3350_CHRG_STEPUP (LTC3350_CHRG_STEPUP_OFFSET << 12 | (LTC3350_CHRG_STEPUP_SIZE - 1) << 8 | LTC3350_CHRG_STEPUP_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_CV CHRG_CV
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_CV Bit Field
 *
 *  The charger is in constant voltage mode
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 2
 *   - MSB: 2
 *   - MASK: 0x0004
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_CV_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_CV "CHRG_CV"
#define LTC3350_CHRG_CV_SIZE 1
#define LTC3350_CHRG_CV_OFFSET 2
#define LTC3350_CHRG_CV_MASK 0x0004
#define LTC3350_CHRG_CV (LTC3350_CHRG_CV_OFFSET << 12 | (LTC3350_CHRG_CV_SIZE - 1) << 8 | LTC3350_CHRG_CV_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_UVLO CHRG_UVLO
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_UVLO Bit Field
 *
 *  The charger is in under voltage lockout
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 3
 *   - MSB: 3
 *   - MASK: 0x0008
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_UVLO_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_UVLO "CHRG_UVLO"
#define LTC3350_CHRG_UVLO_SIZE 1
#define LTC3350_CHRG_UVLO_OFFSET 3
#define LTC3350_CHRG_UVLO_MASK 0x0008
#define LTC3350_CHRG_UVLO (LTC3350_CHRG_UVLO_OFFSET << 12 | (LTC3350_CHRG_UVLO_SIZE - 1) << 8 | LTC3350_CHRG_UVLO_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_INPUT_ILIM CHRG_INPUT_ILIM
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_INPUT_ILIM Bit Field
 *
 *  The charger is in input current limit
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 4
 *   - MSB: 4
 *   - MASK: 0x0010
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_INPUT_ILIM_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_INPUT_ILIM "CHRG_INPUT_ILIM"
#define LTC3350_CHRG_INPUT_ILIM_SIZE 1
#define LTC3350_CHRG_INPUT_ILIM_OFFSET 4
#define LTC3350_CHRG_INPUT_ILIM_MASK 0x0010
#define LTC3350_CHRG_INPUT_ILIM (LTC3350_CHRG_INPUT_ILIM_OFFSET << 12 | (LTC3350_CHRG_INPUT_ILIM_SIZE - 1) << 8 | LTC3350_CHRG_INPUT_ILIM_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_CAPPG CHRG_CAPPG
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_CAPPG Bit Field
 *
 *  The capacitor voltage is above power good threshold
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 5
 *   - MSB: 5
 *   - MASK: 0x0020
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_CAPPG_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_CAPPG "CHRG_CAPPG"
#define LTC3350_CHRG_CAPPG_SIZE 1
#define LTC3350_CHRG_CAPPG_OFFSET 5
#define LTC3350_CHRG_CAPPG_MASK 0x0020
#define LTC3350_CHRG_CAPPG (LTC3350_CHRG_CAPPG_OFFSET << 12 | (LTC3350_CHRG_CAPPG_SIZE - 1) << 8 | LTC3350_CHRG_CAPPG_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_SHNT CHRG_SHNT
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_SHNT Bit Field
 *
 *  The capacitor manager is shunting
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 6
 *   - MSB: 6
 *   - MASK: 0x0040
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_SHNT_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_SHNT "CHRG_SHNT"
#define LTC3350_CHRG_SHNT_SIZE 1
#define LTC3350_CHRG_SHNT_OFFSET 6
#define LTC3350_CHRG_SHNT_MASK 0x0040
#define LTC3350_CHRG_SHNT (LTC3350_CHRG_SHNT_OFFSET << 12 | (LTC3350_CHRG_SHNT_SIZE - 1) << 8 | LTC3350_CHRG_SHNT_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_BAL CHRG_BAL
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_BAL Bit Field
 *
 *  The capacitor manager is balancing
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 7
 *   - MSB: 7
 *   - MASK: 0x0080
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_BAL_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_BAL "CHRG_BAL"
#define LTC3350_CHRG_BAL_SIZE 1
#define LTC3350_CHRG_BAL_OFFSET 7
#define LTC3350_CHRG_BAL_MASK 0x0080
#define LTC3350_CHRG_BAL (LTC3350_CHRG_BAL_OFFSET << 12 | (LTC3350_CHRG_BAL_SIZE - 1) << 8 | LTC3350_CHRG_BAL_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_DIS CHRG_DIS
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_DIS Bit Field
 *
 *  The charger is temporarily disabled for capacitance measurement
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 8
 *   - MSB: 8
 *   - MASK: 0x0100
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_DIS_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_DIS "CHRG_DIS"
#define LTC3350_CHRG_DIS_SIZE 1
#define LTC3350_CHRG_DIS_OFFSET 8
#define LTC3350_CHRG_DIS_MASK 0x0100
#define LTC3350_CHRG_DIS (LTC3350_CHRG_DIS_OFFSET << 12 | (LTC3350_CHRG_DIS_SIZE - 1) << 8 | LTC3350_CHRG_DIS_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_CI CHRG_CI
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_CI Bit Field
 *
 *  The charger is in constant current mode
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 9
 *   - MSB: 9
 *   - MASK: 0x0200
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_CI_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_CI "CHRG_CI"
#define LTC3350_CHRG_CI_SIZE 1
#define LTC3350_CHRG_CI_OFFSET 9
#define LTC3350_CHRG_CI_MASK 0x0200
#define LTC3350_CHRG_CI (LTC3350_CHRG_CI_OFFSET << 12 | (LTC3350_CHRG_CI_SIZE - 1) << 8 | LTC3350_CHRG_CI_SUBADDR)
//!@}
/*! @defgroup LTC3350_CHRG_PFO CHRG_PFO
 *  @ingroup LTC3350_register_map
 *  @brief CHRG_PFO Bit Field
 *
 *  Input voltage is below PFI threshold
 *   - Register: @ref LTC3350_CHRG_STATUS "CHRG_STATUS"
 *   - CommandCode: 27
 *   - Size: 1
 *   - Offset: 11
 *   - MSB: 11
 *   - MASK: 0x0800
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_CHRG_PFO_SUBADDR LTC3350_CHRG_STATUS_SUBADDR //!< @ref LTC3350_CHRG_PFO "CHRG_PFO"
#define LTC3350_CHRG_PFO_SIZE 1
#define LTC3350_CHRG_PFO_OFFSET 11
#define LTC3350_CHRG_PFO_MASK 0x0800
#define LTC3350_CHRG_PFO (LTC3350_CHRG_PFO_OFFSET << 12 | (LTC3350_CHRG_PFO_SIZE - 1) << 8 | LTC3350_CHRG_PFO_SUBADDR)
//!@}

/*! @defgroup LTC3350_MON_STATUS MON_STATUS
 *  @ingroup LTC3350_register_map
 *  @brief MON_STATUS Register
 *
 * | 15:10 |                  9 |                8 |   7 |              6 |              5 |            4 |            3 |                  2 |                    1 |                 0 |
 * |:-----:|:------------------:|:----------------:|:---:|:--------------:|:--------------:|:------------:|:------------:|:------------------:|:--------------------:|:-----------------:|
 * |   n/a | MON_POWER_RETURNED | MON_POWER_FAILED | n/a | MON_ESR_FAILED | MON_CAP_FAILED | MON_ESR_DONE | MON_CAP_DONE | MON_CAPESR_PENDING | MON_CAPESR_SCHEDULED | MON_CAPESR_ACTIVE |
 *
 * Monitor Status: This register provides real time status information about the state of the monitoring system. Each bit is active high.
 *   - CommandCode: 28
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MON_CAPESR_ACTIVE "MON_CAPESR_ACTIVE" : Capacitance/ESR measurement is in progress
 *     + @ref LTC3350_MON_CAPESR_SCHEDULED "MON_CAPESR_SCHEDULED" : Waiting programmed time to begin a capacitance/ESR measurement
 *     + @ref LTC3350_MON_CAPESR_PENDING "MON_CAPESR_PENDING" : Waiting for satisfactory conditions to begin a capacitance/ESR measurement
 *     + @ref LTC3350_MON_CAP_DONE "MON_CAP_DONE" : Capacitance measurement has completed
 *     + @ref LTC3350_MON_ESR_DONE "MON_ESR_DONE" : ESR Measurement has completed
 *     + @ref LTC3350_MON_CAP_FAILED "MON_CAP_FAILED" : The last attempted capacitance measurement was unable to complete
 *     + @ref LTC3350_MON_ESR_FAILED "MON_ESR_FAILED" : The last attempted ESR measurement was unable to complete
 *     + @ref LTC3350_MON_POWER_FAILED "MON_POWER_FAILED" : This bit is set when VIN falls below the PFI threshold or the charger is unable to charge. It is cleared only when power returns and the charger is able to charge.
 *     + @ref LTC3350_MON_POWER_RETURNED "MON_POWER_RETURNED" : This bit is set when the input is above the PFI threshold and the charger is able to charge. It is cleared only when mon_power_failed is set.
*/

//!@{
#define LTC3350_MON_STATUS_SUBADDR 28
#define LTC3350_MON_STATUS (0 << 12 | (16 - 1) << 8 | LTC3350_MON_STATUS_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_CAPESR_ACTIVE MON_CAPESR_ACTIVE
 *  @ingroup LTC3350_register_map
 *  @brief MON_CAPESR_ACTIVE Bit Field
 *
 *  Capacitance/ESR measurement is in progress
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 0
 *   - MSB: 0
 *   - MASK: 0x0001
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_CAPESR_ACTIVE_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_CAPESR_ACTIVE "MON_CAPESR_ACTIVE"
#define LTC3350_MON_CAPESR_ACTIVE_SIZE 1
#define LTC3350_MON_CAPESR_ACTIVE_OFFSET 0
#define LTC3350_MON_CAPESR_ACTIVE_MASK 0x0001
#define LTC3350_MON_CAPESR_ACTIVE (LTC3350_MON_CAPESR_ACTIVE_OFFSET << 12 | (LTC3350_MON_CAPESR_ACTIVE_SIZE - 1) << 8 | LTC3350_MON_CAPESR_ACTIVE_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_CAPESR_SCHEDULED MON_CAPESR_SCHEDULED
 *  @ingroup LTC3350_register_map
 *  @brief MON_CAPESR_SCHEDULED Bit Field
 *
 *  Waiting programmed time to begin a capacitance/ESR measurement
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 1
 *   - MSB: 1
 *   - MASK: 0x0002
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_CAPESR_SCHEDULED_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_CAPESR_SCHEDULED "MON_CAPESR_SCHEDULED"
#define LTC3350_MON_CAPESR_SCHEDULED_SIZE 1
#define LTC3350_MON_CAPESR_SCHEDULED_OFFSET 1
#define LTC3350_MON_CAPESR_SCHEDULED_MASK 0x0002
#define LTC3350_MON_CAPESR_SCHEDULED (LTC3350_MON_CAPESR_SCHEDULED_OFFSET << 12 | (LTC3350_MON_CAPESR_SCHEDULED_SIZE - 1) << 8 | LTC3350_MON_CAPESR_SCHEDULED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_CAPESR_PENDING MON_CAPESR_PENDING
 *  @ingroup LTC3350_register_map
 *  @brief MON_CAPESR_PENDING Bit Field
 *
 *  Waiting for satisfactory conditions to begin a capacitance/ESR measurement
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 2
 *   - MSB: 2
 *   - MASK: 0x0004
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_CAPESR_PENDING_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_CAPESR_PENDING "MON_CAPESR_PENDING"
#define LTC3350_MON_CAPESR_PENDING_SIZE 1
#define LTC3350_MON_CAPESR_PENDING_OFFSET 2
#define LTC3350_MON_CAPESR_PENDING_MASK 0x0004
#define LTC3350_MON_CAPESR_PENDING (LTC3350_MON_CAPESR_PENDING_OFFSET << 12 | (LTC3350_MON_CAPESR_PENDING_SIZE - 1) << 8 | LTC3350_MON_CAPESR_PENDING_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_CAP_DONE MON_CAP_DONE
 *  @ingroup LTC3350_register_map
 *  @brief MON_CAP_DONE Bit Field
 *
 *  Capacitance measurement has completed
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 3
 *   - MSB: 3
 *   - MASK: 0x0008
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_CAP_DONE_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_CAP_DONE "MON_CAP_DONE"
#define LTC3350_MON_CAP_DONE_SIZE 1
#define LTC3350_MON_CAP_DONE_OFFSET 3
#define LTC3350_MON_CAP_DONE_MASK 0x0008
#define LTC3350_MON_CAP_DONE (LTC3350_MON_CAP_DONE_OFFSET << 12 | (LTC3350_MON_CAP_DONE_SIZE - 1) << 8 | LTC3350_MON_CAP_DONE_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_ESR_DONE MON_ESR_DONE
 *  @ingroup LTC3350_register_map
 *  @brief MON_ESR_DONE Bit Field
 *
 *  ESR Measurement has completed
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 4
 *   - MSB: 4
 *   - MASK: 0x0010
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_ESR_DONE_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_ESR_DONE "MON_ESR_DONE"
#define LTC3350_MON_ESR_DONE_SIZE 1
#define LTC3350_MON_ESR_DONE_OFFSET 4
#define LTC3350_MON_ESR_DONE_MASK 0x0010
#define LTC3350_MON_ESR_DONE (LTC3350_MON_ESR_DONE_OFFSET << 12 | (LTC3350_MON_ESR_DONE_SIZE - 1) << 8 | LTC3350_MON_ESR_DONE_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_CAP_FAILED MON_CAP_FAILED
 *  @ingroup LTC3350_register_map
 *  @brief MON_CAP_FAILED Bit Field
 *
 *  The last attempted capacitance measurement was unable to complete
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 5
 *   - MSB: 5
 *   - MASK: 0x0020
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_CAP_FAILED_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_CAP_FAILED "MON_CAP_FAILED"
#define LTC3350_MON_CAP_FAILED_SIZE 1
#define LTC3350_MON_CAP_FAILED_OFFSET 5
#define LTC3350_MON_CAP_FAILED_MASK 0x0020
#define LTC3350_MON_CAP_FAILED (LTC3350_MON_CAP_FAILED_OFFSET << 12 | (LTC3350_MON_CAP_FAILED_SIZE - 1) << 8 | LTC3350_MON_CAP_FAILED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_ESR_FAILED MON_ESR_FAILED
 *  @ingroup LTC3350_register_map
 *  @brief MON_ESR_FAILED Bit Field
 *
 *  The last attempted ESR measurement was unable to complete
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 6
 *   - MSB: 6
 *   - MASK: 0x0040
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_ESR_FAILED_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_ESR_FAILED "MON_ESR_FAILED"
#define LTC3350_MON_ESR_FAILED_SIZE 1
#define LTC3350_MON_ESR_FAILED_OFFSET 6
#define LTC3350_MON_ESR_FAILED_MASK 0x0040
#define LTC3350_MON_ESR_FAILED (LTC3350_MON_ESR_FAILED_OFFSET << 12 | (LTC3350_MON_ESR_FAILED_SIZE - 1) << 8 | LTC3350_MON_ESR_FAILED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_POWER_FAILED MON_POWER_FAILED
 *  @ingroup LTC3350_register_map
 *  @brief MON_POWER_FAILED Bit Field
 *
 *  This bit is set when VIN falls below the PFI threshold or the charger is unable to charge. It is cleared only when power returns and the charger is able to charge.
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 8
 *   - MSB: 8
 *   - MASK: 0x0100
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_POWER_FAILED_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_POWER_FAILED "MON_POWER_FAILED"
#define LTC3350_MON_POWER_FAILED_SIZE 1
#define LTC3350_MON_POWER_FAILED_OFFSET 8
#define LTC3350_MON_POWER_FAILED_MASK 0x0100
#define LTC3350_MON_POWER_FAILED (LTC3350_MON_POWER_FAILED_OFFSET << 12 | (LTC3350_MON_POWER_FAILED_SIZE - 1) << 8 | LTC3350_MON_POWER_FAILED_SUBADDR)
//!@}
/*! @defgroup LTC3350_MON_POWER_RETURNED MON_POWER_RETURNED
 *  @ingroup LTC3350_register_map
 *  @brief MON_POWER_RETURNED Bit Field
 *
 *  This bit is set when the input is above the PFI threshold and the charger is able to charge. It is cleared only when mon_power_failed is set.
 *   - Register: @ref LTC3350_MON_STATUS "MON_STATUS"
 *   - CommandCode: 28
 *   - Size: 1
 *   - Offset: 9
 *   - MSB: 9
 *   - MASK: 0x0200
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MON_POWER_RETURNED_SUBADDR LTC3350_MON_STATUS_SUBADDR //!< @ref LTC3350_MON_POWER_RETURNED "MON_POWER_RETURNED"
#define LTC3350_MON_POWER_RETURNED_SIZE 1
#define LTC3350_MON_POWER_RETURNED_OFFSET 9
#define LTC3350_MON_POWER_RETURNED_MASK 0x0200
#define LTC3350_MON_POWER_RETURNED (LTC3350_MON_POWER_RETURNED_OFFSET << 12 | (LTC3350_MON_POWER_RETURNED_SIZE - 1) << 8 | LTC3350_MON_POWER_RETURNED_SUBADDR)
//!@}

/*! @defgroup LTC3350_ALARM_REG ALARM_REG
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_REG Register
 *
 * |           15 |           14 |              13 |               12 |            11 |           10 |             9 |             8 |             7 |             6 |            5 |            4 |            3 |            2 |            1 |            0 |
 * |:------------:|:------------:|:---------------:|:----------------:|:-------------:|:------------:|:-------------:|:-------------:|:-------------:|:-------------:|:------------:|:------------:|:------------:|:------------:|:------------:|:------------:|
 * | ALARM_CAP_LO | ALARM_ESR_HI | ALARM_DTEMP_HOT | ALARM_DTEMP_COLD | ALARM_ICHG_UC | ALARM_IIN_OC | ALARM_VOUT_OV | ALARM_VOUT_UV | ALARM_VCAP_OV | ALARM_VCAP_UV | ALARM_VIN_OV | ALARM_VIN_UV | ALARM_GPI_OV | ALARM_GPI_UV | ALARM_CAP_OV | ALARM_CAP_UV |
 *
 * Alarms Register: A one in any bit in the register indicates its respective alarm has triggered. All bits are active high.
 *   - CommandCode: 29
 *   - Contains Bit Fields:
 *     + @ref LTC3350_ALARM_CAP_UV "ALARM_CAP_UV" : Capacitor undervoltage alarm
 *     + @ref LTC3350_ALARM_CAP_OV "ALARM_CAP_OV" : Capacitor overvoltage alarm
 *     + @ref LTC3350_ALARM_GPI_UV "ALARM_GPI_UV" : GPI undervoltage alarm
 *     + @ref LTC3350_ALARM_GPI_OV "ALARM_GPI_OV" : GPI overvoltage alarm
 *     + @ref LTC3350_ALARM_VIN_UV "ALARM_VIN_UV" : VIN undervoltage alarm
 *     + @ref LTC3350_ALARM_VIN_OV "ALARM_VIN_OV" : VIN overvoltage alarm
 *     + @ref LTC3350_ALARM_VCAP_UV "ALARM_VCAP_UV" : VCAP undervoltage alarm
 *     + @ref LTC3350_ALARM_VCAP_OV "ALARM_VCAP_OV" : VCAP overvoltage alarm
 *     + @ref LTC3350_ALARM_VOUT_UV "ALARM_VOUT_UV" : VOUT undervoltage alarm
 *     + @ref LTC3350_ALARM_VOUT_OV "ALARM_VOUT_OV" : VOUT overvoltage alarm
 *     + @ref LTC3350_ALARM_IIN_OC "ALARM_IIN_OC" : Input overcurrent alarm
 *     + @ref LTC3350_ALARM_ICHG_UC "ALARM_ICHG_UC" : Charge undercurrent alarm
 *     + @ref LTC3350_ALARM_DTEMP_COLD "ALARM_DTEMP_COLD" : Die temperature cold alarm
 *     + @ref LTC3350_ALARM_DTEMP_HOT "ALARM_DTEMP_HOT" : Die temperature hot alarm
 *     + @ref LTC3350_ALARM_ESR_HI "ALARM_ESR_HI" : ESR high alarm
 *     + @ref LTC3350_ALARM_CAP_LO "ALARM_CAP_LO" : Capacitance low alarm
*/

//!@{
#define LTC3350_ALARM_REG_SUBADDR 29
#define LTC3350_ALARM_REG (0 << 12 | (16 - 1) << 8 | LTC3350_ALARM_REG_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_CAP_UV ALARM_CAP_UV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_CAP_UV Bit Field
 *
 *  Capacitor undervoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 0
 *   - MSB: 0
 *   - MASK: 0x0001
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_CAP_UV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_CAP_UV "ALARM_CAP_UV"
#define LTC3350_ALARM_CAP_UV_SIZE 1
#define LTC3350_ALARM_CAP_UV_OFFSET 0
#define LTC3350_ALARM_CAP_UV_MASK 0x0001
#define LTC3350_ALARM_CAP_UV (LTC3350_ALARM_CAP_UV_OFFSET << 12 | (LTC3350_ALARM_CAP_UV_SIZE - 1) << 8 | LTC3350_ALARM_CAP_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_CAP_OV ALARM_CAP_OV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_CAP_OV Bit Field
 *
 *  Capacitor overvoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 1
 *   - MSB: 1
 *   - MASK: 0x0002
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_CAP_OV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_CAP_OV "ALARM_CAP_OV"
#define LTC3350_ALARM_CAP_OV_SIZE 1
#define LTC3350_ALARM_CAP_OV_OFFSET 1
#define LTC3350_ALARM_CAP_OV_MASK 0x0002
#define LTC3350_ALARM_CAP_OV (LTC3350_ALARM_CAP_OV_OFFSET << 12 | (LTC3350_ALARM_CAP_OV_SIZE - 1) << 8 | LTC3350_ALARM_CAP_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_GPI_UV ALARM_GPI_UV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_GPI_UV Bit Field
 *
 *  GPI undervoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 2
 *   - MSB: 2
 *   - MASK: 0x0004
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_GPI_UV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_GPI_UV "ALARM_GPI_UV"
#define LTC3350_ALARM_GPI_UV_SIZE 1
#define LTC3350_ALARM_GPI_UV_OFFSET 2
#define LTC3350_ALARM_GPI_UV_MASK 0x0004
#define LTC3350_ALARM_GPI_UV (LTC3350_ALARM_GPI_UV_OFFSET << 12 | (LTC3350_ALARM_GPI_UV_SIZE - 1) << 8 | LTC3350_ALARM_GPI_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_GPI_OV ALARM_GPI_OV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_GPI_OV Bit Field
 *
 *  GPI overvoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 3
 *   - MSB: 3
 *   - MASK: 0x0008
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_GPI_OV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_GPI_OV "ALARM_GPI_OV"
#define LTC3350_ALARM_GPI_OV_SIZE 1
#define LTC3350_ALARM_GPI_OV_OFFSET 3
#define LTC3350_ALARM_GPI_OV_MASK 0x0008
#define LTC3350_ALARM_GPI_OV (LTC3350_ALARM_GPI_OV_OFFSET << 12 | (LTC3350_ALARM_GPI_OV_SIZE - 1) << 8 | LTC3350_ALARM_GPI_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_VIN_UV ALARM_VIN_UV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_VIN_UV Bit Field
 *
 *  VIN undervoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 4
 *   - MSB: 4
 *   - MASK: 0x0010
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_VIN_UV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_VIN_UV "ALARM_VIN_UV"
#define LTC3350_ALARM_VIN_UV_SIZE 1
#define LTC3350_ALARM_VIN_UV_OFFSET 4
#define LTC3350_ALARM_VIN_UV_MASK 0x0010
#define LTC3350_ALARM_VIN_UV (LTC3350_ALARM_VIN_UV_OFFSET << 12 | (LTC3350_ALARM_VIN_UV_SIZE - 1) << 8 | LTC3350_ALARM_VIN_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_VIN_OV ALARM_VIN_OV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_VIN_OV Bit Field
 *
 *  VIN overvoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 5
 *   - MSB: 5
 *   - MASK: 0x0020
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_VIN_OV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_VIN_OV "ALARM_VIN_OV"
#define LTC3350_ALARM_VIN_OV_SIZE 1
#define LTC3350_ALARM_VIN_OV_OFFSET 5
#define LTC3350_ALARM_VIN_OV_MASK 0x0020
#define LTC3350_ALARM_VIN_OV (LTC3350_ALARM_VIN_OV_OFFSET << 12 | (LTC3350_ALARM_VIN_OV_SIZE - 1) << 8 | LTC3350_ALARM_VIN_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_VCAP_UV ALARM_VCAP_UV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_VCAP_UV Bit Field
 *
 *  VCAP undervoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 6
 *   - MSB: 6
 *   - MASK: 0x0040
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_VCAP_UV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_VCAP_UV "ALARM_VCAP_UV"
#define LTC3350_ALARM_VCAP_UV_SIZE 1
#define LTC3350_ALARM_VCAP_UV_OFFSET 6
#define LTC3350_ALARM_VCAP_UV_MASK 0x0040
#define LTC3350_ALARM_VCAP_UV (LTC3350_ALARM_VCAP_UV_OFFSET << 12 | (LTC3350_ALARM_VCAP_UV_SIZE - 1) << 8 | LTC3350_ALARM_VCAP_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_VCAP_OV ALARM_VCAP_OV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_VCAP_OV Bit Field
 *
 *  VCAP overvoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 7
 *   - MSB: 7
 *   - MASK: 0x0080
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_VCAP_OV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_VCAP_OV "ALARM_VCAP_OV"
#define LTC3350_ALARM_VCAP_OV_SIZE 1
#define LTC3350_ALARM_VCAP_OV_OFFSET 7
#define LTC3350_ALARM_VCAP_OV_MASK 0x0080
#define LTC3350_ALARM_VCAP_OV (LTC3350_ALARM_VCAP_OV_OFFSET << 12 | (LTC3350_ALARM_VCAP_OV_SIZE - 1) << 8 | LTC3350_ALARM_VCAP_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_VOUT_UV ALARM_VOUT_UV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_VOUT_UV Bit Field
 *
 *  VOUT undervoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 8
 *   - MSB: 8
 *   - MASK: 0x0100
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_VOUT_UV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_VOUT_UV "ALARM_VOUT_UV"
#define LTC3350_ALARM_VOUT_UV_SIZE 1
#define LTC3350_ALARM_VOUT_UV_OFFSET 8
#define LTC3350_ALARM_VOUT_UV_MASK 0x0100
#define LTC3350_ALARM_VOUT_UV (LTC3350_ALARM_VOUT_UV_OFFSET << 12 | (LTC3350_ALARM_VOUT_UV_SIZE - 1) << 8 | LTC3350_ALARM_VOUT_UV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_VOUT_OV ALARM_VOUT_OV
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_VOUT_OV Bit Field
 *
 *  VOUT overvoltage alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 9
 *   - MSB: 9
 *   - MASK: 0x0200
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_VOUT_OV_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_VOUT_OV "ALARM_VOUT_OV"
#define LTC3350_ALARM_VOUT_OV_SIZE 1
#define LTC3350_ALARM_VOUT_OV_OFFSET 9
#define LTC3350_ALARM_VOUT_OV_MASK 0x0200
#define LTC3350_ALARM_VOUT_OV (LTC3350_ALARM_VOUT_OV_OFFSET << 12 | (LTC3350_ALARM_VOUT_OV_SIZE - 1) << 8 | LTC3350_ALARM_VOUT_OV_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_IIN_OC ALARM_IIN_OC
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_IIN_OC Bit Field
 *
 *  Input overcurrent alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 10
 *   - MSB: 10
 *   - MASK: 0x0400
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_IIN_OC_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_IIN_OC "ALARM_IIN_OC"
#define LTC3350_ALARM_IIN_OC_SIZE 1
#define LTC3350_ALARM_IIN_OC_OFFSET 10
#define LTC3350_ALARM_IIN_OC_MASK 0x0400
#define LTC3350_ALARM_IIN_OC (LTC3350_ALARM_IIN_OC_OFFSET << 12 | (LTC3350_ALARM_IIN_OC_SIZE - 1) << 8 | LTC3350_ALARM_IIN_OC_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_ICHG_UC ALARM_ICHG_UC
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_ICHG_UC Bit Field
 *
 *  Charge undercurrent alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 11
 *   - MSB: 11
 *   - MASK: 0x0800
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_ICHG_UC_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_ICHG_UC "ALARM_ICHG_UC"
#define LTC3350_ALARM_ICHG_UC_SIZE 1
#define LTC3350_ALARM_ICHG_UC_OFFSET 11
#define LTC3350_ALARM_ICHG_UC_MASK 0x0800
#define LTC3350_ALARM_ICHG_UC (LTC3350_ALARM_ICHG_UC_OFFSET << 12 | (LTC3350_ALARM_ICHG_UC_SIZE - 1) << 8 | LTC3350_ALARM_ICHG_UC_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_DTEMP_COLD ALARM_DTEMP_COLD
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_DTEMP_COLD Bit Field
 *
 *  Die temperature cold alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 12
 *   - MSB: 12
 *   - MASK: 0x1000
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_DTEMP_COLD_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_DTEMP_COLD "ALARM_DTEMP_COLD"
#define LTC3350_ALARM_DTEMP_COLD_SIZE 1
#define LTC3350_ALARM_DTEMP_COLD_OFFSET 12
#define LTC3350_ALARM_DTEMP_COLD_MASK 0x1000
#define LTC3350_ALARM_DTEMP_COLD (LTC3350_ALARM_DTEMP_COLD_OFFSET << 12 | (LTC3350_ALARM_DTEMP_COLD_SIZE - 1) << 8 | LTC3350_ALARM_DTEMP_COLD_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_DTEMP_HOT ALARM_DTEMP_HOT
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_DTEMP_HOT Bit Field
 *
 *  Die temperature hot alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 13
 *   - MSB: 13
 *   - MASK: 0x2000
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_DTEMP_HOT_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_DTEMP_HOT "ALARM_DTEMP_HOT"
#define LTC3350_ALARM_DTEMP_HOT_SIZE 1
#define LTC3350_ALARM_DTEMP_HOT_OFFSET 13
#define LTC3350_ALARM_DTEMP_HOT_MASK 0x2000
#define LTC3350_ALARM_DTEMP_HOT (LTC3350_ALARM_DTEMP_HOT_OFFSET << 12 | (LTC3350_ALARM_DTEMP_HOT_SIZE - 1) << 8 | LTC3350_ALARM_DTEMP_HOT_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_ESR_HI ALARM_ESR_HI
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_ESR_HI Bit Field
 *
 *  ESR high alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 14
 *   - MSB: 14
 *   - MASK: 0x4000
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_ESR_HI_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_ESR_HI "ALARM_ESR_HI"
#define LTC3350_ALARM_ESR_HI_SIZE 1
#define LTC3350_ALARM_ESR_HI_OFFSET 14
#define LTC3350_ALARM_ESR_HI_MASK 0x4000
#define LTC3350_ALARM_ESR_HI (LTC3350_ALARM_ESR_HI_OFFSET << 12 | (LTC3350_ALARM_ESR_HI_SIZE - 1) << 8 | LTC3350_ALARM_ESR_HI_SUBADDR)
//!@}
/*! @defgroup LTC3350_ALARM_CAP_LO ALARM_CAP_LO
 *  @ingroup LTC3350_register_map
 *  @brief ALARM_CAP_LO Bit Field
 *
 *  Capacitance low alarm
 *   - Register: @ref LTC3350_ALARM_REG "ALARM_REG"
 *   - CommandCode: 29
 *   - Size: 1
 *   - Offset: 15
 *   - MSB: 15
 *   - MASK: 0x8000
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_ALARM_CAP_LO_SUBADDR LTC3350_ALARM_REG_SUBADDR //!< @ref LTC3350_ALARM_CAP_LO "ALARM_CAP_LO"
#define LTC3350_ALARM_CAP_LO_SIZE 1
#define LTC3350_ALARM_CAP_LO_OFFSET 15
#define LTC3350_ALARM_CAP_LO_MASK 0x8000
#define LTC3350_ALARM_CAP_LO (LTC3350_ALARM_CAP_LO_OFFSET << 12 | (LTC3350_ALARM_CAP_LO_SIZE - 1) << 8 | LTC3350_ALARM_CAP_LO_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_CAP_CMD MEAS_CAP_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_CAP_CMD Register
 *
 * |     15:0 |
 * |:--------:|
 * | MEAS_CAP |
 *
 *   - CommandCode: 30
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_CAP "MEAS_CAP" : Measured capacitor stack capacitance value.  When ctl_cap_scale is set to 1, capacitance is 3.36uF * RT/RTST per LSB. When ctl_cap_scale is set to 0 it is 336uF * RT/RTST per LSB.
*/

//!@{
#define LTC3350_MEAS_CAP_CMD_SUBADDR 30
#define LTC3350_MEAS_CAP_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_CAP_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_CAP MEAS_CAP
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_CAP Bit Field
 *
 *  Measured capacitor stack capacitance value.  When ctl_cap_scale is set to 1, capacitance is 3.36uF * RT/RTST per LSB. When ctl_cap_scale is set to 0 it is 336uF * RT/RTST per LSB.
 *   - Register: @ref LTC3350_MEAS_CAP_CMD "MEAS_CAP_CMD"
 *   - CommandCode: 30
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_CAP_SUBADDR LTC3350_MEAS_CAP_CMD_SUBADDR //!< @ref LTC3350_MEAS_CAP "MEAS_CAP"
#define LTC3350_MEAS_CAP_SIZE 16
#define LTC3350_MEAS_CAP_OFFSET 0
#define LTC3350_MEAS_CAP_MASK 0xFFFF
#define LTC3350_MEAS_CAP (LTC3350_MEAS_CAP_OFFSET << 12 | (LTC3350_MEAS_CAP_SIZE - 1) << 8 | LTC3350_MEAS_CAP_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_ESR_CMD MEAS_ESR_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_ESR_CMD Register
 *
 * |     15:0 |
 * |:--------:|
 * | MEAS_ESR |
 *
 *   - CommandCode: 31
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_ESR "MEAS_ESR" : Measured capacitor stack equivalent series resistance (ESR) value. RSNSC/64 per LSB
*/

//!@{
#define LTC3350_MEAS_ESR_CMD_SUBADDR 31
#define LTC3350_MEAS_ESR_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_ESR_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_ESR MEAS_ESR
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_ESR Bit Field
 *
 *  Measured capacitor stack equivalent series resistance (ESR) value. RSNSC/64 per LSB
 *   - Register: @ref LTC3350_MEAS_ESR_CMD "MEAS_ESR_CMD"
 *   - CommandCode: 31
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_ESR_SUBADDR LTC3350_MEAS_ESR_CMD_SUBADDR //!< @ref LTC3350_MEAS_ESR "MEAS_ESR"
#define LTC3350_MEAS_ESR_SIZE 16
#define LTC3350_MEAS_ESR_OFFSET 0
#define LTC3350_MEAS_ESR_MASK 0xFFFF
#define LTC3350_MEAS_ESR (LTC3350_MEAS_ESR_OFFSET << 12 | (LTC3350_MEAS_ESR_SIZE - 1) << 8 | LTC3350_MEAS_ESR_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_VCAP1_CMD MEAS_VCAP1_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP1_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | MEAS_VCAP1 |
 *
 *   - CommandCode: 32
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_VCAP1 "MEAS_VCAP1" : Measured voltage between the CAP1 and CAPRTN pins. 183.5uV per LSB
*/

//!@{
#define LTC3350_MEAS_VCAP1_CMD_SUBADDR 32
#define LTC3350_MEAS_VCAP1_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_VCAP1_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_VCAP1 MEAS_VCAP1
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP1 Bit Field
 *
 *  Measured voltage between the CAP1 and CAPRTN pins. 183.5uV per LSB
 *   - Register: @ref LTC3350_MEAS_VCAP1_CMD "MEAS_VCAP1_CMD"
 *   - CommandCode: 32
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_VCAP1_SUBADDR LTC3350_MEAS_VCAP1_CMD_SUBADDR //!< @ref LTC3350_MEAS_VCAP1 "MEAS_VCAP1"
#define LTC3350_MEAS_VCAP1_SIZE 16
#define LTC3350_MEAS_VCAP1_OFFSET 0
#define LTC3350_MEAS_VCAP1_MASK 0xFFFF
#define LTC3350_MEAS_VCAP1 (LTC3350_MEAS_VCAP1_OFFSET << 12 | (LTC3350_MEAS_VCAP1_SIZE - 1) << 8 | LTC3350_MEAS_VCAP1_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_VCAP2_CMD MEAS_VCAP2_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP2_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | MEAS_VCAP2 |
 *
 *   - CommandCode: 33
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_VCAP2 "MEAS_VCAP2" : Measured voltage between the CAP2 and CAP1 pins. 183.5uV per LSB
*/

//!@{
#define LTC3350_MEAS_VCAP2_CMD_SUBADDR 33
#define LTC3350_MEAS_VCAP2_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_VCAP2_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_VCAP2 MEAS_VCAP2
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP2 Bit Field
 *
 *  Measured voltage between the CAP2 and CAP1 pins. 183.5uV per LSB
 *   - Register: @ref LTC3350_MEAS_VCAP2_CMD "MEAS_VCAP2_CMD"
 *   - CommandCode: 33
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_VCAP2_SUBADDR LTC3350_MEAS_VCAP2_CMD_SUBADDR //!< @ref LTC3350_MEAS_VCAP2 "MEAS_VCAP2"
#define LTC3350_MEAS_VCAP2_SIZE 16
#define LTC3350_MEAS_VCAP2_OFFSET 0
#define LTC3350_MEAS_VCAP2_MASK 0xFFFF
#define LTC3350_MEAS_VCAP2 (LTC3350_MEAS_VCAP2_OFFSET << 12 | (LTC3350_MEAS_VCAP2_SIZE - 1) << 8 | LTC3350_MEAS_VCAP2_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_VCAP3_CMD MEAS_VCAP3_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP3_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | MEAS_VCAP3 |
 *
 *   - CommandCode: 34
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_VCAP3 "MEAS_VCAP3" : Measured voltage between the CAP3 and CAP2 pins. 183.5uV per LSB
*/

//!@{
#define LTC3350_MEAS_VCAP3_CMD_SUBADDR 34
#define LTC3350_MEAS_VCAP3_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_VCAP3_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_VCAP3 MEAS_VCAP3
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP3 Bit Field
 *
 *  Measured voltage between the CAP3 and CAP2 pins. 183.5uV per LSB
 *   - Register: @ref LTC3350_MEAS_VCAP3_CMD "MEAS_VCAP3_CMD"
 *   - CommandCode: 34
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_VCAP3_SUBADDR LTC3350_MEAS_VCAP3_CMD_SUBADDR //!< @ref LTC3350_MEAS_VCAP3 "MEAS_VCAP3"
#define LTC3350_MEAS_VCAP3_SIZE 16
#define LTC3350_MEAS_VCAP3_OFFSET 0
#define LTC3350_MEAS_VCAP3_MASK 0xFFFF
#define LTC3350_MEAS_VCAP3 (LTC3350_MEAS_VCAP3_OFFSET << 12 | (LTC3350_MEAS_VCAP3_SIZE - 1) << 8 | LTC3350_MEAS_VCAP3_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_VCAP4_CMD MEAS_VCAP4_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP4_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | MEAS_VCAP4 |
 *
 *   - CommandCode: 35
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_VCAP4 "MEAS_VCAP4" : Measured voltage between the CAP4 and CAP3 pins. 183.5uV per LSB
*/

//!@{
#define LTC3350_MEAS_VCAP4_CMD_SUBADDR 35
#define LTC3350_MEAS_VCAP4_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_VCAP4_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_VCAP4 MEAS_VCAP4
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP4 Bit Field
 *
 *  Measured voltage between the CAP4 and CAP3 pins. 183.5uV per LSB
 *   - Register: @ref LTC3350_MEAS_VCAP4_CMD "MEAS_VCAP4_CMD"
 *   - CommandCode: 35
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_VCAP4_SUBADDR LTC3350_MEAS_VCAP4_CMD_SUBADDR //!< @ref LTC3350_MEAS_VCAP4 "MEAS_VCAP4"
#define LTC3350_MEAS_VCAP4_SIZE 16
#define LTC3350_MEAS_VCAP4_OFFSET 0
#define LTC3350_MEAS_VCAP4_MASK 0xFFFF
#define LTC3350_MEAS_VCAP4 (LTC3350_MEAS_VCAP4_OFFSET << 12 | (LTC3350_MEAS_VCAP4_SIZE - 1) << 8 | LTC3350_MEAS_VCAP4_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_GPI_CMD MEAS_GPI_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_GPI_CMD Register
 *
 * |     15:0 |
 * |:--------:|
 * | MEAS_GPI |
 *
 *   - CommandCode: 36
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_GPI "MEAS_GPI" : Measurement of GPI pin voltage. 183.5uV per LSB
*/

//!@{
#define LTC3350_MEAS_GPI_CMD_SUBADDR 36
#define LTC3350_MEAS_GPI_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_GPI_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_GPI MEAS_GPI
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_GPI Bit Field
 *
 *  Measurement of GPI pin voltage. 183.5uV per LSB
 *   - Register: @ref LTC3350_MEAS_GPI_CMD "MEAS_GPI_CMD"
 *   - CommandCode: 36
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_GPI_SUBADDR LTC3350_MEAS_GPI_CMD_SUBADDR //!< @ref LTC3350_MEAS_GPI "MEAS_GPI"
#define LTC3350_MEAS_GPI_SIZE 16
#define LTC3350_MEAS_GPI_OFFSET 0
#define LTC3350_MEAS_GPI_MASK 0xFFFF
#define LTC3350_MEAS_GPI (LTC3350_MEAS_GPI_OFFSET << 12 | (LTC3350_MEAS_GPI_SIZE - 1) << 8 | LTC3350_MEAS_GPI_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_VIN_CMD MEAS_VIN_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VIN_CMD Register
 *
 * |     15:0 |
 * |:--------:|
 * | MEAS_VIN |
 *
 *   - CommandCode: 37
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_VIN "MEAS_VIN" : Measured Input Voltage. 2.21mV per LSB
*/

//!@{
#define LTC3350_MEAS_VIN_CMD_SUBADDR 37
#define LTC3350_MEAS_VIN_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_VIN_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_VIN MEAS_VIN
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VIN Bit Field
 *
 *  Measured Input Voltage. 2.21mV per LSB
 *   - Register: @ref LTC3350_MEAS_VIN_CMD "MEAS_VIN_CMD"
 *   - CommandCode: 37
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_VIN_SUBADDR LTC3350_MEAS_VIN_CMD_SUBADDR //!< @ref LTC3350_MEAS_VIN "MEAS_VIN"
#define LTC3350_MEAS_VIN_SIZE 16
#define LTC3350_MEAS_VIN_OFFSET 0
#define LTC3350_MEAS_VIN_MASK 0xFFFF
#define LTC3350_MEAS_VIN (LTC3350_MEAS_VIN_OFFSET << 12 | (LTC3350_MEAS_VIN_SIZE - 1) << 8 | LTC3350_MEAS_VIN_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_VCAP_CMD MEAS_VCAP_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP_CMD Register
 *
 * |      15:0 |
 * |:---------:|
 * | MEAS_VCAP |
 *
 *   - CommandCode: 38
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_VCAP "MEAS_VCAP" : Measured Capacitor Stack Voltage. 1.476mV per LSB.
*/

//!@{
#define LTC3350_MEAS_VCAP_CMD_SUBADDR 38
#define LTC3350_MEAS_VCAP_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_VCAP_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_VCAP MEAS_VCAP
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VCAP Bit Field
 *
 *  Measured Capacitor Stack Voltage. 1.476mV per LSB.
 *   - Register: @ref LTC3350_MEAS_VCAP_CMD "MEAS_VCAP_CMD"
 *   - CommandCode: 38
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_VCAP_SUBADDR LTC3350_MEAS_VCAP_CMD_SUBADDR //!< @ref LTC3350_MEAS_VCAP "MEAS_VCAP"
#define LTC3350_MEAS_VCAP_SIZE 16
#define LTC3350_MEAS_VCAP_OFFSET 0
#define LTC3350_MEAS_VCAP_MASK 0xFFFF
#define LTC3350_MEAS_VCAP (LTC3350_MEAS_VCAP_OFFSET << 12 | (LTC3350_MEAS_VCAP_SIZE - 1) << 8 | LTC3350_MEAS_VCAP_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_VOUT_CMD MEAS_VOUT_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VOUT_CMD Register
 *
 * |      15:0 |
 * |:---------:|
 * | MEAS_VOUT |
 *
 *   - CommandCode: 39
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_VOUT "MEAS_VOUT" : Measured Output Voltage. 2.21mV per LSB.
*/

//!@{
#define LTC3350_MEAS_VOUT_CMD_SUBADDR 39
#define LTC3350_MEAS_VOUT_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_VOUT_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_VOUT MEAS_VOUT
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_VOUT Bit Field
 *
 *  Measured Output Voltage. 2.21mV per LSB.
 *   - Register: @ref LTC3350_MEAS_VOUT_CMD "MEAS_VOUT_CMD"
 *   - CommandCode: 39
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_VOUT_SUBADDR LTC3350_MEAS_VOUT_CMD_SUBADDR //!< @ref LTC3350_MEAS_VOUT "MEAS_VOUT"
#define LTC3350_MEAS_VOUT_SIZE 16
#define LTC3350_MEAS_VOUT_OFFSET 0
#define LTC3350_MEAS_VOUT_MASK 0xFFFF
#define LTC3350_MEAS_VOUT (LTC3350_MEAS_VOUT_OFFSET << 12 | (LTC3350_MEAS_VOUT_SIZE - 1) << 8 | LTC3350_MEAS_VOUT_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_IIN_CMD MEAS_IIN_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_IIN_CMD Register
 *
 * |     15:0 |
 * |:--------:|
 * | MEAS_IIN |
 *
 *   - CommandCode: 40
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_IIN "MEAS_IIN" : Measured Input Current. 1.983uV/RSNSI per LSB
*/

//!@{
#define LTC3350_MEAS_IIN_CMD_SUBADDR 40
#define LTC3350_MEAS_IIN_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_IIN_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_IIN MEAS_IIN
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_IIN Bit Field
 *
 *  Measured Input Current. 1.983uV/RSNSI per LSB
 *   - Register: @ref LTC3350_MEAS_IIN_CMD "MEAS_IIN_CMD"
 *   - CommandCode: 40
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_IIN_SUBADDR LTC3350_MEAS_IIN_CMD_SUBADDR //!< @ref LTC3350_MEAS_IIN "MEAS_IIN"
#define LTC3350_MEAS_IIN_SIZE 16
#define LTC3350_MEAS_IIN_OFFSET 0
#define LTC3350_MEAS_IIN_MASK 0xFFFF
#define LTC3350_MEAS_IIN (LTC3350_MEAS_IIN_OFFSET << 12 | (LTC3350_MEAS_IIN_SIZE - 1) << 8 | LTC3350_MEAS_IIN_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_ICHG_CMD MEAS_ICHG_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_ICHG_CMD Register
 *
 * |      15:0 |
 * |:---------:|
 * | MEAS_ICHG |
 *
 *   - CommandCode: 41
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_ICHG "MEAS_ICHG" : Measured Charge Current. 1.983uV/RSNSC per LSB
*/

//!@{
#define LTC3350_MEAS_ICHG_CMD_SUBADDR 41
#define LTC3350_MEAS_ICHG_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_ICHG_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_ICHG MEAS_ICHG
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_ICHG Bit Field
 *
 *  Measured Charge Current. 1.983uV/RSNSC per LSB
 *   - Register: @ref LTC3350_MEAS_ICHG_CMD "MEAS_ICHG_CMD"
 *   - CommandCode: 41
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_ICHG_SUBADDR LTC3350_MEAS_ICHG_CMD_SUBADDR //!< @ref LTC3350_MEAS_ICHG "MEAS_ICHG"
#define LTC3350_MEAS_ICHG_SIZE 16
#define LTC3350_MEAS_ICHG_OFFSET 0
#define LTC3350_MEAS_ICHG_MASK 0xFFFF
#define LTC3350_MEAS_ICHG (LTC3350_MEAS_ICHG_OFFSET << 12 | (LTC3350_MEAS_ICHG_SIZE - 1) << 8 | LTC3350_MEAS_ICHG_SUBADDR)
//!@}

/*! @defgroup LTC3350_MEAS_DTEMP_CMD MEAS_DTEMP_CMD
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_DTEMP_CMD Register
 *
 * |       15:0 |
 * |:----------:|
 * | MEAS_DTEMP |
 *
 *   - CommandCode: 42
 *   - Contains Bit Fields:
 *     + @ref LTC3350_MEAS_DTEMP "MEAS_DTEMP" : Measured die temperature. Temperature = 0.028C per LSB - 251.4C
*/

//!@{
#define LTC3350_MEAS_DTEMP_CMD_SUBADDR 42
#define LTC3350_MEAS_DTEMP_CMD (0 << 12 | (16 - 1) << 8 | LTC3350_MEAS_DTEMP_CMD_SUBADDR)
//!@}
/*! @defgroup LTC3350_MEAS_DTEMP MEAS_DTEMP
 *  @ingroup LTC3350_register_map
 *  @brief MEAS_DTEMP Bit Field
 *
 *  Measured die temperature. Temperature = 0.028C per LSB - 251.4C
 *   - Register: @ref LTC3350_MEAS_DTEMP_CMD "MEAS_DTEMP_CMD"
 *   - CommandCode: 42
 *   - Size: 16
 *   - Offset: 0
 *   - MSB: 15
 *   - MASK: 0xFFFF
 *   - Access: R
 *   - Default: n/a
 */
//!@{
#define LTC3350_MEAS_DTEMP_SUBADDR LTC3350_MEAS_DTEMP_CMD_SUBADDR //!< @ref LTC3350_MEAS_DTEMP "MEAS_DTEMP"
#define LTC3350_MEAS_DTEMP_SIZE 16
#define LTC3350_MEAS_DTEMP_OFFSET 0
#define LTC3350_MEAS_DTEMP_MASK 0xFFFF
#define LTC3350_MEAS_DTEMP (LTC3350_MEAS_DTEMP_OFFSET << 12 | (LTC3350_MEAS_DTEMP_SIZE - 1) << 8 | LTC3350_MEAS_DTEMP_SUBADDR)
//!@}

#endif
