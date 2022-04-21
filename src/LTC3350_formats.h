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
 *  @brief LTC3350 library file defining data conversion macros and constants used
 *  by LTC3350.c
 *
 *
 *  This file contains macros and constants which can be used to represent real-world
 *  values in the source code, with automatic compile-time conversion to LTC3350
 *  internal register scaling.
 *
 *  Passing runtime variables to these macros should be avoided as it will likely result
 *  in runtime calculations accompanied by associated processor loading and memory usage.
 */

#ifndef LTC3350_FORMATS_H_
#define LTC3350_FORMATS_H_

/*! @name Format Definitions
 *  Constants used by the macros below to convert from real world to LTC3350 referenced numbers.
 */
/*! @name Constants used in real world conversion macros below
 */
#define LTC3350_RSNSI 0.016
#define LTC3350_RSNSC 0.006
#define LTC3350_RTST 121
#define LTC3350_RT 71.5e3
#define LTC3350_CTL_CAP_SCALE_VAL 0
#define LTC3350_GPI_ADC_SCALE 183.5e-6
#define LTC3350_GPI_VREF 2.5
#define LTC3350_RNTCBIAS 10000.0
#define LTC3350_RNTCSER 0.0
#define LTC3350_Rm40 214063.67
#define LTC3350_Rm34 152840.30
#define LTC3350_Rm28 110480.73
#define LTC3350_Rm21 76798.02
#define LTC3350_Rm14 54214.99
#define LTC3350_Rm6 37075.65
#define LTC3350_R4 23649.71
#define LTC3350_R33 7400.97
#define LTC3350_R44 5001.22
#define LTC3350_R53 3693.55
#define LTC3350_R62 2768.21
#define LTC3350_R70 2167.17
#define LTC3350_R78 1714.08
#define LTC3350_R86 1368.87
#define LTC3350_R94 1103.18
#define LTC3350_R102 896.73
#define LTC3350_R110 734.86
#define LTC3350_R118 606.86
#define LTC3350_R126 504.80
#define LTC3350_R134 422.81
#define LTC3350_R142 356.45
#define LTC3350_R150 302.36
/*! @name Use the macros below to convert from real world to LTC3350 referenced numbers.
 */
/*! Convert from °C to the thermistor ADC reading. */
#define LTC3350_NTCS0402E3103FLT(x) (\
__LTC3350_BETWEEN_INCLUSIVE__((-40), (-34), x) ? __LTC3350_LINE__((-40), (-34), ((LTC3350_Rm40 + LTC3350_RNTCSER) / (LTC3350_Rm40 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_Rm34 + LTC3350_RNTCSER) / (LTC3350_Rm34 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((-34), (-28), x) ? __LTC3350_LINE__((-34), (-28), ((LTC3350_Rm34 + LTC3350_RNTCSER) / (LTC3350_Rm34 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_Rm28 + LTC3350_RNTCSER) / (LTC3350_Rm28 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((-28), (-21), x) ? __LTC3350_LINE__((-28), (-21), ((LTC3350_Rm28 + LTC3350_RNTCSER) / (LTC3350_Rm28 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_Rm21 + LTC3350_RNTCSER) / (LTC3350_Rm21 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((-21), (-14), x) ? __LTC3350_LINE__((-21), (-14), ((LTC3350_Rm21 + LTC3350_RNTCSER) / (LTC3350_Rm21 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_Rm14 + LTC3350_RNTCSER) / (LTC3350_Rm14 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((-14), (-6), x) ? __LTC3350_LINE__((-14), (-6), ((LTC3350_Rm14 + LTC3350_RNTCSER) / (LTC3350_Rm14 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_Rm6 + LTC3350_RNTCSER) / (LTC3350_Rm6 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((-6), (4), x) ? __LTC3350_LINE__((-6), (4), ((LTC3350_Rm6 + LTC3350_RNTCSER) / (LTC3350_Rm6 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R4 + LTC3350_RNTCSER) / (LTC3350_R4 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((4), (33), x) ? __LTC3350_LINE__((4), (33), ((LTC3350_R4 + LTC3350_RNTCSER) / (LTC3350_R4 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R33 + LTC3350_RNTCSER) / (LTC3350_R33 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((33), (44), x) ? __LTC3350_LINE__((33), (44), ((LTC3350_R33 + LTC3350_RNTCSER) / (LTC3350_R33 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R44 + LTC3350_RNTCSER) / (LTC3350_R44 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((44), (53), x) ? __LTC3350_LINE__((44), (53), ((LTC3350_R44 + LTC3350_RNTCSER) / (LTC3350_R44 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R53 + LTC3350_RNTCSER) / (LTC3350_R53 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((53), (62), x) ? __LTC3350_LINE__((53), (62), ((LTC3350_R53 + LTC3350_RNTCSER) / (LTC3350_R53 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R62 + LTC3350_RNTCSER) / (LTC3350_R62 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((62), (70), x) ? __LTC3350_LINE__((62), (70), ((LTC3350_R62 + LTC3350_RNTCSER) / (LTC3350_R62 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R70 + LTC3350_RNTCSER) / (LTC3350_R70 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((70), (78), x) ? __LTC3350_LINE__((70), (78), ((LTC3350_R70 + LTC3350_RNTCSER) / (LTC3350_R70 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R78 + LTC3350_RNTCSER) / (LTC3350_R78 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((78), (86), x) ? __LTC3350_LINE__((78), (86), ((LTC3350_R78 + LTC3350_RNTCSER) / (LTC3350_R78 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R86 + LTC3350_RNTCSER) / (LTC3350_R86 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((86), (94), x) ? __LTC3350_LINE__((86), (94), ((LTC3350_R86 + LTC3350_RNTCSER) / (LTC3350_R86 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R94 + LTC3350_RNTCSER) / (LTC3350_R94 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((94), (102), x) ? __LTC3350_LINE__((94), (102), ((LTC3350_R94 + LTC3350_RNTCSER) / (LTC3350_R94 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R102 + LTC3350_RNTCSER) / (LTC3350_R102 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((102), (110), x) ? __LTC3350_LINE__((102), (110), ((LTC3350_R102 + LTC3350_RNTCSER) / (LTC3350_R102 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R110 + LTC3350_RNTCSER) / (LTC3350_R110 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((110), (118), x) ? __LTC3350_LINE__((110), (118), ((LTC3350_R110 + LTC3350_RNTCSER) / (LTC3350_R110 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R118 + LTC3350_RNTCSER) / (LTC3350_R118 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((118), (126), x) ? __LTC3350_LINE__((118), (126), ((LTC3350_R118 + LTC3350_RNTCSER) / (LTC3350_R118 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R126 + LTC3350_RNTCSER) / (LTC3350_R126 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((126), (134), x) ? __LTC3350_LINE__((126), (134), ((LTC3350_R126 + LTC3350_RNTCSER) / (LTC3350_R126 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R134 + LTC3350_RNTCSER) / (LTC3350_R134 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((134), (142), x) ? __LTC3350_LINE__((134), (142), ((LTC3350_R134 + LTC3350_RNTCSER) / (LTC3350_R134 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R142 + LTC3350_RNTCSER) / (LTC3350_R142 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : \
__LTC3350_BETWEEN_INCLUSIVE__((142), (150), x) ? __LTC3350_LINE__((142), (150), ((LTC3350_R142 + LTC3350_RNTCSER) / (LTC3350_R142 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), ((LTC3350_R150 + LTC3350_RNTCSER) / (LTC3350_R150 + LTC3350_RNTCSER + LTC3350_RNTCBIAS) * LTC3350_GPI_VREF / LTC3350_GPI_ADC_SCALE), x) : -1)
/*! Measured capacitor stack capacitance value. When CTL_CAP_SCALE is set to 1, capacitance is 3.36uF * RT/RTST per LSB. When CTL_CAP_SCALE is set to 0 it is 336uF * RT/RTST per LSB. */
#define LTC3350_CAP(x) __LTC3350_LINE__((0), ((3.36e-6 + 332.64e-6 * (1 - LTC3350_CTL_CAP_SCALE_VAL)) * LTC3350_RT / LTC3350_RTST), (0), (1), x)
/*! This is used to program the capacitor voltage feedback loop's reference voltage. Only bits 3:0 are active. CAPFBREF = 37.5mV * vcapfb_dac + 637.5mV. */
#define LTC3350_VCAPFB_DAC_FORMAT(x) __LTC3350_LINE__((0.6375), (0.6375 + 0.0375), (0), (1), x)
/*! Measured voltage between CAP pins or CAP1 and CAPRTN. */
#define LTC3350_CELL(x) __LTC3350_LINE__((0), (183.5e-6), (0), (1), x)
/*! Measured input voltage. */
#define LTC3350_VIN(x) __LTC3350_LINE__((0), (2.21e-3), (0), (1), x)
/*! Measured capacitor stack voltage. */
#define LTC3350_VCAP(x) __LTC3350_LINE__((0), (1.476e-3), (0), (1), x)
/*! Measured output voltage */
#define LTC3350_VOUT(x) __LTC3350_LINE__((0), (2.21e-3), (0), (1), x)
/*! Measured input current. */
#define LTC3350_IIN(x) __LTC3350_LINE__((0), (1.983e-6 / LTC3350_RSNSI), (0), (1), x)
/*! Measured die temperature. */
#define LTC3350_DTEMP(x) __LTC3350_LINE__((-251.4), (-251.4 + 0.028), (0), (1), x)
/*! Measured capacitor stack equivalent series resistance (ESR) value. */
#define LTC3350_ESR(x) __LTC3350_LINE__((0), (LTC3350_RSNSC / 64.0), (0), (1), x)
/*! Measured Charge Current. */
#define LTC3350_ICHARGE(x) __LTC3350_LINE__((0), (1.983e-6 / LTC3350_RSNSC), (0), (1), x)
/*! Capacitance and ESR Measurement Period: This register sets the period of repeated capacitance and ESR measurements. */
#define LTC3350_CAP_PER(x) __LTC3350_LINE__((0), (10.0), (0), (1), x)
/*! Native ADC span. */
#define LTC3350_ADC(x) __LTC3350_LINE__((0), (1 / 24270.0), (0), (1), x)
/*! Unmodified capacitor data */
#define LTC3350_CAP_ZS(x) __LTC3350_LINE__((0), (1.0), (0), (1), x)
/*! @name Private macros for use by formats above.
 */
#define __LTC3350_LINE__(x0,x1,y0,y1,x) (uint16_t)((y0) + ((y1) - (y0))/((x1) - (x0)) * ((x) - (x0)) + 0.5)
#define __LTC3350_BETWEEN_INCLUSIVE__(x0,x1,x) (((x) > (x0) && (x) < (x1)) || (x) == (x0) || (x) == (x1) ? 1 : 0)


#endif /* LTC3350_FORMATS_H_ */
