/*
 * capinfo.h
 *
 *  Created on: Apr 3, 2021
 *      Author: dima
 */

#ifndef CAPINFO_H_
#define CAPINFO_H_

#define R_SNSI				0.015
#define R_SNSC				0.005

#define VCAP_LSB			0.0001835
#define VCAP_ALL_LSB	 	0.001476
#define VIN_LSB				0.00221
#define VOUT_LSB			0.00221
#define IIN_LSB				(0.000001983/R_SNSI)
#define ICHRG_LSB			(0.000001983/R_SNSC)
#define VSHUNT_LSB			0.0001835


typedef union _chrg_status_reg
{
    struct
    {
        uint16_t chrg_stepdown                      :1;
        uint16_t chrg_stepup                        :1;
        uint16_t chrg_cv                   			:1;
        uint16_t chrg_uvlo                        	:1;
        uint16_t chrg_input_ilim                    :1;
        uint16_t chrg_cappg                         :1;
        uint16_t chrg_shnt                          :1;
        uint16_t chrg_bal                          	:1;

        uint16_t chrg_dis                           :1;
        uint16_t chrg_ci                            :1;
        uint16_t not_used                    		:1;
        uint16_t chrg_pfo                        	:1;
        uint16_t reserved1                       	:1;
        uint16_t reserved2                          :1;
        uint16_t reserved3                          :1;
        uint16_t reserved4                          :1;
    };// __attribute__ ((packed));

    uint16_t reg;
} chrg_status_reg;

chrg_status_reg chrg_status;

typedef union _mon_status_reg
{
    struct
    {
        uint16_t mon_capesr_active                  :1;
        uint16_t mon_capesr_scheduled               :1;
        uint16_t mon_capesr_pending                 :1;
        uint16_t mon_cap_done                       :1;
        uint16_t mon_esr_done                       :1;
        uint16_t mon_cap_failed                     :1;
        uint16_t mon_esr_failed                     :1;
        uint16_t not_used                          	:1;

        uint16_t mon_power_failed                   :1;
        uint16_t mon_power_returned                 :1;
        uint16_t reserved1                    		:1;
        uint16_t reserved2                        	:1;
        uint16_t reserved3                       	:1;
        uint16_t reserved4                          :1;
        uint16_t reserved5                          :1;
        uint16_t reserved6                          :1;
    };// __attribute__ ((packed));

    uint16_t reg;
} mon_status_reg;

mon_status_reg mon_status;


#endif /* CAPINFO_H_ */
