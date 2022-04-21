/*
 ============================================================================
 Name        : capInfo.c
 Author      : Dima
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */
#define _BSD_SOURCE
#include "LTC3350.h"
#include <fcntl.h>
#include <linux/i2c-dev.h> //#Make sure to copy corresponding smbus.c from i2c-tools to source dir!
#include <errno.h>
#include <stdio.h>
#include "capinfo.h"

#include <time.h>
#include <signal.h>

uint16_t counter;
timer_t gTimerid;


void start_timer(void)
{
	struct itimerspec value;

	value.it_value.tv_sec = 5;//waits for 5 seconds before sending timer signal
	value.it_value.tv_nsec = 0;

	value.it_interval.tv_sec = 5;//sends timer signal every 5 seconds
	value.it_interval.tv_nsec = 0;

	timer_create (CLOCK_REALTIME, NULL, &gTimerid);
	timer_settime (gTimerid, 0, &value, NULL);
}

void stop_timer(void)
{
	struct itimerspec value;

	value.it_value.tv_sec = 0;
	value.it_value.tv_nsec = 0;

	value.it_interval.tv_sec = 0;
	value.it_interval.tv_nsec = 0;

	timer_settime (gTimerid, 0, &value, NULL);
}

void timer_callback(int sig)
{
	printf(" Catched timer signal: %d … !!\n", sig);
}



/*!  read_register  function  wraps read_word_data and places the returned
data  in  *data. It returns 0 for success and a non-zero error code for failure.
The API functions will return this error code in the event of an error.*/
int read_register(uint8_t addr, //!< Target IC's SMBus address
                  uint8_t command_code, //!< Command Code to be read from
                  uint16_t *data, //!< Pointer to data destination
                  port_configuration_t *port_configuration //!< Pointer to port configuration struct
                 )
{
  if (ioctl(port_configuration->file_descriptor, I2C_SLAVE, addr) < 0)
  {
    printf("Error setting slave address: %s\n", strerror(errno));
    return 1;
  }
  //__s32 i2c_smbus_read_byte_data(int file, __u8 command);
  //__s32 i2c_smbus_read_word_data(int file, __u8 command);
  __s32 ret_val = i2c_smbus_read_word_data(port_configuration->file_descriptor, command_code);
  if (ret_val == -1)
  {
    printf("Read error: %s\n", strerror(errno));
    return 1;
  }
  *data = (uint16_t)ret_val;
  return 0;
}

/*!  write_register  function  wraps  write_word_data.  It  returns  0 for
success  and  a  non-zero  error code for failure. The API functions will return
this error code in the event of an error.*/
int write_register(uint8_t addr, //!< Target IC's SMBus address
                   uint8_t command_code, //!< Command Code to be written to
                   uint16_t data, //!< Data to be written
                   port_configuration_t *port_configuration //!< Pointer to port configuration struct
                  )
{
  if (ioctl(port_configuration->file_descriptor, I2C_SLAVE, addr) < 0)
  {
    printf("Error setting slave address: %s\n", strerror(errno));
    return 1;
  }
  //__s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value);
  //__s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value);
  __s32 ret_val = i2c_smbus_write_word_data(port_configuration->file_descriptor, command_code, data);
  if (ret_val == -1)
  {
    printf("Write error: %s\n", strerror(errno));
    return 1;
  }
  return 0;
}

int main(void)
{
  uint16_t data;
  //choose the correct i2c bus.  You can find the memory mapping at /sys/bus/i2c/devices/i2c*
  int fd = open("/dev/i2c-0", O_RDWR);
  if (fd < 0)
  {
    printf("Error opening file: %s\n", strerror(errno));
    return 1;
  }
  if (ioctl(fd, I2C_PEC, 0) < 0) //set non-zero to enable PEC
  {
    printf("Error disabling Packet Error Checking: %s\n", strerror(errno));
    return 1;
  }
  //Leave slave address ioctl setup for read/write function.
  //Could place here if only ever talking to one address.
  port_configuration_t pc =
  {
    .file_descriptor = fd
  };
  LTC3350_chip_cfg_t cfg =
  {
    .addr = LTC3350_ADDR_09,
    .read_register = read_register,
    .write_register = write_register,
    .port_configuration = &pc
  };

  printf("Initializing LTC3350\n");
  LTC3350 chip = LTC3350_init(&cfg);

  // the API functions can be used to read and write individual bit fields within a command code
/*  printf("Using Read Register for the LTC3350_MSK_GPI_UV bit field\n");
  LTC3350_read_register(chip, LTC3350_MSK_GPI_UV, &data);
  printf("Read: %d\n",data);
  printf("Using Read/Modify/Write Register for the LTC3350_MSK_GPI_UV bit field\n");
  LTC3350_write_register(chip, LTC3350_MSK_GPI_UV, 1);
  printf("Wrote: %d\n",1);
  printf("Using Read Register for the LTC3350_MSK_GPI_UV bit field\n");
  LTC3350_read_register(chip, LTC3350_MSK_GPI_UV, &data);
  printf("Read: %d\n",data);

  // the API functions can also be used to read and write whole command codes
  printf("Using Read Register for the LTC3350_VOUT_OV_LVL_CMD command code\n");
  LTC3350_read_register(chip, LTC3350_VOUT_OV_LVL_CMD, &data);
  printf("Read: %d\n",data);
  printf("Using Read/Modify/Write Register for the LTC3350_VOUT_OV_LVL_CMD command code\n");
  LTC3350_write_register(chip, LTC3350_VOUT_OV_LVL_CMD, 35787);
  printf("Wrote: %d\n",35787);
  printf("Using Read Register for the LTC3350_VOUT_OV_LVL_CMD command code\n");
  LTC3350_read_register(chip, LTC3350_VOUT_OV_LVL_CMD, &data);
  printf("Read: %d\n",data);*/

	(void) signal(SIGALRM, timer_callback);
	start_timer();

  while (1)
  {
	  	  sleep(1);
	  	  counter++;
	  	  printf("counter: %d\n",counter);

		  LTC3350_read_register(chip, LTC3350_NUM_CAPS, &data);
		  printf("LTC3350_NUM_CAPS: %X\n",data);


		  LTC3350_read_register(chip, LTC3350_MEAS_VIN , &data);
		  double vin = data*VIN_LSB;
		  printf("LTC3350_MEAS_VIN  : %04X  VIN  = %.2f V\n",data, vin);

		  LTC3350_read_register(chip, LTC3350_MEAS_VOUT , &data);
		  double vout = data*VOUT_LSB;
		  printf("LTC3350_MEAS_VOUT : %04X VOUT  = %.2f V\n" ,data,vout);

		  LTC3350_read_register(chip, LTC3350_MEAS_IIN  , &data);
		  double iin = data*IIN_LSB;
		  printf("LTC3350_MEAS_IIN  : %04X IIN   = %.2f A\n",data,iin);

		  LTC3350_read_register(chip, LTC3350_MEAS_ICHG , &data);
		  double ichrg = data*ICHRG_LSB;
		  printf("LTC3350_MEAS_ICHG : %04X ICHRG = %.2f A\n",data, ichrg);

		  LTC3350_read_register(chip, LTC3350_MEAS_DTEMP , &data);
		  printf("LTC3350_MEAS_DTEMP: %04X\n",data);
		  printf("---------------------------------\n");


		  LTC3350_read_register(chip, LTC3350_MEAS_VCAP1 , &data);
		  double vcap1 = data*VCAP_LSB;
		  printf("LTC3350_MEAS_VCAP1: %04X VCAP1 = %.2f V\n",data, vcap1);

		  LTC3350_read_register(chip, LTC3350_MEAS_VCAP2 , &data);
		  double vcap2 = data*VCAP_LSB;
		  printf("LTC3350_MEAS_VCAP1: %04X VCAP2 = %.2f V\n",data, vcap2);

		  LTC3350_read_register(chip, LTC3350_MEAS_VCAP3 , &data);
		  double vcap3 = data*VCAP_LSB;
		  printf("LTC3350_MEAS_VCAP1: %04X VCAP3 = %.2f V\n",data, vcap3);

		  LTC3350_read_register(chip, LTC3350_MEAS_VCAP4 , &data);
		  double vcap4 = data*VCAP_LSB;
		  printf("LTC3350_MEAS_VCAP1: %04X VCAP4 = %.2f V\n",data, vcap4);

		  LTC3350_read_register(chip, LTC3350_MEAS_VCAP , &data);
		  double vcap = data*VCAP_ALL_LSB;
		  printf("LTC3350_MEAS_VCAP: %04X  VCAP  = %.2f V\n",data, vcap);

		  LTC3350_read_register(chip, LTC3350_VCAPFB_DAC , &data);
		  printf("LTC3350_VCAPFB_DAC: %04X \n",data);

		  LTC3350_read_register(chip, LTC3350_VSHUNT , &data);
		  double vshunt = data*VSHUNT_LSB;
		  printf("LTC3350_VSHUNT: %04X   VSHUNT = %.2f V\n",data, vshunt);

		  LTC3350_read_register(chip, LTC3350_MEAS_CAP , &data);
		  printf("LTC3350_MEAS_CAP: %X\n",data);
		  printf("---------------------------------\n");


		  LTC3350_read_register(chip, LTC3350_MSK_MON_STATUS , &data);
		  printf("LTC3350_MSK_MON_STATUS: %04X\n",data);
		  mon_status.reg = data;
		  printf("---------------------------------\n");

		  LTC3350_read_register(chip, LTC3350_CHRG_STATUS , &data);
		  chrg_status.reg = data;
		  printf("LTC3350_CHRG_STATUS: %04X\n",data);
		  printf("The synchronous controller is in step-down mode <charging> : %d \n",chrg_status.chrg_stepdown);
		  printf("The synchronous controller is in step-up mode <backup>     : %d \n",chrg_status.chrg_stepup);
		  printf("The charger is in constant voltage mode                    : %d \n",chrg_status.chrg_cv);
		  printf("The charger is in undervoltage lockout                     : %d \n", chrg_status.chrg_uvlo);
		  printf("The charger is in input current limit                      : %d \n",chrg_status.chrg_input_ilim);
		  printf("The capacitor voltage is above power good threshold        : %d \n",chrg_status.chrg_cappg);
		  printf("The capacitor manager is shunting                          : %d \n",chrg_status.chrg_shnt);
		  printf("The capacitor manager is balancing                         : %d \n",chrg_status.chrg_bal);
		  printf("The charger is temporarily disabled for cap measurement    : %d \n",chrg_status.chrg_dis);
		  printf("The charger is in constant current mode                    : %d \n",chrg_status.chrg_ci);
		  printf("Input voltage is below PFI threshold                       : %d \n",chrg_status.chrg_pfo);

		  printf("---------------------------------\n");
		  LTC3350_read_register(chip, LTC3350_ALARM_REG , &data);
		  printf("LTC3350_ALARM_REG: %X\n",data);

  }
  return 0;
}
