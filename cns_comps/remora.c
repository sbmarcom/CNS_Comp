/********************************************************************
* Description:  remora.c
*               This file, 'remora.c', is a HAL component that
*               provides and SPI connection to a external LPC1768 running Remora PRU firmware.
*  				
*				Initially developed for RaspberryPi -> Arduino Due.
*				Further developed for RaspberryPi -> Smoothieboard and clones (LPC1768).
*
* Author: Scott Alford
* License: GPL Version 2
*
*		Credit to GP Orcullo and PICnc V2 which originally inspired this
*		and portions of this code is based on stepgen.c by John Kasunich
*		and hm2_rpspi.c by Matsche
*
* Copyright (c) 2021	All rights reserved.
*
* Last change:
********************************************************************/


#include "rtapi.h"			/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"			/* HAL public API decls */

#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>


// Using BCM2835 driver library by Mike McCauley, why reinvent the wheel!
// http://www.airspayce.com/mikem/bcm2835/index.html
// Include these in the source directory when using "halcompile --install remora.c"
#include "bcm2835.h"
#include "bcm2835.c"

#include "remora.h"


#define MODNAME "remora"
#define PREFIX "remora"

MODULE_AUTHOR("Scott Alford AKA scotta");
MODULE_DESCRIPTION("Driver for Remora LPC1768 control board");
MODULE_LICENSE("GPL v2");


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

typedef struct {
	hal_bit_t		*SPIenable;
	hal_bit_t		*SPIreset;
	hal_bit_t		*PRUreset;
	bool			SPIresetOld;
	hal_bit_t		*SPIstatus;
	hal_bit_t 		*stepperEnable[JOINTS];
	int				pos_mode[JOINTS];
	hal_float_t 	*pos_cmd[JOINTS];			// pin: position command (position units)
	hal_float_t 	*vel_cmd[JOINTS];			// pin: velocity command (position units/sec)
	hal_float_t 	*pos_fb[JOINTS];			// pin: position feedback (position units)
	hal_s32_t		*count[JOINTS];				// pin: psition feedback (raw counts)
	hal_float_t 	pos_scale[JOINTS];			// param: steps per position unit
	float 			freq[JOINTS];				// param: frequency command sent to PRU
	hal_float_t 	*freq_cmd[JOINTS];			// pin: frequency command monitoring, available in LinuxCNC
	hal_float_t 	maxvel[JOINTS];				// param: max velocity, (pos units/sec)
	hal_float_t 	maxaccel[JOINTS];			// param: max accel (pos units/sec^2)
	hal_float_t		*pgain[JOINTS];
	hal_float_t		*ff1gain[JOINTS];
	hal_float_t		*deadband[JOINTS];
	float 			old_pos_cmd[JOINTS];		// previous position command (counts)
	float 			old_pos_cmd_raw[JOINTS];		// previous position command (counts)
	float 			old_scale[JOINTS];			// stored scale value
	float 			scale_recip[JOINTS];		// reciprocal value used for scaling
	float			prev_cmd[JOINTS];
	float			cmd_d[JOINTS];					// command derivative
	hal_float_t 	*setPoint[VARIABLES];
	hal_float_t 	*processVariable[VARIABLES];
	hal_bit_t   	*outputs[DIGITAL_OUTPUTS];
	hal_bit_t   	*inputs[DIGITAL_INPUTS];
	hal_s32_t  	    *home_position_degrees[JOINTS]; //the home position in degrees, with respect to the absolute encoder 0 position on each joint 
	hal_bit_t 		*pose_at_enable; //The pose of the machine when enabled, for kinematics. 
} data_t;

static data_t *data;

uint8_t PACKETCOUNTER = 0x00;
#pragma pack(push, 1)

typedef union
{
  // this allow structured access to the outgoing SPI data without having to move it
  // this is the same structure as the PRU rxData structure
  struct
  {
    uint8_t txBuffer[SPIBUFSIZE];
  };
  struct
  {
	//change datatype of header to match Dario
	uint8_t header;
	uint8_t jointEnable;
    int32_t jointPosCmd[JOINTS];// Not implemented on STM32 yet
    uint8_t digital_outputs;
  };
} txData_t;


typedef union
{
  // this allow structured access to the incoming SPI data without having to move it
  // this is the same structure as the PRU txData structure
  struct
  {
    uint8_t rxBuffer[SPIBUFSIZE];
  };
  struct
  {
		//change datatype of header to match Dario
    uint8_t header;
	uint8_t joint_enable;
    uint32_t jointFeedback[JOINTS];
    uint8_t digital_inputs;
  };
} rxData_t;

#pragma pack(pop)

static txData_t txData;
static rxData_t rxData;

uint32_t        pos_cmd_out[JOINTS];  //Pulses to stm32

/* other globals */
static int 			comp_id;				// component ID
static const char 	*modname = MODNAME;
static const char 	*prefix = PREFIX;
static int 			num_chan = 0;			// number of step generators configured
static long 		old_dtns;				// update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double		dt;						// update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double 		recip_dt;				// recprocal of period, avoids divides

static int64_t 		accum[JOINTS] = { 0 };
static int32_t 		old_count[JOINTS] = { 0 };
static int32_t		accum_diff = 0;

static bool        old_enable_status= 0;

static int 			reset_gpio_pin = 25;				// RPI GPIO pin number used to force watchdog reset of the PRU 

static int 			correction[JOINTS] = {0};

typedef enum CONTROL { POSITION, VELOCITY, INVALID } CONTROL;
char *ctrl_type[JOINTS] = { "p" };
RTAPI_MP_ARRAY_STRING(ctrl_type,JOINTS,"control type (pos or vel)");

enum CHIP { LPC, STM } chip;
char *chip_type = { "STM" }; //default to LPC
RTAPI_MP_STRING(chip_type, "PRU chip type; LPC or STM");

int SPI_clk_div = -1;
RTAPI_MP_INT(SPI_clk_div, "SPI clock divider");



/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
static int rt_bcm2835_init(void);

static void update_freq(void *arg, long period);
static void spi_write();
static void spi_read();
static void spi_transfer();
static void spi_transaction();
static CONTROL parse_ctrl_type(const char *ctrl);
static void update_freq_old(void *arg, long period);

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
	int n, retval;

	// parse stepgen control type
	for (n = 0; n < JOINTS; n++) {
		if(parse_ctrl_type(ctrl_type[n]) == INVALID) {
			rtapi_print_msg(RTAPI_MSG_ERR,
					"STEPGEN: ERROR: bad control type '%s' for axis %i (must be 'p' or 'v')\n",
					ctrl_type[n], n);
			return -1;
		}
    }
	
	// check to see PRU chip type has been set at the command line
	if (!strcmp(chip_type, "LPC") || !strcmp(chip_type, "lpc"))
	{
		rtapi_print_msg(RTAPI_MSG_INFO,"PRU: Chip type set to LPC\n");
		chip = STM;
	}
	else if (!strcmp(chip_type, "STM") || !strcmp(chip_type, "stm"))
	{
		rtapi_print_msg(RTAPI_MSG_INFO,"PRU: Chip type set to STM\n");
		chip = STM;
	}
	else
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: PRU chip type (must be 'LPC' or 'STM')\n");
		return -1;
	}

    // connect to the HAL, initialise the driver
    comp_id = hal_init(modname);
    if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
		return -1;
    }

	// allocate shared memory
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	// Map the RPi BCM2835 peripherals - uses "rtapi_open_as_root" in place of "open"
	if (!rt_bcm2835_init())
    {
      rtapi_print_msg(RTAPI_MSG_ERR,"rt_bcm2835_init failed. Are you running with root privlages??\n");
      return -1;
    }

	// Set the SPI0 pins to the Alt 0 function to enable SPI0 access, setup CS register
	// and clear TX and RX fifos
	if (!bcm2835_spi_begin())
    {
      rtapi_print_msg(RTAPI_MSG_ERR,"bcm2835_spi_begin failed. Are you running with root privlages??\n");
      return -1;
    }

	// Configure SPI0
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default

	//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);		// 3.125MHz on RPI3
	//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);		// 6.250MHz on RPI3
	//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);		// 12.5MHz on RPI3
	//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);		// 25MHz on RPI3

	if (chip == LPC) 
	{
		bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
		rtapi_print_msg(RTAPI_MSG_INFO,"PRU: SPI default clk divider set to 64\n");
	}
	else if (chip == STM) 
	{
		bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
		rtapi_print_msg(RTAPI_MSG_INFO,"PRU: SPI default clk divider set to 16\n");
	}
	
	// check if the default SPI clock divider has been overriden at the command line
	if (SPI_clk_div != -1)
	{
		// check that the setting is a power of 2
		if ((SPI_clk_div & (SPI_clk_div - 1)) == 0)
		{
			bcm2835_spi_setClockDivider(SPI_clk_div);
			rtapi_print_msg(RTAPI_MSG_INFO,"PRU: SPI clk divider overridden and set to %d\n", SPI_clk_div);			
		}
		else
		{
			// it's not a power of 2
			rtapi_print_msg(RTAPI_MSG_ERR,"ERROR: PRU SPI clock divider incorrect\n");
			return -1;
		}	
	}

    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default


	/* RPI_GPIO_P1_19        = 10 		MOSI when SPI0 in use
     * RPI_GPIO_P1_21        =  9 		MISO when SPI0 in use
     * RPI_GPIO_P1_23        = 11 		CLK when SPI0 in use
     * RPI_GPIO_P1_24        =  8 		CE0 when SPI0 in use
     * RPI_GPIO_P1_26        =  7 		CE1 when SPI0 in use
	 */

	// Configure pullups on SPI0 pins - source termination and CS high (does this allows for higher clock frequencies??? wiring is more important here)
	bcm2835_gpio_set_pud(RPI_GPIO_P1_19, BCM2835_GPIO_PUD_DOWN);	// MOSI
	bcm2835_gpio_set_pud(RPI_GPIO_P1_21, BCM2835_GPIO_PUD_DOWN);	// MISO
	bcm2835_gpio_set_pud(RPI_GPIO_P1_24, BCM2835_GPIO_PUD_UP);		// CS0

	// export spiPRU SPI enable and status bits
	retval = hal_pin_bit_newf(HAL_IN, &(data->SPIenable),
			comp_id, "%s.SPI-enable", prefix);
	if (retval != 0) goto error;
	
	retval = hal_pin_bit_newf(HAL_IN, &(data->SPIreset),
			comp_id, "%s.SPI-reset", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_OUT, &(data->SPIstatus),
			comp_id, "%s.SPI-status", prefix);
	if (retval != 0) goto error;

	bcm2835_gpio_fsel(reset_gpio_pin, BCM2835_GPIO_FSEL_OUTP);
	retval = hal_pin_bit_newf(HAL_IN, &(data->PRUreset),
			comp_id, "%s.PRU-reset", prefix);
	if (retval != 0) goto error;


    // export all the variables for each joint
    for (n = 0; n < JOINTS; n++) {
		// export pins

		data->pos_mode[n] = (parse_ctrl_type(ctrl_type[n]) == POSITION);

		retval = hal_pin_bit_newf(HAL_IN, &(data->stepperEnable[n]),
				comp_id, "%s.joint.%01d.enable", prefix, n);
		if (retval != 0) goto error;

		retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
				comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_cmd[n]) = 0.0;
		
		if (data->pos_mode[n] == 0){
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) goto error;
			*(data->vel_cmd[n]) = 0.0;			
		}

		retval = hal_pin_s32_newf(HAL_IN, &(data -> home_position_degrees[n]),
		        comp_id, "%s.joint.%01d.home-degs", prefix, n);
		if (retval < 0) goto error;
				
		retval = hal_pin_float_newf(HAL_OUT, &(data->freq_cmd[n]),
		        comp_id, "%s.joint.%01d.freq-cmd", prefix, n);
		if (retval < 0) goto error;
		*(data->freq_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->pos_fb[n]),
		        comp_id, "%s.joint.%01d.pos-fb", prefix, n);
		if (retval < 0) goto error;
		*(data->pos_fb[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->pos_scale[n]),
		        comp_id, "%s.joint.%01d.scale", prefix, n);
		if (retval < 0) goto error;
		data->pos_scale[n] = 1.0;

		retval = hal_pin_s32_newf(HAL_OUT, &(data->count[n]),
		        comp_id, "%s.joint.%01d.counts", prefix, n);
		if (retval < 0) goto error;
		*(data->count[n]) = 0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->pgain[n]),
				comp_id, "%s.joint.%01d.pgain", prefix, n);
		if (retval < 0) goto error;
		*(data->pgain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->ff1gain[n]),
				comp_id, "%s.joint.%01d.ff1gain", prefix, n);
		if (retval < 0) goto error;
		*(data->ff1gain[n]) = 0.0;
		
		retval = hal_pin_float_newf(HAL_IN, &(data->deadband[n]),
				comp_id, "%s.joint.%01d.deadband", prefix, n);
		if (retval < 0) goto error;
		*(data->deadband[n]) = 0.0;
		
		retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
		        comp_id, "%s.joint.%01d.maxaccel", prefix, n);
		if (retval < 0) goto error;
		data->maxaccel[n] = 1.0;

		

	}

	retval = hal_pin_bit_newf(HAL_OUT, &(data->pose_at_enable),
				comp_id, "%s.enable-pose.%01d", prefix, 0);
		if (retval != 0) goto error;
		*(data->pose_at_enable)=0;

	for (n = 0; n < VARIABLES; n++) {
	// export pins

		retval = hal_pin_float_newf(HAL_IN, &(data->setPoint[n]),
		        comp_id, "%s.SP.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->setPoint[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->processVariable[n]),
		        comp_id, "%s.PV.%01d", prefix, n);
		if (retval < 0) goto error;
		*(data->processVariable[n]) = 0.0;
	}

	for (n = 0; n < DIGITAL_OUTPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_IN, &(data->outputs[n]),
				comp_id, "%s.output.%01d", prefix, n);
		if (retval != 0) goto error;
		*(data->outputs[n])=0;
	}

	for (n = 0; n < DIGITAL_INPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n]),
				comp_id, "%s.input.%01d", prefix, n);
		if (retval != 0) goto error;
		*(data->inputs[n])=0;
	}

	error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
		        modname, retval);
		hal_exit(comp_id);
		return -1;
	}

	// Export functions
	rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
	/* no FP operations */
	retval = hal_export_funct(name, spi_write, 0, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: write function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, spi_read, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}


/***********************************************************************
*                   LOCAL FUNCTION DEFINITIONS                         *
************************************************************************/


// This is the same as the standard bcm2835 library except for the use of
// "rtapi_open_as_root" in place of "open"

int rt_bcm2835_init(void)
{
    int  memfd;
    int  ok;
    FILE *fp;

    if (debug) 
    {
        bcm2835_peripherals = (uint32_t*)BCM2835_PERI_BASE;

	bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
	bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
	bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
	bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
	bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
	bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4;
	bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4;
	bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
	bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
	bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;

	return 1; /* Success */
    }

    /* Figure out the base and size of the peripheral address block
    // using the device-tree. Required for RPi2/3/4, optional for RPi 1
    */
    if ((fp = fopen(BMC2835_RPI2_DT_FILENAME , "rb")))
    {
        unsigned char buf[16];
        uint32_t base_address;
        uint32_t peri_size;
        if (fread(buf, 1, sizeof(buf), fp) >= 8)
        {
            base_address = (buf[4] << 24) |
              (buf[5] << 16) |
              (buf[6] << 8) |
              (buf[7] << 0);
            
            peri_size = (buf[8] << 24) |
              (buf[9] << 16) |
              (buf[10] << 8) |
              (buf[11] << 0);
            
            if (!base_address)
            {
                /* looks like RPI 4 */
                base_address = (buf[8] << 24) |
                      (buf[9] << 16) |
                      (buf[10] << 8) |
                      (buf[11] << 0);
                      
                peri_size = (buf[12] << 24) |
                (buf[13] << 16) |
                (buf[14] << 8) |
                (buf[15] << 0);
            }
            /* check for valid known range formats */
            if ((buf[0] == 0x7e) &&
                    (buf[1] == 0x00) &&
                    (buf[2] == 0x00) &&
                    (buf[3] == 0x00) &&
                    ((base_address == BCM2835_PERI_BASE) || (base_address == BCM2835_RPI2_PERI_BASE) || (base_address == BCM2835_RPI4_PERI_BASE)))
            {
                bcm2835_peripherals_base = (off_t)base_address;
                bcm2835_peripherals_size = (size_t)peri_size;
                if( base_address == BCM2835_RPI4_PERI_BASE )
                {
                    pud_type_rpi4 = 1;
                }
            }
        
        }
        
	fclose(fp);
    }
    /* else we are prob on RPi 1 with BCM2835, and use the hardwired defaults */

    /* Now get ready to map the peripherals block 
     * If we are not root, try for the new /dev/gpiomem interface and accept
     * the fact that we can only access GPIO
     * else try for the /dev/mem interface and get access to everything
     */
    memfd = -1;
    ok = 0;
    if (geteuid() == 0)
    {
      /* Open the master /dev/mem device */
      if ((memfd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC) ) < 0) 
	{
	  fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n",
		  strerror(errno)) ;
	  goto exit;
	}
      
      /* Base of the peripherals block is mapped to VM */
      bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
      if (bcm2835_peripherals == MAP_FAILED) goto exit;
      
      /* Now compute the base addresses of various peripherals, 
      // which are at fixed offsets within the mapped peripherals block
      // Caution: bcm2835_peripherals is uint32_t*, so divide offsets by 4
      */
      bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
      bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
      bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
      bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
      bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
      bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4; /* I2C */
      bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4; /* I2C */
      bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
      bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
      bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;

      ok = 1;
    }
    else
    {
      /* Not root, try /dev/gpiomem */
      /* Open the master /dev/mem device */
      if ((memfd = open("/dev/gpiomem", O_RDWR | O_SYNC) ) < 0) 
	{
	  fprintf(stderr, "bcm2835_init: Unable to open /dev/gpiomem: %s\n",
		  strerror(errno)) ;
	  goto exit;
	}
      
      /* Base of the peripherals block is mapped to VM */
      bcm2835_peripherals_base = 0;
      bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
      if (bcm2835_peripherals == MAP_FAILED) goto exit;
      bcm2835_gpio = bcm2835_peripherals;
      ok = 1;
    }

exit:
    if (memfd >= 0)
        close(memfd);

    if (!ok)
	bcm2835_close();

    return ok;
}

void spi_read()
{
	
	int i;
	double curr_pos;
	hal_float_t non_corrected_angle;

	*(data->SPIstatus) = 1;
	// update the PRUreset output
	spi_transfer(); //read dummy buffer from stm32

	spi_transfer(); //Actual read

	for (i = 0; i < JOINTS-1; i++){
		non_corrected_angle = (float)(rxData.jointFeedback[i] / data->pos_scale[i])- *(data-> home_position_degrees[i]);
		correction[i] = non_corrected_angle/360;
		
			*(data->pos_fb[i]) =  non_corrected_angle - (correction[i]) * 360;
			if (*(data->pos_fb[i])>180){
				*(data->pos_fb[i])= (360-(*(data->pos_fb[i])))*-1;
				correction[i]+= 1;
			}
		
		//*(data->pos_fb[i]) = (float)(rxData.jointFeedback[i] / data->pos_scale[i]);
	}
	*(data->pos_fb[1])= -1* (*(data->pos_fb[1]));
	*(data->pos_fb[2])= (float)((rxData.jointFeedback[2])/(data->pos_scale[2]));
//THE FOLLOWING LINES SHOULD BE COMMENTED OUT IF THE Iflag is Set in Kinematics

	if (*(data -> stepperEnable[0]) != old_enable_status){

		if(*(data ->pos_fb[1]) < 0){
			*(data ->pose_at_enable) =1;
		}
		else{
			*(data ->pose_at_enable) =0;
		}

		old_enable_status = *(data ->stepperEnable[0]);
	}
	
	// Read Digital inputs at end of packet
	*(data->inputs[0]) = rxData.digital_inputs ;

}

void spi_write()
{
int i;

// Data header
txData.header = PRU_GENERIC;

if (PACKETCOUNTER <= 0x0F){
	PACKETCOUNTER = PACKETCOUNTER++;
}
else{
	PACKETCOUNTER = 0x00;
}

txData.header = PRU_WRITE || PACKETCOUNTER;
*(data ->pos_cmd[1]) = -1 * (*(data -> pos_cmd[1]));
// Joint frequency commands
for (i = 0; i < JOINTS-1; i++)
{	
	txData.jointPosCmd[i] = (uint32_t)((*(data->pos_cmd[i])+ (correction[i])*360 + *(data ->home_position_degrees[i])) * data->pos_scale[i]);
	//txData.jointPosCmd[i] = (uint32_t)(*(data->pos_cmd[i])* data->pos_scale[i]);
}
txData.jointPosCmd[2] = (uint32_t)((*(data->pos_cmd[2])) * data->pos_scale[2]);

for (i = 0; i < JOINTS; i++)
{
	if (*(data->stepperEnable[i]) == 1)
	{
		txData.jointEnable |= (1 << i);		
	}
	else
	{
		txData.jointEnable &= ~(1 << i);	
	}

}

spi_transfer();

}


void spi_transfer()
{
// send and receive data to and from the Remora PRU concurrently
	bcm2835_spi_transfernb(txData.txBuffer, rxData.rxBuffer, SPIBUFSIZE);
}


static CONTROL parse_ctrl_type(const char *ctrl)
{
if(!ctrl || !*ctrl || *ctrl == 'p' || *ctrl == 'P') return POSITION;
if(*ctrl == 'v' || *ctrl == 'V') return VELOCITY;
return INVALID;
}
