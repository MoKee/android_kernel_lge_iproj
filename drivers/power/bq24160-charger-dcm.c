/* 
 *  TI BQ24160 Charger Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#define DEBUG
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/bq24160-charger.h>
#include <linux/power_supply.h>
#include <linux/mfd/pm8xxx/batt-alarm.h> 

#ifdef CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY
#include <mach/gpio.h>
#include <linux/msm-charger.h>
#include <linux/msm_adc.h>
#include <linux/wakelock.h>
#endif

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#include "../../lge/include/lg_power_common.h"
#endif



#define BQ24160_CABLE_PWRON_INT
#define BQ24160_WORK_AROUND_CODE
#define BQ24160_LOCAL_WQ

// define register map
#define BQ24160_REG_STAT_CTRL 		0x00		/* Status / Control Register (read/write) */

#ifdef BQ24160_WORK_AROUND_CODE
#define BQ24160_WDOG_TMR_MASK		0x88
#else
#define BQ24160_WDOG_TMR_MASK		0x80
#endif

#define BQ24160_WDOG_TMR_SHFT		7
#define BQ24160_STAT_MASK			0x70
#define BQ24160_STAT_SHFT			4
#define BQ24160_SUPPLY_SEL_MASK		0x08
#define BQ24160_SUPPLY_SEL_SHFT		3
#define BQ24160_FAULT_MASK			0x07
#define BQ24160_FAULT_SHFT			0

#define BQ24160_REG_BATTNPS_STAT	0x01		/* Battery / Supply Status Register (read/write) */

#define BQ24160_INSTAT_MASK			0xC0
#define BQ24160_INSTAT_SHFT			6
#define BQ24160_USBSTAT_MASK		0x30
#define BQ24160_USBSTAT_SHFT		4
#define BQ24160_OTGLOCK_MASK		0x08
#define BQ24160_OTGLOCK_SHFT		3
#define BQ24160_BATTSTAT_MASK		0x06
#define BQ24160_BATTSTAT_SHFT		1

#define BQ24160_REG_CTRL			0x02		/* Control Register (read/write) */

#define BQ24160_RESET_MASK			0x80
#define BQ24160_RESET_SHFT			7
#define BQ24160_IUSB_LMT_MASK		0x70
#define BQ24160_IUSB_LMT_SHFT		4
#define BQ24160_ENSTAT_MASK			0x08
#define BQ24160_ENSTAT_SHFT			3
#define BQ24160_TE_MASK				0x04
#define BQ24160_TE_SHFT				2
#define BQ24160_CE_MASK				0x02
#define BQ24160_CE_SHFT				1
#define BQ24160_HZMODE_MASK			0x01
#define BQ24160_HZMODE_SHFT			0

#define BQ24160_REG_CTRL_BATVOLT	0x03		/* Control / Battery Voltage Register (read/write) */

#if !defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY_4P2V)
#define BQ24160_VBATTREG_MASK		0xAE
#else
#define BQ24160_VBATTREG_MASK		0x8E
#endif
#define BQ24160_VBATTREG_SHFT		2
#define BQ24160_INLMT_IN_MASK		0x02
#define BQ24160_INLMT_IN_SHFT		1
#define BQ24160_DPDM_EN_MASK		0x01
#define BQ24160_DPDM_EN_SHFT		0

#define BQ24160_REG_VEND_PART_REV	0x04		/* Vendor / Part / Revision (read only) */

#define BQ24160_VENDOR_MASK			0xE0
#define BQ24160_VENDOR_SHFT			5
#define BQ24160_PN_MASK				0x18
#define BQ24160_PN_SHFT				3
#define BQ24160_REV_MASK			0x07
#define BQ24160_REV_SHFT			0

#define BQ24160_REG_BATTTERM_FCHGCRNT	0x05	/* Battery Termination / Fast Charge Current (read/write) */

#define BQ24160_ICHGCRNT_MASK		0xF8
#define BQ24160_ICHGCRNT_SHFT		3
#define BQ24160_ITERMCRNT_MASK		0x07
#define BQ24160_ITERMCRNT_SHFT		0

#define BQ24160_REG_VINDPM_STAT	0x06			/* Vin-dpm Voltage / DPPM Status */

#define BQ24160_MINSYS_STAT_MASK	0x80
#define BQ24160_MINSYS_STAT_SHFT	7
#define BQ24160_DPM_STAT_MASK		0x40
#define BQ24160_DPM_STAT_SHFT		6
#define BQ24160_USB_INDPM_MASK		0x38
#define BQ24160_USB_INDPM_SHFT		3
#define BQ24160_IN_INDPM_MASK		0x07
#define BQ24160_IN_INDPM_SHFT		0

#define BQ24160_REG_SAFETMR_NTCMON		0x07	/* Safety Timer / NTC Monitor (read/write) */

#define BQ24160_2XTMR_EN_MASK		0x80
#define BQ24160_2XTMR_EN_SHFT		7
#define BQ24160_TMR_MASK			0x60
#define BQ24160_TMR_SHFT			5
#define BQ24160_TS_EN_MASK			0x08
#define BQ24160_TS_EN_SHFT			3
#define BQ24160_TS_FAULT_MASK		0x06
#define BQ24160_TS_FAULT_SHFT		1

#define BQ24160_STAT_NO_VALID_SRC_DETECTED		0
#define BQ24160_STAT_IN_READY 					1
#define BQ24160_STAT_USB_READY 					2
#define BQ24160_STAT_CHARGING_FROM_IN 			3
#define BQ24160_STAT_CHARGING_FROM_USB 			4
#define BQ24160_STAT_CHARGE_DONE 				5
#define BQ24160_STAT_NA 						6
#define BQ24160_STAT_FAULT						7

#define BQ24160_CHG_WORK_PERIOD	 ((HZ) * 1)
#define BQ24160_CHG_TEMP_SENARIO ((HZ) * 1)
#define BQ24160_CHG_WATCH_DOG_RESET_PERIOD ((HZ) * 20)

#define WIRELESS_CHARGE_STATUS	71
#define WIRELESS_CHARGE_COMPLETE	141

#define SWITCHING_CHG_IRQ_N	124


#define BQ24160_ICHG_CURRENT_450    450
#define BQ24160_ICHG_CURRENT_500    500
#define BQ24160_ICHG_CURRENT_600    600
#define BQ24160_ICHG_CURRENT_800    800
#define BQ24160_ICHG_CURRENT_1500   1500


enum
{
  BQ24160_CHG_STATE_NO_CABLE = 0,
  BQ24160_CHG_STATE_CHARGING,
  BQ24160_CHG_STATE_DONE,
  BQ24160_CHG_STATE_STOP,
};


enum
{
  BQ24160_CHG_USB = 0,
  BQ24160_CHG_TA
};


enum
{
  BQ24160_CHG_STATE_1 = 0,
  BQ24160_CHG_STATE_2,
  BQ24160_CHG_STATE_3,
  BQ24160_CHG_STATE_4,
  BQ24160_CHG_STATE_5,
  BQ24160_CHG_STATE_6,
  BQ24160_CHG_STATE_7,
  BQ24160_CHG_STATE_8,
  BQ24160_CHG_STATE_9,
  BQ24160_CHG_STATE_UNKNOWN
};

struct bq24160_chip {
	struct i2c_client *client;
	struct delayed_work charge_work;
  struct delayed_work watch_dog_reset;
  struct delayed_work charge_start_from_temp_senario;
  struct delayed_work charge_stop_from_temp_senario;
#ifdef BQ24160_HZMODE_CTRL
	struct delayed_work hzmode_ctrl_work;
#endif	
	struct power_supply charger;
	struct bq24160_platform_data *pdata;
    int irq;
    int chg_online;
#ifdef CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY
	struct msm_hardware_charger	adapter_hw_chg;
#endif
};


static bool at_cmd_chg = false;
static bool resume_chg_start = false;

#ifdef BQ24160_WORK_AROUND_CODE
bool chg_complete = false;
bool chg_resume = false;
bool bq24160_chg_done_once = false;
static bool chg_over_heat_detect = false;
static bool chg_stop_temp_detect = false;
#endif

static int ichg_current = 0;
static int batt_high_temp = 5;
static int old_chg = -1;
static int chg_step_now = 10;
static int chg_step_prev = 10;
static int polling_time_prev_voltage = -1;
static int prev_chg_state = -1;


int batt_volt = 0;
int batt_temp = 1000;
int fuel_percent = 0;
int now_chg_state = 0;

extern int pre_soc;
extern int pre_volt;




extern uint16_t battery_info_get(void);
extern int usb_cable_info;
extern int usb_chg_type;

extern const struct adc_map_pt adcmap_batttherm[THERM_LAST];

#ifdef BQ24160_LOCAL_WQ
struct mutex bq24160_status_lock;
static struct workqueue_struct *local_workqueue;
#endif


#ifdef CONFIG_LGE_PM_FACTORY_CURRENT_DOWN
extern uint16_t battery_info_get(void);
extern int usb_cable_info;
#endif

extern unsigned msm_otg_get_chg_current(void);
extern int pm8058_get_battery_temperature_adc(void);
//extern int get_battery_temperature_adc(void);

#ifdef CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY
static void bq24160_irq_init(int gpio_num)
{
	int rc;
  
	rc = gpio_request(gpio_num, "bq24160_irq_gpio");
	if(rc < 0){
		return ;
	}
	gpio_direction_input(gpio_num);
	gpio_set_value(gpio_num, 1);
}
#endif


static int bq24160_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int bq24160_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}


void bq24160_set_chg_current(int ichg)
{
  ichg_current = ichg;
}
EXPORT_SYMBOL(bq24160_set_chg_current);


static void bq24160_start_chg_from_temp_senario(struct work_struct *bq24160_work)
{
  struct bq24160_chip *bq24160_chg;


	bq24160_chg = container_of(bq24160_work, struct bq24160_chip,
			charge_start_from_temp_senario.work);

#ifdef BQ24160_LOCAL_WQ
  queue_delayed_work(local_workqueue, &bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
  queue_delayed_work(local_workqueue, &bq24160_chg->charge_work, BQ24160_CHG_WORK_PERIOD);
#else
  schedule_delayed_work(&bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
  schedule_delayed_work(&bq24160_chg->charge_work, BQ24160_CHG_WORK_PERIOD);
#endif
}


static void bq24160_stop_chg_from_temp_senario(struct work_struct *bq24160_work)
{
  struct bq24160_chip *bq24160_chg;

  bq24160_chg = container_of(bq24160_work, struct bq24160_chip,
			charge_stop_from_temp_senario.work);
    
  /* Watchdog timer reset */
	if((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
  {
    bq24160_write_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL, 0xB8);
  }

  /* REG_#2  Set current limit and enable STAT in Control register */
  if(usb_chg_type == 3)
  {
    bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
    bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2C);
  }
  else
  {
    bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
    bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3C);
  }
}


void bq24160_determine_the_collect_chg(int start)
{
  batt_high_temp = start;
}
EXPORT_SYMBOL(bq24160_determine_the_collect_chg);


bool bq24160_chg_plug_in(void)
{
  if(now_chg_state == BQ24160_CHG_STATE_NO_CABLE || now_chg_state == BQ24160_CHG_STATE_STOP)
    return false;
  else
    return true;
}
EXPORT_SYMBOL(bq24160_chg_plug_in);


static void bq24160_usb_chg_start_reg_setting(struct bq24160_chip *chip)
{
  /* REG_#2  Set current limit and enable STAT in Control register */
  bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x29);  // USB current limit 500mA, Enable hi-z mode

  /* REG_#3  Set Battery Regulation Voltage */
  bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK);  // VBAT : 4.36[V]

  /* REG_#0 */
	bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x88);  // USB input select

  /* REG_#1 */
  bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
		
  /* REG_#5  Set Battery Termination and Fast Chg current */
  bq24160_write_reg(chip->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);  //Charging current : 850[mA], Termination current : 150[mA]

  /* REG_#6  Set Vin-DPM voltage */
  bq24160_write_reg(chip->client, BQ24160_REG_VINDPM_STAT, 0xD2);

	/* REG_#7  Set timer and monitor */
  bq24160_write_reg(chip->client, BQ24160_REG_SAFETMR_NTCMON, 0xA0);  // Charging current: 425mA

  /* REG_#2  Set current limit and enable STAT in Control register */
	bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x28); // Disable Hi-z mode
}



static void bq24160_ta_chg_start_reg_setting(struct bq24160_chip *chip)
{
  /* REG_#2  Set current limit and enable STAT in Control register */
	bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x39);  // USB current limit 500mA, Enable hi-z mode

  /* REG_#3  Set Battery Regulation Voltage */
  bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK);  // VBAT : 4.36[V]

	/* REG_#0 */
	bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x88);  // USB input select

  /* REG_#1 */
  bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
		
	/* REG_#5  Set Battery Termination and Fast Chg current */
	bq24160_write_reg(chip->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x11);  //Charging current : 850[mA], Termination current : 150[mA]

	/* REG_#6  Set Vin-DPM voltage */
	bq24160_write_reg(chip->client, BQ24160_REG_VINDPM_STAT, 0xD2);

  /* REG_#7  Set timer and monitor */
	bq24160_write_reg(chip->client, BQ24160_REG_SAFETMR_NTCMON, 0xA0);

  /* REG_#2  Set current limit and enable STAT in Control register */
	bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x38); // Disable Hi-z mode
}


static void bq24160_pif_chg_start_reg_setting(struct bq24160_chip *chip)
{
  /* REG_#0 */
  bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x00);  // USB input select
    
  /* REG_#1 */
  bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
    
  /* REG_#2  Set current limit and enable STAT in Control register */
  bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x08); // Disable Hi-z mode

  /* REG_#3  Set Battery Regulation Voltage */
	bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, 0x66);  // VBAT : 4.36[V]
}


static int bq24160_chg_refresh_reg_normal(struct bq24160_chip *chip, int enable)
{    
	if (enable) {
    if(usb_chg_type == 3 && (0 != battery_info_get()))
    {  
      /* REG_#3  Set Battery Regulation Voltage */
		  bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK);  // VBAT : 4.36[V]

		  /* REG_#0 */
		  bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x88);  // USB input select

      /* REG_#1 */
      bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
		
		  /* REG_#5  Set Battery Termination and Fast Chg current */
		  bq24160_write_reg(chip->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);  //Charging current : 850[mA], Termination current : 150[mA]

		  /* REG_#6  Set Vin-DPM voltage */
		  bq24160_write_reg(chip->client, BQ24160_REG_VINDPM_STAT, 0xD2);

		  /* REG_#7  Set timer and monitor */
		  bq24160_write_reg(chip->client, BQ24160_REG_SAFETMR_NTCMON, 0xA0);  // Charging current: 425mA

      /* REG_#2  Set current limit and enable STAT in Control register */
		  bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x28); // Disable Hi-z mode
    }
    else if(usb_chg_type == 2 && (0 != battery_info_get()))
    {          
      /* REG_#3  Set Battery Regulation Voltage */
		  bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK);  // VBAT : 4.36[V]

		  /* REG_#0 */
		  bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x88);  // USB input select

      /* REG_#1 */
      bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
		
		  /* REG_#5  Set Battery Termination and Fast Chg current */
		  bq24160_write_reg(chip->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x11);  //Charging current : 850[mA], Termination current : 150[mA]

		  /* REG_#6  Set Vin-DPM voltage */
		  bq24160_write_reg(chip->client, BQ24160_REG_VINDPM_STAT, 0xD2);

      /* REG_#7  Set timer and monitor */
		  bq24160_write_reg(chip->client, BQ24160_REG_SAFETMR_NTCMON, 0xA0);

      /* REG_#2  Set current limit and enable STAT in Control register */
		  bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x38); // Disable Hi-z mode
    }
    else
    {
      /* REG_#0 */
      bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x00);  // USB input select
    
      /* REG_#1 */
      bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
    
      /* REG_#2  Set current limit and enable STAT in Control register */
      bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x08); // Disable Hi-z mode

      /* REG_#3  Set Battery Regulation Voltage */
	    bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, 0x66);  // VBAT : 4.36[V]
    }
	} 
  else {
		/* disable charge */
	}
	dev_dbg(&chip->client->dev, "%s %s\n", __func__, (enable) ? "Enable charger" : "Disable charger");
	return 0;
}



static void bq24160_chg_refresh_reg_temp_senario(struct bq24160_chip *chip)
{
  if(usb_chg_type == 3 && (0 != battery_info_get()))
  {  
    /* REG_#3  Set Battery Regulation Voltage */
		bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK);  // VBAT : 4.36[V]

		/* REG_#0 */
		bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x88);  // USB input select

    /* REG_#1 */
    bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
		
		/* REG_#5  Set Battery Termination and Fast Chg current */
		bq24160_write_reg(chip->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);

		/* REG_#6  Set Vin-DPM voltage */
		bq24160_write_reg(chip->client, BQ24160_REG_VINDPM_STAT, 0xD2);

		/* REG_#7  Set timer and monitor */
		bq24160_write_reg(chip->client, BQ24160_REG_SAFETMR_NTCMON, 0xA0);  // Charging current: 425mA

    /* REG_#2  Set current limit and enable STAT in Control register */
		bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x28); // Disable Hi-z mode
  }
  else if(usb_chg_type == 2 && (0 != battery_info_get()))
  {    
    /* REG_#2  Set current limit and enable STAT in Control register */
		bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x29); // Disable Hi-z mode
		
    /* REG_#3  Set Battery Regulation Voltage */
	  bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK);  // VBAT : 4.36[V]

		/* REG_#0 */
		bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x88);  // USB input select

    /* REG_#1 */
    bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
		
		/* REG_#5  Set Battery Termination and Fast Chg current */
		bq24160_write_reg(chip->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);

		/* REG_#6  Set Vin-DPM voltage */
		bq24160_write_reg(chip->client, BQ24160_REG_VINDPM_STAT, 0xD2);

    /* REG_#7  Set timer and monitor */
		bq24160_write_reg(chip->client, BQ24160_REG_SAFETMR_NTCMON, 0xA0);

    /* REG_#2  Set current limit and enable STAT in Control register */
		bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x28); // Disable Hi-z mode
  }
  else
  {    
    /* REG_#0 */
    bq24160_write_reg(chip->client, BQ24160_REG_STAT_CTRL, 0x00);  // USB input select
    
    /* REG_#1 */
    bq24160_write_reg(chip->client, BQ24160_REG_BATTNPS_STAT, 0x01); // Enable no battery operation to disable boost back circuit
    
    /* REG_#2  Set current limit and enable STAT in Control register */
    bq24160_write_reg(chip->client, BQ24160_REG_CTRL, 0x08); // Disable Hi-z mode

    /* REG_#3  Set Battery Regulation Voltage */
	  bq24160_write_reg(chip->client, BQ24160_REG_CTRL_BATVOLT, 0x66);  // VBAT : 4.36[V]
  }
}


static void bq24160_chg_get_batt_temp(void)
{
  int temp_adc;

  temp_adc = batt_temp; //get_battery_temperature_adc();

  if(temp_adc < adcmap_batttherm[THERM_55].x)
    batt_high_temp = CHG_BATT_TEMP_OVER_55;
  else if(temp_adc < adcmap_batttherm[THERM_45].x)
    batt_high_temp = CHG_BATT_TEMP_46_55;
  else if(temp_adc <= adcmap_batttherm[THERM_42].x)
    batt_high_temp = CHG_BATT_TEMP_42_45;
  else if(temp_adc < adcmap_batttherm[THERM_M5].x)
    batt_high_temp = CHG_BATT_TEMP_M4_41;
  else if(temp_adc <= adcmap_batttherm[THERM_M10].x)
    batt_high_temp = CHG_BATT_TEMP_M10_M5;
  else
    batt_high_temp = CHG_BATT_TEMP_UNDER_M10;

  printk(KERN_DEBUG "[bq24160_chg_get_batt_temp]: TEMP ADC = %d TEMP = %d \n", temp_adc, batt_high_temp);
}


static void bq24160_charge(struct work_struct *bq24160_work)
{
  u8 temp = 0;
	u8 status = 0;
  
  //int rc = 0;
  int now_chg = -1;
  int mv = 0;
  int ret = -1;
  
  struct bq24160_chip *bq24160_chg;


	bq24160_chg = container_of(bq24160_work, struct bq24160_chip,
			charge_work.work);

	dev_dbg(&bq24160_chg->client->dev, "%s\n", __func__);

  if(((now_chg_state == BQ24160_CHG_STATE_CHARGING) || (now_chg_state == BQ24160_CHG_STATE_DONE)) && \
    ((status != BQ24160_STAT_NO_VALID_SRC_DETECTED) && (status != BQ24160_STAT_NA) && (status != BQ24160_STAT_FAULT)))
    return;

	/* Watchdog timer reset */
	if((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
  {
    ret = bq24160_write_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL, 0xB8);
    if(ret < 0)
      return;
  }

  temp = bq24160_read_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL);
  dev_dbg(&bq24160_chg->client->dev, "%s STATUS Register[#0]: 0x%x\n", __func__,temp);

  /*********************************************************************************************/
  /*                                                         HIGH TEMPERATURE CHECK                                                              */
  /*********************************************************************************************/
  mv = pre_volt/*batt_volt*/;
  
  /* Stop Charging Condition Check for BATTERY */
  if(batt_high_temp == CHG_BATT_TEMP_OVER_55 || batt_high_temp == CHG_BATT_TEMP_UNDER_M10 || \
    ((batt_high_temp == CHG_BATT_TEMP_46_55) && (mv > 4000)))
  {
    return;
  }

  status = temp & BQ24160_STAT_MASK;
	status = status >> BQ24160_STAT_SHFT;

  if(((status != BQ24160_STAT_NO_VALID_SRC_DETECTED) && (status != BQ24160_STAT_NA) && (status != BQ24160_STAT_FAULT)) && \
    (chg_complete || chg_resume))
  {
    return;
  }

  if(prev_chg_state == status)
  {
    return;
  }

  printk(KERN_DEBUG "[bq24160_charge]: CHARGE STATUS = %d Current = %d Cable = %d \n", status, usb_chg_type, usb_cable_info);

	switch(status)
	{
		case BQ24160_STAT_NO_VALID_SRC_DETECTED:
      now_chg_state = BQ24160_CHG_STATE_NO_CABLE;
      usb_chg_type = -1;
      old_chg = -1;
      polling_time_prev_voltage = -1;
      chg_resume = false;
      chg_complete = false;
      
      //rc = pm8058_batt_alarm_state_set(1, 0);
      cancel_delayed_work_sync(&bq24160_chg->watch_dog_reset);
      bq24160_chg->chg_online = 0;
      msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_REMOVED_EVENT);
			break;

		case BQ24160_STAT_IN_READY:
      if(usb_chg_type != 0)
      {
        now_chg_state = BQ24160_CHG_STATE_CHARGING;
        bq24160_chg_done_once = false;
      
        now_chg = usb_chg_type;

        if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
        {
          if(now_chg != old_chg)
          {
            bq24160_pif_chg_start_reg_setting(bq24160_chg);
          }
        }
        else
        {
          if(now_chg == 3)
          {
            if(now_chg != old_chg)
            {
              printk(KERN_DEBUG "[bq24160_charge]: USB \n");
              bq24160_usb_chg_start_reg_setting(bq24160_chg);
            }
          }
          else if(now_chg == 2)
          {
            if(now_chg != old_chg)
            {
              printk(KERN_DEBUG "[bq24160_charge]: TA \n");
              bq24160_ta_chg_start_reg_setting(bq24160_chg);
            }
          }
        }
      
        old_chg = now_chg;

        //rc = pm8058_batt_alarm_state_set(0, 0);
        bq24160_chg->chg_online = 1;
        msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      }
      break;
      
		case BQ24160_STAT_USB_READY:
      if(usb_chg_type != 0)
      {
        now_chg_state = BQ24160_CHG_STATE_CHARGING;
        bq24160_chg_done_once = false;

        now_chg = usb_chg_type;

        if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
        {
          if(now_chg != old_chg)
          {
            bq24160_pif_chg_start_reg_setting(bq24160_chg);
          }
        }
        else
        {
          if(now_chg == 3)
          {
            if(now_chg != old_chg)
            {
              printk(KERN_DEBUG "[bq24160_charge]: USB \n");
              bq24160_usb_chg_start_reg_setting(bq24160_chg);
            }
          }
          else
          {
            if(now_chg != old_chg)
            {
              printk(KERN_DEBUG "[bq24160_charge]: TA \n");
              bq24160_ta_chg_start_reg_setting(bq24160_chg);
            }
          }
        }

        old_chg = now_chg;

        //rc = pm8058_batt_alarm_state_set(0, 0);
        bq24160_chg->chg_online = 1;
        msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      }
			break;

		case BQ24160_STAT_CHARGING_FROM_IN:
      now_chg_state = BQ24160_CHG_STATE_CHARGING;
      bq24160_chg_done_once = false;

      now_chg = usb_chg_type;

      if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
      {
        if(now_chg != old_chg)
        {
          bq24160_pif_chg_start_reg_setting(bq24160_chg);
        }
      }
      else
      {
        if(now_chg == 3)
        {
          if(now_chg != old_chg)
          {
            printk(KERN_DEBUG "[bq24160_charge]: USB \n");
            bq24160_usb_chg_start_reg_setting(bq24160_chg);
          }
        }
        else
        {
          if(now_chg != old_chg)
          {
            printk(KERN_DEBUG "[bq24160_charge]: TA \n");
            bq24160_ta_chg_start_reg_setting(bq24160_chg);
          }
        }
      }

      old_chg = now_chg;

      //rc = pm8058_batt_alarm_state_set(0, 0);
      bq24160_chg->chg_online = 1;
      msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      break;
     
		case BQ24160_STAT_CHARGING_FROM_USB:
      now_chg_state = BQ24160_CHG_STATE_CHARGING;
      bq24160_chg_done_once = false;

      now_chg = usb_chg_type;
      
      if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
      {
        if(now_chg != old_chg)
        {
          bq24160_pif_chg_start_reg_setting(bq24160_chg);
        }
      }
      else
      {
        if(now_chg == 3)
        {
          if(now_chg != old_chg)
          {
            printk(KERN_DEBUG "[bq24160_charge]: USB \n");
            bq24160_usb_chg_start_reg_setting(bq24160_chg);
          }
        }
        else
        {
          if(now_chg != old_chg)
          {
            printk(KERN_DEBUG "[bq24160_charge]: TA \n");
            bq24160_ta_chg_start_reg_setting(bq24160_chg);
          }
        }
      }
      

      old_chg = now_chg;

      //rc = pm8058_batt_alarm_state_set(0, 0);
      bq24160_chg->chg_online = 1;
      msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
			break;
      
		case BQ24160_STAT_CHARGE_DONE:
      bq24160_chg->chg_online = 1;
      //rc = pm8058_batt_alarm_state_set(1, 0);
			break;
      
		case BQ24160_STAT_NA:
      now_chg_state = BQ24160_CHG_STATE_STOP;
      usb_chg_type = -1;
      old_chg = -1;
      polling_time_prev_voltage = -1;
      chg_resume = false;
      chg_complete = false;
      bq24160_chg_done_once = false;
      
      cancel_delayed_work_sync(&bq24160_chg->watch_dog_reset);
      bq24160_chg->chg_online = 0;
      msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_REMOVED_EVENT);
			break;
      
		case BQ24160_STAT_FAULT:
      usb_chg_type = -1;
      old_chg = -1;
      polling_time_prev_voltage = -1;
      chg_resume = false;
      chg_complete = false;
      bq24160_chg_done_once = false;
      
      if((0 != battery_info_get()) && ((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11)))
      {
        now_chg_state = BQ24160_CHG_STATE_STOP;
      
        cancel_delayed_work_sync(&bq24160_chg->watch_dog_reset);
        bq24160_chg->chg_online = 0;
        msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_REMOVED_EVENT);
      }
			break;
	}
}

static irqreturn_t bq24160_valid_handler(int irq, void *data)
{
  struct bq24160_chip *chip = (struct bq24160_chip *)data;

#ifdef BQ24160_LOCAL_WQ
  queue_delayed_work(local_workqueue, &chip->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
  queue_delayed_work(local_workqueue, &chip->charge_work, BQ24160_CHG_WORK_PERIOD);
#else
  schedule_delayed_work(&chip->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
  schedule_delayed_work(&chip->charge_work, BQ24160_CHG_WORK_PERIOD);
#endif
  
  return IRQ_HANDLED;
}

static int bq24160_charger_get_property(struct power_supply *psy,
                                        enum power_supply_property psp,
                                        union power_supply_propval *val)
{
	struct bq24160_chip *chip = container_of(psy, struct bq24160_chip, charger);
	int ret = 0;
    int chg_status = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->chg_online;
		break;
    case POWER_SUPPLY_PROP_STATUS:
		ret = bq24160_read_reg(chip->client, BQ24160_REG_STAT_CTRL);
		if(ret >= 0)
		{
        	chg_status = ret & BQ24160_STAT_MASK;
			chg_status = chg_status >> BQ24160_STAT_SHFT;
		}
        if (chip->chg_online) {
            if (chg_status == BQ24160_STAT_CHARGE_DONE) {
                val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
            }
            else if ((chg_status == BQ24160_STAT_CHARGING_FROM_IN) ||
                     (chg_status == BQ24160_STAT_CHARGING_FROM_USB) ) {
                val->intval = POWER_SUPPLY_STATUS_CHARGING;
            }
            else {
                val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
            }
        }
        else {
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        }
        ret = 0;
		break;	
    default:
		ret = -ENODEV;
		break;
	}
	return ret;
}

int bq24160_charger_get_status(void)
{
  if(now_chg_state == BQ24160_CHG_STATE_STOP)
    return POWER_SUPPLY_STATUS_DISCHARGING;
  else if(now_chg_state == BQ24160_CHG_STATE_NO_CABLE)
    return POWER_SUPPLY_STATUS_DISCHARGING;
  else if(now_chg_state == BQ24160_CHG_STATE_CHARGING)
    return POWER_SUPPLY_STATUS_CHARGING;
  else if(now_chg_state == BQ24160_CHG_STATE_DONE)
    return POWER_SUPPLY_STATUS_FULL;
  else
    return POWER_SUPPLY_STATUS_UNKNOWN;
}
EXPORT_SYMBOL(bq24160_charger_get_status);


static enum power_supply_property bq24160_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply bq24160_charger_ps = {
   .name = "bq24160-charger",
   .type = POWER_SUPPLY_TYPE_MAINS,
   .properties = bq24160_charger_props,
   .num_properties = ARRAY_SIZE(bq24160_charger_props),
   .get_property = bq24160_charger_get_property,
};

#ifdef CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY
/*=========================================================================*/

extern int pm8058_get_battery_mvolts(void);

extern int batt_read_adc(int channel, int *mv_reading);

extern int pm8058_get_battery_temperature(void);

extern int pm8058_is_battery_present(void);

extern int pm8058_is_battery_temp_within_range(void);

extern int pm8058_is_battery_id_valid(void);


#if 0
#define BATT_THERM_OPEN_MV  2000
static int bq24160_is_battery_present(void)
{
	int mv_reading;

	mv_reading = 0;
	batt_read_adc(CHANNEL_ADC_BATT_THERM, &mv_reading);
	pr_info("%s: therm_raw is %d\n", __func__, mv_reading);
#ifndef CONFIG_LGE_FUEL_GAUGE
	if (mv_reading > 0 && mv_reading < BATT_THERM_OPEN_MV)
#endif
		return 1;

	return 0;
}

#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
#define BATT_THERM_OPERATIONAL_MAX_CELCIUS 100
#else
#define BATT_THERM_OPERATIONAL_MAX_CELCIUS 40
#endif
#define BATT_THERM_OPERATIONAL_MIN_CELCIUS 0
static int bq24160_is_battery_temp_within_range(void)
{
	int therm_celcius;

	therm_celcius = bq24160_is_battery_present();
	pr_info("%s: therm_celcius is %d\n", __func__, therm_celcius);
#ifndef CONFIG_LGE_FUEL_GAUGE 
	if (therm_celcius > 0
		&& therm_celcius > BATT_THERM_OPERATIONAL_MIN_CELCIUS
		&& therm_celcius < BATT_THERM_OPERATIONAL_MAX_CELCIUS)
#endif
		return 1;

	return 0;
}
#endif

void bq24160_at_cmd_chg_set(bool set_value)
{
  at_cmd_chg = set_value;
}
EXPORT_SYMBOL(bq24160_at_cmd_chg_set);


bool bq24160_at_cmd_chg_get(void)
{
  return at_cmd_chg;
}


#if 0
static struct msm_battery_gauge bq24160_batt_gauge = {
#ifdef CONFIG_LGE_FUEL_GAUGE
	.get_battery_mvolts = pm8058_get_battery_mvolts,
#endif
	.get_battery_temperature = pm8058_get_battery_temperature,
	.is_battery_present = pm8058_is_battery_present,
	.is_battery_temp_within_range = bq24160_is_battery_temp_within_range,
	.is_battery_id_valid = pm8058_is_battery_id_valid,
#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO	
    .get_battery_temperature_adc = pm8058_get_battery_temperature_adc,
#endif
};
#endif
/*=======================================================================*/

static int bq24160_start_charging(struct msm_hardware_charger *hw_chg,
		int chg_voltage, int chg_current)
{
	struct bq24160_chip *bq24160_chg;
	int ret = 0;

	bq24160_chg = container_of(hw_chg, struct bq24160_chip, adapter_hw_chg);

  printk(KERN_DEBUG "[bq24160_start_charging] Online = %d AT CMD = %d Resume = %d \n", 
    bq24160_chg->chg_online, bq24160_at_cmd_chg_get(), resume_chg_start);

  if(!bq24160_at_cmd_chg_get() && (chg_complete || chg_resume))
    return 0;
  
	if (bq24160_chg->chg_online && !bq24160_at_cmd_chg_get())
  { 
    if(resume_chg_start)
    {
      /* Watchdog timer reset */
	    if((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL, 0xB8);
      }

      /* REG_#2  Set current limit and enable STAT in Control register */
      if(usb_chg_type == 3)
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x28);
      }
      else
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x38);
      }

      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK); // Batt Regulation Volt: 4.38V

      if(usb_chg_type == 3)
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);
      }
      else
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x11);
      }

      bq24160_chg->chg_online = 1;
      msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
    
      resume_chg_start = false;
    }
    else
    {
		  /* we are already charging */
		  return 0;
    }
  }
  else
  {
    /* Watchdog timer reset */
	  if((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL, 0xB8);
    }

    /* REG_#2  Set current limit and enable STAT in Control register */
    if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x38); // Disable Hi-z mode
    }
    else
    {
      if(usb_chg_type == 3)
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x28);
      }
      else
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x38);
      }
    }
    
    bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL_BATVOLT, BQ24160_VBATTREG_MASK); // Batt Regulation Volt: 4.38V

    if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x11);
    }
    else
    {
      if(usb_chg_type == 3)
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x01);
      }
      else
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTTERM_FCHGCRNT, 0x11);
      }
    }

    bq24160_chg->chg_online = 1;
    msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);

#ifdef BQ24160_LOCAL_WQ
    queue_delayed_work(local_workqueue, &bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
#else
    schedule_delayed_work(&bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
#endif
  }

	dev_dbg(&bq24160_chg->client->dev, "%s\n", __func__);
	return ret;
}

static int bq24160_stop_charging(struct msm_hardware_charger *hw_chg)
{
	struct bq24160_chip *bq24160_chg;
	int ret = 0;

	bq24160_chg = container_of(hw_chg, struct bq24160_chip, adapter_hw_chg);

  printk(KERN_DEBUG "[bq24160_stop_charging] Online = %d AT CMD = %d Resume = %d \n", 
    bq24160_chg->chg_online, bq24160_at_cmd_chg_get(), resume_chg_start);
   
	if (!(bq24160_chg->chg_online)  && !bq24160_at_cmd_chg_get())
		/* we arent charging */
		return 0;
  else
  {    
    /* Watchdog timer reset */
	  if((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL, 0xB8);
    }

    /* REG_#2  Set current limit and enable STAT in Control register */
    if(usb_chg_type == 3)
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2C);
    }
    else
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3C);
    }

    bq24160_chg->chg_online = 0;
    msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_REMOVED_EVENT);

    if(now_chg_state != BQ24160_CHG_STATE_DONE)
    {
      cancel_delayed_work_sync(&bq24160_chg->watch_dog_reset);
    }
  }

	dev_dbg(&bq24160_chg->client->dev, "%s\n", __func__);
	return ret;
}

static int bq24160_charging_switched(struct msm_hardware_charger *hw_chg)
{
	struct bq24160_chip *bq24160_chg;

	bq24160_chg = container_of(hw_chg, struct bq24160_chip, adapter_hw_chg);
	dev_dbg(&bq24160_chg->client->dev, "%s\n", __func__);
	return 0;
}
#endif


#ifdef CONFIG_LGE_CHARGER_TEMP_SCENARIO
extern void arch_reset(char mode, const char *cmd);
#endif

static void bq24160_watchdog_reset(struct work_struct *bq24160_work)
{
  struct bq24160_chip *bq24160_chg;
  int mv;
  
#ifdef BQ24160_WORK_AROUND_CODE
  int batt_percent = 0;
#endif

	bq24160_chg = container_of(bq24160_work, struct bq24160_chip,
			watch_dog_reset.work);


  mutex_lock(&bq24160_status_lock);

  if((!chg_complete && chg_resume) && (0 != battery_info_get()))
  {
    mv = pre_volt/*batt_volt*/;
    batt_percent = 100;
    bq24160_chg_get_batt_temp();
  }
  else if(!chg_complete && (0 != battery_info_get()))
  {
    batt_percent = pre_soc/*fuel_percent*/;
    mv = pre_volt/*batt_volt*/;
    bq24160_chg_get_batt_temp();
  }
  else
  {
    batt_high_temp = CHG_BATT_TEMP_M4_41;
    mv = pre_volt/*batt_volt*/;

    if(0 == battery_info_get())
      batt_percent = 70;
    else
      batt_percent = 100;
  }

  printk(KERN_DEBUG "[bq24160_pollig_timer]: [CHG CUR = %d CHG STATE = %d VOLT = %d TEMP1 = %d COMPLETE = %d\n", 
    usb_chg_type, now_chg_state, mv, batt_high_temp, chg_complete);

  if(chg_complete && (mv > 4249))
  {    
    now_chg_state = BQ24160_CHG_STATE_DONE;
    queue_delayed_work(local_workqueue, &bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);

    mutex_unlock(&bq24160_status_lock);

    return;
  }

  if(((batt_high_temp != CHG_BATT_TEMP_OVER_55) && (batt_high_temp != CHG_BATT_TEMP_UNDER_M10) && \
    (batt_high_temp != CHG_BATT_TEMP_46_55)) && chg_resume)
  {
    if(mv > 4337)
    {
      chg_complete = true;
      chg_resume = false;
      bq24160_chg_done_once = true;
      now_chg_state = BQ24160_CHG_STATE_DONE;

      if(usb_chg_type == 3)
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2C);
      }
      else
      {
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
        bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3C);
      }
    }
    
    queue_delayed_work(local_workqueue, &bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);

    mutex_unlock(&bq24160_status_lock);
    
    return;
  }

/*==========================================================================*/
/*                                                      BATTERY TEMPERATURE SENARIO                                                               */
/*==========================================================================*/
  /* Stop Charging Condition Check for BATTERY */
  if(batt_high_temp == CHG_BATT_TEMP_OVER_55 || batt_high_temp == CHG_BATT_TEMP_UNDER_M10)
  {    
    /* REG_#2  Set current limit and enable STAT in Control register */
    printk(KERN_DEBUG "[bq24160_pollig_timer]: BATTERY HEAT is OVER 55 Deg!!! \n");

    if(usb_chg_type == 3)
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2E);
    }
    else
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3E);
    }

    now_chg_state = BQ24160_CHG_STATE_STOP;
    bq24160_chg->chg_online = 0;

    /* Watchdog timer reset */
    if((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL, 0xB8);
    }

    if (!pm8058_is_battery_present())
    {
      printk(KERN_DEBUG "[bq24160_pollig_timer]: BATTERY IS NOT CONNECTED!!! \n");
      arch_reset(0,NULL);
    }

#ifdef BQ24160_LOCAL_WQ
    queue_delayed_work(local_workqueue, &bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
#else
    schedule_delayed_work(&bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
#endif

    chg_over_heat_detect = true;
    chg_resume = false;

    mutex_unlock(&bq24160_status_lock);

    return;
  }


/*==========================================================================*/
/*                                          BATTERY FULL or RESUME CHARGING SENARIO                                                        */
/*==========================================================================*/  
#ifdef BQ24160_WORK_AROUND_CODE
  /* Determine the Resume Charging or Charging Termination */
#if !defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY_4P2V)
  if((batt_percent == 100) && (mv > 4337)) /* Charing Termination */
#else
  if((batt_percent == 100) && (mv > 4180)) /* Charing Termination */
#endif
  {
    chg_complete = true;
    chg_resume = false;
    bq24160_chg_done_once = true;

    if(usb_chg_type == 3)
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2C);
    }
    else
    {
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
      bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3C);
    }

    if(now_chg_state != BQ24160_CHG_STATE_DONE)
    {
      msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_DONE_EVENT);
    }

    now_chg_state = BQ24160_CHG_STATE_DONE;
  }
#if !defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY_4P2V)
  else if((mv < 4250) && chg_complete) /* Resume Charing */
#else
  else if((mv < 4100) && chg_complete) /* Resume Charing */
#endif
  {
    if(chg_complete && (batt_high_temp != CHG_BATT_TEMP_46_55))
    {
      msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      
      chg_complete = false;
      chg_resume = true;
      bq24160_chg_done_once = true;

      if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
      {
        bq24160_pif_chg_start_reg_setting(bq24160_chg);
      }
      else
      {
        if(usb_chg_type == 3)
        {
          bq24160_usb_chg_start_reg_setting(bq24160_chg);
        }
        else
        {
          bq24160_ta_chg_start_reg_setting(bq24160_chg);
        }
      }
    }
    else if(chg_complete && (batt_high_temp == CHG_BATT_TEMP_46_55))
    {
      chg_complete = false;
      printk(KERN_DEBUG "[bq24160_pollig_timer]: Can't resume charging because of termperature!!! \n");
    }
  }
#if !defined(CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY_4P2V)
  else if(chg_complete && (mv > 4249) && (mv < 4339))
#else
  else if(chg_complete && (mv > 4099) && (mv < 4181))
#endif
  {
    //SKIP
    chg_complete = true;
    chg_resume = false;
    bq24160_chg_done_once = true;
  }
  else
  {
    chg_resume = false;
    
    if(polling_time_prev_voltage != usb_chg_type)
    {
      printk(KERN_DEBUG "[bq24160_pollig_timer][0]: Status Change = %d !!! \n", usb_chg_type);
      bq24160_chg_refresh_reg_normal(bq24160_chg, 1);
    }
    
    // Normal
    if(!chg_over_heat_detect && !chg_stop_temp_detect && ((batt_high_temp > CHG_BATT_TEMP_46_55) && (batt_high_temp < CHG_BATT_TEMP_UNDER_M10)))
    {      
      chg_stop_temp_detect = false;
      chg_step_now = BQ24160_CHG_STATE_1;
      now_chg_state = BQ24160_CHG_STATE_CHARGING;

      if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
      {
        printk(KERN_DEBUG "[bq24160_pollig_timer][0]: Charging Normal State!!! \n");
        
        bq24160_chg_refresh_reg_normal(bq24160_chg, 1);
      }
    }
    // Reduce charging current
    else if(!chg_over_heat_detect && !chg_stop_temp_detect && (batt_high_temp == CHG_BATT_TEMP_46_55) && (mv < 4001))
    {      
      chg_stop_temp_detect = false;
      chg_step_now = BQ24160_CHG_STATE_2;
      now_chg_state = BQ24160_CHG_STATE_CHARGING;

      if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
      {
        printk(KERN_DEBUG "[bq24160_pollig_timer][1]: Reduce charging current because of termperature!!! \n");
        
        bq24160_chg_refresh_reg_temp_senario(bq24160_chg);
      }
    }
    // Stop charging
    else if(!chg_over_heat_detect && !chg_stop_temp_detect && (batt_high_temp == CHG_BATT_TEMP_46_55) && (mv > 4000))
    {      
      chg_stop_temp_detect = true;
      chg_step_now = BQ24160_CHG_STATE_3;
      if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
      {
        bq24160_pif_chg_start_reg_setting(bq24160_chg);
      }
      else
      {
        if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
        {
          printk(KERN_DEBUG "[bq24160_pollig_timer][2]:Stop charging current because of termperature!!! \n");
          
          if(usb_chg_type == 3)
          {
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2E);
          }
          else
          {
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3E);
          }
        }
      }

      now_chg_state = BQ24160_CHG_STATE_STOP;

      bq24160_chg->chg_online = 0;
    }
    // Reduce charging current
    else if(!chg_over_heat_detect && chg_stop_temp_detect && (batt_high_temp == CHG_BATT_TEMP_46_55) && (mv < 4001))
    {
      //int rc = 0;
      
      chg_stop_temp_detect = false;
      chg_step_now = BQ24160_CHG_STATE_4;
      now_chg_state = BQ24160_CHG_STATE_CHARGING;

      if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
      {
        printk(KERN_DEBUG "[bq24160_pollig_timer][3]: Reduce charging current because of termperature!!! \n");
        
        bq24160_chg_refresh_reg_temp_senario(bq24160_chg);

        //rc = pm8058_batt_alarm_state_set(0, 0);
        bq24160_chg->chg_online = 1;
        msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      }
    }
    // Stop charging
    else if(!chg_over_heat_detect && chg_stop_temp_detect && (batt_high_temp == CHG_BATT_TEMP_46_55) && (mv > 4000))
    {      
      chg_stop_temp_detect = true;
      chg_step_now = BQ24160_CHG_STATE_5;
      now_chg_state = BQ24160_CHG_STATE_STOP;

      if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
      {
        bq24160_pif_chg_start_reg_setting(bq24160_chg);
      }
      else
      {
        if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
        {
          printk(KERN_DEBUG "[bq24160_pollig_timer][4]:Stop charging current because of termperature!!! \n");
          
          if(usb_chg_type == 3)
          {
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2E);
          }
          else
          {
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
            bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3E);
          }
        }
      }

      bq24160_chg->chg_online = 0;
    }
    else if(!chg_over_heat_detect && chg_stop_temp_detect && ((batt_high_temp > CHG_BATT_TEMP_46_55) && (batt_high_temp < CHG_BATT_TEMP_UNDER_M10)))
    {
      //int rc = 0;
      
      chg_stop_temp_detect = false;
      chg_step_now = BQ24160_CHG_STATE_6;
      now_chg_state = BQ24160_CHG_STATE_CHARGING;

      if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
      {
        bq24160_pif_chg_start_reg_setting(bq24160_chg);
      }
      else
      {
        if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
        {
          printk(KERN_DEBUG "[bq24160_pollig_timer][5]: Charging Restart!!! \n");
          
          if(usb_chg_type == 3)
          {
            bq24160_usb_chg_start_reg_setting(bq24160_chg);
          }
          else
          {
            bq24160_ta_chg_start_reg_setting(bq24160_chg);
          }
        }
      }

      if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
      {
        //rc = pm8058_batt_alarm_state_set(0, 0);
        bq24160_chg->chg_online = 1;
        msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      }
    }
    else if(chg_over_heat_detect && !chg_stop_temp_detect && (batt_high_temp == CHG_BATT_TEMP_46_55) && (mv < 4001))
    {
      //int rc = 0;
      
      chg_stop_temp_detect = false;
      chg_step_now = BQ24160_CHG_STATE_7;
      now_chg_state = BQ24160_CHG_STATE_CHARGING;

      if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
      {
        printk(KERN_DEBUG "[bq24160_pollig_timer][6]: Reduce charging current because of termperature!!! \n");
        
        bq24160_chg_refresh_reg_temp_senario(bq24160_chg);

        //rc = pm8058_batt_alarm_state_set(0, 0);
        bq24160_chg->chg_online = 1;
        msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      }
    }
    else if(chg_over_heat_detect && !chg_stop_temp_detect && (batt_high_temp == CHG_BATT_TEMP_46_55) && (mv > 4001))
    {
      chg_stop_temp_detect = true;
      chg_step_now = BQ24160_CHG_STATE_8;
      now_chg_state = BQ24160_CHG_STATE_STOP;

      if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
      {
        printk(KERN_DEBUG "[bq24160_pollig_timer][7]: Stop charging current because of termperature!!! \n");
        
        if(usb_chg_type == 3)
        {
          bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
          bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x2E);
        }
        else
        {
          bq24160_write_reg(bq24160_chg->client, BQ24160_REG_BATTNPS_STAT, 0x00);
          bq24160_write_reg(bq24160_chg->client, BQ24160_REG_CTRL, 0x3E);
        }

        bq24160_chg->chg_online = 0;
      }
    }
    else if(chg_over_heat_detect && !chg_stop_temp_detect && ((batt_high_temp > CHG_BATT_TEMP_46_55) && (batt_high_temp < CHG_BATT_TEMP_UNDER_M10)))
    {
      //int rc = 0;
      
      chg_stop_temp_detect = false;
      now_chg_state = BQ24160_CHG_STATE_CHARGING;

      if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
      {
        bq24160_pif_chg_start_reg_setting(bq24160_chg);
      }
      else
      {
        if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
        {
          printk(KERN_DEBUG "[bq24160_pollig_timer][8]: Charging Restart!!!\n");
          
          if(usb_chg_type == 3)
          {
            bq24160_usb_chg_start_reg_setting(bq24160_chg);
          }
          else
          {
            bq24160_ta_chg_start_reg_setting(bq24160_chg);
          }
        }
      }

      if((chg_step_now != chg_step_prev) || (polling_time_prev_voltage != usb_chg_type))
      {
        //rc = pm8058_batt_alarm_state_set(0, 0);
        bq24160_chg->chg_online = 1;
        msm_charger_notify_event(&bq24160_chg->adapter_hw_chg, CHG_INSERTED_EVENT);
      }
    }
  }
#endif

  chg_over_heat_detect = false;
  chg_step_prev = chg_step_now;
  polling_time_prev_voltage = usb_chg_type;

  /* Watchdog timer reset */
  if((usb_cable_info != 6) && (usb_cable_info != 7) && (usb_cable_info != 11))
  {
    bq24160_write_reg(bq24160_chg->client, BQ24160_REG_STAT_CTRL, 0xB8);

    if (!pm8058_is_battery_present())
    {
      printk(KERN_DEBUG "[bq24160_pollig_timer]: BATTERY IS NOT CONNECTED!!! \n");
      arch_reset(0,NULL);
    }
  }

#ifdef BQ24160_LOCAL_WQ
  queue_delayed_work(local_workqueue, &bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
#else
  schedule_delayed_work(&bq24160_chg->watch_dog_reset, BQ24160_CHG_WATCH_DOG_RESET_PERIOD);
#endif

  mutex_unlock(&bq24160_status_lock);
}


static __devinit int bq24160_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bq24160_chip *chip;
  int ret = 0;

  printk(KERN_DEBUG "########## [bq24160_probe]: I-DCM SWITCHING CHARGER PROBE START!!! #################\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

  mutex_init(&bq24160_status_lock);

	INIT_DELAYED_WORK(&chip->charge_work, bq24160_charge);
  INIT_DELAYED_WORK(&chip->watch_dog_reset, bq24160_watchdog_reset);
  INIT_DELAYED_WORK(&chip->charge_start_from_temp_senario, bq24160_start_chg_from_temp_senario);
  INIT_DELAYED_WORK(&chip->charge_stop_from_temp_senario, bq24160_stop_chg_from_temp_senario);

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);
  chip->charger = bq24160_charger_ps;

	ret = power_supply_register(&client->dev, &chip->charger);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		goto out;
	}

#ifdef CONFIG_LGE_SWITCHING_CHARGER_BQ24160_DOCOMO_ONLY
	/* fill hw_chg structure for registering msm_charger */
	chip->adapter_hw_chg.type = CHG_TYPE_AC;
	chip->adapter_hw_chg.rating = 1;
	chip->adapter_hw_chg.name = "bq24160-adapter";
	chip->adapter_hw_chg.start_charging = bq24160_start_charging;
	chip->adapter_hw_chg.stop_charging = bq24160_stop_charging;
	chip->adapter_hw_chg.charging_switched = bq24160_charging_switched;
	
	ret = msm_charger_register(&chip->adapter_hw_chg);
	if (ret) {
		dev_err(&client->dev,
			"%s msm_charger_register failed for ret =%d\n",
			__func__, ret);
		goto out;
	}


	bq24160_irq_init(SWITCHING_CHG_IRQ_N);
#endif


	ret = request_threaded_irq(client->irq, NULL, bq24160_valid_handler,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, client->name, chip);



#if 0
  ret = pm8058_batt_alarm_state_set(0, 0);
	if (ret) {
		pr_err("%s: unable to set batt alarm state\n", __func__);
		goto out;
	}

	/*
	 * The batt-alarm driver requires sane values for both min / max,
	 * regardless of whether they're both activated.
	 */
	ret = pm8058_batt_alarm_hold_time_set(PM8058_BATT_ALARM_HOLD_TIME_16_MS);
	if (ret) {
		pr_err("%s: unable to set batt alarm hold time\n", __func__);
		goto out;
	}
#endif

	chip->chg_online = 0;

#ifdef BQ24160_WORK_AROUND_CODE
  if((0 == battery_info_get()) && ((usb_cable_info == 6) || (usb_cable_info == 7) ||(usb_cable_info == 11)))
  {
    bq24160_pif_chg_start_reg_setting(chip);
  }
  else
  {
    if(usb_chg_type == 3)
    {
      bq24160_usb_chg_start_reg_setting(chip);
    }
    else
    {
      bq24160_ta_chg_start_reg_setting(chip);
    }
  }
#endif

  
	return 0;
out:
	kfree(chip);
	return ret;
}

static __devexit int bq24160_remove(struct i2c_client *client)
{
  //int rc = 0;
  struct bq24160_chip *chip = i2c_get_clientdata(client);

	free_irq(client->irq, chip);
  cancel_delayed_work_sync(&chip->watch_dog_reset);
	power_supply_unregister(&bq24160_charger_ps);
	kfree(chip);

  //rc = pm8058_batt_alarm_state_set(0, 0);

#ifdef BQ24160_WORK_AROUND_CODE
  mutex_destroy(&bq24160_status_lock);
  chg_complete = false;
  chg_resume = false;
  chg_over_heat_detect = false;
  usb_chg_type = -1;
  now_chg_state = BQ24160_CHG_STATE_NO_CABLE;

  msm_charger_notify_event(&chip->adapter_hw_chg, CHG_REMOVED_EVENT);
#endif

	return 0;
}

static const struct i2c_device_id bq24160_id[] = {
	{ "bq24160", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bq24160_id);

static struct i2c_driver bq24160_i2c_driver = {
	.driver = {
		.name = "bq24160",
	},
	.probe		= bq24160_probe,
	.remove		= __devexit_p(bq24160_remove),
	.id_table	= bq24160_id,
};

static int __init bq24160_init(void)
{
#ifdef BQ24160_LOCAL_WQ  
  local_workqueue = create_workqueue("bq24160 charger") ;
  
  if(!local_workqueue)
     return -ENOMEM;
#endif

	return i2c_add_driver(&bq24160_i2c_driver);
}
module_init(bq24160_init);

static void __exit bq24160_exit(void)
{
#ifdef BQ24160_LOCAL_WQ
  printk(KERN_DEBUG "######### [bq24160_exit][8]: Charger REMOVE!!! ########\n");
  if(local_workqueue)
    destroy_workqueue(local_workqueue);

  local_workqueue = NULL;
#endif

	i2c_del_driver(&bq24160_i2c_driver);
}
module_exit(bq24160_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kyungho Kong <kyungho.kong@lge.com>");
MODULE_DESCRIPTION("Power supply driver for BQ24160");
MODULE_ALIAS("platform:bq24160-charger");
