/*
 * Copyright(c) 2013 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
/*
 * Intel Quark Thermal driver
 */
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/intel_qrk_sb.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/thermal.h>
#include <linux/timer.h>

#define DRIVER_NAME		"intel-qrk-thrm"

/* Definition of register locations for thermal management */
#define THRM_CTRL_REG		(0x80)		/* Thermal control */
#define THRM_MODE_REG		(0xB0)		/* Thermal mode */
#define THRM_MODE_SENSOR_EN	(0x00008000)	/* Thermal mode sensor enable */
#define THRM_TEMP_REG		(0xB1)		/* Thermal sensor temperature */
#define THRM_TRPCLR_REG		(0xB2)		/* Catastropic/Hot trip/clear */
#define THRM_AUXTRP_REG		(0xB3)		/* Aux0-Aux3 trip point */
#define THRM_AUXCLR_REG		(0xB4)		/* Aux0-Aux3 clear trip */
#define THRM_STATUS_REG		(0xB5)		/* Thermal sensor status */
#define THRM_TRIPBEHAVE_REG	(0xB6)		/* Trip point behavior */
#define THRM_MSIADDR_REG	(0xC5)		/* Thermal MSI addres reg */
#define THRM_MSIDATA_REG	(0xC6)		/* Thermal MSI data reg */
#define THRM_CTRL_READ		(0x10)		/* Config reg */
#define THRM_CTRL_WRITE		(0x11)		/* Config reg */

#define SOC_TSENSOR_REG		(0x34)
#define SOC_TSENSOR_RST		(0x00000001)
#define SOC_CTRL_READ		(0x06)
#define SOC_CTRL_WRITE		(0x07)


#define THRM_ZONE_COUNT		2		/* Only hot/critical relevant */
#define ACTIVE_INTERVAL		(1000)
#define IDLE_INTERVAL		(20000)
#define MCELSIUS(x)		((x) * 1000)

/* CPU Zone information */
#define CATASTROPIC_ZONE	0
#define HOT_ZONE		1
#define AUX0_ZONE		2		/* Unused */
#define AUX1_ZONE		3		/* Unused */
#define AUX2_ZONE		4		/* Unused */
#define AUX3_ZONE		5		/* Unused */
#define MIN_USED_ZONE		CATASTROPIC_ZONE
#define MAX_USED_ZONE		HOT_ZONE
/*
 * Default catastrophic/hot trip values - in degrees celsius
 * Maximum temperature is 105 degrees
 */
#define CRIT_TEMP	104
#define HOT_TEMP	95
#define RAW2CELSIUS_DIFF	50

static int driver_enable = 1;
module_param(driver_enable, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(driver_enable, "Disable Thermal Driver Polling");

/* Shorten fn names to fit 80 char limit */
#ifndef sb_read
#define sb_read				intel_qrk_sb_read_reg
#endif
#ifndef sb_write
#define sb_write			intel_qrk_sb_write_reg
#endif

struct intel_qrk_therm_zone {
	enum thermal_trip_type type;
	int trip_value;
};

/**
 * struct intel_qrk_thermal_dev
 *
 */
struct intel_qrk_thermal_dev {
	enum thermal_device_mode mode;
	struct intel_qrk_therm_zone tzone[THRM_ZONE_COUNT];
	struct mutex lock;
	struct platform_device *pldev;		/* Platform device */
	struct thermal_zone_device *therm_dev;	/* Thermal device */
};

static struct intel_qrk_thermal_dev qrk_tdev;

/******************************************************************************
 *                        Thermal API implementation
 ******************************************************************************/

/**
 * get_temp
 *
 * @param tz: Thermal zone descriptor
 *
 * Get the current temperature
 * We have exactly one thermal zone/sensor
 * Value passed is an unsigned long - our sensor reports up to -50 celsius so we
 * just clip at zero if the temperature is negative.
 */
static int intel_qrk_thermal_get_temp(struct thermal_zone_device *tz,
				unsigned long *temp)
{
	sb_read(SB_ID_THERMAL, THRM_CTRL_READ, THRM_TEMP_REG, (u32 *)temp, 1);
	*temp -= RAW2CELSIUS_DIFF;

	/* Clip to unsigned output value if sensor is reporting sub-zero */
	if ((int)*temp < 0)
		*temp = 0;

	*temp = MCELSIUS(*temp&0x000000FF);

	return 0;
}

/**
 * get_trend
 *
 * Wears good clothes
 */
static int intel_qrk_thermal_get_trend(struct thermal_zone_device *tz,
			int trip, enum thermal_trend *trend)
{
	if (tz->temperature >= trip)
		*trend = THERMAL_TREND_RAISING;
	else
		*trend = THERMAL_TREND_DROPPING;

	return 0;
}

/**
 * intel_qrk_thermal_get_mode
 *
 * Get the mode
 */
static int intel_qrk_thermal_get_mode(struct thermal_zone_device *tz,
				enum thermal_device_mode *mode)
{
	mutex_lock(&qrk_tdev.lock);
	*mode = qrk_tdev.mode;
	mutex_unlock(&qrk_tdev.lock);

	return 0;
}

/**
 * intel_qrk_thermal_set_mode
 *
 * Set the mode
 */
static int intel_qrk_thermal_set_mode(struct thermal_zone_device *tz,
				enum thermal_device_mode mode)
{
	mutex_lock(&qrk_tdev.lock);

	if (mode == THERMAL_DEVICE_ENABLED)
		qrk_tdev.therm_dev->polling_delay = IDLE_INTERVAL;
	else
		qrk_tdev.therm_dev->polling_delay = 0;
	qrk_tdev.mode = mode;

	mutex_unlock(&qrk_tdev.lock);

	thermal_zone_device_update(qrk_tdev.therm_dev);
	pr_info("thermal polling set for duration=%d msec\n",
				qrk_tdev.therm_dev->polling_delay);
	return 0;
}

/**
 * intel_qrk_thermal_get_trip_type
 *
 * Get trip type
 */
static int intel_qrk_thermal_get_trip_type(struct thermal_zone_device *tz,
				int trip, enum thermal_trip_type *type)
{
	if (trip < MIN_USED_ZONE || trip > MAX_USED_ZONE)
		return -EINVAL;

	*type = qrk_tdev.tzone[trip].type;
	return 0;
}

/**
 * intel_qrk_thermal_get_trip_temp
 *
 * Get trip temp
 */
static int intel_qrk_thermal_get_trip_temp(struct thermal_zone_device *tz,
				int trip, unsigned long *temp)
{
	if (trip < MIN_USED_ZONE || trip > MAX_USED_ZONE)
		return -EINVAL;

	/* Convert the temperature into millicelsius */
	*temp = qrk_tdev.tzone[trip].trip_value;

	return 0;
}

/**
 * intel_qrk_thermal_get_trip_type
 *
 * Get trip temp
 */
static int intel_qrk_thermal_get_crit_temp(struct thermal_zone_device *tz,
				unsigned long *temp)
{
	/* Critical zone */
	*temp = qrk_tdev.tzone[CATASTROPIC_ZONE].trip_value;
	return 0;
}

static struct thermal_zone_device_ops intel_qrk_thrm_dev_ops = {
	.get_temp = intel_qrk_thermal_get_temp,
	.get_trend = intel_qrk_thermal_get_trend,
	.get_mode = intel_qrk_thermal_get_mode,
	.set_mode = intel_qrk_thermal_set_mode,
	.get_trip_type = intel_qrk_thermal_get_trip_type,
	.get_trip_temp = intel_qrk_thermal_get_trip_temp,
	.get_crit_temp = intel_qrk_thermal_get_crit_temp,
};



/**
 * intel_qrk_init_zone
 *
 * Initialise a zone
 */
static void intel_qrk_thermal_init_zone(struct intel_qrk_therm_zone *tz,
				enum thermal_trip_type type, int trip_value)
{
	tz->type = type;
	tz->trip_value = MCELSIUS(trip_value);
}

/******************************************************************************
 *                        Module Entry/Exit hooks
 ******************************************************************************/

/**
 * intel_qrk_thermal_probe
 *
 * @param pdev: Platform device
 * @return 0 success < 0 failure
 *
 * Callback from platform sub-system to probe
 *
 * This routine registers a thermal device with the kernel's thermal management
 * sub-system
 */
static int intel_qrk_thermal_probe(struct platform_device *pdev)
{
	int err = 0;
	int critical_temp = 0, hot_temp = 0;
	uint32_t regval = 0;

	if (driver_enable == 0)
		return 0;

	memset(&qrk_tdev, 0x00, sizeof(qrk_tdev));

	critical_temp = CRIT_TEMP;
	hot_temp = HOT_TEMP;

	/* Enumerate zone type data */
	memset(&qrk_tdev, 0x00, sizeof(qrk_tdev));
	mutex_init(&qrk_tdev.lock);

	/* Set initial state disabled */
	qrk_tdev.mode = THERMAL_DEVICE_ENABLED;

	intel_qrk_thermal_init_zone(&qrk_tdev.tzone[CATASTROPIC_ZONE],
				  THERMAL_TRIP_CRITICAL, critical_temp);
	intel_qrk_thermal_init_zone(&qrk_tdev.tzone[HOT_ZONE],
				  THERMAL_TRIP_HOT, hot_temp);

	/* Register a thermal zone */
	qrk_tdev.therm_dev = thermal_zone_device_register(DRIVER_NAME,
			THRM_ZONE_COUNT, 0, 0, &intel_qrk_thrm_dev_ops,
			0, IDLE_INTERVAL, ACTIVE_INTERVAL);

	if (IS_ERR(qrk_tdev.therm_dev)) {
		err = PTR_ERR(qrk_tdev.therm_dev);
		return err;
	}

	/* Read the BIOS configured hardware catastrophic trip temp */
	sb_read(SB_ID_THERMAL, THRM_CTRL_READ, THRM_TRPCLR_REG, &regval, 1);
	regval = (regval & 0xff) - 50;

	pr_info("THRM: critical reset %d c hot %d c hardware failover %d c\n",
		critical_temp, hot_temp, regval);

	return 0;
}

/**
 * intel_qrk_thermal_remove
 *
 * @return 0 success < 0 failure
 *
 * Removes a platform device
 */
static int intel_qrk_thermal_remove(struct platform_device *pdev)
{
	if (qrk_tdev.therm_dev != NULL) {
		thermal_zone_device_unregister(qrk_tdev.therm_dev);
		return 0;
	}
	return -EINVAL;
}

/*
 * Platform structures useful for interface to PM subsystem
 */
static struct platform_driver intel_qrk_thermal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = intel_qrk_thermal_probe,
	.remove = intel_qrk_thermal_remove,
};

module_platform_driver(intel_qrk_thermal_driver);


MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@linux.intel.com>");
MODULE_DESCRIPTION("Intel Quark Thermal driver");
MODULE_LICENSE("Dual BSD/GPL");
