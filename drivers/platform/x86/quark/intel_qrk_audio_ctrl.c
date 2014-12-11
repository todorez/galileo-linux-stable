/*
 * Intel Quark platform audio control driver
 *
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
 *
 * The Intel Clanton Hill platform hardware design includes an audio subsystem
 * with a number of interconnected audio interfaces.  This driver enables
 * applications to choose which audio connections to enable for various
 * application use cases.  The interconnections are selectable using GPIO output
 * pins on the CPU.  This driver is also responsible for configuring a Maxim
 * 9867 audio codec, a component of this audio subsystem, connected to the CPU
 * via I2C.
 */

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <uapi/linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>

#include "intel_qrk_audio_ctrl.h"

#define DRIVER_NAME			"intel_qrk_audio_ctrl"

/*
 * GPIO numbers to use for switching audio paths
 */
#define GPIO_AUDIO_S0   11
#define GPIO_AUDIO_S1   12
#define GPIO_AUDIO_S2   13

#define GPIO_AUDIO_DEFAULT (INTEL_QRK_AUDIO_MODE_SPKR_MIC)

/**
 * struct intel_qrk_audio_ctrl_data
 *
 * Structure to represent module state/data/etc
 */
struct intel_qrk_audio_ctrl_priv {

	/* i2c device descriptor for read/write access to MAX9867 registers */
	struct i2c_client *max9867_i2c;

	/* Char dev to provide user-space ioctl interface for audio control */
	struct cdev cdev;
	dev_t cdev_no;
	struct class *cl;

	/* Mutex to protect against concurrent access to the ioctl() handler */
	struct mutex lock;

	/* Current GPIO switch value */
	unsigned char gpio_val;
};

static int
intel_qrk_audio_ctrl_open(struct inode *inode, struct file *filp)
{
	struct intel_qrk_audio_ctrl_priv *priv;

	priv = container_of(inode->i_cdev,
			    struct intel_qrk_audio_ctrl_priv,
			    cdev);
	filp->private_data = priv;

	return 0;
}

static int
intel_qrk_audio_ctrl_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/*
 * Logic truth table for AUDIO_S[0-3] outputs, illustrating which paths are
 * connected between audio interfaces A, B, C.  Each audio interface has one
 * effective input (I) port and one effective output (O) port
 *
 * A = USB Codec (to Quark CPU)
 * B = Spkr/Mic  (to car audio system)
 * C = I2S Codec (to Telit HE910)
 *
 * PATH examples:
 *   AO-CO: A-Output connected to C-Output
 *   BI-AI: B-Input connected to A-Input
 *
 * NOTE: Assume a CI-AI connection is available in ALL cases (sometimes unused)
 *
 * S2 S1 S0  PATHS             USE CASE
 * -- -- --  ----------------- -------------------------------------------------
 *  0  0  0  AO-CO             BT Headset call
 *  0  0  1  AO-BO             Analog Driver Alerts (CI unused)
 *  0  1  0  AO-CO,BI-AI       XX Unused/invalid (BI *and* CI connected to AI)
 *  0  1  1  AO-BO,BI-AI       Archival Voice Record/Playback (or Driver Alerts)
 *  1  0  0  AO-CO,BI-CO       XX Unused/invalid (A0 *and* BI connected to CO)
 *  1  0  1  AO-BO,BI-CO       Analog hands-free call
 *  1  1  0  AO-CO,BI-AI,BI-CO XX Unused/invalid (BI connected to AI *and* CO)
 *  1  1  1  AO-BO,BI-AI,BI-CO XX Unused/invalid (BI connected to AI *and* CO)
 *
 *
 * Mapping to IOCTLs (using more intuitive naming on the API):
 *
 * PATHS           IOCTL
 * --------------- -------------------------------------------------------------
 * AO-CO           INTEL_QRK_AUDIO_MODE_GSM_ONLY
 * AO-BO           INTEL_QRK_AUDIO_MODE_SPKR_ONLY
 * AO-BO,BI-AI     INTEL_QRK_AUDIO_MODE_SPKR_MIC
 * AO-BO,BI-CO     INTEL_QRK_AUDIO_MODE_GSM_SPKR_MIC
 */

static int
intel_qrk_audio_ctrl_gpio_update(struct intel_qrk_audio_ctrl_priv *priv)
{
	int ret = 0;
	struct gpio audio_sw_gpios[] = {
		{
			GPIO_AUDIO_S2,
			GPIOF_OUT_INIT_LOW,
			"audio_s2"
		},
		{
			GPIO_AUDIO_S1,
			GPIOF_OUT_INIT_LOW,
			"audio_s1"
		},
		{
			GPIO_AUDIO_S0,
			GPIOF_OUT_INIT_LOW,
			"audio_s0"
		}
	};

	/*
	 * Update the Audio Switch GPIO outputs according to the user selection
	 */
	ret = gpio_request_array(audio_sw_gpios,
				 ARRAY_SIZE(audio_sw_gpios));
	if (ret) {
		pr_err("%s: Failed to allocate audio control GPIO pins\n",
		       __func__);
		return ret;
	}

	gpio_set_value(GPIO_AUDIO_S2, (priv->gpio_val >> 2) & 0x1);
	gpio_set_value(GPIO_AUDIO_S1, (priv->gpio_val >> 1) & 0x1);
	gpio_set_value(GPIO_AUDIO_S0, (priv->gpio_val >> 0) & 0x1);

	gpio_free_array(audio_sw_gpios,
			ARRAY_SIZE(audio_sw_gpios));

	return 0;
}

static long
intel_qrk_audio_ctrl_ioctl(struct file *filp,
			   unsigned int cmd,
			   unsigned long arg)
{
	struct intel_qrk_audio_ctrl_priv *priv = filp->private_data;
	int ret = 0;

	ret = mutex_lock_interruptible(&priv->lock);
	if (ret)
		return ret;

	switch (cmd) {
	case INTEL_QRK_AUDIO_MODE_IOC_GSM_ONLY:
	case INTEL_QRK_AUDIO_MODE_IOC_SPKR_ONLY:
	case INTEL_QRK_AUDIO_MODE_IOC_SPKR_MIC:
	case INTEL_QRK_AUDIO_MODE_IOC_GSM_SPKR_MIC:
		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	priv->gpio_val = _IOC_NR(cmd) & 0x7;
	ret = intel_qrk_audio_ctrl_gpio_update(priv);
exit:
	mutex_unlock(&priv->lock);
	return ret;
}

static const struct file_operations intel_qrk_audio_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = intel_qrk_audio_ctrl_open,
	.release = intel_qrk_audio_ctrl_release,
	.unlocked_ioctl = intel_qrk_audio_ctrl_ioctl
};

static int
intel_qrk_audio_ctrl_chrdev_init(struct intel_qrk_audio_ctrl_priv *priv)
{
	/* Register a character dev interface (with ioctls)
	 * to allow control of the audio subsystem switch
	 */
	int ret;
	struct device *dev;

	ret = alloc_chrdev_region(&priv->cdev_no, 0, 1,
				  "intel_qrk_audio_ctrl");
	if (ret) {
		pr_err("Failed to alloc chrdev: %d", ret);
		return ret;
	}

	cdev_init(&priv->cdev, &intel_qrk_audio_ctrl_fops);

	ret = cdev_add(&priv->cdev, priv->cdev_no, 1);
	if (ret) {
		pr_err("Failed to add cdev: %d", ret);
		unregister_chrdev_region(priv->cdev_no, 1);
		return ret;
	}

	priv->cl = class_create(THIS_MODULE, "char");
	if (IS_ERR(priv->cl)) {
		pr_err("Failed to create device class: %ld",
		       PTR_ERR(priv->cl));
		cdev_del(&priv->cdev);
		unregister_chrdev_region(priv->cdev_no, 1);
		return PTR_ERR(priv->cl);
	}

	dev = device_create(priv->cl, NULL, priv->cdev_no, NULL,
			    "intel_qrk_audio_ctrl");
	if (IS_ERR(dev)) {
		pr_err("Failed to create device: %ld",
		       PTR_ERR(priv->cl));
		class_destroy(priv->cl);
		cdev_del(&priv->cdev);
		unregister_chrdev_region(priv->cdev_no, 1);
		return PTR_ERR(dev);
	}

	return 0;
}

static int
intel_qrk_audio_ctrl_chrdev_remove(struct intel_qrk_audio_ctrl_priv *priv)
{
	device_destroy(priv->cl, priv->cdev_no);
	class_destroy(priv->cl);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->cdev_no, 1);

	return 0;
}


ssize_t intel_qrk_audio_ctrl_sysfs_show_mode(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct intel_qrk_audio_ctrl_priv *priv = dev_get_drvdata(dev);
	int ret;
	char *mode;

	ret = mutex_lock_interruptible(&priv->lock);
	if (ret)
		return ret;

	switch (priv->gpio_val) {
	case INTEL_QRK_AUDIO_MODE_GSM_ONLY:
		mode = "gsm";
		break;
	case INTEL_QRK_AUDIO_MODE_SPKR_ONLY:
		mode = "spkr";
		break;
	case INTEL_QRK_AUDIO_MODE_SPKR_MIC:
		mode = "spkr_mic";
		break;
	case INTEL_QRK_AUDIO_MODE_GSM_SPKR_MIC:
		mode = "gsm_spkr_mic";
		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%s\n", mode);

exit:
	mutex_unlock(&priv->lock);
	return ret;
}

ssize_t intel_qrk_audio_ctrl_sysfs_store_mode(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct intel_qrk_audio_ctrl_priv *priv = dev_get_drvdata(dev);
	char mode[16];
	unsigned char gpio_val;
	int ret = count;

	sscanf(buf, "%15s", mode);

	if (!strcmp(mode, "gsm"))
		gpio_val = INTEL_QRK_AUDIO_MODE_GSM_ONLY;
	else if (!strcmp(mode, "spkr"))
		gpio_val = INTEL_QRK_AUDIO_MODE_SPKR_ONLY;
	else if (!strcmp(mode, "spkr_mic"))
		gpio_val = INTEL_QRK_AUDIO_MODE_SPKR_MIC;
	else if (!strcmp(mode, "gsm_spkr_mic"))
		gpio_val = INTEL_QRK_AUDIO_MODE_GSM_SPKR_MIC;
	else
		return -EINVAL;

	ret = mutex_lock_interruptible(&priv->lock);
	if (ret)
		return ret;

	priv->gpio_val = gpio_val;
	ret = intel_qrk_audio_ctrl_gpio_update(priv);
	if (ret)
		goto exit;

	ret = count;

exit:
	mutex_unlock(&priv->lock);

	return ret;
}

/* Sysfs attribute descriptor (for alternative user-space interface) */
static DEVICE_ATTR(audio_switch_mode, S_IWUSR | S_IRUGO,
		   intel_qrk_audio_ctrl_sysfs_show_mode,
		   intel_qrk_audio_ctrl_sysfs_store_mode);

/******************************************************************************
 *                                Module hooks
 ******************************************************************************/

static int
intel_qrk_max9867_init(struct i2c_client *client)
{
	int ret;

	/* MAX9867 register configuration, from Telit HE910 DVI app-note */

	u8 reg_cfg_seq1[] = {
		0x04, /* Starting register address, followed by data */
		0x00, /* 0x04 Interrupt Enable */
		0x10, /* 0x05 System Clock */
		0x90, /* 0x06 Audio Clock High */
		0x00, /* 0x07 Audio Clock Low */
		0x10, /* 0x08 Interface 1a */
		0x0A, /* 0x09 Interface 1d */
		0x33, /* 0x0A Codec Filters */
		0x00, /* 0x0B DAC Gain/Sidetone */
		0x00, /* 0x0C DAC Level */
		0x33, /* 0x0D ADC Level */
		0x4C, /* 0x0E Left Line Input Level */
		0x4C, /* 0x0F Right Line Input Level */
		0x00, /* 0x10 Left Volume Control */
		0x00, /* 0x11 Right Volume Control */
		0x14, /* 0x12 Left Mic Gain */
		0x14, /* 0x13 Right Mic Gain */
		/* Configuration */
		0xA0, /* 0x14 Input */
		0x00, /* 0x15 Microphone */
		0x65  /* 0x16 Mode */
	};

	u8 reg_cfg_seq2[] = {
		0x17, /* Starting register address, followed by data */
		0xEF  /* 0x17 System Shutdown */
	};

	ret = i2c_master_send(client,
			      reg_cfg_seq1, sizeof(reg_cfg_seq1));
	if (ret != sizeof(reg_cfg_seq1)) {
		pr_err("Failed to write MAX9867 config registers (set 1/2)");
		return -EIO;
	}

	ret = i2c_master_send(client,
			      reg_cfg_seq2, sizeof(reg_cfg_seq2));
	if (ret != sizeof(reg_cfg_seq2)) {
		pr_err("Failed to write MAX9867 config registers (set 2/2)");
		return -EIO;
	}

	return 0;
}

static int
intel_qrk_max9867_get_chip_rev(struct i2c_client *client)
{
	struct i2c_msg msg[2];
	u8 data[2];
	int ret;

	data[0] = 0xFF;  /* Chip-revision register address = 0xFF */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &data[0];
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &data[1];
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, &msg[0], 2);
	return (ret == 2) ? data[1] : -EIO;
}

static int intel_qrk_max9867_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct intel_qrk_audio_ctrl_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv),
			    GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->max9867_i2c = client;
	mutex_init(&priv->lock);

	ret = intel_qrk_max9867_get_chip_rev(client);
	if (ret >= 0)
		pr_info("%s: Detected MAX9867 chip revision 0x%02X\n",
			__func__, ret);
	else {
		pr_err("%s: Failed to read MAX9867 chip revision\n", __func__);
		goto exit;
	}

	ret = intel_qrk_max9867_init(client);
	if (ret)
		goto exit;

	priv->gpio_val = GPIO_AUDIO_DEFAULT;
	ret = intel_qrk_audio_ctrl_gpio_update(priv);
	if (ret)
		goto exit;

	/* Create a char dev interface, providing an ioctl config option */
	ret = intel_qrk_audio_ctrl_chrdev_init(priv);
	if (ret)
		goto exit;

	/* Also create a sysfs interface, providing a cmd line config option */
	ret = sysfs_create_file(&client->dev.kobj,
				&dev_attr_audio_switch_mode.attr);

exit:
	return ret;
}

static int intel_qrk_max9867_i2c_remove(struct i2c_client *client)
{
	struct intel_qrk_audio_ctrl_priv *priv = i2c_get_clientdata(client);

	intel_qrk_audio_ctrl_chrdev_remove(priv);

	sysfs_remove_file(&client->dev.kobj, &dev_attr_audio_switch_mode.attr);

	return 0;
}

static const struct i2c_device_id intel_qrk_max9867_i2c_id[] = {
	{"intel-qrk-max9867", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, intel_qrk_max9867_i2c_id);

/* i2c codec control layer */
static struct i2c_driver intel_qrk_audio_ctrl_i2c_driver = {
	.driver = {
		.name = "intel_qrk_audio_ctrl",
		.owner = THIS_MODULE,
	},
	.probe = intel_qrk_max9867_i2c_probe,
	.remove =  intel_qrk_max9867_i2c_remove,
	.id_table = intel_qrk_max9867_i2c_id,
};

module_i2c_driver(intel_qrk_audio_ctrl_i2c_driver);

MODULE_AUTHOR("Dan O'Donovan <dan@emutex.com>");
MODULE_DESCRIPTION("Intel Quark platform audio control driver");
MODULE_LICENSE("Dual BSD/GPL");
