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
 * Intel Quark IMR driver
 *
 * IMR stand for Insolate Memory Region, supported by Quark SoC.
 *
 * A total number of 8 IMRs have implemented by Quark SoC,
 * Some IMRs might be already occupied by BIOS or Linux during
 * booting time.
 *
 * A user can cat /sys/devices/platform/intel-qrk-imr/status for current IMR
 * status 
 *
 * To allocate an IMR addresses must be alinged to 1k
 *
 * The IMR alloc API will locate the next available IMR slot set up
 * with input memory region, then apply the input access right masks
 *
 * The IMR can be freed with the pre-allocated memory addresses.
 */

#include <asm-generic/uaccess.h>
#include <linux/intel_qrk_sb.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/proc_fs.h>

#include "intel_qrk_imr.h"
#include <asm/imr.h>

#define DRIVER_NAME	"intel-qrk-imr"

#define IMR_READ_MASK	0x1
#define IMR_MAX_ID	7

#ifndef phys_to_virt
#define phys_to_virt __va
#endif

/* IMR HW register address structre */
struct qrk_imr_reg_t {
	u8  imr_xl;   /* high address register */
	u8  imr_xh;   /* low address register */
	u8  imr_rm;   /* read mask register */
	u8  imr_wm;   /* write mask register */
} qrk_imr_reg_t;

/**
 * struct qrk_imr_addr_t
 *
 * IMR memory address structure
 */
struct qrk_imr_addr_t {
	u32 addr_low;      /* low boundary memroy address */
	u32 addr_high;     /* high boundary memory address */
	u32 read_mask;     /* read access right mask */
	u32 write_mask;    /* write access right mask */
} qrk_imr_addr_t;

/**
 * struct qrk_imr_pack
 *
 * local IMR pack structure
 */
struct qrk_imr_pack {
	bool occupied;       /* IMR occupied */
	bool locked;         /* IMR lock */
	struct qrk_imr_reg_t reg;   /* predefined imr register address */
	struct qrk_imr_addr_t addr; /* IMR address region structure */
	unsigned char info[MAX_INFO_SIZE]; /* IMR info */
} qrk_imr_pack;


/* Predefined HW register address */
static struct qrk_imr_reg_t imr_reg_value[] = {
	{ IMR0L, IMR0H, IMR0RM, IMR0WM },
	{ IMR1L, IMR1H, IMR1RM, IMR1WM },
	{ IMR2L, IMR2H, IMR2RM, IMR2WM },
	{ IMR3L, IMR3H, IMR3RM, IMR3WM },
	{ IMR4L, IMR4H, IMR4RM, IMR4WM },
	{ IMR5L, IMR5H, IMR5RM, IMR5WM },
	{ IMR6L, IMR6H, IMR6RM, IMR6WM },
	{ IMR7L, IMR7H, IMR7RM, IMR7WM }
};

static struct platform_device *pdev;

/**
 * module parameter
 * IMR slot should repersant the available IMR region from
 * linux boot and BIOS.
 *
 * For example: imr_bit_mask = 0x10111001
 * occupied IMR: 0, 3, 4, 5, 7
 * un-occupied IMR: 1, 2, 6
 */
static int imr_bit_mask = 0xFF;
module_param(imr_bit_mask, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(imr_bit_mask, "IMR bit mask");

/**
 * module parameter
 * if IMR lock is a nozero value, all unlocked
 * imrs will be locked regardless the usage.
 */
static int imr_lock = 0;
module_param(imr_lock, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(imr_lock, "switch to lock unused IMR");

/* local IMR data structure */
struct qrk_imr_pack local_imr[IMR_MAXID];

static unsigned short host_id;

/**
 * intel_qrk_imr_read_reg
 *
 * @param reg: register address
 * @return nothing
 *
 * return register value from input address.
 */
static inline uint32_t intel_qrk_imr_read_reg(uint8_t reg)
{
	uint32_t temp = 0;

	intel_qrk_sb_read_reg(SB_ID_ESRAM, CFG_READ_OPCODE, reg, &temp, 0);
	return temp;
}

/**
 * intel_clm_imr_latch_data
 *
 * @return nothing
 *
 * Populate IMR data structure from HW.
 */
static inline void intel_clm_imr_latch_data(void)
{
	int i = 0;

	for (i = 0; i < IMR_MAXID; i++) {

		local_imr[i].addr.addr_low =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_xl);
		local_imr[i].addr.addr_high =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_xh);
		local_imr[i].addr.read_mask =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_rm);
		local_imr[i].addr.write_mask =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_wm);

		if (local_imr[i].addr.addr_low & IMR_LOCK_BIT)
			local_imr[i].locked = true;

		if (local_imr[i].addr.read_mask > 0 &&
			local_imr[i].addr.read_mask < IMR_READ_ENABLE_ALL){
			local_imr[i].occupied = true;
		} else {
			local_imr[i].occupied = false;
			memcpy(local_imr[i].info, "NOT USED", MAX_INFO_SIZE);
		}
	}
}

/**
 * prepare_input_addr
 *
 * @param addr: input physical memory address
 * @return formated memory address
 *
 * 1. verify input memory address alignment
 * 2. apply IMR_REG_MASK to match the format required by HW
 */
static inline uint32_t prepare_input_addr(uint32_t addr)
{
	if (addr & (IMR_MEM_ALIGN - 1))
		return 0;

	addr = (addr >> 8) & IMR_REG_MASK;
	return addr;
}

/**
 * intel_qrk_imr_find_free_entry
 *
 * @return the next free imr slot
 */
static int intel_qrk_imr_find_free_entry(void)
{
	int i = 0;

	intel_clm_imr_latch_data();

	for (i = 0; i < IMR_MAXID; i++) {
		if ((!local_imr[i].occupied) && (!local_imr[i].locked))
			return i;
	}

	pr_err("%s: No more free IMR available.\n", __func__);
	return -ENOMEM;
}


/**
 * sb_write_chk
 *
 * @param id: Sideband identifier
 * @param cmd: Command to send to destination identifier
 * @param reg: Target register w/r to qrk_sb_id
 * @param data: Data to write to target register
 * @return nothing
 *
 * Set SB register and read back to verify register has been updated.
 */
static void sb_write_chk(qrk_sb_id id, u8 cmd, u8 reg, u32 data)
{
	u32 data_verify = 0;

	intel_qrk_sb_write_reg(id, cmd, reg, data, 0);
	intel_qrk_sb_read_reg(id, cmd, reg, &data_verify, 0);
	BUG_ON(data != data_verify);
}

/**
 * imr_add_entry
 *
 * @param id: imr slot id
 * @param hi: hi memory address
 * @param lo: lo memory address
 * @param read: read access mask
 * @param write: write access mask
 * @return nothing
 *
 */
static inline void imr_add_entry(int id, uint32_t hi, uint32_t lo,
				 uint32_t read, uint32_t write, bool lock)
{
	u32 val = 0;

	intel_qrk_sb_read_reg(SB_ID_ESRAM, CFG_READ_OPCODE,
			      imr_reg_value[id].imr_xl, &val, 0);
	val &= ~IMR_EN;
	sb_write_chk(SB_ID_ESRAM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_xl,
		     val);

	sb_write_chk(SB_ID_ESRAM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_rm,
		     read);
	sb_write_chk(SB_ID_ESRAM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_wm,
		     write);
	sb_write_chk(SB_ID_ESRAM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_xh,
		      hi);
	sb_write_chk(SB_ID_ESRAM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_xl,
		      lo);
}

/**
 * x1000_imr_add_entry
 *
 * @param id: imr slot id
 * @param hi: hi memory address
 * @param lo: lo memory address
 * @param read: read access mask
 * @param write: write access mask
 * @return nothing
 *
 */
static inline void x1000_imr_add_entry(int id, uint32_t hi, uint32_t lo,
				       uint32_t read, uint32_t write, bool lock)
{
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_xl, lo, 0);
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_xh, hi, 0);
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_rm, read, 0);
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_wm, write, 0);
}

/**
 * intel_qrk_imr_add_entry
 *
 * @param id: imr slot id
 * @param hi: hi memory address
 * @param lo: lo memory address
 * @param read: read access mask
 * @param write: write access mask
 * @return nothing
 *
 * Setup an IMR entry
 */
static void intel_qrk_imr_add_entry(int id, uint32_t hi,
		uint32_t lo, uint32_t read, uint32_t write, bool lock)
{
	if (PCI_DEVICE_ID_X1000_HOST_BRIDGE == host_id)
		x1000_imr_add_entry(id, hi, lo, read, write, lock);
	else
		imr_add_entry(id, hi, lo, read, write, lock);

	if (lock) {
		lo |= IMR_LOCK_BIT;
		intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
					imr_reg_value[id].imr_xl, lo, 0);
	}
}

/**
 * get_phy_addr
 * @return phy address value
 *
 * convert register format to physical address format.
 */
static uint32_t get_phy_addr(uint32_t reg_value)
{
	reg_value = ((reg_value & IMR_REG_MASK) << 8);
	return reg_value;
}



/**
 * intel_qrk_imr_init_mask
 *
 * @param mask: module parameter
 * @return nothing
 *
 * prepare local IMR data structure from input module parameter.
 */
static void intel_qrk_imr_init_mask(int mask)
{
	int i = 0;

	BUG_ON((mask > 255 || mask < 0));

	for (i = 0; i < IMR_MAXID; i++) {
		local_imr[i].addr.addr_low =
			intel_qrk_imr_read_reg(imr_reg_value[i].imr_xl);

		/* mask bit 1 means imr occupied*/
		if (((mask>>i) & IMR_READ_MASK) == 0) {
			if (!(local_imr[i].addr.addr_low & IMR_LOCK_BIT))
				intel_qrk_remove_imr_entry(i);
		}
	}
}

/**
 * imr_rm_entry
 *
 * @param id: imr slot id
 * @return nothing
 *
 */
static inline void imr_rm_entry(int id)
{
	u32 val = 0;

	intel_qrk_sb_read_reg(SB_ID_ESRAM, CFG_READ_OPCODE,
			      imr_reg_value[id].imr_xl, &val, 0);
	val &= ~IMR_EN;
	sb_write_chk(SB_ID_ESRAM, CFG_WRITE_OPCODE, imr_reg_value[id].imr_xl,
		     val);
}

/**
 * x1000_imr_rm_entry
 *
 * @param id: imr slot id
 * @return nothing
 *
 */
static inline void x1000_imr_rm_entry(int id)
{
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_rm, IMR_READ_ENABLE_ALL,
				0);
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_wm, IMR_WRITE_ENABLE_ALL,
				0);
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_xl, IMR_BASE_ADDR, 0);
	intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
				imr_reg_value[id].imr_xh, IMR_BASE_ADDR, 0);
}

/**
 * intel_qrk_remove_imr_entry
 *
 * @param id: imr slot id
 * @return nothing
 *
 * remove imr slot based on input id
 */
void intel_qrk_remove_imr_entry(int id)
{
	if (id >= IMR_MAXID || local_imr[id].locked)
		return;

	if (PCI_DEVICE_ID_X1000_HOST_BRIDGE == host_id)
		x1000_imr_rm_entry(id);
	else
		imr_rm_entry(id);

	intel_clm_imr_latch_data();

}
EXPORT_SYMBOL(intel_qrk_remove_imr_entry);

/**
 * intel_qrk_imr_alloc
 *
 * @param high: high boundary of memory address
 * @param low: low boundary of memorry address
 * @param read: IMR read mask value
 * @param write: IMR write mask value
 * @return nothing
 *
 * setup the next available IMR with customized read and write masks
 */
int intel_qrk_imr_alloc(uint32_t high, uint32_t low, uint32_t read,
			uint32_t write, unsigned char *info, bool lock)
{
	int id = 0;

	if (info == NULL)
		return -EINVAL;

	if ((low & IMR_LOCK_BIT) || (read == 0 || write == 0)) {
		pr_err("%s: Invalid acces mode\n", __func__);
		return -EINVAL;
	}

	/* Calculate aligned addresses and validate range */
	high = prepare_input_addr(high);
	low = prepare_input_addr(low);

	/* Find a free entry */
	id = intel_qrk_imr_find_free_entry();
	if (id < 0)
		return -ENOMEM;

	/* Add entry - locking as necessary */
	intel_qrk_imr_add_entry(id, high, low, (read & IMR_READ_ENABLE_ALL),
				write, lock);

	/* Name the new entry */
	memcpy(local_imr[id].info, info, MAX_INFO_SIZE);

	/* Update local data structures */
	intel_clm_imr_latch_data();

	pr_info("IMR alloc id %d 0x%08x - 0x%08x %s\n", id, low, high,
		lock ? "locked" : "unlocked");

	return 0;
}
EXPORT_SYMBOL(intel_qrk_imr_alloc);

/**
 * intel_qrk_imr_free
 *
 * @param high: high boundary of memory address
 * @param low: low boundary of memorry address
 * @return nothing
 *
 * remove the imr based on input memory region
 */
int intel_qrk_imr_free(uint32_t high, uint32_t low)
{
	int i = 0;

	if (low > high) {
		pr_err("%s: Invalid input address values.\n", __func__);
		return -EINVAL;
	}

	high = prepare_input_addr(high);
	if (!high) {
		pr_err("%s: Invalid input memory address.\n", __func__);
		return -EINVAL;
	}

	low = prepare_input_addr(low);
	if (!low) {
		pr_err("%s: Invalid input memory address.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < IMR_MAXID; i++) {
		if (local_imr[i].occupied
			&& (local_imr[i].addr.addr_low == low)
			&& (local_imr[i].addr.addr_high == high)
			&& (!local_imr[i].locked)) {
				intel_qrk_remove_imr_entry(i);
				return 0;
			}
	}

	return -EINVAL;
}
EXPORT_SYMBOL(intel_qrk_imr_free);

/**
 * intel_qrk_imr_init_data
 *
 * @return nothing
 * initialize local_imr data structure
 */
static void intel_qrk_imr_init_data(void)
{
	int i = 0;
	char * res_str = "System Reserved Region";
	int len = strlen(res_str);

	intel_clm_imr_latch_data();

	for (i = 0; i < IMR_MAXID; i++) {
		local_imr[i].reg = imr_reg_value[i];
		memcpy(local_imr[i].info, res_str, len);
	}
}

/**
 * intel_qrk_imr_lockall
 *
 * @param mask: module parameter
 * @return nothing
 *
 * lock up all un-locked IMRs
 */
int intel_qrk_imr_lockall(void)
{
	int i = 0;
	uint32_t temp_addr;

	/* Enumerate IMR data structures */
	intel_qrk_imr_init_data();
	intel_qrk_imr_init_mask(imr_bit_mask);

	/* Cycle through IMRs locking whichever are unlocked */
	for (i = 0; i < IMR_MAXID; i++) {

		temp_addr = local_imr[i].addr.addr_low;
		if (!(temp_addr & IMR_LOCK_BIT)) {

			DBG("%s: locking IMR %d\n", __func__, i);
			temp_addr |= IMR_LOCK_BIT;
			intel_qrk_sb_write_reg(SB_ID_ESRAM, CFG_WRITE_OPCODE,
						local_imr[i].reg.imr_xl,
						temp_addr, 0);
		}
	}

	return 0;
}
EXPORT_SYMBOL(intel_qrk_imr_lockall);

/**
 * intel_qrk_imr_stat_show
 *
 * @param dev: pointer to device
 * @param attr: attribute pointer
 * @param buf: output buffer
 * @return number of bytes successfully read
 *
 * Populates IMR state via /sys/device/intel-qrk-imr/stat
 */
static int intel_qrk_imr_stat_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int len = 0;
	int i = 0;
	int size, count = PAGE_SIZE;
	uint32_t hi_phy_addr, lo_phy_addr;

	for (i = 0; i < IMR_MAXID; i++) {

		/* read back the actual  input physical memory address */
		hi_phy_addr = get_phy_addr(local_imr[i].addr.addr_high);
		lo_phy_addr = get_phy_addr(local_imr[i].addr.addr_low);

		/* the IMR always protect extra 1k memory size above the input
		 * high reg value
		 */
		size = ((hi_phy_addr - lo_phy_addr) / IMR_MEM_ALIGN) + 1;

		size = snprintf(buf+len, count,
				"imr - id : %d\n"
				"info     : %s\n"
				"occupied : %s\n"
				"locked   : %s\n"
				"size     : %d kb\n"
				"hi addr (phy): 0x%08x\n"
				"lo addr (phy): 0x%08x\n"
				"hi addr (vir): 0x%08x\n"
				"lo addr (vir): 0x%08x\n"
				"read mask  : 0x%08x\n"
				"write mask : 0x%08x\n\n",
				i,
				local_imr[i].info,
				local_imr[i].occupied ? "yes" : "no",
				local_imr[i].locked ? "yes" : "no",
				size,
				hi_phy_addr,
				lo_phy_addr,
				(uint32_t)phys_to_virt(hi_phy_addr),
				(uint32_t)phys_to_virt(lo_phy_addr),
				local_imr[i].addr.read_mask,
				local_imr[i].addr.write_mask);
		len += size;
		count -= size;
	}
	return len;
}

static struct device_attribute dev_attr_stats = {
	.attr = {
		.name = "stat",
		.mode = 0444, },
	.show = intel_qrk_imr_stat_show,
};

static struct attribute *platform_attributes[] = {
	&dev_attr_stats.attr,
	NULL,
};

static struct attribute_group imr_attrib_group = {
	.attrs = platform_attributes
};

/**
 * intel_qrk_imr_init
 *
 * @param dev_id: Host Bridge's PCI device ID
 * @return 0 success < 0 failue
 *
 * module entry point
 */
int intel_qrk_imr_init(unsigned short dev_id)
{
	int ret;

	host_id = dev_id;

	pdev = platform_device_alloc(DRIVER_NAME, -1);
	if (!pdev)
		return -ENOMEM;

	ret = platform_device_add(pdev);
	if (ret)
		goto fail_platform;

	/* initialise local imr data structure */
	intel_qrk_imr_init_data();

	ret = sysfs_create_group(&pdev->dev.kobj, &imr_attrib_group);
	if (ret)
		goto fail_platform;

	if(intel_qrk_imr_runt_setparams() == 0 && imr_lock == 1){
                intel_qrk_imr_lockall();
        }

	return 0;

fail_platform:
	platform_device_del(pdev);
	return ret;
}
EXPORT_SYMBOL(intel_qrk_imr_init);

MODULE_DESCRIPTION("Intel Quark SOC IMR API ");
MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("Dual BSD/GPL");

