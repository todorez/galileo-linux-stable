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
 * Intel Quark eSRAM overlay driver
 *
 * eSRAM is an on-chip fast access SRAM.
 *
 * This driver provides the ability to map a kallsyms derived symbol of
 * arbitrary length or a struct page entitiy.
 * A sysfs interface is provided to allow map of kernel structures, without
 * having to use the API from your code directly.
 *
 * Example:
 * echo idt_table > /sys/devices/intel-qrk-esram.0/map
 *
 * An API is provided to allow for mapping of a) kernel symbols or b) pages.
 * eSRAM requires 4k physically aligned addresses to work - so a struct page
 * fits neatly into this.
 *
 * intel_qrk_esram_map_sym(ohci_irq);
 * intel_qrk_esram_map_page(virt_to_page(ohci_irq), "ohci_irq");
 * Are equivalent - with the exception that map_sym() can detect if a mapping
 * crosses a page-boundary, whereas map_page just maps one page. Generally use
 * map_sym() for code and map_page() for data
 *
 * To populte eSRAM we must copy data to a temporary buffer, overlay and
 * then copy data back to the eSRAM region.
 * 
 * When entering S3 - we must save eSRAM state to DRAM, and similarly on restore
 * to S0 we must repopulate eSRAM
 * Unmap code is included for reference however the cache coherency of unmap is
 * not guaranteed so the functionality is not exported by this code
 * 
 */
#include <asm/cacheflush.h>
#include <asm/desc.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/special_insns.h>
#include <asm-generic/uaccess.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/intel_qrk_sb.h>
#include <linux/kallsyms.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>

#include "intel_qrk_esram.h"

#define DRIVER_NAME			"intel-qrk-esram"

/* Shorten fn names to fit 80 char limit */
#ifndef sb_read
#define sb_read				intel_qrk_sb_read_reg
#endif
#ifndef sb_write
#define sb_write			intel_qrk_sb_write_reg
#endif

/* Define size of pages, ECC scrub demark etc */
#define MAX_PAGE_RETRIES		(100)
#define MS_PER_HOUR			(3600000UL)
#define ESRAM_PAGE_COUNT		INTEL_QRK_ESRAM_PAGE_COUNT
#define ESRAM_PAGE_MASK			(0xFFFFF000)
#define ESRAM_PAGE_SIZE			INTEL_QRK_ESRAM_PAGE_SIZE
#define ESRAM_TOTAL_SIZE		(ESRAM_PAGE_COUNT * ESRAM_PAGE_SIZE)
#define ECC_MAX_REFRESH_PERIOD		(48)
#define ECC_DEFAULT_REFRESH_PERIOD	(24)
#define ECC_DRAM_READSIZE		(512)		/* bytes per DRAM ECC */
#define ECC_ESRAM_READSIZE		ESRAM_PAGE_SIZE	/* bytes per SRAM ECC */

/* Register ID */
#define ESRAM_PGPOOL_REG		(0x80)		/* PGPOOL */
#define ESRAM_CTRL_REG			(0x81)		/* ESRAMCTRL */
#define ESRAM_PGBLOCK_REG		(0x82)		/* Global page ctrl */
#define ESCRM_ECCCERR_REG		(0x83)		/* Correctable ECC */
#define ESRAM_ECCUCERR_REG		(0x84)		/* Uncorrectable ECC */

/* Reg commands */
#define ESRAM_CTRL_READ			(0x10)		/* Config reg */
#define ESRAM_CTRL_WRITE		(0x11)		/* Config reg */
#define ESRAM_PAGE_READ			(0x12)		/* Page config read */
#define ESRAM_PAGE_WRITE		(0x13)		/* Page config write */

/* ESRAMPGPOOL reg 0x80 - r/w opcodes 0x10/0x11 */
#define ESRAM_PGPOOL_FLUSHING(x)	((x>>18)&0x1FF)
#define ESRAM_PGPOOL_PGBUSY(x)		((x>>9)&0x1FF)

/* ESRAMCTRL reg 0x81 - r/w opcodes 0x10/0x11 */
#define ESRAM_CTRL_FLUSHPRI(x)		((x>>25)&0x03)	/* DRAM flush priority */
#define ESRAM_CTRL_SIZE(x)		((x>>16)&0xFF)	/* # of 4k pages */
#define ESRAM_CTRL_ECCTHRESH(x)		((x>>8)&0xFF)	/* ECC threshold */
#define ESRAM_CTRL_THRESHMSG_EN		(0x00000080)	/* ECC notification */
#define ESRAM_CTRL_ISAVAIL		(0x00000010)	/* ESRAM on die ? */
#define ESRAM_CTRL_BLOCK_MODE		(0x00000008)	/* Block mode enable */
#define ESRAM_CTRL_GLOBAL_LOCK		(0x00000004)	/* Global lock status */
#define ESRAM_CTRL_FLUSHDISABLE		(0x00000002)	/* Global flush/dis */
#define ESRAM_CTRL_SECDEC		(0x00000001)	/* ECC enable bit */

/* PGBLOCK reg 0x82 - opcode 0x10/0x11 */
#define ESRAM_PGBLOCK_FLUSHEN		(0x80000000)	/* Block flush enable */
#define ESRAM_PGBLOCK_PGFLUSH		(0x40000000)	/* Flush the block */
#define ESRAM_PGBLOCK_DISABLE		(0x20000000)	/* Block mode disable */
#define ESRAM_PGBLOCK_ENABLE		(0x10000000)	/* Block mode enable */
#define ESRAM_PGBLOCK_LOCK		(0x08000000)	/* Block mode lock en */
#define ESRAM_PGBLOCK_INIT		(0x04000000)	/* Block init in prog */
#define ESRAM_PGBLOCK_BUSY		(0x01000000)	/* Block is enabled */
#define ESRAM_PGBLOCK_SYSADDR(x)	(x&0x000000FF)

/* ESRAMPGCTRL - opcode 0x12/0x13 */
#define ESRAM_PAGE_FLUSH_PAGE_EN	(0x80000000)	/* S3 autoflush */
#define ESRAM_PAGE_FLUSH		(0x40000000)	/* Flush page to DRAM */
#define ESRAM_PAGE_DISABLE		(0x20000000)	/* page disable bit */
#define ESRAM_PAGE_EN			(0x10000000)	/* Page enable */
#define ESRAM_PAGE_LOCK			(0x08000000)	/* Page lock en */
#define ESRAM_PAGE_INITIALISING		(0x04000000)	/* Init in progress */
#define ESRAM_PAGE_BUSY			(0x01000000)	/* Page busy */
#define ESRAM_PAGE_MAP_SHIFT		(12)		/* Shift away 12 LSBs */

/* Extra */
#define ESRAM_MAP_OP			(0x01)
#define ESRAM_UNMAP_OP			(0x00)

/**
 * struct esram_refname
 *
 * Structure to hold a linked list of names
 */
struct esram_refname {
	char name[KSYM_SYMBOL_LEN];	/* Name of mapping */
	struct list_head list;
};

/**
 * struct esram_page
 *
 * Represents an eSRAM page in our linked list
 */
struct esram_page {

	struct list_head list;		/* List entry descriptor */
	struct list_head name_list;	/* Linked list for name references */
	u32 id;				/* Page ID */
	u32 phys_addr;			/* Physial address of page */
	u32 refcount;			/* Reference count */
	u32 vaddr;			/* Virtual address of page */

};

/**
 * struct intel_qrk_esram_dev
 *
 * Structre to represent module state/data/etc
 */
struct intel_qrk_esram_dev{

	/* Linux kernel structures */
	struct list_head page_used;	/* Used pages */
	struct list_head page_free;	/* Free pages */
	spinlock_t slock;		/* Spinlock */
	struct platform_device *pldev;	/* Platform device */

	/* Actual data */
	struct esram_page * pages;
	u8 cbuf[ESRAM_PAGE_SIZE];

	/* Stats */
	u32 page_count;			/* As reported by silicon */
	u32 page_disable_retries;	/* Aggreate count on disable */
	u32 page_enable_retries;	/* Aggregate spin count page enable */
	u32 page_free_ct;		/* Free pages for mapping code section */
};

static struct intel_qrk_esram_dev esram_dev;

/* 
 * Kallsyms does not provide data addresses. To map important structures such as
 * the idt and gdt, we need to frig the lookup with the below. Other entities
 * can similarly be added. Note we map a page from the given address - anything
 * larger will require additional code to handle
 */
struct esram_symex {
	char * name;
	void * vaddr;
	u32 size;
};

static struct esram_symex esram_symex[] = 
{
	{
		.name = "idt_table",
		.vaddr = &idt_table,
		.size = ESRAM_PAGE_SIZE,
	},
	{
		.name = "gdt_page",
		.vaddr = &gdt_page,
		.size = ESRAM_PAGE_SIZE,
	},
};

/**
 * intel_qrk_esram_stat_show
 *
 * @param dev: pointer to device
 * @param attr: attribute pointer
 * @param buf: output buffer
 * @return number of bytes successfully read
 *
 * Populates eSRAM state via /sys/device/intel-qrk-esram.0/stat
 */
static ssize_t intel_qrk_esram_stat_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)

{
	struct esram_page * epage = NULL;
	int len = 0;
	unsigned int count = PAGE_SIZE, size;
	u32 pgpool = 0, ctrl = 0, pgblock = 0;
	char * enabled = "enabled";
	char * disabled = "disabled";

	/* Display page-pool relevant data */
	sb_read(SB_ID_ESRAM, ESRAM_CTRL_READ, ESRAM_PGPOOL_REG, &pgpool, 1);
	size = snprintf(buf, count,
			"esram-pgpool\t\t\t: 0x%08x\n" 
			"esram-pgpool.free\t\t: %u\n"
			"esram-pgpool.flushing\t\t: %u\n",
			pgpool,	ESRAM_PGPOOL_PGBUSY(pgpool)+1,
			ESRAM_PGPOOL_FLUSHING(pgpool) + 1);
	len += size;
	count -= size;
	
	/* Display ctrl reg - most of this is of interest */
	sb_read(SB_ID_ESRAM, ESRAM_CTRL_READ, ESRAM_CTRL_REG, &ctrl, 1);
	size = snprintf(buf + len, count - len,
			"esram-ctrl\t\t\t: 0x%08x\n"
			"esram-ctrl.ecc\t\t\t: %s\n"
			"esram-ctrl.ecc-theshold\t\t: %u\n"
			"esram-ctrl.pages\t\t: %u\n"
			"esram-ctrl.dram-flush-priorityi\t: %u\n",
			ctrl, (ctrl & ESRAM_CTRL_SECDEC) ? enabled : disabled,
			ESRAM_CTRL_ECCTHRESH(ctrl), ESRAM_CTRL_SIZE(ctrl)+1,
			ESRAM_CTRL_FLUSHPRI(ctrl));
	len += size;
	count -= size;

	/* Display block ctrl/stat - we should be !block mode */
	sb_read(SB_ID_ESRAM, ESRAM_CTRL_READ, ESRAM_PGBLOCK_REG, &pgblock, 1);
	size = snprintf(buf + len, count - len, "esram-block\t\t\t: 0x%08x\n",
			pgblock);
	len += size;
	count -= size;

	/* Print ECC status regs */

	/* Print per-page info */
	size = snprintf(buf + len, count - len, 
			"free page\t\t\t: %u\nused page\t\t\t: %u\n"
			"refresh  \t\t\t: %ums\npage enable retries\t\t: %u\n"
			"page disable retries\t: %u\n",
			esram_dev.page_free_ct, 
			esram_dev.page_count-esram_dev.page_free_ct,
			0,
			esram_dev.page_enable_retries,
			esram_dev.page_disable_retries);
	len += size;
	count -= size;

	spin_lock(&esram_dev.slock);
	if(!list_empty(&esram_dev.page_free)){

		epage = list_first_entry(&esram_dev.page_free, struct esram_page, list);
		size = snprintf(buf + len, count - len, 
			"ecc next page \t\t\t: %u\n",epage->id);
		len += size;
		count -= size;


	}
	spin_unlock(&esram_dev.slock);

	/* Return len indicate eof */
	return len;
}

/**
 * intel_qrk_esram_map_show
 *
 * @param dev: pointer to device
 * @param attr: attribute pointer
 * @param buf: output buffer
 * @return number of bytes successfully read
 * 
 * Read back eSRAM mapped entries
 */
static ssize_t
intel_qrk_esram_map_show(struct device *dev,struct device_attribute *attr,
			 char *buf)
{
	struct esram_page * epage = NULL;
	struct esram_refname * refname = NULL;
	int len = 0, size = 0;
	unsigned int count = PAGE_SIZE;

	spin_lock(&esram_dev.slock);
	list_for_each_entry(epage, &esram_dev.page_used, list){
		/* Print references */
		list_for_each_entry(refname, &epage->name_list, list){
			size = snprintf(buf + len, count - len,
				"%s ", refname->name);
			len += size;
			count -= size;
		}
		/* Print data */
		size += snprintf(buf + len, count - len,
			"\n\tPage virt 0x%08x phys 0x%08x\n"
			"\tRefcount %u\n",
			epage->vaddr, epage->phys_addr,
			epage->refcount);
		len += size;
		count -= size;
	}
	spin_unlock(&esram_dev.slock);

	/* Return len indicate eof */
	return len;
}

/**
 * intel_qrk_esram_map_store
 *
 * @param dev: pointer to device
 * @param attr: attribute pointer
 * @param buf: input buffer
 * @param size: size of input data
 * @return number of bytes successfully written
 *
 * Function allows user-space to switch mappings on/off with a simple
 * echo idt_table > /sys/devices/intel-qrk-esram.0/map type command
 */
static ssize_t
intel_qrk_esram_map_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t size)
{
	ssize_t ret = 0;
	char * sbuf = NULL;
	unsigned long vaddr = 0, i = 0;
	unsigned int count = PAGE_SIZE;

	if(count <= 1){
		return -EINVAL;
	}

	/* Get input */
	sbuf = (char*)buf;

	/* Fixup entity to scrub spaces */
	while(sbuf < (buf + count)){
		if(*sbuf == ' ' || *sbuf == '\r' || *sbuf =='\n'){
			*sbuf = 0;
			break;
		}
		sbuf++;
	}

	/* Check to see if we are being asked to map a non-kallsyms addr */
	for(i = 0; i < sizeof(esram_symex)/sizeof(struct esram_symex); i++){
		if(strcmp(buf, esram_symex[i].name) == 0){
			ret = intel_qrk_esram_map_range(
				esram_symex[i].vaddr,
				esram_symex[i].size,
				esram_symex[i].name);
			goto done;
		}
	}

	/* This path relies on kallsyms to provide name/address data */
	vaddr = kallsyms_lookup_name(buf);
	if(vaddr == 0)
		goto done;

	ret = intel_qrk_esram_map_symbol((void*)vaddr);
done:
	if(ret == 0)
		ret = (ssize_t)count;
	return ret;
}

static struct device_attribute dev_attr_stats = {
	.attr = {
		.name = "stats",
		.mode = 0444,
		},
	.show = intel_qrk_esram_stat_show,
};

static struct device_attribute dev_attr_map = {
	.attr = {
		.name = "map",
		.mode = 0644,
		},
	.show = intel_qrk_esram_map_show,
	.store = intel_qrk_esram_map_store,
};

static struct attribute *platform_attributes[] = {
	&dev_attr_stats.attr,
	&dev_attr_map.attr,
	NULL,
};

static struct attribute_group esram_attrib_group = {
	.attrs = platform_attributes
};

/******************************************************************************
 *                                eSRAM Core
 ******************************************************************************/

/**
 * intel_qrk_esram_page_busy
 *
 * @param epage: Pointer to the page descriptor
 * @return boolean indicating whether or not a page is enabled
 */
static int intel_qrk_esram_page_busy(struct esram_page * epage, u8 lock)
{
	u32 reg = 0;

	sb_read(SB_ID_ESRAM, ESRAM_PAGE_READ, epage->id, &reg, lock);
	return (reg&(ESRAM_PAGE_BUSY | ESRAM_PAGE_FLUSH | ESRAM_PAGE_DISABLE));
}

/**
 * intel_qrk_esram_fault
 *
 * Dump eSRAM registers and kernel panic
 * Nothing else to do at this point
 */
void intel_qrk_esram_fault(struct esram_page * epage, u32 lineno)
{
	u32 reg = 0, next = 0, prev = 0, prev_reg = 0;
	u32 next_reg = 0, block = 0, ctrl = 0;

	pr_err("eSRAM: fault @ %s:%d\n", __FILE__, lineno);
	sb_read(SB_ID_ESRAM, ESRAM_PAGE_READ, epage->id, &reg, 1);
	pr_err("read page %d state 0x%08x\n", epage->id, reg);
	if(epage->id == 0){
		next = 1; prev = 127;
	}else if(epage->id == 127){
		next = 0; prev = 126;
	}else{
		next = epage->id+1;
		prev = epage->id-1;
	}
	sb_read(SB_ID_ESRAM, ESRAM_PAGE_READ, next, &next_reg, 1);
	sb_read(SB_ID_ESRAM, ESRAM_PAGE_READ, prev, &prev_reg, 1);

	/* Get state */
	sb_read(SB_ID_ESRAM, ESRAM_CTRL_READ, ESRAM_CTRL_REG, &ctrl, 1);
	sb_read(SB_ID_ESRAM, ESRAM_CTRL_READ, ESRAM_PGBLOCK_REG, &block, 1);

	pr_err("eSRAM: CTRL 0x%08x block 0x%08x\n", ctrl, block);
	pr_err("Prev page %d state 0x%08x Next page %d state 0x%08x\n"
		, next, next_reg, prev, prev_reg);
	BUG();
}


/**
 * intel_qrk_esram_page_enable
 *
 * @param epage: struct esram_page carries data to program to register
 * @param lock: Indicates whether to attain sb spinlock or not
 * 
 * Enable an eSRAM page spinning for page to become ready.
 */
static void intel_qrk_esram_page_enable(struct esram_page *epage, u8 lock)
{
	u32 ret = 0;

	/* Fault if we try to enable a disabled page */
	if(intel_qrk_esram_page_busy(epage, lock)){
		intel_qrk_esram_fault(epage, __LINE__);
	}

	/* Program page mapping */
	sb_write(SB_ID_ESRAM, ESRAM_PAGE_WRITE, epage->id, 
		ESRAM_PAGE_FLUSH_PAGE_EN | ESRAM_PAGE_EN | 
			(epage->phys_addr>>ESRAM_PAGE_MAP_SHIFT), lock);
	do {
		/* Poll until page busy bit becomes true */
		ret = intel_qrk_esram_page_busy(epage, lock);

		/* This branch should rarely if ever be true */
		if(unlikely(ret == 0)){
			esram_dev.page_enable_retries++;
		}
		
	}while(ret == 0);
}

/**
 * intel_qrk_esram_page_disable_sync
 *
 * @param epage: pointer to eSRAM page descriptor
 *
 * This function spins waiting for disable bit to clear, useful right after a
 * disable/disable-flush command. Interrupts are enabled here, sleeping is OK
 */
static void intel_qrk_esram_page_disable_sync(struct esram_page * epage)
{
	u32 ret = 0, retries = 0;
	do {
		/* Poll for busy bit clear */
		ret = intel_qrk_esram_page_busy(epage, 1);

		/* This branch should rarely if ever be true */
		if(unlikely(ret)){
			esram_dev.page_disable_retries++;
			retries++;
		}

		if(retries == MAX_PAGE_RETRIES){
			intel_qrk_esram_fault(epage, __LINE__);
		}
	}while(ret);
}

/**
 * intel_qrk_esram_page_disable
 *
 * @param epage: struct esram_page carries data to program to register
 *
 * Disable the eSRAM page no flush. Interrupts are enabled here, sleeping is OK
 */
static void intel_qrk_esram_page_disable(struct esram_page *epage)
{
	sb_write(SB_ID_ESRAM, ESRAM_PAGE_WRITE, epage->id,
		ESRAM_PAGE_DISABLE, 1);
	intel_qrk_esram_page_disable_sync(epage);	
}

/**
 * intel_qrk_esram_page_flush_disable
 *
 * @param epage: struct esram_page carries data to program to register
 *
 * Disable the eSRAM page - with flush. Note the architecture will block access
 * to the overlayed region until the flush has completed => irqs may be switched
 * on during this operation.
 */
static void intel_qrk_esram_page_flush_disable(struct esram_page *epage)
{
	

	/* Do flush */
	sb_write(SB_ID_ESRAM, ESRAM_PAGE_WRITE, epage->id,
		ESRAM_PAGE_FLUSH | ESRAM_PAGE_DISABLE, 1);

	intel_qrk_esram_page_disable_sync(epage);	
}

#if 0
/**
 * intel_qrk_esram_flush_disable_all
 *
 * Flushes and disables all enabled eSRAM pages
 */
static void intel_qrk_esram_page_flush_disable_all(void)
{
	struct esram_page * epage = NULL;
	
	spin_lock(&esram_dev.slock);
	list_for_each_entry(epage, &esram_dev.page_used, list){
		intel_qrk_esram_page_flush_disable(epage);
	}
	spin_unlock(&esram_dev.slock);
}
#endif

/**
 * intel_qrk_esram_page_populate_atomic
 *
 * @param epage: Pointer to eSRAM page desciptor.
 * @return 0 placeholder, later versions may return error
 *
 * Function takes the mappings given in epage and uses the values to populate
 * an eSRAM page. The copy/enable/copy routine must be done atomically, since we
 * may be doing a memcpy() of an ISR for example.
 * For this reason we wrapper this entire call into a callback provided by 
 * side-band, which does a spin_lock_irqsave calls this function and then does
 * a spin_lock_irqrestore - thus guaranteeing atomicity of the below code and
 * respect for the locking strategy of the side-band driver
  */
static int intel_qrk_esram_page_populate_atomic(struct esram_page * epage)
{
	unsigned long crz;

	/* Copy away */	
	memcpy(&esram_dev.cbuf, (void*)epage->vaddr, ESRAM_PAGE_SIZE);

	/* If CR0.WP is true - flip it HSD # 4930660 */
	crz = read_cr0();
	if (crz & X86_CR0_WP){
		write_cr0(crz & (~X86_CR0_WP));
	}

	/* Disable NMI */
	outb(0x80, 0x70);
	
	/*  Enable page mapping */
	intel_qrk_esram_page_enable(epage, 0);
	
	/* Copy back - populating memory overlay */
	memcpy((void*)epage->vaddr, &esram_dev.cbuf,  ESRAM_PAGE_SIZE);

	/* Re-enable NMI */
	outb(0x00, 0x70);

	/* Restore CR0.WP if appropriate HSD # 4930660 */
	if (crz & X86_CR0_WP){
		write_cr0(crz);
	}
	return 0;
}

/**
 * intel_qrk_esram_page_populate
 *
 * @param epage: Pointer to eSRAM page desciptor.
 * @return 0 on success < 0 on failure
 *
 * Populates the page. set_memory_rw/set_memory_ro require local irqs enabled.
 * intel_qrk_esram_page_populate_atomic - needs irqs switched off since memory
 * can be inconsistent during the populate operation. Depopulate operations are
 * architecturally guaranteed
 */
static int intel_qrk_esram_page_populate(struct esram_page * epage)
{
	int flip_rw = 0, level = 0, ret = 0;
	pte_t * pte = epage != NULL ? lookup_address(epage->vaddr, &level):NULL;

	if(unlikely(pte == NULL)){
		return -EINVAL;
	}

	/* Determine if we need to set writable */
	flip_rw = !(pte_write(*pte));
	
	/* Ensure memory is r/w - do so before spin_lock_irqsave */
	if(flip_rw){
		ret = set_memory_rw(epage->vaddr, 1);
		if (ret != 0){
			pr_err("%s error during set_memory_rw = %d\n",
				__func__, ret);
			return ret;
		}
	}

	/* Force ECC update @ disable only */
	intel_qrk_esram_page_enable(epage, 1);
	intel_qrk_esram_page_disable(epage);

	/* Enable and populate eSRAM page using callback in sb with irqs off */
	ret |= intel_qrk_sb_runfn_lock(
		(int (*)(void*))intel_qrk_esram_page_populate_atomic,(void*)epage);

	/* If we set memory writable - restore previous state */
	if(flip_rw){
		ret |= set_memory_ro(epage->vaddr, 1);
		if (ret != 0){
			pr_err("%s error during set_memory_ro = %d\n",
				__func__, ret);
			return ret;
		}
	}

	return ret;
}
/**
 * intel_qrk_esram_page_addref
 *
 * @param epage: eSRAM page descriptor
 * @param name: Name of reference to add
 * @return zero on success negative on error
 *
 */
static int intel_qrk_esram_page_addref(struct esram_page * epage, char * name)
{
	struct esram_refname * refname = NULL;
	if(unlikely(epage == NULL || name == NULL)){
		return -EINVAL;
	}

	refname = kzalloc(sizeof(struct esram_refname), GFP_KERNEL);
	if(unlikely(refname == NULL)){
		return -ENOMEM;
	}
		
	/* Add to list */
	strncpy(refname->name, name, sizeof(refname->name));
	list_add(&refname->list, &epage->name_list);

	/* Bump reference count */
	epage->refcount++;
	return 0;
}


/**
 * __intel_qrk_esram_map_page
 *
 * @param page: Page to map
 * @param name: Name of the mapping
 * @return 0 success < 0 failure
 *
 * Overlay a vritual address rangne eeds to be aligned to a 4k address.
 * Since multiple items can live in a 4k range, it is possible when calling
 * into map_page() that a previous mapping will have already covered some or all
 * of the mapping we want. This is not an error case, if the map function finds
 * it is being asked to map a 4k range already mapped it returns 0, to indicate
 * the mapping has suceeded i.e. it's already been mapped. This is logical if
 * you think about it. In contrast being asked to unmap a region not mapped is
 * clearly an error...
 *
 */
static int __intel_qrk_esram_map_page(u32 vaddr, char * name)
{
	int ret = 0;	
	struct esram_page * epage = NULL;
	struct esram_refname * refname = NULL;

	if(unlikely(name == NULL)){
		return -EINVAL;
	}

	if(unlikely(esram_dev.page_free_ct == 0)){
		return -ENOMEM;
	}

	/* Verify if we have already mapped */
	list_for_each_entry(epage, &esram_dev.page_used, list){
		if(epage->vaddr == vaddr){

			/* Page already mapped */
			list_for_each_entry(refname, &epage->name_list, list){
				if(strcmp(refname->name, name)==0){
					/* Page mapped at this name */
					return -EINVAL;
				}
			}
			/* New symbol in previous mapping */
			return intel_qrk_esram_page_addref(epage, name);
		}
	}

	/* Enumerate eSRAM page structure */
	epage = list_first_entry(&esram_dev.page_free, struct esram_page, list);
	epage->phys_addr = virt_to_phys((void*)vaddr);
	epage->vaddr = vaddr;
	ret = intel_qrk_esram_page_addref(epage, name);
	if(unlikely(ret < 0)){
		return ret;
	}
	
	/* Populate page */
	ret = intel_qrk_esram_page_populate(epage);

	/* Move to used list */
	list_move(&epage->list, &esram_dev.page_used);
	esram_dev.page_free_ct--;

	return ret;
}

/**
 * __intel_qrk_esram_unmap_page
 *
 * @param page: Page to unmap
 * @param name: Name of the mapping
 * @return 0 success < 0 failure
 *
 * Unmap a previously mapped virutal address range.
 * Must be 4k aligned
 *
 */
static int __intel_qrk_esram_unmap_page(u32 vaddr, char * name)
{
	u8 found = 0;
	struct esram_page * epage = NULL;
	struct esram_refname * refname = NULL;

	/* Find physical address */
	list_for_each_entry(epage, &esram_dev.page_used, list){
		if(epage->vaddr == vaddr){
			found = 1;
			break;
		}
	}

	/* Bail out on error */
	if(found == 0){
		pr_err("0x%08x not mapped\n", vaddr);
		return -EINVAL;
	}

	/* Determine reference to delete */
	found = 0;
	list_for_each_entry(refname, &epage->name_list, list){
		if(strcmp(refname->name,name)==0){
			found = 1;
			break;
		}
	}
	if(unlikely(found == 0)){
		pr_err("No mapping %s!\n", name);
		return -EINVAL;
	}

	/* Remove entry decrement reference count */	
	list_del(&refname->list);
	kfree(refname);
	if(--epage->refcount > 0){
		return 0;
	}

	/* Flush and disable page */
	intel_qrk_esram_page_flush_disable(epage);

	/* Move to free list tail - scrub entries come from head */
	list_move_tail(&epage->list, &esram_dev.page_free);
	esram_dev.page_free_ct++;

	return 0;
}

/**
 *
 * __intel_qrk_esram_page_op
 *
 * @param vaddr: Virtual address of symbol
 * @param size: Size/length of symbol
 * @param name: Name of mapping
 * @param map: Boolean indicates whether to map or unmap the page
 * @return 0 success < 0 failure
 *
 * This function maps/unmaps a pages/pages given at the given vaddr. If
 * the extent of the symbol @ vaddr crosses a page boundary, then we map
 * multiple pages. Other stuff inside the page, gets a performance boost 'for
 * free'. Any other data in the page that crosses the physical page boundary
 * will be partially mapped.
 */
static int __intel_qrk_esram_page_op(u32 vaddr, u32 size, char *name, u8 map)
{
	unsigned long offset = 0, page_offset = 0;
	u32  pages = size/ESRAM_PAGE_SIZE + ((size%ESRAM_PAGE_SIZE) ? 1 : 0);
	int ret = 0;

	/* Compare required pages to available pages */
	if(map == ESRAM_MAP_OP){
		if(pages > esram_dev.page_free_ct)
			return -ENOMEM;
	}else{
		if(pages > esram_dev.page_count - esram_dev.page_free_ct)
			return -ENOMEM;
	}

	/* Align to 4k and iterate the mappings */	
	vaddr = vaddr&ESRAM_PAGE_MASK;
	while(size > 0){

		/* Map the page */
		spin_lock(&esram_dev.slock);
		if(map == ESRAM_MAP_OP){
			ret = __intel_qrk_esram_map_page(vaddr, name);
							 
		}else{
			ret = __intel_qrk_esram_unmap_page(vaddr, name);
		}
		spin_unlock(&esram_dev.slock);
		if(unlikely(ret != 0)){
			break;
		}

		/* Calc appropriate offsets */
		page_offset = offset_in_page(vaddr);
		if(page_offset + size > ESRAM_PAGE_SIZE){

			offset = ESRAM_PAGE_SIZE - page_offset;
			size -= offset;
			vaddr += ESRAM_PAGE_SIZE; 

		}else{
			size = 0;
		}
	}

	return ret;
}

/******************************************************************************
 *                                 eSRAM API
 ******************************************************************************/

/**
 * intel_qrk_esram_map_range
 *
 * @param vaddr: Virtual address to start mapping (must be 4k aligned)
 * @param size: Size to map from
 * @param mapname: Mapping name
 * @return 0 success < 0 failure
 *
 * Map 4k increments at given address to eSRAM.
 */
int intel_qrk_esram_map_range(void * vaddr, u32 size, char * mapname)
{
	if(size == 0 || mapname == NULL || vaddr == NULL){
		return -EINVAL;
	}
	return __intel_qrk_esram_page_op((u32)vaddr, size, mapname, ESRAM_MAP_OP);
}
EXPORT_SYMBOL(intel_qrk_esram_map_range);

/**
 * intel_qrk_esram_map_symbol
 *
 * @param vaddr: Virtual address of the symbol
 * @return 0 success < 0 failure
 *
 * Maps a series of 4k chunks starting at vaddr&0xFFFFF000. vaddr shall be a
 * kernel text section symbol (kernel or loaded module)
 *
 * We get the size of the symbol from kallsyms. We guarantee to map the entire
 * size of the symbol - plus whatever padding is necessary to get alignment to
 * eSRAM_PAGE_SIZE 
 * Other stuff inside the mapped pages will get a performance boost 'for free'.
 * If this free boost is not what you want then 
 *
 *	1. Align to 4k
 *	2. Pad to 4k
 *	3. Call intel_qrk_esram_map_range()
 */
int intel_qrk_esram_map_symbol(void * vaddr)
{
	long unsigned int size = 0, offset = 0;
	char symname[KSYM_SYMBOL_LEN];
	
	kallsyms_lookup_size_offset((long unsigned int)vaddr, &size, &offset);
	if(size == 0){
		return -EINVAL;
	}
	sprint_symbol(symname, (u32)vaddr);

	return __intel_qrk_esram_page_op((u32)vaddr, size, symname, 1);
}
EXPORT_SYMBOL(intel_qrk_esram_map_symbol);

/******************************************************************************
 *                        Module/PowerManagement hooks 
 ******************************************************************************/

/**
 * intel_qrk_esram_suspend
 *
 * @param pdev: Platform device structure (unused)
 * @param pm: Power managment descriptor
 * @return 0 success < 0 failure
 *
 * For each enabled page - flush to DRAM and disable eSRAM page.
 * For each 4k region the architecture guarantees atomicity of flush/disable.
 * Hence any memory transactions to the affected region will stall until
 * flush/disable completes - hence interrupts are left on.
 */
static int intel_qrk_esram_suspend(struct device * pdev)
{
	/* Flush and disable of eSRAM pages is carried out automatically */
	return 0;
}

/**
 * intel_qrk_esram_resume
 *
 * @param pm: Power management descriptor
 * @return 0 success < 0 failure
 *
 * Runs after resume_noirq. Switches pages back to ro, if appropriate. We do
 * this here since interrupts will be on, as required by the function
 * set_memory_ro. If it were possible to set memory ro in resume_noirq we would
 * do it there instead
 */
static int intel_qrk_esram_resume(struct device * pdev)
{
	struct esram_page * epage = NULL;
	int ret = 0;

	list_for_each_entry(epage, &esram_dev.page_used, list){
		ret |= intel_qrk_esram_page_populate(epage);
	}
	
	return ret;
}


/**
 * intel_qrk_esram_probe
 *
 * @param pdev: Platform device
 * @return 0 success < 0 failure
 *
 * Callback from platform sub-system to probe
 *
 * This driver manages eSRAM on a per-page basis. Therefore if we find block
 * mode is enabled, or any global, block-level or page-level locks are in place
 * at module initialisation time - we bail out.
 */
static int intel_qrk_esram_probe(struct platform_device * pdev)
{
	int ret = 0;
	u32 block = 0, ctrl = 0, i = 0, pgstat = 0;

	memset(&esram_dev, 0x00, sizeof(esram_dev));
	INIT_LIST_HEAD(&esram_dev.page_used);
	INIT_LIST_HEAD(&esram_dev.page_free);
	spin_lock_init(&esram_dev.slock);
	esram_dev.page_free_ct = 0;
	
	/* Ensure block mode disabled */
	block = ESRAM_PGBLOCK_DISABLE;
	sb_write(SB_ID_ESRAM, ESRAM_CTRL_WRITE, ESRAM_PGBLOCK_REG, block, 1);

	/* Get state */
	sb_read(SB_ID_ESRAM, ESRAM_CTRL_READ, ESRAM_CTRL_REG, &ctrl, 1);
	sb_read(SB_ID_ESRAM, ESRAM_CTRL_READ, ESRAM_PGBLOCK_REG, &block, 1);

	/* Verify state is good to go */
	if (ctrl & ESRAM_CTRL_GLOBAL_LOCK){
		pr_err ("eSRAM: global lock @ 0x%08x\n", ctrl);
		return -ENODEV;
	}

	if (block & (ESRAM_PGBLOCK_LOCK | ESRAM_PGBLOCK_ENABLE)){
		pr_err ("eSRAM: lock @ 0x%08x\n", block);
		return -ENODEV;
	}
	pr_info("eSRAM: CTRL 0x%08x block 0x%08x\n", ctrl, block);

	/* Calculate # of pages silicon supports */
	esram_dev.page_count = ESRAM_CTRL_SIZE(ctrl) + 1;
	esram_dev.page_free_ct = esram_dev.page_count;
	pr_info("eSRAM: pages %d\n", esram_dev.page_free_ct);

	if(esram_dev.page_free_ct <= 1){
		pr_err("Too few pages reported by eSRAM sub-system\n");
		return -ENOMEM;
	}

	/* Allocate an appropriate number of pages */
	esram_dev.pages = kzalloc(esram_dev.page_count *
		sizeof(struct esram_page), GFP_KERNEL);
	if (esram_dev.pages == NULL){
		return -ENOMEM;
	}

	/* Initialise list of free pages, explicitely disable as we go */
	for(i = 0; i < esram_dev.page_count; i++){
		INIT_LIST_HEAD(&esram_dev.pages[i].name_list);
		esram_dev.pages[i].id = i;
		
		/* Read & verify page state */
		sb_read(SB_ID_ESRAM, ESRAM_PAGE_READ, i, &pgstat, 1);
		if(pgstat & (ESRAM_PAGE_BUSY | ESRAM_PAGE_LOCK)){
			pr_err("eSRAM: page %d state 0x%08x err\n", i, pgstat);
			ret = -ENODEV;
			goto err;
		}

		list_add(&esram_dev.pages[i].list, &esram_dev.page_free);
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &esram_attrib_group);
	if (ret)
		goto err;

	return 0;
err:
	kfree(esram_dev.pages);
	return ret;
}

/*
 * Power management operations
 */
static const struct dev_pm_ops intel_qrk_esram_pm_ops = {
	.suspend = intel_qrk_esram_suspend,
	.resume = intel_qrk_esram_resume,
};

/*
 * Platform structures useful for interface to PM subsystem
 */
static struct platform_driver intel_qrk_esram_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &intel_qrk_esram_pm_ops,
	},
	.probe = intel_qrk_esram_probe,
};

module_platform_driver(intel_qrk_esram_driver);

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@linux.intel.com>");
MODULE_DESCRIPTION("Intel Quark eSRAM overlay/ECC-scrub driver");
MODULE_LICENSE("Dual BSD/GPL");

