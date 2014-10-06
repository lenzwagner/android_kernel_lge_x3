/*
 * drivers/mfd/max77663-core.c
 * Max77663 mfd driver (I2C bus access)
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#ifdef CONFIG_MACH_X3
#include <linux/notifier.h>
#include <linux/reboot.h>
#endif

#include <linux/mfd/max77663-core.h>

#ifdef CONFIG_MACH_X3
#include "../../arch/arm/mach-tegra/x3/include/lge/board-x3.h"
#endif

/* RTC i2c slave address */
#define MAX77663_RTC_I2C_ADDR		0x48

/* Registers */
#ifdef CONFIG_MACH_X3
#define MAX77663_REG_32KHZ_OSC		0x03
#endif
#define MAX77663_REG_IRQ_TOP		0x05
#define MAX77663_REG_LBT_IRQ		0x06
#define MAX77663_REG_SD_IRQ		0x07
#define MAX77663_REG_LDOX_IRQ		0x08
#define MAX77663_REG_LDO8_IRQ		0x09
#define MAX77663_REG_GPIO_IRQ		0x0A
#define MAX77663_REG_ONOFF_IRQ		0x0B
#define MAX77663_REG_NVER		0x0C
#define MAX77663_REG_IRQ_TOP_MASK	0x0D
#define MAX77663_REG_LBT_IRQ_MASK	0x0E
#define MAX77663_REG_SD_IRQ_MASK	0x0F
#define MAX77663_REG_LDOX_IRQ_MASK	0x10
#define MAX77663_REG_LDO8_IRQ_MASK	0x11
#define MAX77663_REG_ONOFF_IRQ_MASK	0x12
#ifdef CONFIG_MACH_X3
#define MAX77663_REG_ONOFF_STAT		0x15
#endif
#define MAX77663_REG_GPIO_CTRL0		0x36
#define MAX77663_REG_GPIO_CTRL1		0x37
#define MAX77663_REG_GPIO_CTRL2		0x38
#define MAX77663_REG_GPIO_CTRL3		0x39
#define MAX77663_REG_GPIO_CTRL4		0x3A
#define MAX77663_REG_GPIO_CTRL5		0x3B
#define MAX77663_REG_GPIO_CTRL6		0x3C
#define MAX77663_REG_GPIO_CTRL7		0x3D
#define MAX77663_REG_GPIO_PU		0x3E
#define MAX77663_REG_GPIO_PD		0x3F
#define MAX77663_REG_GPIO_ALT		0x40
#define MAX77663_REG_ONOFF_CFG1		0x41
#define MAX77663_REG_ONOFF_CFG2		0x42
#define MAX77663_REG_CID4			0x5C
#define MAX77663_REG_CID5			0x5D

#define IRQ_TOP_GLBL_MASK		(1 << 7)
#define IRQ_TOP_GLBL_SHIFT		7
#define IRQ_TOP_SD_MASK			(1 << 6)
#define IRQ_TOP_SD_SHIFT		6
#define IRQ_TOP_LDO_MASK		(1 << 5)
#define IRQ_TOP_LDO_SHIFT		5
#define IRQ_TOP_GPIO_MASK		(1 << 4)
#define IRQ_TOP_GPIO_SHIFT		4
#define IRQ_TOP_RTC_MASK		(1 << 3)
#define IRQ_TOP_RTC_SHIFT		3
#define IRQ_TOP_32K_MASK		(1 << 2)
#define IRQ_TOP_32K_SHIFT		2
#define IRQ_TOP_ONOFF_MASK		(1 << 1)
#define IRQ_TOP_ONOFF_SHIFT		1
#define IRQ_TOP_NVER_MASK		(1 << 0)
#define IRQ_TOP_NVER_SHIFT		0

#define IRQ_GLBL_MASK			(1 << 0)

#define IRQ_LBT_BASE			MAX77663_IRQ_LBT_LB
#define IRQ_LBT_END			MAX77663_IRQ_LBT_THERM_ALRM2

#define IRQ_GPIO_BASE			MAX77663_IRQ_GPIO0
#define IRQ_GPIO_END			MAX77663_IRQ_GPIO7

#define IRQ_ONOFF_BASE			MAX77663_IRQ_ONOFF_HRDPOWRN
#define IRQ_ONOFF_END			MAX77663_IRQ_ONOFF_ACOK_RISING

#define GPIO_REG_ADDR(offset)		(MAX77663_REG_GPIO_CTRL0 + offset)

#define GPIO_CTRL_DBNC_MASK		(3 << 6)
#define GPIO_CTRL_DBNC_SHIFT		6
#define GPIO_CTRL_REFE_IRQ_MASK		(3 << 4)
#define GPIO_CTRL_REFE_IRQ_SHIFT	4
#define GPIO_CTRL_DOUT_MASK		(1 << 3)
#define GPIO_CTRL_DOUT_SHIFT		3
#define GPIO_CTRL_DIN_MASK		(1 << 2)
#define GPIO_CTRL_DIN_SHIFT		2
#define GPIO_CTRL_DIR_MASK		(1 << 1)
#define GPIO_CTRL_DIR_SHIFT		1
#define GPIO_CTRL_OUT_DRV_MASK		(1 << 0)
#define GPIO_CTRL_OUT_DRV_SHIFT		0

#define GPIO_REFE_IRQ_NONE		0
#define GPIO_REFE_IRQ_EDGE_FALLING	1
#define GPIO_REFE_IRQ_EDGE_RISING	2
#define GPIO_REFE_IRQ_EDGE_BOTH		3

#define GPIO_DBNC_NONE			0
#define GPIO_DBNC_8MS			1
#define GPIO_DBNC_16MS			2
#define GPIO_DBNC_32MS			3

#ifdef CONFIG_MACH_X3
#define OSC_32KHZ_LOAD_12PF		(0b00 << 4)
#define OSC_32KHZ_LOAD_22PF		(0b01 << 4)
#define OSC_32KHZ_LOAD_NO		(0b10 << 4)
#define OSC_32KHZ_LOAD_10PF		(0b11 << 4)

#define OSC_32KHZ_LOAD_MASK		(1 << 5 | 1 << 4)

#define	ONOFF_MRT_MASK_2S		(0b000 << 3)
#define	ONOFF_MRT_MASK_3S		(0b001 << 3)
#define	ONOFF_MRT_MASK_4S		(0b010 << 3)
#define	ONOFF_MRT_MASK_5S		(0b011 << 3)
#define	ONOFF_MRT_MASK_6S		(0b100 << 3)
#define	ONOFF_MRT_MASK_8S		(0b101 << 3)
#define	ONOFF_MRT_MASK_10S		(0b110 << 3)
#define	ONOFF_MRT_MASK_12S		(0b111 << 3)
#endif

#define ONOFF_SFT_RST_MASK		(1 << 7)
#ifdef CONFIG_MACH_X3
#define ONOFF_MRT_MASK			(1 << 5 | 1 << 4 | 1 << 3)
#endif
#define ONOFF_SLPEN_MASK		(1 << 2)
#define ONOFF_PWR_OFF_MASK		(1 << 1)

#define ONOFF_SLP_LPM_MASK		(1 << 5)

#ifdef CONFIG_MACH_X3
#define ONOFF_SFT_PWR_OFF_MASK		(1 << 1)
#define ONOFF_SFT_RST_WK_MASK		(1 << 7)

#define ONOFF_STAT_EN0_MASK		(1 << 2)
#define ONOFF_STAT_ACOK_MASK		(1 << 1)
#define ONOFF_STAT_LID_MASK		(1 << 0)
#endif
#define ONOFF_IRQ_EN0_RISING		(1 << 3)

enum {
	CACHE_IRQ_LBT,
	CACHE_IRQ_SD,
	CACHE_IRQ_LDO,
	CACHE_IRQ_ONOFF,
	CACHE_IRQ_NR,
};

struct max77663_irq_data {
	int mask_reg;
	u16 mask;
	u8 top_mask;
	u8 top_shift;
	int cache_idx;
	bool is_rtc;
	bool is_unmask;
	u8 trigger_type;
};

struct max77663_chip {
	struct device *dev;
	struct i2c_client *i2c_power;
	struct i2c_client *i2c_rtc;
	struct regmap *regmap_power;
	struct regmap *regmap_rtc;

	struct max77663_platform_data *pdata;
	struct mutex io_lock;

	struct irq_chip irq;
	struct mutex irq_lock;
	int irq_base;
	int irq_top_count[8];
	u8 cache_irq_top_mask;
	u16 cache_irq_mask[CACHE_IRQ_NR];

	u8 rtc_i2c_addr;
};

struct max77663_chip *max77663_chip;

static struct resource gpio_resources[] = {
	{
		.start	= MAX77663_IRQ_INT_TOP_GPIO,
		.end	= MAX77663_IRQ_INT_TOP_GPIO,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource rtc_resources[] = {
	{
		.start	= MAX77663_IRQ_RTC,
		.end	= MAX77663_IRQ_RTC,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct mfd_cell max77663_cells[] = {
	{
		.name = "max77663-gpio",
		.num_resources	= ARRAY_SIZE(gpio_resources),
		.resources	= &gpio_resources[0],
	}, {
		.name = "max77663-pmic",
	}, {
		.name = "max77663-rtc",
		.num_resources	= ARRAY_SIZE(rtc_resources),
		.resources	= &rtc_resources[0],
	},
};

#define IRQ_DATA_LBT(_name, _shift)			\
	[MAX77663_IRQ_LBT_##_name] = {			\
		.mask_reg = MAX77663_REG_LBT_IRQ_MASK,	\
		.mask = (1 << _shift),			\
		.top_mask = IRQ_TOP_GLBL_MASK,		\
		.top_shift = IRQ_TOP_GLBL_SHIFT,	\
		.cache_idx = CACHE_IRQ_LBT,		\
	}

#define IRQ_DATA_GPIO_TOP()				\
	[MAX77663_IRQ_INT_TOP_GPIO] = {			\
		.top_mask = IRQ_TOP_GPIO_MASK,		\
		.top_shift = IRQ_TOP_GPIO_SHIFT,	\
		.cache_idx = -1,			\
	}

#define IRQ_DATA_ONOFF(_name, _shift)			\
	[MAX77663_IRQ_ONOFF_##_name] = {		\
		.mask_reg = MAX77663_REG_ONOFF_IRQ_MASK,\
		.mask = (1 << _shift),			\
		.top_mask = IRQ_TOP_ONOFF_MASK,		\
		.top_shift = IRQ_TOP_ONOFF_SHIFT,	\
		.cache_idx = CACHE_IRQ_ONOFF,		\
	}

static struct max77663_irq_data max77663_irqs[MAX77663_IRQ_NR] = {
	IRQ_DATA_GPIO_TOP(),
	IRQ_DATA_LBT(LB, 3),
	IRQ_DATA_LBT(THERM_ALRM1, 2),
	IRQ_DATA_LBT(THERM_ALRM2, 1),
	IRQ_DATA_ONOFF(HRDPOWRN,     0),
	IRQ_DATA_ONOFF(EN0_1SEC,     1),
	IRQ_DATA_ONOFF(EN0_FALLING,  2),
	IRQ_DATA_ONOFF(EN0_RISING,   3),
	IRQ_DATA_ONOFF(LID_FALLING,  4),
	IRQ_DATA_ONOFF(LID_RISING,   5),
	IRQ_DATA_ONOFF(ACOK_FALLING, 6),
	IRQ_DATA_ONOFF(ACOK_RISING,  7),
	[MAX77663_IRQ_RTC] = {
		.top_mask = IRQ_TOP_RTC_MASK,
		.top_shift = IRQ_TOP_RTC_SHIFT,
		.cache_idx = -1,
		.is_rtc = 1,
	},
	[MAX77663_IRQ_SD_PF] = {
		.mask_reg = MAX77663_REG_SD_IRQ_MASK,
		.mask = 0xF8,
		.top_mask = IRQ_TOP_SD_MASK,
		.top_shift = IRQ_TOP_SD_SHIFT,
		.cache_idx = CACHE_IRQ_SD,
	},
	[MAX77663_IRQ_LDO_PF] = {
		.mask_reg = MAX77663_REG_LDOX_IRQ_MASK,
		.mask = 0x1FF,
		.top_mask = IRQ_TOP_LDO_MASK,
		.top_shift = IRQ_TOP_LDO_SHIFT,
		.cache_idx = CACHE_IRQ_LDO,
	},
	[MAX77663_IRQ_32K] = {
		.top_mask = IRQ_TOP_32K_MASK,
		.top_shift = IRQ_TOP_32K_SHIFT,
		.cache_idx = -1,
	},
	[MAX77663_IRQ_NVER] = {
		.top_mask = IRQ_TOP_NVER_MASK,
		.top_shift = IRQ_TOP_NVER_SHIFT,
		.cache_idx = -1,
	},
};

/* MAX77663 PMU doesn't allow PWR_OFF and SFT_RST setting in ONOFF_CFG1
 * at the same time. So if it try to set PWR_OFF and SFT_RST to ONOFF_CFG1
 * simultaneously, handle only SFT_RST and ignore PWR_OFF.
 */
#define CHECK_ONOFF_CFG1_MASK	(ONOFF_SFT_RST_MASK | ONOFF_PWR_OFF_MASK)
#define CHECK_ONOFF_CFG1(_addr, _val)			\
	unlikely((_addr == MAX77663_REG_ONOFF_CFG1) &&	\
		 ((_val & CHECK_ONOFF_CFG1_MASK) == CHECK_ONOFF_CFG1_MASK))

static inline int max77663_i2c_write(struct max77663_chip *chip, u8 addr,
				     void *src, u32 bytes, bool is_rtc)
{
	int ret = 0;

	dev_dbg(chip->dev, "i2c_write: addr=0x%02x, src=0x%02x, bytes=%u\n",
		addr, *((u8 *)src), bytes);

	if (is_rtc) {
		/* RTC registers support sequential writing */
		ret = regmap_bulk_write(chip->regmap_rtc, addr, src, bytes);
	} else {
		/* Power registers support register-data pair writing */
		u8 *src8 = (u8 *)src;
		unsigned int val;
		int i;

		for (i = 0; i < bytes; i++) {
			if (CHECK_ONOFF_CFG1(addr, *src8))
				val = *src8++ & ~ONOFF_PWR_OFF_MASK;
			else
				val = *src8++;
			ret = regmap_write(chip->regmap_power, addr, val);
			if (ret < 0)
				break;
			addr++;
		}
	}
	if (ret < 0)
		dev_err(chip->dev, "%s() failed, e %d\n", __func__, ret);
	return ret;
}

static inline int max77663_i2c_read(struct max77663_chip *chip, u8 addr,
				    void *dest, u32 bytes, bool is_rtc)
{
	int ret = 0;
	struct regmap *regmap = chip->regmap_power;

	if (is_rtc)
		regmap = chip->regmap_rtc;
		ret = regmap_bulk_read(regmap, addr, dest, bytes);
	if (ret < 0) {
		dev_err(chip->dev, "%s() failed, e %d\n", __func__, ret);
		return ret;
	}

	dev_dbg(chip->dev, "i2c_read: addr=0x%02x, dest=0x%02x, bytes=%u\n",
		addr, *((u8 *)dest), bytes);
	return ret;
}

int max77663_read(struct device *dev, u8 addr, void *values, u32 len,
		  bool is_rtc)
{
	struct max77663_chip *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = max77663_i2c_read(chip, addr, values, len, is_rtc);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77663_read);

int max77663_write(struct device *dev, u8 addr, void *values, u32 len,
		   bool is_rtc)
{
	struct max77663_chip *chip = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = max77663_i2c_write(chip, addr, values, len, is_rtc);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77663_write);

int max77663_set_bits(struct device *dev, u8 addr, u8 mask, u8 value,
		      bool is_rtc)
{
	struct max77663_chip *chip = dev_get_drvdata(dev);
	int ret;
	struct regmap *regmap = chip->regmap_power;

	if (is_rtc)
		regmap = chip->regmap_rtc;

	mutex_lock(&chip->io_lock);
	ret = regmap_update_bits(regmap, addr, mask, value);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77663_set_bits);

#ifdef CONFIG_MACH_X3
int max77663_get_bits(struct device *dev, u8 addr, u8 mask,
		      bool is_rtc)
{
	struct max77663_chip *chip = dev_get_drvdata(dev);
//	struct i2c_client *client = NULL;
	u8 tmp = 0;
	int ret = 0;

	mutex_lock(&chip->io_lock);
#if 0
	if (!is_rtc)
		client = chip->i2c_power;
	else
		client = chip->i2c_rtc;
#endif

	ret = max77663_i2c_read(chip, addr, &tmp, 1, is_rtc);
	if (ret == 0) {
		ret = tmp & mask;
	}
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(max77663_get_bits);

int max77663_power_off(void)
#else
static void max77663_power_off(void)
#endif
{
	struct max77663_chip *chip = max77663_chip;

	if (!chip)
#ifdef CONFIG_MACH_X3
		return -EINVAL;
#else
		return;
#endif

#ifdef CONFIG_MACH_X3
	if (max77663_get_bits(chip->dev, MAX77663_REG_ONOFF_STAT, ONOFF_STAT_ACOK_MASK, 0)) {
		max77663_power_rst_wkup(1);
		//arm_machine_restart('h', NULL);
		return 0;
	} else {
		dev_info(chip->dev, "%s: Global shutdown\n", __func__);
		return max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
					 ONOFF_SFT_PWR_OFF_MASK, ONOFF_SFT_PWR_OFF_MASK, 0);
	}
#else
	dev_info(chip->dev, "%s: Global shutdown\n", __func__);
	max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
			  ONOFF_SFT_RST_MASK, ONOFF_SFT_RST_MASK, 0);
#endif
}
#ifdef CONFIG_MACH_X3
EXPORT_SYMBOL(max77663_power_off);

#define MAX77663_RTC_UPDATE0		0x04
#define MAX77663_RTC_ALARM_SEC2		0x15
int max77663_set_ScratchRegister(u8 bit)
{
	struct max77663_chip *chip = max77663_chip;
	u8 val = 0x0b; //write

	if (!chip)
		return -EINVAL;

	max77663_set_bits(chip->dev, MAX77663_RTC_ALARM_SEC2, bit, bit, 1);

	max77663_write(chip->dev, MAX77663_RTC_UPDATE0, &val, 1, 1);
	mdelay(15);

	return 0;
}
EXPORT_SYMBOL(max77663_set_ScratchRegister);

int max77663_get_acok_stat(void)
{
	struct max77663_chip *chip = max77663_chip;

	if (!chip)
		return -EINVAL;

	return max77663_get_bits(chip->dev, MAX77663_REG_ONOFF_STAT,
				ONOFF_STAT_ACOK_MASK, 0);
}
EXPORT_SYMBOL(max77663_get_acok_stat);

int max77663_power_rst_wkup(int on)
{
	struct max77663_chip *chip = max77663_chip;
	int ret = 0;

	if (!chip)
		return -EINVAL;

	ret = max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG2,
				 ONOFF_SFT_RST_WK_MASK, on ?ONOFF_SFT_RST_WK_MASK:0, 0);

	dev_info(chip->dev, "%s: restart 1\n", __func__);
	if(ret < 0)
		return ret;

	udelay(100);
	dev_info(chip->dev, "%s: restart 2\n", __func__);

	return max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
				 ONOFF_SFT_RST_MASK, ONOFF_SFT_RST_MASK, 0);
}
EXPORT_SYMBOL(max77663_power_rst_wkup);
#endif

static int max77663_sleep(struct max77663_chip *chip, bool on)
{
	int ret = 0;

#ifdef CONFIG_MACH_X3
#if defined(CONFIG_MFD_MAX77663_LPM)
	if (chip->pdata->flags & SLP_LPM_ENABLE) {
	/* Put the power rails into Low-Power mode during sleep mode,
	 * if the power rail's power mode is GLPM. */
		ret = max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG2,
			ONOFF_SLP_LPM_MASK, ONOFF_SLP_LPM_MASK, 0);
		if (ret < 0)
			return ret;
	}
 	/* Enable sleep that AP can be placed into sleep mode
 	 * by pulling EN1 low */
	return max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
				ONOFF_SLPEN_MASK, on ? ONOFF_SLPEN_MASK : 0, 0);
#else
	/* Enable sleep that AP can be placed into sleep mode
	 * by pulling EN1 low */
	ret = max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
					ONOFF_SLPEN_MASK, on ? ONOFF_SLPEN_MASK : 0, 0);

	if (ret < 0)
		dev_err(chip->dev,
			"sleep_enable: Failed to set on/off config1 (%d) Sleep\n",
			MAX77663_REG_ONOFF_CFG1);

	return ret;
#endif
#else
	if (chip->pdata->flags & SLP_LPM_ENABLE) {
		/* Put the power rails into Low-Power mode during sleep mode,
		 * if the power rail's power mode is GLPM. */
		ret = max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG2,
					ONOFF_SLP_LPM_MASK,
					on ? ONOFF_SLP_LPM_MASK : 0, 0);
		if (ret < 0)
			return ret;
	}

	/* Enable sleep that AP can be placed into sleep mode
	 * by pulling EN1 low */
	return max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
				 ONOFF_SLPEN_MASK,
				 on ? ONOFF_SLPEN_MASK : 0, 0);
#endif /* CONFIG_MACH_X3 */
}

#ifdef CONFIG_MACH_X3
static int max77663_set_manual_resettime(struct max77663_chip *chip)
{
	int ret;

	ret = max77663_set_bits(chip->dev, MAX77663_REG_ONOFF_CFG1,
					ONOFF_MRT_MASK, ONOFF_MRT_MASK_10S, 0);
	if (ret < 0)
		dev_err(chip->dev,
			"sleep_enable: Failed to set manual reset time config1\n");

	return ret;
}
#endif

static inline int max77663_cache_write(struct device *dev, u8 addr, u8 mask,
				       u8 val, u8 *cache)
{
	u8 new_val;
	int ret;

	new_val = (*cache & ~mask) | (val & mask);
	if (*cache != new_val) {
		ret = max77663_write(dev, addr, &new_val, 1, 0);
		if (ret < 0)
			return ret;
		*cache = new_val;
	}
	return 0;
}

static void max77663_irq_mask(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);

	max77663_irqs[data->irq - chip->irq_base].is_unmask = 0;
}

static void max77663_irq_unmask(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);

	max77663_irqs[data->irq - chip->irq_base].is_unmask = 1;
}

static void max77663_irq_lock(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);

	mutex_lock(&chip->irq_lock);
}

static void max77663_irq_sync_unlock(struct irq_data *data)
{
	struct max77663_chip *chip = irq_data_get_irq_chip_data(data);
	struct max77663_irq_data *irq_data =
			&max77663_irqs[data->irq - chip->irq_base];
	int idx = irq_data->cache_idx;
	u8 irq_top_mask = chip->cache_irq_top_mask;
	u16 irq_mask = chip->cache_irq_mask[idx];
	int update_irq_top = 0;
	u32 len = 1;
	int ret;

	if (irq_data->is_unmask) {
		if (chip->irq_top_count[irq_data->top_shift] == 0)
			update_irq_top = 1;
		chip->irq_top_count[irq_data->top_shift]++;

		if (irq_data->top_mask != IRQ_TOP_GLBL_MASK)
			irq_top_mask &= ~irq_data->top_mask;

		if (idx != -1)
			irq_mask &= ~irq_data->mask;
	} else {
		if (chip->irq_top_count[irq_data->top_shift] == 1)
			update_irq_top = 1;

		if (--chip->irq_top_count[irq_data->top_shift] < 0)
			chip->irq_top_count[irq_data->top_shift] = 0;

		if (irq_data->top_mask != IRQ_TOP_GLBL_MASK)
			irq_top_mask |= irq_data->top_mask;

		if (idx != -1)
			irq_mask |= irq_data->mask;
	}

	if ((idx != -1) && (irq_mask != chip->cache_irq_mask[idx])) {
		if (irq_data->top_mask == IRQ_TOP_LDO_MASK)
			len = 2;

		ret = max77663_write(chip->dev, irq_data->mask_reg,
				     &irq_mask, len, irq_data->is_rtc);
		if (ret < 0)
			goto out;

		chip->cache_irq_mask[idx] = irq_mask;
	}

	if (update_irq_top && (irq_top_mask != chip->cache_irq_top_mask)) {
		ret = max77663_cache_write(chip->dev, MAX77663_REG_IRQ_TOP_MASK,
					   irq_data->top_mask, irq_top_mask,
					   &chip->cache_irq_top_mask);
		if (ret < 0)
			goto out;
	}

out:
	mutex_unlock(&chip->irq_lock);
}

static inline int max77663_do_irq(struct max77663_chip *chip, u8 addr,
				  int irq_base, int irq_end)
{
	struct max77663_irq_data *irq_data = NULL;
	int irqs_to_handle[irq_end - irq_base + 1];
	int handled = 0;
	u16 val;
	u32 len = 1;
	int i;
	int ret;

	ret = max77663_read(chip->dev, addr, &val, len, 0);
	if (ret < 0)
		return ret;

	for (i = irq_base; i <= irq_end; i++) {
		irq_data = &max77663_irqs[i];
		if (val & irq_data->mask) {
			irqs_to_handle[handled] = i + chip->irq_base;
			handled++;
		}
	}

	for (i = 0; i < handled; i++)
		handle_nested_irq(irqs_to_handle[i]);

	return 0;
}

static irqreturn_t max77663_irq(int irq, void *data)
{
	struct max77663_chip *chip = data;
	u8 irq_top;
	int ret;

	ret = max77663_read(chip->dev, MAX77663_REG_IRQ_TOP, &irq_top, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "irq: Failed to get irq top status\n");
		return IRQ_NONE;
	}

#ifdef CONFIG_MACH_X3
	printk("max77663_irq: irq_mask=0x%02x \n",irq_top);
#endif

	if (irq_top & IRQ_TOP_GLBL_MASK) {
		ret = max77663_do_irq(chip, MAX77663_REG_LBT_IRQ, IRQ_LBT_BASE,
				      IRQ_LBT_END);
		if (ret < 0)
			return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP_GPIO_MASK)
		handle_nested_irq(MAX77663_IRQ_INT_TOP_GPIO + chip->irq_base);

	if (irq_top & IRQ_TOP_ONOFF_MASK) {
		ret = max77663_do_irq(chip, MAX77663_REG_ONOFF_IRQ,
				      IRQ_ONOFF_BASE, IRQ_ONOFF_END);
		if (ret < 0)
			return IRQ_NONE;
	}

	if (irq_top & IRQ_TOP_RTC_MASK)
		handle_nested_irq(MAX77663_IRQ_RTC + chip->irq_base);

	if (irq_top & IRQ_TOP_SD_MASK)
		handle_nested_irq(MAX77663_IRQ_SD_PF + chip->irq_base);

	if (irq_top & IRQ_TOP_LDO_MASK)
		handle_nested_irq(MAX77663_IRQ_LDO_PF + chip->irq_base);

	if (irq_top & IRQ_TOP_32K_MASK)
		handle_nested_irq(MAX77663_IRQ_32K + chip->irq_base);

	if (irq_top & IRQ_TOP_NVER_MASK)
		handle_nested_irq(MAX77663_IRQ_NVER + chip->irq_base);

	return IRQ_HANDLED;
}

static struct irq_chip max77663_irq_chip = {
	.name = "max77663-irq",
	.irq_mask = max77663_irq_mask,
	.irq_unmask = max77663_irq_unmask,
	.irq_bus_lock = max77663_irq_lock,
	.irq_bus_sync_unlock = max77663_irq_sync_unlock,
};

static int max77663_irq_init(struct max77663_chip *chip)
{
	u32 temp;
	int i, ret = 0;

	mutex_init(&chip->irq_lock);

	/* Mask all interrupts */
	chip->cache_irq_top_mask = 0xFF;
	chip->cache_irq_mask[CACHE_IRQ_LBT] = 0x0F;
	chip->cache_irq_mask[CACHE_IRQ_SD] = 0xFF;
	chip->cache_irq_mask[CACHE_IRQ_LDO] = 0xFFFF;
	chip->cache_irq_mask[CACHE_IRQ_ONOFF] = 0xFF;

	max77663_write(chip->dev, MAX77663_REG_IRQ_TOP_MASK,
		       &chip->cache_irq_top_mask, 1, 0);
	max77663_write(chip->dev, MAX77663_REG_LBT_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_LBT], 1, 0);
	max77663_write(chip->dev, MAX77663_REG_SD_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_SD], 1, 0);
	max77663_write(chip->dev, MAX77663_REG_LDOX_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_LDO], 2, 0);
	max77663_write(chip->dev, MAX77663_REG_ONOFF_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_ONOFF], 1, 0);

	/* Clear all interrups */
	max77663_read(chip->dev, MAX77663_REG_LBT_IRQ, &temp, 1, 0);
	max77663_read(chip->dev, MAX77663_REG_SD_IRQ, &temp, 1, 0);
	max77663_read(chip->dev, MAX77663_REG_LDOX_IRQ, &temp, 2, 0);
	//max77663_read(chip->dev, MAX77663_REG_GPIO_IRQ, &temp, 1, 0);
	max77663_read(chip->dev, MAX77663_REG_ONOFF_IRQ, &temp, 1, 0);

	for (i = chip->irq_base; i < (MAX77663_IRQ_NR + chip->irq_base); i++) {
		int irq = i - chip->irq_base;
		if (i >= NR_IRQS) {
			dev_err(chip->dev,
				"irq_init: Can't set irq chip for irq %d\n", i);
			continue;
		}
		if ((irq >= MAX77663_IRQ_GPIO0) && (irq <= MAX77663_IRQ_GPIO7))
			continue;

		irq_set_chip_data(i, chip);

		irq_set_chip_and_handler(i, &max77663_irq_chip, handle_edge_irq);
#ifdef CONFIG_ARM
		set_irq_flags(i, IRQF_VALID);
#else
		irq_set_noprobe(i);
#endif
		irq_set_nested_thread(i, 1);
	}

	ret = request_threaded_irq(chip->i2c_power->irq, NULL, max77663_irq,
				   IRQF_ONESHOT, "max77663", chip);
	if (ret) {
		dev_err(chip->dev, "irq_init: Failed to request irq %d\n",
			chip->i2c_power->irq);
		return ret;
	}

	device_init_wakeup(chip->dev, 1);
	enable_irq_wake(chip->i2c_power->irq);

	chip->cache_irq_top_mask &= ~IRQ_TOP_GLBL_MASK;
	max77663_write(chip->dev, MAX77663_REG_IRQ_TOP_MASK,
		       &chip->cache_irq_top_mask, 1, 0);

	chip->cache_irq_mask[CACHE_IRQ_LBT] &= ~IRQ_GLBL_MASK;
	max77663_write(chip->dev, MAX77663_REG_LBT_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_LBT], 1, 0);

	chip->cache_irq_mask[CACHE_IRQ_ONOFF] &= ~ONOFF_IRQ_EN0_RISING;
	max77663_write(chip->dev, MAX77663_REG_ONOFF_IRQ_MASK,
		       &chip->cache_irq_mask[CACHE_IRQ_ONOFF], 1, 0);

	return 0;
}

static void max77663_irq_exit(struct max77663_chip *chip)
{
	if (chip->i2c_power->irq)
		free_irq(chip->i2c_power->irq, chip);
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *max77663_dentry_regs;

static int max77663_debugfs_dump_regs(struct max77663_chip *chip, char *label,
				      u8 *addrs, int num_addrs, char *buf,
				      ssize_t *len, int is_rtc)
{
	ssize_t count = *len;
	u8 val;
	int ret = 0;
	int i;

	count += sprintf(buf + count, "%s\n", label);
	if (count >= PAGE_SIZE - 1)
		return -ERANGE;

	for (i = 0; i < num_addrs; i++) {
		count += sprintf(buf + count, "0x%02x: ", addrs[i]);
		if (count >= PAGE_SIZE - 1)
			return -ERANGE;

		ret = max77663_read(chip->dev, addrs[i], &val, 1, is_rtc);
		if (ret == 0)
			count += sprintf(buf + count, "0x%02x\n", val);
		else
			count += sprintf(buf + count, "<read fail: %d>\n", ret);

		if (count >= PAGE_SIZE - 1)
			return -ERANGE;
	}

	*len = count;
	return 0;
}

static int max77663_debugfs_regs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t max77663_debugfs_regs_read(struct file *file,
					  char __user *user_buf,
					  size_t count, loff_t *ppos)
{
	struct max77663_chip *chip = file->private_data;
	char *buf;
	size_t len = 0;
	ssize_t ret;

	/* Excluded interrupt status register to prevent register clear */
	u8 global_regs[] = { 0x00, 0x01, 0x02, 0x05, 0x0D, 0x0E, 0x13 };
	u8 sd_regs[] = {
		0x07, 0x0F, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
		0x1E, 0x1F, 0x20, 0x21, 0x22
	};
	u8 ldo_regs[] = {
		0x10, 0x11, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
		0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34,
		0x35
	};
	u8 gpio_regs[] = {
		0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
		0x40
	};
	u8 rtc_regs[] = {
		0x01, 0x02, 0x03, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
		0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
		0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B
	};
	u8 osc_32k_regs[] = { 0x03 };
	u8 bbc_regs[] = { 0x04 };
	u8 onoff_regs[] = { 0x12, 0x15, 0x41, 0x42 };
	u8 fps_regs[] = {
		0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C,
		0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
		0x57
	};
	u8 cid_regs[] = { 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D };

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += sprintf(buf + len, "MAX77663 Registers\n");
	max77663_debugfs_dump_regs(chip, "[Global]", global_regs,
				   ARRAY_SIZE(global_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Step-Down]", sd_regs,
				   ARRAY_SIZE(sd_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[LDO]", ldo_regs,
				   ARRAY_SIZE(ldo_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[GPIO]", gpio_regs,
				   ARRAY_SIZE(gpio_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[RTC]", rtc_regs,
				   ARRAY_SIZE(rtc_regs), buf, &len, 1);
	max77663_debugfs_dump_regs(chip, "[32kHz Oscillator]", osc_32k_regs,
				   ARRAY_SIZE(osc_32k_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Backup Battery Charger]", bbc_regs,
				   ARRAY_SIZE(bbc_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[On/OFF Controller]", onoff_regs,
				   ARRAY_SIZE(onoff_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Flexible Power Sequencer]", fps_regs,
				   ARRAY_SIZE(fps_regs), buf, &len, 0);
	max77663_debugfs_dump_regs(chip, "[Chip Identification]", cid_regs,
				   ARRAY_SIZE(cid_regs), buf, &len, 0);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return ret;
}

static const struct file_operations max77663_debugfs_regs_fops = {
	.open = max77663_debugfs_regs_open,
	.read = max77663_debugfs_regs_read,
};

static void max77663_debugfs_init(struct max77663_chip *chip)
{
	max77663_dentry_regs = debugfs_create_file(chip->i2c_power->name,
						   0444, 0, chip,
						   &max77663_debugfs_regs_fops);
	if (!max77663_dentry_regs)
		dev_warn(chip->dev,
			 "debugfs_init: Failed to create debugfs file\n");
}

static void max77663_debugfs_exit(struct max77663_chip *chip)
{
	debugfs_remove(max77663_dentry_regs);
}

#if 0 //def CONFIG_MACH_X3
static void max77663_dumpRegister(struct max77663_chip *chip, char *label,
				      u8 *addrs, int num_addrs, int is_rtc)
{
	int i;
	u8 val;
	int ret = 0;

	printk("%s\n", label);

	for (i = 0; i < num_addrs; i++) {
		printk("0x%02x: ", addrs[i]);

		ret = max77663_read(chip->dev, addrs[i], &val, 1, is_rtc);
		if (ret == 0)
			printk("0x%02x\n", val);
		else
			printk("<read fail: %d>\n", ret);
	}
}

static void max77663_AlldumpRegister(void)
{
	struct max77663_chip *chip = max77663_chip;

	u8 global_regs[] = { 0x00, 0x01, 0x02, 0x05, 0x0D, 0x0E, 0x13 };
	u8 sd_regs[] = {
		0x07, 0x0F, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
		0x1E, 0x1F, 0x20, 0x21, 0x22
	};
	u8 ldo_regs[] = {
		0x10, 0x11, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
		0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34,
		0x35
	};
	u8 gpio_regs[] = {
		0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
		0x40
	};
	u8 rtc_regs[] = {
		0x01, 0x02, 0x03, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
		0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
		0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B
	};
	u8 osc_32k_regs[] = { 0x03 };
	u8 bbc_regs[] = { 0x04 };
	u8 onoff_regs[] = { 0x12, 0x15, 0x41, 0x42 };
	u8 fps_regs[] = {
		0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C,
		0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
		0x57
	};
	u8 cid_regs[] = { 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D };


	max77663_dumpRegister(chip, "[Global]", global_regs,
				   ARRAY_SIZE(global_regs), 0);
	max77663_dumpRegister(chip, "[Step-Down]", sd_regs,
				   ARRAY_SIZE(sd_regs), 0);
	max77663_dumpRegister(chip, "[LDO]", ldo_regs,
				   ARRAY_SIZE(ldo_regs), 0);
	max77663_dumpRegister(chip, "[GPIO]", gpio_regs,
				   ARRAY_SIZE(gpio_regs), 0);
	max77663_dumpRegister(chip, "[RTC]", rtc_regs,
				   ARRAY_SIZE(rtc_regs),1);
	max77663_dumpRegister(chip, "[32kHz Oscillator]", osc_32k_regs,
				   ARRAY_SIZE(osc_32k_regs), 0);
	max77663_dumpRegister(chip, "[Backup Battery Charger]", bbc_regs,
				   ARRAY_SIZE(bbc_regs), 0);
	max77663_dumpRegister(chip, "[On/OFF Controller]", onoff_regs,
				   ARRAY_SIZE(onoff_regs), 0);
	max77663_dumpRegister(chip, "[Flexible Power Sequencer]", fps_regs,
				   ARRAY_SIZE(fps_regs), 0);
	max77663_dumpRegister(chip, "[Chip Identification]", cid_regs,
				   ARRAY_SIZE(cid_regs), 0);

}
#endif

#else
static inline void max77663_debugfs_init(struct max77663_chip *chip)
{
}

static inline void max77663_debugfs_exit(struct max77663_chip *chip)
{
}
#endif /* CONFIG_DEBUG_FS */

static bool rd_wr_reg_power(struct device *dev, unsigned int reg)
{
	if (reg < 0x60)
		return true;

	dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
	BUG();
	return false;
}

static bool rd_wr_reg_rtc(struct device *dev, unsigned int reg)
{
	if (reg < 0x1C)
		return true;

	dev_err(dev, "non-existing reg %s() reg 0x%x\n", __func__, reg);
	BUG();
	return false;
}

int max77663_read_chip_version(struct device *dev, u8 *val)
{
	int ret, version;

	version = MAX77663_DRV_NOT_DEFINED;
	ret = max77663_read(dev, MAX77663_REG_CID4, val, 1, 0);

	if (!ret) {
		if (*val == 0x24)
			version = MAX77663_DRV_24;
		return version;
	}
	return ret;
}

static const struct regmap_config max77663_regmap_config_power = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x60,
	.writeable_reg = rd_wr_reg_power,
	.readable_reg = rd_wr_reg_power,
	.cache_type = REGCACHE_RBTREE,
};

static const struct regmap_config max77663_regmap_config_rtc = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x1C,
	.writeable_reg = rd_wr_reg_rtc,
	.readable_reg = rd_wr_reg_rtc,
	.cache_type = REGCACHE_RBTREE,
};

#ifdef CONFIG_MACH_X3
static int max77663_reboot_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
	struct max77663_chip *chip = max77663_chip;

	dev_info(chip->dev, "%s: [%lu]\n", __func__, event);

	switch (event) {
		case SYS_RESTART:
		case SYS_HALT:
		case SYS_POWER_OFF: {
			max77663_set_bits(chip->dev, 0x47, 0xFF, 0x6a, 0);
			max77663_set_bits(chip->dev, 0x48, 0xFF, 0x5c, 0);
			max77663_set_bits(chip->dev, 0x49, 0xFF, 0x00, 0);
			max77663_set_bits(chip->dev, 0x4a, 0xFF, 0x07, 0);
			max77663_set_bits(chip->dev, 0x4c, 0xFF, 0x6a, 0);
			max77663_set_bits(chip->dev, 0x4f, 0xFF, 0x00, 0);
			max77663_set_bits(chip->dev, 0x51, 0xFF, 0x23, 0);
			max77663_set_bits(chip->dev, 0x52, 0xFF, 0x5c, 0);
			max77663_set_bits(chip->dev, 0x53, 0xFF, 0x00, 0);
//			max77663_set_bits(chip->dev, 0x03, 0x03, 0x03, 0);

#ifdef CONFIG_DEBUG_FS
//			max77663_AlldumpRegister();
#endif

			return NOTIFY_OK;
		}
	}

	return NOTIFY_DONE;
}

static struct notifier_block max77663_reboot_nb = {
	.notifier_call = max77663_reboot_notify,
};
#endif

static int max77663_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct max77663_platform_data *pdata = client->dev.platform_data;
	struct max77663_chip *chip;
	int ret = 0;
	u8 val;

	if (pdata == NULL) {
		dev_err(&client->dev, "probe: Invalid platform_data\n");
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&client->dev, "probe: kzalloc() failed\n");
		return -ENOMEM;
	}
	max77663_chip = chip;

	chip->i2c_power = client;
	i2c_set_clientdata(client, chip);

	chip->regmap_power = devm_regmap_init_i2c(client,
					&max77663_regmap_config_power);
	if (IS_ERR(chip->regmap_power)) {
		ret = PTR_ERR(chip->regmap_power);
		dev_err(&client->dev, "power regmap init failed: %d\n", ret);
		return ret;
	}

	if (pdata->rtc_i2c_addr)
		chip->rtc_i2c_addr = pdata->rtc_i2c_addr;
	else
		chip->rtc_i2c_addr = MAX77663_RTC_I2C_ADDR;

	chip->i2c_rtc = i2c_new_dummy(client->adapter, chip->rtc_i2c_addr);
	if (!chip->i2c_rtc) {
		dev_err(&client->dev, "can't attach client at addr 0x%x\n",
				chip->rtc_i2c_addr);
		return -ENOMEM;
	}
	i2c_set_clientdata(chip->i2c_rtc, chip);

	chip->regmap_rtc = devm_regmap_init_i2c(chip->i2c_rtc,
					&max77663_regmap_config_rtc);
	if (IS_ERR(chip->regmap_rtc)) {
		ret = PTR_ERR(chip->regmap_rtc);
		dev_err(&client->dev, "rtc tegmap init failed: %d\n", ret);
		goto out_rtc_regmap_fail;
	}

	chip->dev = &client->dev;
	chip->pdata = pdata;
	chip->irq_base = pdata->irq_base;
	mutex_init(&chip->io_lock);

	/* Dummy read to see if chip is present or not*/
	ret = max77663_read(chip->dev, MAX77663_REG_CID5, &val, 1, 0);
	if (ret < 0) {
		dev_err(chip->dev, "preinit: Failed to get register 0x%x\n",
				MAX77663_REG_CID5);
		return ret;
	}

	/* Reading chip version */
	ret = max77663_read_chip_version(chip->dev, &val);
	if (ret < 0)
		dev_err(chip->dev, "Failed to read chip version\n");
	else
		dev_dbg(chip->dev, "Chip version - 0x%x\n", val);

	max77663_irq_init(chip);
	max77663_debugfs_init(chip);
	ret = max77663_sleep(chip, false);
	if (ret < 0) {
		dev_err(&client->dev, "probe: Failed to disable sleep\n");
		goto out_exit;
	}

	if (pdata->use_power_off && !pm_power_off)
		pm_power_off = (void (*)(void))max77663_power_off;

	ret =  mfd_add_devices(&client->dev, -1, max77663_cells,
			ARRAY_SIZE(max77663_cells), NULL, chip->irq_base);
	if (ret < 0) {
		 dev_err(&client->dev, "mfd add dev failed, e = %d\n", ret);
		goto out_exit;
	}

#ifdef CONFIG_MACH_X3
	max77663_set_manual_resettime(chip);
#endif

	ret = mfd_add_devices(&client->dev, 0, pdata->sub_devices,
			      pdata->num_subdevs, NULL, 0);
	if (ret != 0) {
		dev_err(&client->dev, "probe: Failed to add subdev: %d\n", ret);
		goto out_mfd_clean;
	}

#ifdef CONFIG_MACH_X3
	register_reboot_notifier(&max77663_reboot_nb);
#endif

	return 0;

out_mfd_clean:
	mfd_remove_devices(chip->dev);
out_exit:
	max77663_debugfs_exit(chip);
	max77663_irq_exit(chip);
	mutex_destroy(&chip->io_lock);

out_rtc_regmap_fail:
	i2c_unregister_device(chip->i2c_rtc);
	max77663_chip = NULL;
	return ret;
}

static int __devexit max77663_remove(struct i2c_client *client)
{
	struct max77663_chip *chip = i2c_get_clientdata(client);

	mfd_remove_devices(chip->dev);
	max77663_debugfs_exit(chip);
	max77663_irq_exit(chip);
	mutex_destroy(&chip->io_lock);
	i2c_unregister_device(chip->i2c_rtc);
	max77663_chip = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77663_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max77663_chip *chip = i2c_get_clientdata(client);
	int ret;

	if (client->irq)
		disable_irq(client->irq);

	ret = max77663_sleep(chip, true);
	if (ret < 0) {
#ifdef CONFIG_MACH_X3
		if (client->irq)
			enable_irq(client->irq);
#endif
		dev_err(dev, "suspend: Failed to enable sleep\n");
	}

	return ret;
}

#if defined(CONFIG_MACH_DUMP_GPIO)
static DEFINE_SPINLOCK(dump_spinlock);
char gpio_regs_buf_suspend[PAGE_SIZE];
extern const struct tegra_drive_pingroup_desc tegra_soc_drive_pingroups[TEGRA_MAX_DRIVE_PINGROUP];
extern u32 pinmux_reg[TEGRA_MAX_PINGROUP + ARRAY_SIZE(tegra_soc_drive_pingroups)];
#endif

static int max77663_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max77663_chip *chip = i2c_get_clientdata(client);
	int ret;
#if defined(CONFIG_MACH_DUMP_GPIO)
	int i;
	u32 *ctx = pinmux_reg;
#endif

	ret = max77663_sleep(chip, false);
	if (ret < 0) {
		dev_err(dev, "resume: Failed to disable sleep\n");
		return ret;
	}

	if (client->irq)
		enable_irq(client->irq);

#if defined(CONFIG_MACH_DUMP_GPIO)
	spin_lock(&dump_spinlock);

	pr_info("\n==>dump pinmux regs\n");
	for (i = 0; i < TEGRA_MAX_PINGROUP; i++)
		pr_info("%16s %8x\n", tegra_soc_pingroups[i].name, *ctx++);

	pr_info("\n==>dump pinmux drive regs\n");
	for (i = 0; i < ARRAY_SIZE(tegra_soc_drive_pingroups); i++)
		pr_info("%16s %8x\n", tegra_soc_drive_pingroups[i].name, *ctx++);

	pr_info("\n==>dump gpio regs\n");
	pr_info("Bank:Port CNF OE OUT\n");
	pr_info("%s\n", gpio_regs_buf_suspend);

	spin_unlock(&dump_spinlock);
#endif
	return 0;
}
#else
#define max77663_suspend      NULL
#define max77663_resume       NULL
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_MACH_X3
static int __init max77663_reset_cause_cmdline(char *str)
{
	int output[4] = { 0 };
 
	sscanf(str, "%d,%d,%d,%d ", &output[0], &output[1], &output[2], &output[3]);

	printk("%s bootcause[%d] max77663RTCStatus[%x] max77663rebootcause[%x] max77663OnOffStat[%x]\n",
		__func__, output[0], output[1], output[2], output[3]);

	return 1;
}
__setup("pmicresetcause=", max77663_reset_cause_cmdline);
#endif

static const struct i2c_device_id max77663_id[] = {
	{"max77663", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, max77663_id);

static const struct dev_pm_ops max77663_pm = {
	.suspend = max77663_suspend,
	.resume = max77663_resume,
};

static struct i2c_driver max77663_driver = {
	.driver = {
		.name = "max77663",
		.owner = THIS_MODULE,
		.pm = &max77663_pm,
	},
	.probe = max77663_probe,
	.remove = __devexit_p(max77663_remove),
	.id_table = max77663_id,
};

static int __init max77663_init(void)
{
	return i2c_add_driver(&max77663_driver);
}
#ifdef CONFIG_MACH_X3
arch_initcall(max77663_init);
#else
subsys_initcall(max77663_init);
#endif

static void __exit max77663_exit(void)
{
	i2c_del_driver(&max77663_driver);
}
module_exit(max77663_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77663 Multi Function Device Core Driver");
MODULE_VERSION("1.0");
