/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/gpio-tegra.h>
#include "board.h"
#include "tegra-board-id.h"
#include "gpio-names.h"
#include "pm-irq.h"

/* Tegra USB1 wake source index */
#define USB1_VBUS_WAKE 19
#define USB1_ID_WAKE 21
#define USB1_REM_WAKE 39

/* constants for USB1 wake sources - VBUS and ID */
#define USB1_IF_USB_PHY_VBUS_SENSORS_0 0x408
#define VBUS_WAKEUP_STS_BIT 10
#define ID_STS_BIT 2

static int tegra_gpio_wakes[] = {
	TEGRA_GPIO_PO5,				/* wake0 */
	TEGRA_GPIO_PV1,				/* wake1 */
	-EINVAL,				/* wake2 */
	-EINVAL,				/* wake3 */
	-EINVAL,				/* wake4 */
	-EINVAL,				/* wake5 */
	TEGRA_GPIO_PU5,				/* wake6 */
	TEGRA_GPIO_PU6,				/* wake7 */
	TEGRA_GPIO_PC7,				/* wake8 */
	TEGRA_GPIO_PS2,				/* wake9 */
	-EINVAL,				/* wake10 */
	TEGRA_GPIO_PW3,				/* wake11 */
	TEGRA_GPIO_PW2,				/* wake12 */
	-EINVAL,				/* wake13 */
	TEGRA_GPIO_PDD3,			/* wake14 */
	-EINVAL,				/* wake15 */
	-EINVAL,				/* wake16 */
	-EINVAL,				/* wake17 */
	-EINVAL,				/* wake18 */
	-EINVAL,				/* wake19 */
	-EINVAL,				/* wake20 */
	-EINVAL,				/* wake21 */
	-EINVAL,				/* wake22 */
	TEGRA_GPIO_PI5,				/* wake23 */
	TEGRA_GPIO_PV0,				/* wake24 */
	-EINVAL,				/* wake25 */
	-EINVAL,				/* wake26 */
	TEGRA_GPIO_PS0,				/* wake27 */
	-EINVAL,				/* wake28 */
	-EINVAL,				/* wake29 */
	-EINVAL,				/* wake30 */
	-EINVAL,				/* wake31 */
	-EINVAL,				/* wake32 */
	TEGRA_GPIO_PJ0,				/* wake33 */
	TEGRA_GPIO_PK2,				/* wake34 */
	TEGRA_GPIO_PI6,				/* wake35 */
	-EINVAL,				/* wake36 */
	-EINVAL,				/* wake37 */
	-EINVAL,				/* wake38 */
	-EINVAL,				/* wake39 */
	-EINVAL,				/* wake40 */
	-EINVAL,				/* wake41 */
	-EINVAL,				/* wake42 */
	-EINVAL,				/* wake43 */
	-EINVAL,				/* wake44 */
	TEGRA_GPIO_PBB6,			/* wake45 */
	-EINVAL,				/* wake46 */
	TEGRA_GPIO_PT6,				/* wake47 */
	-EINVAL,				/* wake48 */
	TEGRA_GPIO_PR7,				/* wake49 */
	TEGRA_GPIO_PR4,				/* wake50 */
	TEGRA_GPIO_PQ0,				/* wake51 */
	-EINVAL,				/* wake52 */
	-EINVAL,				/* wake53 */
	TEGRA_GPIO_PQ5,				/* wake54 */
	-EINVAL,				/* wake55 */
	TEGRA_GPIO_PV2,				/* wake56 */
	-EINVAL,				/* wake57 */
	-EINVAL,				/* wake58 */
};

static int tegra_wake_event_irq[] = {
	-EAGAIN, /* ULPI DATA4 */		/* wake0 */
	-EAGAIN,				/* wake1 */
	-EAGAIN,				/* wake2 */
	INT_SDMMC3, /* SDMMC3 DAT1 */		/* wake3 */
	INT_HDMI, /* HDMI INT */		/* wake4 */
	-EAGAIN,				/* wake5 */
	-EAGAIN,				/* wake6 */
	-EAGAIN,				/* wake7 */
	-EAGAIN,				/* wake8 */
	INT_UARTC, /* UART3 RXD */		/* wake9 */
	INT_SDMMC4, /* SDMMC4 DAT1 */		/* wake10 */
	-EAGAIN,				/* wake11 */
	-EAGAIN,				/* wake12 */
	INT_SDMMC1, /* SDMMC1 DAT1 */		/* wake13 */
	-EAGAIN,				/* wake14 */
	INT_THERMAL,				/* wake15 */
	INT_RTC,				/* wake16 */
	INT_KBC,				/* wake17 */
	INT_EXTERNAL_PMU,			/* wake18 */
	-EINVAL,				/* wake19 */
	-EINVAL,				/* wake20 */
	-EINVAL,				/* wake21 */
	-EINVAL,				/* wake22 */
	-EAGAIN,				/* wake23 */
	-EAGAIN,				/* wake24 */
	-EAGAIN,				/* wake25 */
	-EAGAIN,				/* wake26 */
	-EAGAIN,				/* wake27 */
	-EAGAIN,				/* wake28 */
	-EAGAIN,				/* wake29 */
	INT_AUDIO_CLUSTER, /* I2S0 SDATA OUT */		/* wake30 */
	-EINVAL,				/* wake31 */
	-EINVAL, /* ULPI DATA3 */		/* wake32 */
	-EAGAIN,				/* wake33 */
	-EAGAIN,				/* wake34 */
	-EAGAIN,				/* wake35 */
	-EAGAIN,				/* wake36 */
	-EINVAL, /* TEGRA_USB3_VBUS, */		/* wake37 */
	-EINVAL, /* TEGRA_USB3_ID, */		/* wake38 */
	INT_USB, /* TEGRA_USB1_UTMIP, */	/* wake39 */
	-EINVAL,				/* wake40 */
	-EINVAL,				/* wake41 */
	INT_USB2, /* USB1 UHSIC PHY */		/* wake42 */
	INT_USB3, /* USB3 UHSIC PHY */		/* wake43 */
	INT_I2C, /* I2C1 DAT */		/* wake44 */
	-EAGAIN,				/* wake45 */
	INT_I2C5, /* PWR I2C DAT */		/* wake46 */
	INT_I2C2, /* I2C2 DAT */		/* wake47 */
	INT_I2C3, /* I2C3 DAT */		/* wake48 */
	-EAGAIN,				/* wake49 */
	-EAGAIN,				/* wake50 */
	INT_KBC, /* KBC11 */			/* wake51 */
	INT_HDMI, /* HDMI CEC */		/* wake52 */
	INT_I2C3, /* I2C3 CLK */		/* wake53 */
	-EAGAIN,				/* wake54 */
	INT_UARTC, /* UART3 CTS */		/* wake55 */
	INT_SDMMC3, /* SDMMC3 CD */		/* wake56 */
	INT_USB, /* TEGRA_USB1_VBUS_EN1, */	/* wake57 */
	INT_XUSB_PADCTL, /* XUSB superspeed wake */	/* wake58 */
};

static int last_gpio = -1;

/* USB1 VBUS and ID wake sources are handled as special case
 * Note: SD card detect is an ANY wake source but is
 * mostly a GPIO which can handle any edge wakeup.
 */
static u8 any_wake_t11x[] = {
	/* DO NOT EDIT this list */
	[ANY_WAKE_INDEX_VBUS] = USB1_VBUS_WAKE,
	[ANY_WAKE_INDEX_ID] = USB1_ID_WAKE,
};

void tegra_get_internal_any_wake_list(u8 *wake_count, u8 **any_wake,
	u8 *remote_usb)
{
	*wake_count = ARRAY_SIZE(any_wake_t11x);
	*any_wake = any_wake_t11x;
	*remote_usb = USB1_REM_WAKE;
}

/* Needed on dalmore today hence exposed this API */
int get_vbus_id_cable_connect_state(bool *is_vbus_connected,
		bool *is_id_connected)
{
	static void __iomem *usb1_base = IO_ADDRESS(TEGRA_USB_BASE);
	u32 reg;

	reg = readl(usb1_base + USB1_IF_USB_PHY_VBUS_SENSORS_0);

	/* ID bit when 0 - ID cable connected */
	*is_id_connected = (reg & (1 << ID_STS_BIT)) ? false : true;

	/*
	 * VBUS_WAKEUP_STS_BIT is also set when ID is connected
	 * and we are supplying VBUS, hence below conditional assignment
	 */
	if (*is_id_connected)
		*is_vbus_connected = false;
	else
		/* VBUS bit when 1 - VBUS cable connected */
		*is_vbus_connected = (reg & (1 << VBUS_WAKEUP_STS_BIT)) ?
			true : false;
	return 0;
}

int tegra_gpio_to_wake(int gpio)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra_gpio_wakes); i++) {
		if (tegra_gpio_wakes[i] == gpio) {
			pr_info("gpio wake%d for gpio=%d\n", i, gpio);
			last_gpio = i;
			return i;
		}
	}

	return -EINVAL;
}

void tegra_set_usb_wake_source(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	/* For Dalmore */
	if (board_info.board_id == BOARD_E1611) {
		tegra_wake_event_irq[41] = INT_USB3;
		tegra_wake_event_irq[43] = -EINVAL;
	}
}

void tegra_irq_to_wake(int irq, int *wak_list, int *wak_size)
{
	int i;

	*wak_size = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_wake_event_irq); i++) {
		if (tegra_wake_event_irq[i] == irq) {
			pr_info("Wake%d for irq=%d\n", i, irq);
			wak_list[*wak_size] = i;
			*wak_size = *wak_size + 1;
		}
	}
	if (*wak_size)
		goto out;

	/* The gpio set_wake code bubbles the set_wake call up to the irq
	 * set_wake code. This insures that the nested irq set_wake call
	 * succeeds, even though it doesn't have to do any pm setup for the
	 * bank.
	 *
	 * This is very fragile - there's no locking, so two callers could
	 * cause issues with this.
	 */
	if (last_gpio < 0)
		goto out;

	if (tegra_gpio_get_bank_int_nr(tegra_gpio_wakes[last_gpio]) == irq) {
		pr_info("gpio bank wake found: wake%d for irq=%d\n", i, irq);
		wak_list[*wak_size] = last_gpio;
		*wak_size = 1;
	}

out:
	return;
}

int tegra_wake_to_irq(int wake)
{
	int ret;

	if (wake < 0)
		return -EINVAL;

	if (wake >= ARRAY_SIZE(tegra_wake_event_irq))
		return -EINVAL;

	ret = tegra_wake_event_irq[wake];
	if (ret == -EAGAIN) {
		ret = tegra_gpio_wakes[wake];
		if (ret != -EINVAL)
			ret = gpio_to_irq(ret);
	}

	return ret;
}

int tegra_disable_wake_source(int wake)
{
	if (wake >= ARRAY_SIZE(tegra_wake_event_irq))
		return -EINVAL;

	tegra_wake_event_irq[wake] = -EINVAL;
	return 0;
}
