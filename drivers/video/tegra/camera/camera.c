/*
 * drivers/video/tegra/camera/camera.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "camera_priv_defs.h"
#include "camera_clk.h"
#include "camera_power.h"
#include "camera_emc.h"

#define TEGRA_CAMERA_NAME "tegra_camera"

static struct clock_data clock_init[] = {
	{ CAMERA_ISP_CLK, "isp", true, 0},
	{ CAMERA_VI_CLK, "vi", true, 0},
	{ CAMERA_VI_SENSOR_CLK, "vi_sensor", true, 0},
	{ CAMERA_CSUS_CLK, "csus", true, 0},
	{ CAMERA_CSI_CLK, "csi", true, 0},
	{ CAMERA_EMC_CLK, "emc", true, 0},
#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	{ CAMERA_CILAB_CLK, "cilab", true, 0},
	{ CAMERA_CILCD_CLK, "cilcd", true, 0},
	{ CAMERA_CILE_CLK, "cile", true, 0},
	{ CAMERA_PLL_D2_CLK, "pll_d2", false, 0},
#endif
	{ CAMERA_SCLK, "sclk", true, 80000000},
};

#ifdef CONFIG_MACH_X3
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>

static struct tegra_camera *tegra_camera_dev;

//#define POWER_SAVE_REC_CPU_USER_CAP_RATE	640000
#define POWER_SAVE_BOOST_STEP			1
#define POWER_SAVE_CPU_FREQ_MIN			640000
#define POWER_SAVE_CPU_FREQ_MAX			640000
#define POWER_SAVE_MIN_CPUS			2
#define POWER_SAVE_MAX_CPUS			2

static unsigned long boost_step_default;

static inline void tegra_camera_do_power_save(struct tegra_camera *dev)
{
#if 0
	int preview, rec;
	
	pr_info("%s \n", __func__);

	preview = dev->power_save_preview;
	rec = dev->power_save_rec;

	if (!dev->power_save && (preview || rec)) {
		boost_step_default = cpufreq_interactive_get_boost_step();
		dev->power_save = true;
	}

	if (!dev->power_save)
		return;

	if (preview && rec) {    
		pr_info("%s : when preview && rec \n", __func__);
		cpufreq_interactive_set_boost_step(POWER_SAVE_BOOST_STEP);

		cpufreq_set_max_freq(NULL, POWER_SAVE_CPU_FREQ_MAX);
		if ((dev->xres == 1280 && dev->yres == 720) ||
			(dev->xres == 1440 && dev->yres == 1080) ||
				(dev->xres == 1920 && dev->yres == 1080)) {
			cpufreq_set_min_freq(NULL, POWER_SAVE_CPU_FREQ_MIN);
#ifndef CONFIG_CPUQUIET_FRAMEWORK
			tegra_auto_hotplug_set_min_cpus(POWER_SAVE_MIN_CPUS);
			tegra_auto_hotplug_set_max_cpus(POWER_SAVE_MAX_CPUS);
#endif
		}
	} else if (preview && !rec) {
		pr_info("%s : preview && !rec \n", __func__);

		cpufreq_interactive_set_boost_step(POWER_SAVE_BOOST_STEP);
#if 0 // kwanghee.choi 20120912 Vu1.0 Global fix the frame drop on Camera by setting full CPU clock (start)
		cpufreq_set_min_freq(NULL, POWER_SAVE_CPU_FREQ_MIN);
		cpufreq_set_max_freq(NULL, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
#ifndef CONFIG_CPUQUIET_FRAMEWORK
		tegra_auto_hotplug_set_min_cpus(0);
		tegra_auto_hotplug_set_max_cpus(0);
#endif
#else
		cpufreq_set_min_freq(NULL, PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
		cpufreq_set_max_freq(NULL, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
#ifndef CONFIG_CPUQUIET_FRAMEWORK
		tegra_auto_hotplug_set_min_cpus(0);
		tegra_auto_hotplug_set_max_cpus(0);
#endif
#endif // kwanghee.choi 20120912 Vu1.0 Global fix the frame drop on Camera by setting full CPU clock (end)
	} else if (!preview && !rec) {
		pr_info("%s : !preview && !rec \n", __func__);

		cpufreq_interactive_set_boost_step(boost_step_default);

		cpufreq_set_min_freq(NULL, PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
		cpufreq_set_max_freq(NULL, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
#ifndef CONFIG_CPUQUIET_FRAMEWORK
		tegra_auto_hotplug_set_min_cpus(0);
		tegra_auto_hotplug_set_max_cpus(0);
#endif

		dev->power_save = false;
	}
#else
	pr_info("%s: Test This Camera.\n", __func__);
#endif
}

int tegra_camera_set_size(int xres, int yres)
{
	struct tegra_camera *camera = tegra_camera_dev;
	if ((xres <= 0) || (yres <= 0))
		return -EINVAL;

	mutex_lock(&camera->tegra_camera_lock);
	camera->xres = xres;
	camera->yres = yres;
	tegra_camera_do_power_save(camera);
	mutex_unlock(&camera->tegra_camera_lock);

	return 0;
}

static ssize_t tegra_camera_show_power_save_preview(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tegra_camera *camera = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", camera->power_save_preview);
}

static ssize_t tegra_camera_store_power_save_preview(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct tegra_camera *camera = dev_get_drvdata(dev);
	int val, ret;

	ret = sscanf(buf, "%d", &val);
	if (!ret)
		return ret;

	mutex_lock(&camera->tegra_camera_lock);
	if (val == 1 && !camera->power_save_preview)
		camera->power_save_preview = true;
	else if (val == 0 && camera->power_save_preview)
		camera->power_save_preview = false;

	tegra_camera_do_power_save(camera);
	mutex_unlock(&camera->tegra_camera_lock);

	return count;
}

static DEVICE_ATTR(power_save_preview, S_IRUGO | S_IWUSR | S_IRGRP | S_IWGRP,
		   tegra_camera_show_power_save_preview,
		   tegra_camera_store_power_save_preview);

static ssize_t tegra_camera_show_power_save_rec(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tegra_camera *camera = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", camera->power_save_rec);
}

static ssize_t tegra_camera_store_power_save_rec(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct tegra_camera *camera = dev_get_drvdata(dev);
	int val, ret;

	ret = sscanf(buf, "%d", &val);
	if (!ret)
		return ret;

	mutex_lock(&camera->tegra_camera_lock);
	if (val == 1 && !camera->power_save_rec)
		camera->power_save_rec = true;
	else if (val == 0 && camera->power_save_rec)
		camera->power_save_rec = false;

	tegra_camera_do_power_save(camera);
	mutex_unlock(&camera->tegra_camera_lock);

	return count;
}

static DEVICE_ATTR(power_save_rec, S_IRUGO | S_IWUSR | S_IRGRP | S_IWGRP,
		   tegra_camera_show_power_save_rec,
		   tegra_camera_store_power_save_rec);

bool tegra_camera_get_power_save_rec(void)
{
	if (tegra_camera_dev != NULL) {
		return tegra_camera_dev->power_save_rec;
	}
	return false;
}
EXPORT_SYMBOL(tegra_camera_get_power_save_rec);
#endif

static long tegra_camera_ioctl(struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	uint id;
	struct tegra_camera *camera = file->private_data;

	/* first element of arg must be u32 with id of module to talk to */
	if (copy_from_user(&id, (const void __user *)arg, sizeof(uint))) {
		dev_err(camera->dev,
				"%s: Failed to copy arg from user", __func__);
		return -EFAULT;
	}

	if (id >= TEGRA_CAMERA_MODULE_MAX) {
		dev_err(camera->dev,
				"%s: Invalid id to tegra isp ioctl%d\n",
				__func__, id);
		return -EINVAL;
	}

	switch (cmd) {
	/*
	 * Clock enable/disable and reset should be handled in kernel.
	 * In order to support legacy code in user space, we don't remove
	 * these IOCTL.
	 */
	case TEGRA_CAMERA_IOCTL_ENABLE:
	case TEGRA_CAMERA_IOCTL_DISABLE:
	case TEGRA_CAMERA_IOCTL_RESET:
		return 0;
	case TEGRA_CAMERA_IOCTL_CLK_SET_RATE:
	{
		int ret;

		if (copy_from_user(&camera->info, (const void __user *)arg,
				   sizeof(struct tegra_camera_clk_info))) {
			dev_err(camera->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}
		ret = tegra_camera_clk_set_rate(camera);
		if (ret)
			return ret;
		if (copy_to_user((void __user *)arg, &camera->info,
				 sizeof(struct tegra_camera_clk_info))) {
			dev_err(camera->dev,
				"%s: Failed to copy arg to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	default:
		dev_err(camera->dev,
				"%s: Unknown tegra_camera ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int tegra_camera_open(struct inode *inode, struct file *file)
{
	int ret;
	struct miscdevice *miscdev = file->private_data;
	struct tegra_camera *camera = container_of(miscdev,
						struct tegra_camera,
						misc_dev);

	dev_info(camera->dev, "%s: ++\n", __func__);

	if (atomic_xchg(&camera->in_use, 1))
		return -EBUSY;

	file->private_data = camera;

	mutex_lock(&camera->tegra_camera_lock);

	/* turn on CSI regulator */
	ret = tegra_camera_power_on(camera);
	if (ret)
		goto power_on_fail;
	/* set EMC request */
	ret = tegra_camera_enable_emc(camera);
	if (ret)
		goto enable_emc_fail;

	/* read initial clock info */
	tegra_camera_init_clk(camera, clock_init);

	/* enable camera HW clock */
	ret = tegra_camera_enable_clk(camera);
	if (ret)
		goto enable_clk_fail;

	mutex_unlock(&camera->tegra_camera_lock);

	return 0;

enable_clk_fail:
	tegra_camera_disable_emc(camera);
enable_emc_fail:
	tegra_camera_power_off(camera);
power_on_fail:
	mutex_unlock(&camera->tegra_camera_lock);
	return ret;
}

static int tegra_camera_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct tegra_camera *camera = file->private_data;

	dev_info(camera->dev, "%s++\n", __func__);

	mutex_lock(&camera->tegra_camera_lock);
	/* disable HW clock */
	ret = tegra_camera_disable_clk(camera);
	if (ret)
		goto release_exit;
	/* nullify EMC request */
	ret = tegra_camera_disable_emc(camera);
	if (ret)
		goto release_exit;
	/* turn off CSI regulator */
	ret = tegra_camera_power_off(camera);
	if (ret)
		goto release_exit;

release_exit:
	mutex_unlock(&camera->tegra_camera_lock);
	WARN_ON(!atomic_xchg(&camera->in_use, 0));
	return 0;
}

static const struct file_operations tegra_camera_fops = {
	.owner = THIS_MODULE,
	.open = tegra_camera_open,
	.unlocked_ioctl = tegra_camera_ioctl,
	.release = tegra_camera_release,
};

static int tegra_camera_clk_get(struct platform_device *ndev, const char *name,
				struct clk **clk)
{
	*clk = clk_get(&ndev->dev, name);
	if (IS_ERR_OR_NULL(*clk)) {
		dev_err(&ndev->dev, "%s: unable to get clock for %s\n",
			__func__, name);
		*clk = NULL;
		return PTR_ERR(*clk);
	}
	return 0;
}

struct tegra_camera *tegra_camera_register(struct platform_device *ndev)
{
	struct tegra_camera *camera = NULL;
	int ret = 0;
	int i;

	dev_info(&ndev->dev, "%s: ++\n", __func__);

	camera = kzalloc(sizeof(struct tegra_camera), GFP_KERNEL);
	if (!camera) {
		dev_err(&ndev->dev, "can't allocate memory for tegra_camera\n");
		return camera;
	}

	mutex_init(&camera->tegra_camera_lock);

	/* Powergate VE when boot */
	mutex_lock(&camera->tegra_camera_lock);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	ret = tegra_camera_powergate_init(camera);
	if (ret)
		goto regulator_fail;
#endif
	mutex_unlock(&camera->tegra_camera_lock);

	camera->dev = &ndev->dev;

	/* Get regulator pointer */
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	camera->reg = regulator_get(&ndev->dev, "vcsi");
#else
	camera->reg = regulator_get(&ndev->dev, "avdd_dsi_csi");
#endif

	if (IS_ERR_OR_NULL(camera->reg)) {
		ret = -ENODEV;
		if (camera->reg == ERR_PTR(-ENODEV)) {
			camera->reg = NULL;
			dev_info(&ndev->dev,
				"%s: no regulator device, overriding\n",
							__func__);
		} else {
			dev_err(&ndev->dev, "%s: couldn't get regulator\n",
							__func__);
			goto regulator_fail;
		}
	}

	camera->misc_dev.minor = MISC_DYNAMIC_MINOR;
	camera->misc_dev.name = TEGRA_CAMERA_NAME;
	camera->misc_dev.fops = &tegra_camera_fops;
	camera->misc_dev.parent = &ndev->dev;
	ret = misc_register(&camera->misc_dev);
	if (ret) {
		dev_err(&ndev->dev, "%s: unable to register misc device!\n",
			TEGRA_CAMERA_NAME);
		goto misc_register_fail;
	}

	for (i = 0; i < CAMERA_CLK_MAX; i++) {
		ret = tegra_camera_clk_get(ndev, clock_init[i].name,
				&camera->clock[clock_init[i].index].clk);
		if (ret)
			goto clk_get_fail;
	}

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	/* Dedicated bw is what VI could ask for at most */
	camera->isomgr_handle = tegra_isomgr_register(TEGRA_ISO_CLIENT_VI_0,
					/* dedicated bw, KBps*/
					tegra_camera_get_max_bw(camera),
					NULL,	/* tegra_isomgr_renegotiate */
					NULL);	/* *priv */
	if (!camera->isomgr_handle) {
		dev_err(&ndev->dev, "%s: unable to register isomgr\n",
					__func__);
		goto clk_get_fail;
	}
#endif

#ifdef CONFIG_MACH_X3
	tegra_camera_dev = camera;
	camera->power_save = false;

	ret = device_create_file(camera->dev, &dev_attr_power_save_preview);
	if (ret < 0)
		dev_warn(camera->dev,
			 "%s: failed to add power_save_preview sysfs: %d\n",
			 __func__, ret);

	ret = device_create_file(camera->dev, &dev_attr_power_save_rec);
	if (ret < 0)
		dev_warn(camera->dev,
			 "%s: failed to add power_save_rec sysfs: %d\n",
			 __func__, ret);
#endif

	return camera;

clk_get_fail:
	for (; i > 0; i--)
		clk_put(camera->clock[clock_init[i].index].clk);
	misc_deregister(&camera->misc_dev);
misc_register_fail:
	regulator_put(camera->reg);
regulator_fail:
	kfree(camera);
	camera = NULL;
	return camera;
}

int tegra_camera_unregister(struct tegra_camera *camera)
{
	int i;

	dev_info(camera->dev, "%s: ++\n", __func__);

	for (i = 0; i < CAMERA_CLK_MAX; i++)
		clk_put(camera->clock[i].clk);

#ifdef CONFIG_ARCH_TEGRA_11x_SOC
	/*
	 * Return memory bandwidth to isomgr.
	 * If bandwidth is zero, then latency will be ignored
	 * in tegra_isomgr_reserve().
	 */
	{
		int ret = 0;

		ret = tegra_isomgr_reserve(camera->isomgr_handle,
					0,	/* KB/sec */
					0);	/* usec */
		if (!ret)
			return -ENOMEM;

		tegra_isomgr_unregister(camera->isomgr_handle);
		camera->isomgr_handle = NULL;
	}
#endif
	kfree(camera);

	return 0;
}

#ifdef CONFIG_PM
int tegra_camera_suspend(struct tegra_camera *camera)
{
	int ret = 0;

	dev_dbg(camera->dev, "%s: ++\n", __func__);
	mutex_lock(&camera->tegra_camera_lock);
	if (camera->power_on) {
		ret = -EBUSY;
		dev_err(camera->dev,
		"tegra_camera cannot suspend, "
		"application is holding on to camera.\n");
	}
	mutex_unlock(&camera->tegra_camera_lock);

	return ret;
}

int tegra_camera_resume(struct tegra_camera *camera)
{
	dev_info(camera->dev, "%s: ++\n", __func__);
	return 0;
}
#endif
