/*
 * tegra_max98088.c - Tegra machine ASoC driver for boards using MAX98088 codec.
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION. All rights reserved.
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/tegra_asoc_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/max98088.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra30_ahub.h"
#include "tegra30_i2s.h"
#include "tegra30_dam.h"
#endif

#ifdef CONFIG_MACH_X3
#include <mach/gpio-tegra.h>
#include <linux/wakelock.h>
#endif

#define DRV_NAME "tegra-snd-max98088"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET	BIT(4)

#define DAI_LINK_HIFI		0
#define DAI_LINK_SPDIF		1
#define DAI_LINK_BTSCO		2
#define DAI_LINK_VOICE_CALL	3
#define DAI_LINK_BT_VOICE_CALL	4
#ifdef CONFIG_MACH_X3
 #define DAI_LINK_FM_RADIO	5
 #define NUM_DAI_LINKS	6
#else
 #define NUM_DAI_LINKS	5
#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
const char *tegra_max98088_i2s_dai_name[TEGRA30_NR_I2S_IFC] = {
	"tegra30-i2s.0",
	"tegra30-i2s.1",
	"tegra30-i2s.2",
	"tegra30-i2s.3",
	"tegra30-i2s.4",
};
#endif

extern int g_is_call_mode;

#ifdef CONFIG_MACH_X3
struct wake_lock x3_callmodelock;
struct headset_switch_data	*headset_sw_data;
#endif

struct tegra_max98088 {
	struct tegra_asoc_utils_data util_data;
	struct tegra_asoc_platform_data *pdata;
	int gpio_requested;
	bool init_done;
	int is_call_mode;
	int is_device_bt;
#ifdef CONFIG_MACH_X3
	int is_radio_mode;
	int call_record_tx_gain;
	int call_record_rx_gain;
#endif
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct codec_config codec_info[NUM_I2S_DEVICES];
#endif
	enum snd_soc_bias_level bias_level;
	struct snd_soc_card *pcard;
	volatile int clock_enabled;
};

static int tegra_call_mode_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int tegra_call_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->is_call_mode;

	return 0;
}

static int tegra_call_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);
	int is_call_mode_new = ucontrol->value.integer.value[0];
	int codec_index;
	unsigned int i;

	if (machine->is_call_mode == is_call_mode_new)
		return 0;

	if (machine->is_device_bt)
		codec_index = BT_SCO;
	else
		codec_index = HIFI_CODEC;

	if (is_call_mode_new) {
#ifndef CONFIG_ARCH_TEGRA_2x_SOC

		if (machine->codec_info[codec_index].rate == 0 ||
			machine->codec_info[codec_index].channels == 0)
				return -EINVAL;

		wake_lock(&x3_callmodelock);

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 1;

		tegra30_make_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND], 0);
#endif
	} else {
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		tegra30_break_voice_call_connections(
			&machine->codec_info[codec_index],
			&machine->codec_info[BASEBAND], 0);

		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 0;

		wake_unlock(&x3_callmodelock);
#endif
	}

	machine->is_call_mode = is_call_mode_new;
	g_is_call_mode = machine->is_call_mode;

	return 1;
}

#ifdef CONFIG_MACH_X3
static int tegra_voice_dl_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 2048; // 4096;
	return 0;
}

static int volume_get_voice_dl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		machine->codec_info[HIFI_CODEC].dam_gain[TEGRA30_DAM_CHIN0_SRC];

	return 0;
}

static int volume_put_voice_dl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	machine->codec_info[HIFI_CODEC].dam_gain[TEGRA30_DAM_CHIN0_SRC] =
						ucontrol->value.integer.value[0];
	machine->codec_info[BT_SCO].dam_gain[TEGRA30_DAM_CHIN0_SRC] =
						ucontrol->value.integer.value[0];
		
	tegra30_set_dam_ifc_gain(&machine->codec_info[HIFI_CODEC]);
	tegra30_set_dam_ifc_gain(&machine->codec_info[BT_SCO]);

	return 1;
}

static int volume_get_voice_ul(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		machine->codec_info[BASEBAND].dam_gain[TEGRA30_DAM_CHIN0_SRC];
	
	return 0;
}

static int tegra_voice_ul_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 2048; // 4096;
	return 0;
}

static int volume_put_voice_ul(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	machine->codec_info[BASEBAND].dam_gain[TEGRA30_DAM_CHIN0_SRC] =
						ucontrol->value.integer.value[0];

	tegra30_set_dam_ifc_gain(&machine->codec_info[BASEBAND]);
	
	return 1;
}

static int tegra_multi_dl_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 2048; // 4096;
	return 0;
}

static int volume_get_multi_dl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		machine->codec_info[HIFI_CODEC].dam_gain[TEGRA30_DAM_CHIN1];

	return 0;
}

static int volume_put_multi_dl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	machine->codec_info[HIFI_CODEC].dam_gain[TEGRA30_DAM_CHIN1] =
						ucontrol->value.integer.value[0];
	machine->codec_info[BT_SCO].dam_gain[TEGRA30_DAM_CHIN1] =
						ucontrol->value.integer.value[0];
	
	tegra30_set_dam_ifc_gain(&machine->codec_info[HIFI_CODEC]);
	tegra30_set_dam_ifc_gain(&machine->codec_info[BT_SCO]);
	
	return 1;
}

static int tegra_voice_record_dl_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 28672; /* 0x7000 */
	return 0;
}

static int volume_get_voice_dl_record(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{

	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->call_record_rx_gain;

	return 0;
}

static int volume_put_voice_dl_record(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	machine->call_record_rx_gain = ucontrol->value.integer.value[0];

	if (machine->is_device_bt)
		tegra30_set_dam_ifc_gain_of_call_record(&machine->codec_info[BT_SCO],
							machine->call_record_rx_gain,
							machine->call_record_tx_gain);
	else
		tegra30_set_dam_ifc_gain_of_call_record(&machine->codec_info[HIFI_CODEC],
							machine->call_record_rx_gain,
							machine->call_record_tx_gain);

	return 1;
}

static int tegra_voice_record_ul_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 28672;
	return 0;
}

static int volume_get_voice_ul_record(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] = machine->call_record_tx_gain;

	return 0;
}

static int volume_put_voice_ul_record(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_max98088 *machine = snd_kcontrol_chip(kcontrol);

	machine->call_record_tx_gain = ucontrol->value.integer.value[0];

	if (machine->is_device_bt)
		tegra30_set_dam_ifc_gain_of_call_record(&machine->codec_info[BT_SCO],
							machine->call_record_rx_gain,
							machine->call_record_tx_gain);
	else
		tegra30_set_dam_ifc_gain_of_call_record(&machine->codec_info[HIFI_CODEC],
							machine->call_record_rx_gain,
							machine->call_record_tx_gain);

	return 1;
}
#endif

struct snd_kcontrol_new tegra_call_mode_control = {
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Call Mode Switch",
	.private_value = 0xffff,
	.info = tegra_call_mode_info,
	.get = tegra_call_mode_get,
	.put = tegra_call_mode_put
};

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_max98088_set_dam_cif(int dam_ifc, int srate,
			int channels, int bit_size, int src_on, int src_srate,
			int src_channels, int src_bit_size)
{
#ifndef CONFIG_MACH_X3
	tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN1, 0x1000);
#endif
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHOUT,
				srate);
	tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN1,
				srate);
#ifndef CONFIG_ARCH_TEGRA_3x_SOC
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN1,
		channels, bit_size, channels,
				32);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHOUT,
		channels, bit_size, channels,
				32);
#else
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN1,
		channels, bit_size, channels,
				bit_size);
	tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHOUT,
		channels, bit_size, channels,
				bit_size);
#endif

	if (src_on) {
#ifndef CONFIG_MACH_X3
		tegra30_dam_set_gain(dam_ifc, TEGRA30_DAM_CHIN0_SRC, 0x1000);
#endif
		tegra30_dam_set_samplerate(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_srate);
#ifndef CONFIG_ARCH_TEGRA_3x_SOC
		tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_channels, src_bit_size, 1, 32);
#else
		tegra30_dam_set_acif(dam_ifc, TEGRA30_DAM_CHIN0_SRC,
			src_channels, src_bit_size, 1, 16);
#endif
	}

	return 0;
}
#endif

static int tegra_max98088_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
#endif
	int srate, mclk, sample_size, i2s_daifmt, i2s_master;
	int err;
	int rate;

	i2s_master = pdata->i2s_param[HIFI_CODEC].is_i2s_master;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = 12000000;
		break;
	}

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF;
	i2s_daifmt |= i2s_master ? SND_SOC_DAIFMT_CBS_CFS :
				SND_SOC_DAIFMT_CBM_CFM;

	switch (pdata->i2s_param[HIFI_CODEC].i2s_mode) {
		case TEGRA_DAIFMT_I2S :
			i2s_daifmt |= SND_SOC_DAIFMT_I2S;
			break;
		case TEGRA_DAIFMT_DSP_A :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
			break;
		case TEGRA_DAIFMT_DSP_B :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_B;
			break;
		case TEGRA_DAIFMT_LEFT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_LEFT_J;
			break;
		case TEGRA_DAIFMT_RIGHT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_RIGHT_J;
			break;
		default :
			dev_err(card->dev, "Can't configure i2s format\n");
			return -EINVAL;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	rate = clk_get_rate(machine->util_data.clk_cdev1);

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, rate, SND_SOC_CLOCK_IN);

	/* ULP specific use case for 44.1kHz stream. */
	if ((!i2s_master) && (srate == 44100) &&
		machine_is_tegra_enterprise()) {
		clk_set_rate(machine->util_data.clk_cdev1, (256 * srate));
		rate = clk_get_rate(machine->util_data.clk_cdev1);
		err = snd_soc_dai_set_sysclk(codec_dai, 0, rate,
				SND_SOC_CLOCK_IN);
	}

	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
#ifdef CONFIG_MACH_X3
	if (i2s->is_dam_used)
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
#else
	if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		&& (i2s->is_dam_used))
#endif
		tegra_max98088_set_dam_cif(i2s->dam_ifc, srate,
			params_channels(params), sample_size, 0, 0, 0, 0);
#endif

	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}

static int tegra_bt_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
	struct snd_soc_card *card = rtd->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int err, srate, mclk, sample_size;
#ifndef CONFIG_MACH_X3
	int min_mclk;
#endif
	int i2s_daifmt;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	default:
		return -EINVAL;
	}

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
#ifdef CONFIG_MACH_X3
	case 24000:
#endif
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
#ifdef CONFIG_MACH_X3
		mclk = 12288000;
#endif
		return -EINVAL;
	}

#ifndef CONFIG_MACH_X3
	min_mclk = 64 * srate;
#endif

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
#ifdef CONFIG_MACH_X3
		if (!(machine->util_data.set_mclk % mclk))
#else
		if (!(machine->util_data.set_mclk % min_mclk))
#endif
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF;
	i2s_daifmt |= pdata->i2s_param[BT_SCO].is_i2s_master ?
			SND_SOC_DAIFMT_CBS_CFS : SND_SOC_DAIFMT_CBM_CFM;

	switch (pdata->i2s_param[BT_SCO].i2s_mode) {
		case TEGRA_DAIFMT_I2S :
			i2s_daifmt |= SND_SOC_DAIFMT_I2S;
			break;
		case TEGRA_DAIFMT_DSP_A :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
			break;
		case TEGRA_DAIFMT_DSP_B :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_B;
			break;
		case TEGRA_DAIFMT_LEFT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_LEFT_J;
			break;
		case TEGRA_DAIFMT_RIGHT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_RIGHT_J;
			break;
		default :
			dev_err(card->dev, "Can't configure i2s format\n");
			return -EINVAL;
	}

	err = snd_soc_dai_set_fmt(rtd->cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(rtd->codec->card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
#ifdef CONFIG_MACH_X3
	if (i2s->is_dam_used)
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
#else
	if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		&& (i2s->is_dam_used))
#endif
		tegra_max98088_set_dam_cif(i2s->dam_ifc, params_rate(params),
			params_channels(params), sample_size, 0, 0, 0, 0);
#endif

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

#ifdef CONFIG_MACH_X3
#define x3_dapm_ignore_suspend(rtd,en) \
	/* +++ Playback DAPM +++ */				\
	snd_soc_dapm_ignore_suspend(rtd, "DACL1",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "DACR1",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "DACL2",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "DACR2",	   en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "Left SPK Mixer", en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Right SPK Mixer",en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Left HP Mixer",  en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Right HP Mixer", en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Left REC Mixer", en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Right REC Mixer",en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "HP Left Out",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "HP Right Out",   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "SPK Left Out",   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "SPK Right Out",  en);	\
	snd_soc_dapm_ignore_suspend(rtd, "REC Left Out",   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "REC Right Out",  en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "HPL",		   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "HPR",		   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "SPKL",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "SPKR",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "RECL",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "RECR",	   en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "Int Spk",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Headphone Jack", en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Earpiece",	   en);	\
								\
	/* +++ Capture DAPM +++ */				\
	snd_soc_dapm_ignore_suspend(rtd, "ADCL",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "ADCR",	   en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "Left ADC Mixer", en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Right ADC Mixer",en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "INA1 Input",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "INA2 Input",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "MIC1 Input",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "MIC2 Input",	   en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "INA1",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "MIC1",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "MIC2",	   en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "MICBIAS",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "SUBMICBIAS",	   en);	\
								\
	snd_soc_dapm_ignore_suspend(rtd, "Mic Jack",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Int Mic",	   en);	\
	snd_soc_dapm_ignore_suspend(rtd, "Int Sub Mic",	   en);


static int tegra_voice_call_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	x3_dapm_ignore_suspend(&rtd->codec->dapm, 1);

	return 0;
}

static int tegra_fmradio_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(rtd->card);
	int i;

	machine->is_radio_mode = true;

	pr_debug("[X3-MAX98088] Starting FM Radio...\n");

	if (machine->is_call_mode == false) {
		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 1;

		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Left SPK Mixer", 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Right SPK Mixer",1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Left HP Mixer",	 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Right HP Mixer", 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HP Left Out",	 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HP Right Out",	 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPK Left Out",	 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPK Right Out",	 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HPL",		 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HPR",		 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPKL",		 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPKR",		 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "INB1",		 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "INB2",		 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Int Spk",	 1);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Headphone Jack", 1);		
	}

	pr_debug("[X3-MAX98088] FM Radio started up.\n");

	return 0;
}

static void tegra_fmradio_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(rtd->card);
	int i;

	pr_debug("[X3-MAX98088] Shutting down FM Radio...\n");

	if (machine->is_call_mode == false ){
		for (i = 0; i < machine->pcard->num_links; i++)
			machine->pcard->dai_link[i].ignore_suspend = 0;

		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Left SPK Mixer",  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Right SPK Mixer", 0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Left HP Mixer",	  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Right HP Mixer",  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HP Left Out",	  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HP Right Out",	  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPK Left Out",	  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPK Right Out",	  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HPL",		  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "HPR",		  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPKL",		  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "SPKR",		  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "INB1",		  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "INB2",		  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Int Spk",	  0);
		snd_soc_dapm_ignore_suspend(&rtd->codec->dapm, "Headphone Jack",  0); 	
	}
	machine->is_radio_mode = false;

	pr_debug("[X3-MAX98088] FM Radio shutdown complete.\n");

	return;
}
#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra_max98088_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(rtd->card);
	struct codec_config *codec_info;
	struct codec_config *bb_info;
	int codec_index;

	if (!i2s->is_dam_used)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*dam configuration*/
		if (!i2s->dam_ch_refcount)
			i2s->dam_ifc = tegra30_dam_allocate_controller();
		if (i2s->dam_ifc < 0)
			return i2s->dam_ifc;
		tegra30_dam_allocate_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
		i2s->dam_ch_refcount++;
		tegra30_dam_enable_clock(i2s->dam_ifc);

#ifdef CONFIG_MACH_X3
		tegra30_set_dam_ifc_gain(&machine->codec_info[HIFI_CODEC]);
		tegra30_set_dam_ifc_gain(&machine->codec_info[BT_SCO]);
#endif

		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
				(i2s->dam_ifc*2), i2s->txcif);

		/*
		*make the dam tx to i2s rx connection if this is the only client
		*using i2s for playback
		*/
		if (i2s->playback_ref_count == 1)
			tegra30_ahub_set_rx_cif_source(
				TEGRA30_AHUB_RXCIF_I2S0_RX0 + i2s->id,
				TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->dam_ifc);

		/* enable the dam*/
		tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN1);
	} else {

		i2s->is_call_mode_rec = machine->is_call_mode;

		if (!i2s->is_call_mode_rec)
			return 0;

		if (machine->is_device_bt)
			codec_index = BT_SCO;
		else
			codec_index = HIFI_CODEC;

		codec_info = &machine->codec_info[codec_index];
		bb_info = &machine->codec_info[BASEBAND];

		/* allocate a dam for voice call recording */

		i2s->call_record_dam_ifc = tegra30_dam_allocate_controller();
		if (i2s->call_record_dam_ifc < 0)
			return i2s->call_record_dam_ifc;
		tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_allocate_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN1);
		tegra30_dam_enable_clock(i2s->call_record_dam_ifc);

		/* configure the dam */
		tegra_max98088_set_dam_cif(i2s->call_record_dam_ifc,
			codec_info->rate, codec_info->channels,
			codec_info->bitsize, 1, bb_info->rate,
			bb_info->channels, bb_info->bitsize);

#ifdef CONFIG_MACH_X3
		/* Tune audio gain */
		tegra30_set_dam_ifc_gain_of_call_record(codec_info,
							machine->call_record_rx_gain,
							machine->call_record_tx_gain);
#endif

		/* setup the connections for voice call record */

		tegra30_ahub_unset_rx_cif_source(i2s->rxcif);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(i2s->call_record_dam_ifc*2),
			TEGRA30_AHUB_TXCIF_I2S0_TX0 + bb_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
			(i2s->call_record_dam_ifc*2),
			TEGRA30_AHUB_TXCIF_I2S0_TX0 + codec_info->i2s_id);
		tegra30_ahub_set_rx_cif_source(i2s->rxcif,
			TEGRA30_AHUB_TXCIF_DAM0_TX0 + i2s->call_record_dam_ifc);

		/* enable the dam*/

		tegra30_dam_enable(i2s->call_record_dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN1);
		tegra30_dam_enable(i2s->call_record_dam_ifc, TEGRA30_DAM_ENABLE,
				TEGRA30_DAM_CHIN0_SRC);
	}

	return 0;
}

static void tegra_max98088_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

	if (!i2s->is_dam_used)
		return;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* disable the dam*/
		tegra30_dam_enable(i2s->dam_ifc, TEGRA30_DAM_DISABLE,
				TEGRA30_DAM_CHIN1);

		/* disconnect the ahub connections*/
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
					(i2s->dam_ifc*2));

		/* disable the dam and free the controller */
		tegra30_dam_disable_clock(i2s->dam_ifc);
		tegra30_dam_free_channel(i2s->dam_ifc, TEGRA30_DAM_CHIN1);
		i2s->dam_ch_refcount--;
		if (!i2s->dam_ch_refcount)
			tegra30_dam_free_controller(i2s->dam_ifc);
	 } else {

		if (!i2s->is_call_mode_rec)
			return;

		i2s->is_call_mode_rec = 0;

		/* disable the dam*/
		tegra30_dam_enable(i2s->call_record_dam_ifc,
			TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN1);
		tegra30_dam_enable(i2s->call_record_dam_ifc,
			TEGRA30_DAM_DISABLE, TEGRA30_DAM_CHIN0_SRC);

		/* disconnect the ahub connections*/
		tegra30_ahub_unset_rx_cif_source(i2s->rxcif);
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX0 +
			(i2s->call_record_dam_ifc*2));
		tegra30_ahub_unset_rx_cif_source(TEGRA30_AHUB_RXCIF_DAM0_RX1 +
			(i2s->call_record_dam_ifc*2));

		/* free the dam channels and dam controller */
		tegra30_dam_disable_clock(i2s->call_record_dam_ifc);
		tegra30_dam_free_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN1);
		tegra30_dam_free_channel(i2s->call_record_dam_ifc,
			TEGRA30_DAM_CHIN0_SRC);
		tegra30_dam_free_controller(i2s->call_record_dam_ifc);
	 }

	return;
}
#endif

static int tegra_voice_call_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int srate, mclk, i2s_daifmt;
	int err, rate;

	srate = params_rate(params);
	switch (srate) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
		break;
	}

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF;
	i2s_daifmt |= pdata->i2s_param[HIFI_CODEC].is_i2s_master ?
			SND_SOC_DAIFMT_CBS_CFS : SND_SOC_DAIFMT_CBM_CFM;

	switch (pdata->i2s_param[HIFI_CODEC].i2s_mode) {
		case TEGRA_DAIFMT_I2S :
			i2s_daifmt |= SND_SOC_DAIFMT_I2S;
			break;
		case TEGRA_DAIFMT_DSP_A :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
			break;
		case TEGRA_DAIFMT_DSP_B :
			i2s_daifmt |= SND_SOC_DAIFMT_DSP_B;
			break;
		case TEGRA_DAIFMT_LEFT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_LEFT_J;
			break;
		case TEGRA_DAIFMT_RIGHT_J :
			i2s_daifmt |= SND_SOC_DAIFMT_RIGHT_J;
			break;
		default :
			dev_err(card->dev, "Can't configure i2s format\n");
			return -EINVAL;
	}

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	rate = clk_get_rate(machine->util_data.clk_cdev1);

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, rate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* codec configuration */
	machine->codec_info[HIFI_CODEC].rate = params_rate(params);
	machine->codec_info[HIFI_CODEC].channels = params_channels(params);
 #ifdef CONFIG_MACH_X3
	machine->codec_info[BASEBAND].rate = pdata->i2s_param[BASEBAND].rate;
	machine->codec_info[BASEBAND].channels = pdata->i2s_param[BASEBAND].channels;
 #endif
#endif

	machine->is_device_bt = 0;

	return 0;
}

static void tegra_voice_call_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_max98088 *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	machine->codec_info[HIFI_CODEC].rate = 0;
	machine->codec_info[HIFI_CODEC].channels = 0;
#endif

#ifdef CONFIG_MACH_X3
	x3_dapm_ignore_suspend(&rtd->codec->dapm, 0);

	/* Reconfigure DAPM for FM Radio, if FM is ON */
	if (machine->is_radio_mode == true)
		tegra_fmradio_startup(substream); 
#endif

	return;
}

static int tegra_bt_voice_call_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	int err, srate, mclk, min_mclk;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* codec configuration */
	machine->codec_info[BT_SCO].rate = params_rate(params);
	machine->codec_info[BT_SCO].channels = params_channels(params);
 #ifdef CONFIG_MACH_X3
	machine->codec_info[BASEBAND].rate = params_rate(params);
	machine->codec_info[BASEBAND].channels = params_channels(params);
 #endif
#endif

	machine->is_device_bt = 1;

	return 0;
}

static void tegra_bt_voice_call_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_max98088 *machine  =
			snd_soc_card_get_drvdata(rtd->codec->card);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	machine->codec_info[BT_SCO].rate = 0;
	machine->codec_info[BT_SCO].channels = 0;
#endif

	return;
}

static struct snd_soc_ops tegra_max98088_ops = {
	.hw_params = tegra_max98088_hw_params,
	.hw_free = tegra_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	.startup = tegra_max98088_startup,
	.shutdown = tegra_max98088_shutdown,
#endif
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_voice_call_ops = {
#ifdef CONFIG_MACH_X3
	.startup = tegra_voice_call_startup,
#endif
	.hw_params = tegra_voice_call_hw_params,
	.shutdown = tegra_voice_call_shutdown,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_bt_voice_call_ops = {
	.hw_params = tegra_bt_voice_call_hw_params,
	.shutdown = tegra_bt_voice_call_shutdown,
	.hw_free = tegra_hw_free,
};

#ifdef CONFIG_MACH_X3
static struct snd_soc_ops tegra_fm_radio_ops = {
	.startup = tegra_fmradio_startup,
	.shutdown = tegra_fmradio_shutdown,
};
#endif

static struct snd_soc_ops tegra_bt_ops = {
	.hw_params = tegra_bt_hw_params,
	.hw_free = tegra_hw_free,
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	.startup = tegra_max98088_startup,
	.shutdown = tegra_max98088_shutdown,
#endif
};

static struct snd_soc_jack tegra_max98088_hp_jack;

#ifdef CONFIG_MACH_X3
static struct snd_soc_jack_pin tegra_max98088_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_gpio tegra_max98088_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

/* These values are copied from WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static int x3_handle_hs_connect(struct notifier_block *self,
	unsigned long action, void *dev)
{
	int hs_gpio = gpio_get_value(headset_sw_data->gpio);
	int hook_gpio = gpio_get_value(headset_sw_data->hook_gpio);
	int hstype = 0;

	int state = 0;

	if (!hs_gpio) {
		if (hook_gpio)
			hstype = SND_JACK_HEADSET;
		else
			hstype = SND_JACK_HEADPHONE;
	} else {
		/*
		 * LGE X3 encounters a tedious HW bug: when disconnecting an
		 * headset/headphone from 3.5mm jack the KEY_MICMUTE event
		 * will be generated because of temporary short-circuit of
		 * the 3.5mm TX PIN. Therefore, we need to use this workaround
		 * to get sure we won't trigger false-positives.
		 */
		input_report_key(headset_sw_data->ip_dev, KEY_MICMUTE, 0); /* Avoid to get MICMUTE stuck. */
	}

	switch (hstype) {
		case SND_JACK_HEADSET:
			tegra_gpio_enable(headset_sw_data->hook_gpio);
			state |= BIT_HEADSET;
			pr_info("Connecting HEADSET.\n");
			break;
		case SND_JACK_HEADPHONE:
			pr_info("Connecting HEADPHONE (no mic!)\n");
			tegra_gpio_disable(headset_sw_data->hook_gpio);
			state |= BIT_HEADSET_NO_MIC;
			break;
		default:
			tegra_gpio_enable(headset_sw_data->hook_gpio);
			state = 0;
	}

	switch_set_state(&headset_sw_data->sdev, state);

	return NOTIFY_OK;
}

static struct notifier_block x3_headset_notifier = {
	.notifier_call = x3_handle_hs_connect,
};

#else /* (!)CONFIG_MACH_X3 */

#ifdef CONFIG_SWITCH
static struct switch_dev wired_switch_dev = {
	.name = "h2w",
};

/* These values are copied from WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static int headset_switch_notify(struct notifier_block *self,
	unsigned long action, void *dev)
{
	int state = 0;

	switch (action) {
	case SND_JACK_HEADPHONE:
		state |= BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state |= BIT_HEADSET;
		break;
	default:
		state |= BIT_NO_HEADSET;
	}

	switch_set_state(&wired_switch_dev, state);

	return NOTIFY_OK;
}

static struct notifier_block headset_switch_nb = {
	.notifier_call = headset_switch_notify,
};
#else
static struct snd_soc_jack_pin tegra_max98088_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};
#endif /* CONFIG_SWITCH */
#endif /* CONFIG_MACH_X3 */

static int tegra_max98088_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_max98088_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget tegra_max98088_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_max98088_event_int_spk),
	SND_SOC_DAPM_OUTPUT("Earpiece"),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_max98088_event_hp),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_INPUT("Int Mic"),
#ifdef CONFIG_MACH_X3
	SND_SOC_DAPM_INPUT("Int Sub Mic"),
#endif
};

static const struct snd_soc_dapm_route enterprise_audio_map[] = {
	{"Int Spk", NULL, "SPKL"},
	{"Int Spk", NULL, "SPKR"},
	{"Earpiece", NULL, "RECL"},
	{"Earpiece", NULL, "RECR"},
	{"Headphone Jack", NULL, "HPL"},
	{"Headphone Jack", NULL, "HPR"},
#ifdef CONFIG_MACH_X3
	{"INA1", NULL, "Mic Jack"},
	{"MIC2", NULL, "SUBMICBIAS"},
	{"MICBIAS", NULL, "Int Mic"},
	{"SUBMICBIAS", NULL, "Int Sub Mic"},
#endif
	{"MICBIAS", NULL, "Mic Jack"},
	{"MIC2", NULL, "MICBIAS"},
	{"MICBIAS", NULL, "Int Mic"},
	{"MIC1", NULL, "MICBIAS"},
};

static const struct snd_kcontrol_new tegra_max98088_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
#ifdef CONFIG_MACH_X3
	SOC_DAPM_PIN_SWITCH("Int Sub Mic"),

	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Voice DL Volume",
		.info = tegra_voice_dl_info,
		.get = volume_get_voice_dl,
		.put = volume_put_voice_dl,
		.private_value = 0xffff,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Voice UL Volume",
		.info = tegra_voice_ul_info,
		.get = volume_get_voice_ul,
		.put = volume_put_voice_ul,
		.private_value = 0xffff,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Multi DL Volume",
		.info = tegra_multi_dl_info,
		.get = volume_get_multi_dl,
		.put = volume_put_multi_dl,
		.private_value = 0xffff,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Call Record DL Volume",
		.info = tegra_voice_record_dl_info,
		.get = volume_get_voice_dl_record,
		.put = volume_put_voice_dl_record,
		.private_value = 0xffff,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Call Record UL Volume",
		.info = tegra_voice_record_ul_info,
		.get = volume_get_voice_ul_record,
		.put = volume_put_voice_ul_record,
		.private_value = 0xffff,
	},
#endif
};

#ifdef CONFIG_MACH_X3
static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	const char *state;

	if (switch_get_state(sdev))
		state = headset_sw_data->state_on;
	else
		state = headset_sw_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);

	return -1;
}
#endif

static int tegra_max98088_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	struct tegra30_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
#endif
	int ret;

#ifdef CONFIG_MACH_X3
	struct headset_switch_data *switch_data;
	struct input_dev *ip_dev;
#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	if (machine->codec_info[BASEBAND].i2s_id != -1)
		i2s->is_dam_used = true;
#endif

	if (machine->init_done)
		return 0;

	machine->init_done = true;

	machine->pcard = card;
	machine->bias_level = SND_SOC_BIAS_STANDBY;
	machine->clock_enabled = 1;

#ifdef CONFIG_MACH_X3
	switch_data = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio_hp_det;
	switch_data->name_on = NULL;
	switch_data->name_off = NULL;
	switch_data->state_on = NULL;
	switch_data->state_off = NULL;
	switch_data->sdev.print_state = switch_gpio_print_state;
	switch_data->hook_gpio = pdata->gpio_hook;
	switch_data->ear_mic = pdata->gpio_ear_mic;

/* X3: Use tegra_asoc_switch_register. */
	ret = tegra_asoc_switch_register(&switch_data->sdev);
//	ret = switch_dev_register(&switch_data->sdev);
	ip_dev = input_allocate_device();
	switch_data->ip_dev = ip_dev;
	set_bit(EV_SYN, switch_data->ip_dev->evbit);
	set_bit(EV_KEY, switch_data->ip_dev->evbit);
	set_bit(KEY_MICMUTE, switch_data->ip_dev->keybit); 

	switch_data->ip_dev->name = "tegra-snd-max98088";
	ret = input_register_device(switch_data->ip_dev);  
	headset_sw_data = switch_data;   
printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
printk("!!!!X3!!!! PDATA: sdevname: %s, gpio: %d, hook: %d, ear_mic: %d\n",
	pdata->name, pdata->gpio_hp_det, pdata->gpio_hook, pdata->gpio_ear_mic);
printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
#endif

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

#ifdef CONFIG_MACH_X3
	tegra_gpio_enable(pdata->gpio_int_mic_en);
#endif

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_int_mic_en)) {
		ret = gpio_request(pdata->gpio_int_mic_en, "int_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_MIC_EN;

#ifdef CONFIG_MACH_X3
		/* Disable int mic; enable signal is active-low (X3) */
		gpio_direction_output(pdata->gpio_int_mic_en, 1);
#else
		/* Disable int mic; enable signal is active-high */
		gpio_direction_output(pdata->gpio_int_mic_en, 0);
#endif
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

#ifdef CONFIG_MACH_X3
		gpio_direction_output(pdata->gpio_ext_mic_en, 1);
#else
		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 0);
#endif
	}

	ret = snd_soc_add_card_controls(card, tegra_max98088_controls,
				   ARRAY_SIZE(tegra_max98088_controls));
	if (ret < 0)
		return ret;

	snd_soc_dapm_new_controls(dapm, tegra_max98088_dapm_widgets,
					ARRAY_SIZE(tegra_max98088_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, enterprise_audio_map,
					ARRAY_SIZE(enterprise_audio_map));

#ifdef CONFIG_MACH_X3
	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_max98088_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				 &tegra_max98088_hp_jack);

		snd_soc_jack_add_pins(&tegra_max98088_hp_jack,
				      ARRAY_SIZE(tegra_max98088_hp_jack_pins),
				      tegra_max98088_hp_jack_pins);

		snd_soc_jack_add_gpios(&tegra_max98088_hp_jack,
					1,
					&tegra_max98088_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;

		snd_soc_jack_notifier_register(&tegra_max98088_hp_jack,
			&x3_headset_notifier);

		max98088_headset_detect(codec, &tegra_max98088_hp_jack,
			SND_JACK_HEADSET);
	}
#else
	ret = snd_soc_jack_new(codec, "Headset Jack", SND_JACK_HEADSET,
			&tegra_max98088_hp_jack);
	if (ret < 0)
		return ret;

#ifdef CONFIG_SWITCH
	snd_soc_jack_notifier_register(&tegra_max98088_hp_jack,
		&headset_switch_nb);
#else /*gpio based headset detection*/
	snd_soc_jack_add_pins(&tegra_max98088_hp_jack,
		ARRAY_SIZE(tegra_max98088_hp_jack_pins),
		tegra_max98088_hp_jack_pins);
#endif

	max98088_headset_detect(codec, &tegra_max98088_hp_jack,
		SND_JACK_HEADSET);
#endif /* CONFIG_MACH_X3 */

       /* Add call mode switch control */
	ret = snd_ctl_add(codec->card->snd_card,
			snd_ctl_new1(&tegra_call_mode_control, machine));
	if (ret < 0)
		return ret;

	ret = tegra_asoc_utils_register_ctls(&machine->util_data);
	if (ret < 0)
		return ret;

#ifndef CONFIG_MACH_X3
	snd_soc_dapm_nc_pin(dapm, "INA1");
	snd_soc_dapm_nc_pin(dapm, "INA2");
	snd_soc_dapm_nc_pin(dapm, "INB1");
	snd_soc_dapm_nc_pin(dapm, "INB2");
#else
	/* Enable Headset Microphone GPIO */
	ret = gpio_request(headset_sw_data->ear_mic, "ear_mic");
	if (ret)
		pr_err("Failed to request GPIO %d (ear_mic).\n",
					headset_sw_data->ear_mic);

	ret = gpio_direction_output(headset_sw_data->ear_mic, 0);
	if (ret)
		return ret;

	tegra_gpio_enable(headset_sw_data->ear_mic);
	gpio_set_value(headset_sw_data->ear_mic, 1);

	/* Enable wakeup source from MICMUTE button GPIO */
	headset_sw_data->hook_irq = gpio_to_irq(headset_sw_data->hook_gpio);
	ret = enable_irq_wake(headset_sw_data->hook_irq);
	if (ret)
		return ret;
#endif
	snd_soc_dapm_sync(dapm);

	wake_lock_init(&x3_callmodelock, WAKE_LOCK_SUSPEND, "x3-call-mode");

	return 0;
}

static struct snd_soc_dai_link tegra_max98088_dai[NUM_DAI_LINKS] = {
	[DAI_LINK_HIFI] = {
			.name = "MAX98088",
			.stream_name = "MAX98088 HIFI",
			.codec_name = "max98088.0-0010",
			.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_MACH_X3
			.cpu_dai_name = "tegra30-i2s.0",
#endif
			.codec_dai_name = "HiFi",
			.init = tegra_max98088_init,
			.ops = &tegra_max98088_ops,
		},
	[DAI_LINK_SPDIF] = {
			.name = "SPDIF",
			.stream_name = "SPDIF PCM",
			.codec_name = "spdif-dit.0",
			.platform_name = "tegra-pcm-audio",
			.cpu_dai_name = "tegra30-spdif",
			.codec_dai_name = "dit-hifi",
			.ops = &tegra_spdif_ops,
		},
	[DAI_LINK_BTSCO] = {
			.name = "BT SCO",
			.stream_name = "BT SCO PCM",
			.codec_name = "spdif-dit.1",
			.platform_name = "tegra-pcm-audio",
#ifdef CONFIG_MACH_X3
			.cpu_dai_name = "tegra30-i2s.3",
#endif
			.codec_dai_name = "dit-hifi",
			.init = tegra_max98088_init,
			.ops = &tegra_bt_ops,
		},
	[DAI_LINK_VOICE_CALL] = {
			.name = "VOICE CALL",
			.stream_name = "VOICE CALL PCM",
			.codec_name = "max98088.0-0010",
			.platform_name = "tegra-pcm-audio",
			.cpu_dai_name = "dit-hifi",
#ifdef CONFIG_MACH_X3
			.codec_dai_name = "CALL",
#else
			.codec_dai_name = "HiFi",
#endif
			.ops = &tegra_voice_call_ops,
		},
	[DAI_LINK_BT_VOICE_CALL] = {
			.name = "BT VOICE CALL",
			.stream_name = "BT VOICE CALL PCM",
			.codec_name = "spdif-dit.2",
			.platform_name = "tegra-pcm-audio",
			.cpu_dai_name = "dit-hifi",
			.codec_dai_name = "dit-hifi",
			.ops = &tegra_bt_voice_call_ops,
		},
#ifdef CONFIG_MACH_X3
	[DAI_LINK_FM_RADIO] = {
			.name = "FM RADIO",
			.stream_name = "FM RADIO ANALOG",
			.codec_name = "max98088.0-0010",
			.platform_name = "tegra-pcm-audio",
			.cpu_dai_name = "dit-hifi",
			.codec_dai_name = "FM",
			.ops = &tegra_fm_radio_ops,
		},
#endif
};

static int tegra30_soc_set_bias_level(struct snd_soc_card *card,
					struct snd_soc_dapm_context *dapm,
					enum snd_soc_bias_level level)
{
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level == SND_SOC_BIAS_OFF &&
		level != SND_SOC_BIAS_OFF && (!machine->clock_enabled)) {
		machine->clock_enabled = 1;
		tegra_asoc_utils_clk_enable(&machine->util_data);
		machine->bias_level = level;
	}

	return 0;
}

static int tegra30_soc_set_bias_level_post(struct snd_soc_card *card,
					struct snd_soc_dapm_context *dapm,
					enum snd_soc_bias_level level)
{
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level != SND_SOC_BIAS_OFF &&
		level == SND_SOC_BIAS_OFF && (machine->clock_enabled)) {
		machine->clock_enabled = 0;
		tegra_asoc_utils_clk_disable(&machine->util_data);
	}

	machine->bias_level = level;

	return 0 ;
}

static int tegra_max98088_suspend_post(struct snd_soc_card *card)
{
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
#ifdef CONFIG_MACH_X3
	struct tegra_asoc_platform_data *pdata = machine->pdata;
 
	if (machine->is_call_mode == false) {
		if (machine->gpio_requested & GPIO_EXT_MIC_EN)
			gpio_direction_output(pdata->gpio_ext_mic_en, 0);
		if (machine->gpio_requested & GPIO_INT_MIC_EN)
			gpio_direction_output(pdata->gpio_int_mic_en, 0);
	}
#else
	if (machine->clock_enabled)
		tegra_asoc_utils_clk_disable(&machine->util_data);
#endif
	return 0;

}

static int tegra_max98088_resume_pre(struct snd_soc_card *card)
{
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
#ifdef CONFIG_MACH_X3
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_direction_output(pdata->gpio_ext_mic_en, 1);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_direction_output(pdata->gpio_int_mic_en, 1);

	gpio_set_value(headset_sw_data->ear_mic, 1);
#else
	if (!machine->clock_enabled)
		tegra_asoc_utils_clk_enable(&machine->util_data);

#endif

	return 0;
}

static struct snd_soc_card snd_soc_tegra_max98088 = {
	.name = "tegra-max98088",
	.owner = THIS_MODULE,
	.dai_link = tegra_max98088_dai,
	.num_links = ARRAY_SIZE(tegra_max98088_dai),
	.set_bias_level = tegra30_soc_set_bias_level,
	.set_bias_level_post = tegra30_soc_set_bias_level_post,
	.suspend_post = tegra_max98088_suspend_post,
	.resume_pre = tegra_max98088_resume_pre,
};

static __devinit int tegra_max98088_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_max98088;
	struct tegra_max98088 *machine;
	struct tegra_asoc_platform_data *pdata;
	int ret, i;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_max98088), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_max98088 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;

#ifdef CONFIG_MACH_X3
	/* Set call record gain */
//	machine->call_record_rx_gain = 0x1000;
//	machine->call_record_tx_gain = 0x1000;
#endif

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev, card);
	if (ret)
		goto err_free_machine;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

#ifndef CONFIG_MACH_X3
#ifdef CONFIG_SWITCH
	/* Add h2w switch class support */
	ret = tegra_asoc_switch_register(&wired_switch_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "not able to register switch device\n");
		goto err_fini_utils;
	}
#endif
#endif

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	for (i = 0; i < NUM_I2S_DEVICES ; i++) {
 #ifdef CONFIG_MACH_X3
		machine->codec_info[i].dam_gain[0] = 0x1A00;
		machine->codec_info[i].dam_gain[1] = 0x1A00;
 #endif
		machine->codec_info[i].i2s_id =
			pdata->i2s_param[i].audio_port_id;
		machine->codec_info[i].bitsize =
			pdata->i2s_param[i].sample_size;
		machine->codec_info[i].is_i2smaster =
			pdata->i2s_param[i].is_i2s_master;
		machine->codec_info[i].rate =
			pdata->i2s_param[i].rate;
		machine->codec_info[i].channels =
			pdata->i2s_param[i].channels;
		machine->codec_info[i].i2s_mode =
			pdata->i2s_param[i].i2s_mode;
		machine->codec_info[i].bit_clk =
			pdata->i2s_param[i].bit_clk;
	}

 #ifndef CONFIG_MACH_X3
	tegra_max98088_dai[DAI_LINK_HIFI].cpu_dai_name =
	tegra_max98088_i2s_dai_name[machine->codec_info[HIFI_CODEC].i2s_id];

	tegra_max98088_dai[DAI_LINK_BTSCO].cpu_dai_name =
	tegra_max98088_i2s_dai_name[machine->codec_info[BT_SCO].i2s_id];
 #endif
#endif

	card->dapm.idle_bias_off = 1;
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_switch_unregister;
	}

	if (!card->instantiated) {
		dev_err(&pdev->dev, "No MAX98088 codec\n");
		goto err_unregister_card;
	}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	ret = tegra_asoc_utils_set_parent(&machine->util_data,
				pdata->i2s_param[HIFI_CODEC].is_i2s_master);
	if (ret) {
		dev_err(&pdev->dev, "tegra_asoc_utils_set_parent failed (%d)\n",
			ret);
		goto err_unregister_card;
	}
#endif

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_switch_unregister:
#ifndef CONFIG_MACH_X3
#ifdef CONFIG_SWITCH
	tegra_asoc_switch_unregister(&wired_switch_dev);
#endif
err_fini_utils:
	tegra_asoc_utils_fini(&machine->util_data);
#endif
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_max98088_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_max98088 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

#ifdef CONFIG_MACH_X3
	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_max98088_hp_jack,
					1,
					&tegra_max98088_hp_jack_gpio);
#endif

	snd_soc_unregister_card(card);

#ifdef CONFIG_MACH_X3
	tegra_asoc_switch_unregister(&headset_sw_data->sdev);
#else
 #ifdef CONFIG_SWITCH
	tegra_asoc_switch_unregister(&wired_switch_dev);
 #endif
#endif /* CONFIG_MACH_X3 */

	tegra_asoc_utils_fini(&machine->util_data);

	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_INT_MIC_EN)
		gpio_free(pdata->gpio_int_mic_en);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);

	kfree(machine);

	return 0;
}

static struct platform_driver tegra_max98088_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_max98088_driver_probe,
	.remove = __devexit_p(tegra_max98088_driver_remove),
};

static int __init tegra_max98088_modinit(void)
{
	return platform_driver_register(&tegra_max98088_driver);
}
module_init(tegra_max98088_modinit);

static void __exit tegra_max98088_modexit(void)
{
	platform_driver_unregister(&tegra_max98088_driver);
}
module_exit(tegra_max98088_modexit);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra+MAX98088 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
