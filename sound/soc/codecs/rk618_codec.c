/*
 * rk618.c  --  rk618 CODEC ALSA SoC audio driver
 *
 * Copyright 2013 Rockship
 * Author: chenjq <chenjq@rock-chips.com>
 */

#include "linux/clk.h"
#include "linux/dev_printk.h"
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/rk618.h>
#include "rk618_codec.h"


/* volume setting
 *  0: -39dB
 *  26: 0dB
 *  31: 6dB
 *  Step: 1.5dB
*/
#ifdef VIRTUAL_HPGND
#define  SPKOUT_VOLUME    24 //0~31
#define  HPOUT_VOLUME     20 //0~31
#else
#define  SPKOUT_VOLUME    24 //0~31
#define  HPOUT_VOLUME     24 //0~31
#endif

/* volume setting
 *  0: -18dB
 *  12: 0dB
 *  31: 28.5dB
 *  Step: 1.5dB
*/
#define   CAPTURE_VOL     24 //0-31

//sleep for MOSFET or SPK power amplifier chip
#define SPK_AMP_DELAY 150
#define HP_MOS_DELAY 50

//for route
#define RK618_CODEC_PLAYBACK	1
#define RK618_CODEC_CAPTURE	2
#define RK618_CODEC_INCALL	4
#define RK618_CODEC_ALL	(RK618_CODEC_PLAYBACK | RK618_CODEC_CAPTURE | RK618_CODEC_INCALL)

//for gpio
#define RK618_CODEC_SET_SPK	1
#define RK618_CODEC_SET_HP	2
#define RK618_CODEC_SET_RCV	4
#define RK618_CODEC_SET_MIC	8

struct rk618_codec_priv {
	struct snd_soc_component *component;
	struct clk *mclk;

    struct rk618 *rk618;

	unsigned int stereo_sysclk;
	unsigned int rate;

	struct gpio_desc *spk_ctl_gpio;
	struct gpio_desc *hp_ctl_gpio;
	struct gpio_desc *rcv_ctl_gpio;
	struct gpio_desc *mic_sel_gpio;

	int spk_gpio_level;
	int hp_gpio_level;
	int rcv_gpio_level;
	int mic_gpio_level;
};

static struct rk618_codec_priv *rk618_priv = NULL;
static bool is_hdmi_in = false;

static const unsigned int rk618_reg_defaults[RK618_PGAR_AGC_CTL5 + 1] = {
	[RK618_RESET] = 0x0003,
	[RK618_DAC_VOL] = 0x0046,
	[RK618_ADC_INT_CTL1] = 0x0050,
	[RK618_ADC_INT_CTL2] = 0x000e,
	[RK618_DAC_INT_CTL1] = 0x0050,
	[RK618_DAC_INT_CTL2] = 0x000e,
	[RK618_CLK_CHPUMP] = 0x0021,
	[RK618_PGA_AGC_CTL] = 0x000c,
	[RK618_PWR_ADD1] = 0x007c,
	[RK618_BST_CTL] = 0x0099,
	[RK618_DIFFIN_CTL] = 0x0024,
	[RK618_MIXINL_CTL] = 0x001f,
	[RK618_MIXINL_VOL1] = 0x0024,
	[RK618_MIXINL_VOL2] = 0x0004,
	[RK618_MIXINR_CTL] = 0x003f,
	[RK618_MIXINR_VOL1] = 0x0024,
	[RK618_MIXINR_VOL2] = 0x0024,
	[RK618_PGAL_CTL] = 0x00cc,
	[RK618_PGAR_CTL] = 0x00cc,
	[RK618_PWR_ADD2] = 0x00ff,
	[RK618_DAC_CTL] = 0x003f,
	[RK618_LINEMIX_CTL] = 0x001f,
	[RK618_MUXHP_HPMIX_CTL] = 0x003c,
	[RK618_HPMIX_CTL] = 0x00ff,
	[RK618_HPMIX_VOL1] = 0x0000,
	[RK618_HPMIX_VOL2] = 0x0000,
	[RK618_LINEOUT1_CTL] = 0x0060,
	[RK618_LINEOUT2_CTL] = 0x0060,
	[RK618_SPKL_CTL] = 0x00e0,
	[RK618_SPKR_CTL] = 0x00e0,
	[RK618_HPL_CTL] = 0x00e0,
	[RK618_HPR_CTL] = 0x00e0,
	[RK618_MICBIAS_CTL] = 0x00ff,
	[RK618_MICKEY_DET_CTL] = 0x0028,
	[RK618_PWR_ADD3] = 0x000f,
	[RK618_ADC_CTL] = 0x0036,
	[RK618_SINGNAL_ZC_CTL1] = 0x003f,
	[RK618_SINGNAL_ZC_CTL2] = 0x00ff,
	[RK618_PGAL_AGC_CTL1] = 0x0010,
	[RK618_PGAL_AGC_CTL2] = 0x0025,
	[RK618_PGAL_AGC_CTL3] = 0x0041,
	[RK618_PGAL_AGC_CTL4] = 0x002c,
	[RK618_PGAL_ASR_CTL] = 0x0000,
	[RK618_PGAL_AGC_MAX_H] = 0x0026,
	[RK618_PGAL_AGC_MAX_L] = 0x0040,
	[RK618_PGAL_AGC_MIN_H] = 0x0036,
	[RK618_PGAL_AGC_MIN_L] = 0x0020,
	[RK618_PGAL_AGC_CTL5] = 0x0038,
	[RK618_PGAR_AGC_CTL1] = 0x0010,
	[RK618_PGAR_AGC_CTL2] = 0x0025,
	[RK618_PGAR_AGC_CTL3] = 0x0041,
	[RK618_PGAR_AGC_CTL4] = 0x002c,
	[RK618_PGAR_ASR_CTL] = 0x0000,
	[RK618_PGAR_AGC_MAX_H] = 0x0026,
	[RK618_PGAR_AGC_MAX_L] = 0x0040,
	[RK618_PGAR_AGC_MIN_H] = 0x0036,
	[RK618_PGAR_AGC_MIN_L] = 0x0020,
	[RK618_PGAR_AGC_CTL5] = 0x0038,
};

/* mfd registers default list */
static struct rk618_reg_val_typ rk618_mfd_reg_defaults[] = {
	{RK618_CODEC_DIV, 0x00000000},
	{RK618_IO_CON0, (I2S1_OUTPUT_DISABLE | I2S0_OUTPUT_DISABLE | I2S1_IO_PULL_DOWN_DISABLE | I2S0_IO_PULL_DOWN_DISABLE) },
	{RK618_IO_CON1, (I2S1_IO_SCHMITT_INPUT_ENABLE | I2S0_IO_SCHMITT_INPUT_ENABLE)},
	{RK618_PCM2IS2_CON2, (0) | ((PCM_TO_I2S_MUX | APS_SEL | APS_CLR | I2S_CHANNEL_SEL) << 16)},
	{RK618_CFGMISC_CON, 0x00000000},
};
#define RK618_MFD_REG_LEN ARRAY_SIZE(rk618_mfd_reg_defaults)

static int rk618_reset(struct snd_soc_component *component)
{
	int i;

	snd_soc_component_write(component, RK618_RESET, 0xfc);
	mdelay(10);
	snd_soc_component_write(component, RK618_RESET, 0x43);
	mdelay(10);

	for (i = 0; i < RK618_MFD_REG_LEN; i++)
		snd_soc_component_write(component, rk618_mfd_reg_defaults[i].reg,
			rk618_mfd_reg_defaults[i].value);

	//close charge pump
	snd_soc_component_write(component, RK618_CLK_CHPUMP, 0x41);

	//bypass zero-crossing detection
	snd_soc_component_write(component, RK618_SINGNAL_ZC_CTL1, 0x3f);
	snd_soc_component_write(component, RK618_SINGNAL_ZC_CTL2, 0xff);

	//set ADC Power for MICBIAS
	snd_soc_component_update_bits(component, RK618_PWR_ADD1,
		RK618_ADC_PWRD, 0);

	return 0;
}

bool get_hdmi_state(void)
{
	return is_hdmi_in;
}

void rk618_codec_set_spk(bool on)
{
	struct rk618_codec_priv *rk618 = rk618_priv;
	struct snd_soc_component *component;

	dev_dbg(rk618_priv->component->dev, "%s : %s\n", __func__, on ? "enable spk" : "disable spk");

	if (!rk618 || !rk618->component) {
		printk("%s : rk618_priv or rk618_priv->codec is NULL\n", __func__);
		return;
	}

	component = rk618->component;

	if (on) {
        mutex_lock(&component->io_mutex);
        snd_soc_dapm_enable_pin(&component->dapm, "Headphone Jack");
        snd_soc_dapm_enable_pin(&component->dapm, "Ext Spk");
        snd_soc_dapm_sync(&component->dapm);
        mutex_unlock(&component->io_mutex);
	} else {
		gpiod_direction_output(rk618->spk_ctl_gpio, 0);
		gpiod_direction_output(rk618->hp_ctl_gpio, 0);

        mutex_lock(&component->io_mutex);
        snd_soc_dapm_disable_pin(&component->dapm, "Headphone Jack");
        snd_soc_dapm_disable_pin(&component->dapm, "Ext Spk");
        snd_soc_dapm_sync(&component->dapm);
        mutex_unlock(&component->io_mutex);
	}

	is_hdmi_in = on ? 0 : 1;
}
EXPORT_SYMBOL_GPL(rk618_codec_set_spk);

static const DECLARE_TLV_DB_SCALE(out_vol_tlv, -3900, 150, 0);
static const DECLARE_TLV_DB_SCALE(pga_vol_tlv, -1800, 150, 0);
static const DECLARE_TLV_DB_SCALE(bst_vol_tlv, 0, 2000, 0);
static const DECLARE_TLV_DB_SCALE(mix_vol_tlv, -1200, 300, 0);
static const DECLARE_TLV_DB_SCALE(pga_agc_max_vol_tlv, -1350, 600, 0);
static const DECLARE_TLV_DB_SCALE(pga_agc_min_vol_tlv, -1800, 600, 0);

static const char *rk618_input_mode[] = {"Differential", "Single-Ended"}; 

static const char *rk618_micbias_ratio[] = {"1.0 Vref", "1.1 Vref",
		"1.2 Vref", "1.3 Vref", "1.4 Vref", "1.5 Vref", "1.6 Vref", "1.7 Vref",};

static const char *rk618_dis_en_sel[] = {"Disable", "Enable"};

static const char *rk618_mickey_range[] = {"100uA", "300uA",
		"500uA", "700uA", "900uA", "1100uA", "1300uA", "1500uA"};

static const char *rk618_pga_gain_control[] = {"Normal", "AGC"};

static const char *rk618_pga_agc_way[] = {"Normal", "Jack"};

static const char *rk618_pga_agc_hold_time[] = {"0ms", "2ms",
		"4ms", "8ms", "16ms", "32ms", "64ms", "128ms", "256ms", "512ms", "1s"};

static const char *rk618_pga_agc_ramp_up_time[] = {"500us", "1ms", "2ms",
		"4ms", "8ms", "16ms", "32ms", "64ms", "128ms", "256ms", "512ms"};

static const char *rk618_pga_agc_ramp_down_time[] = {"Normal:125us Jack:32us",
		"Normal:250us Jack:64us", "Normal:500us Jack:125us", "Normal:1ms Jack:250us",
		"Normal:2ms Jack:500us", "Normal:4ms Jack:1ms", "Normal:8ms Jack:2ms",
		"Normal:16ms Jack:4ms", "Normal:32ms Jack:8ms", "Normal:64ms Jack:16ms",
		"Normal:128ms Jack:32ms"};

static const char *rk618_pga_agc_mode[] = {"Normal", "Limiter"};

static const char *rk618_pga_agc_recovery_mode[] = {"Right Now", "After AGC to Limiter"};

static const char *rk618_pga_agc_noise_gate_threhold[] = {"-39dB", "-45dB", "-51dB",
		"-57dB", "-63dB", "-69dB", "-75dB", "-81dB"};

static const char *rk618_pga_agc_update_gain[] = {"Right Now", "After 1st Zero Cross"};

static const char *rk618_pga_agc_approximate_sample_rate[] = {"48KHz", "32KHz",
		"24KHz", "16KHz", "12KHz", "8KHz"};

static const char *rk618_gpio_sel[] = {"Low", "High"};

static const struct soc_enum rk618_bst_enum[] = {
SOC_ENUM_SINGLE(RK618_BST_CTL, RK618_BSTL_MODE_SFT, 2, rk618_input_mode),
SOC_ENUM_SINGLE(RK618_BST_CTL, RK618_BSTR_MODE_SFT, 2, rk618_input_mode),
};

static const struct soc_enum rk618_diffin_enum =
	SOC_ENUM_SINGLE(RK618_DIFFIN_CTL, RK618_DIFFIN_MODE_SFT, 2, rk618_input_mode);

static const struct soc_enum rk618_micbias_enum[] = {
SOC_ENUM_SINGLE(RK618_MICBIAS_CTL, RK618_MICBIAS1_V_SFT, 8, rk618_micbias_ratio),
SOC_ENUM_SINGLE(RK618_MICBIAS_CTL, RK618_MICBIAS2_V_SFT, 8, rk618_micbias_ratio),
};

static const struct soc_enum rk618_mickey_enum[] = {
SOC_ENUM_SINGLE(RK618_MICKEY_DET_CTL, RK618_MK1_DET_SFT, 2, rk618_dis_en_sel),
SOC_ENUM_SINGLE(RK618_MICKEY_DET_CTL, RK618_MK2_DET_SFT, 2, rk618_dis_en_sel),
SOC_ENUM_SINGLE(RK618_MICKEY_DET_CTL, RK618_MK1_DET_I_SFT, 8, rk618_mickey_range),
SOC_ENUM_SINGLE(RK618_MICKEY_DET_CTL, RK618_MK2_DET_I_SFT, 8, rk618_mickey_range),
};

static const struct soc_enum rk618_agcl_enum[] = {
SOC_ENUM_SINGLE(RK618_PGA_AGC_CTL, RK618_PGAL_AGC_EN_SFT, 2, rk618_pga_gain_control),/*0*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL1, RK618_PGA_AGC_WAY_SFT, 2, rk618_pga_agc_way),/*1*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL1, RK618_PGA_AGC_HOLD_T_SFT, 11, rk618_pga_agc_hold_time),/*2*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL2, RK618_PGA_AGC_GRU_T_SFT, 11, rk618_pga_agc_ramp_up_time),/*3*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL2, RK618_PGA_AGC_GRD_T_SFT, 11, rk618_pga_agc_ramp_down_time),/*4*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL3, RK618_PGA_AGC_MODE_SFT, 2, rk618_pga_agc_mode),/*5*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL3, RK618_PGA_AGC_ZO_SFT, 2, rk618_dis_en_sel),/*6*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL3, RK618_PGA_AGC_REC_MODE_SFT, 2, rk618_pga_agc_recovery_mode),/*7*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL3, RK618_PGA_AGC_FAST_D_SFT, 2, rk618_dis_en_sel),/*8*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL3, RK618_PGA_AGC_NG_SFT, 2, rk618_dis_en_sel),/*9*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL3, RK618_PGA_AGC_NG_THR_SFT, 8, rk618_pga_agc_noise_gate_threhold),/*10*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL4, RK618_PGA_AGC_ZO_MODE_SFT, 2, rk618_pga_agc_update_gain),/*11*/
SOC_ENUM_SINGLE(RK618_PGAL_ASR_CTL, RK618_PGA_SLOW_CLK_SFT, 2, rk618_dis_en_sel),/*12*/
SOC_ENUM_SINGLE(RK618_PGAL_ASR_CTL, RK618_PGA_ASR_SFT, 6, rk618_pga_agc_approximate_sample_rate),/*13*/
SOC_ENUM_SINGLE(RK618_PGAL_AGC_CTL5, RK618_PGA_AGC_SFT, 2, rk618_dis_en_sel),/*14*/
};

static const struct soc_enum rk618_agcr_enum[] = {
SOC_ENUM_SINGLE(RK618_PGA_AGC_CTL, RK618_PGAR_AGC_EN_SFT, 2, rk618_pga_gain_control),/*0*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL1, RK618_PGA_AGC_WAY_SFT, 2, rk618_pga_agc_way),/*1*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL1, RK618_PGA_AGC_HOLD_T_SFT, 11, rk618_pga_agc_hold_time),/*2*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL2, RK618_PGA_AGC_GRU_T_SFT, 11, rk618_pga_agc_ramp_up_time),/*3*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL2, RK618_PGA_AGC_GRD_T_SFT, 11, rk618_pga_agc_ramp_down_time),/*4*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL3, RK618_PGA_AGC_MODE_SFT, 2, rk618_pga_agc_mode),/*5*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL3, RK618_PGA_AGC_ZO_SFT, 2, rk618_dis_en_sel),/*6*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL3, RK618_PGA_AGC_REC_MODE_SFT, 2, rk618_pga_agc_recovery_mode),/*7*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL3, RK618_PGA_AGC_FAST_D_SFT, 2, rk618_dis_en_sel),/*8*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL3, RK618_PGA_AGC_NG_SFT, 2, rk618_dis_en_sel),/*9*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL3, RK618_PGA_AGC_NG_THR_SFT, 8, rk618_pga_agc_noise_gate_threhold),/*10*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL4, RK618_PGA_AGC_ZO_MODE_SFT, 2, rk618_pga_agc_update_gain),/*11*/
SOC_ENUM_SINGLE(RK618_PGAR_ASR_CTL, RK618_PGA_SLOW_CLK_SFT, 2, rk618_dis_en_sel),/*12*/
SOC_ENUM_SINGLE(RK618_PGAR_ASR_CTL, RK618_PGA_ASR_SFT, 6, rk618_pga_agc_approximate_sample_rate),/*13*/
SOC_ENUM_SINGLE(RK618_PGAR_AGC_CTL5, RK618_PGA_AGC_SFT, 2, rk618_dis_en_sel),/*14*/
};

static const struct soc_enum rk618_loop_enum =
	SOC_ENUM_SINGLE(RK618_CFGMISC_CON, AD_DA_LOOP_SFT, 2, rk618_dis_en_sel);

static const struct soc_enum rk618_gpio_enum[] = {
	SOC_ENUM_SINGLE(RK618_CODEC_SET_SPK, 0, 2, rk618_gpio_sel),
	SOC_ENUM_SINGLE(RK618_CODEC_SET_HP, 0, 2, rk618_gpio_sel),
	SOC_ENUM_SINGLE(RK618_CODEC_SET_RCV, 0, 2, rk618_gpio_sel),
	SOC_ENUM_SINGLE(RK618_CODEC_SET_MIC, 0, 2, rk618_gpio_sel),
};

int snd_soc_put_pgal_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	unsigned int val;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;

	val = (ucontrol->value.integer.value[0] & mask);

	//set for capture pop noise
	if (val) {
		snd_soc_component_update_bits(component, RK618_PGA_AGC_CTL, 0x0f, 0x09);
	}

	return snd_soc_put_volsw(kcontrol, ucontrol);
}

//for setting volume pop noise, turn volume step up/down.
int snd_soc_put_step_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err = 0;
	unsigned int val, val2, val_mask, old_l, old_r, old_reg_l, old_reg_r, step = 1;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	old_reg_l = snd_soc_component_read(component, reg);
	old_l = (old_reg_l & val_mask) >> shift;

	old_reg_r = snd_soc_component_read(component, reg);
	old_r = (old_reg_r & val_mask) >> shift;

	old_reg_l &= ~mask;
	old_reg_r &= ~mask;

	while (old_l != val || old_r != val2) {
		if (old_l != val) {
			if (old_l > val) {
				old_l -= step;
				if (old_l < val)
					old_l = val;
			} else {
				old_l += step;
				if (old_l > val)
					old_l = val;
			}

			if (invert) {
				old_l = max - old_l;
			}

			old_l = old_l << shift;

			err = snd_soc_component_write(component, reg, old_reg_l | old_l);
			if (err < 0)
				return err;
		}
		if (old_r != val2) {
			if (old_r > val2) {
				old_r -= step;
				if (old_r < val2)
					old_r = val2;
			} else {
				old_r += step;
				if (old_r > val2)
					old_r = val2;
			}

			if (invert) {
				old_r = max - old_r;
			}

			old_r = old_r << shift;

			err = snd_soc_component_write(component, reg2, old_reg_r | old_r);
			if (err < 0)
				return err;
		}
	}
	return err;
}

int snd_soc_get_gpio_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct rk618_codec_priv *rk618 = rk618_priv;

	if (!rk618) {
		printk("%s : rk618_priv is NULL\n", __func__);
		return -EINVAL;
	}

	switch(e->reg) {
	case RK618_CODEC_SET_SPK:
		ucontrol->value.enumerated.item[0] = rk618->spk_gpio_level;
		break;
	case RK618_CODEC_SET_HP:
		ucontrol->value.enumerated.item[0] = rk618->hp_gpio_level;
		break;
	case RK618_CODEC_SET_RCV:
		ucontrol->value.enumerated.item[0] = rk618->rcv_gpio_level;
		break;
	case RK618_CODEC_SET_MIC:
		ucontrol->value.enumerated.item[0] = rk618->mic_gpio_level;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int snd_soc_put_gpio_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct rk618_codec_priv *rk618 = rk618_priv;

	if (!rk618) {
		printk("%s : rk618_priv is NULL\n", __func__);
		return -EINVAL;
	}

	if (ucontrol->value.enumerated.item[0] > e->items - 1)
		return -EINVAL;

	//The gpio of SPK HP and RCV will be setting in digital_mute for pop noise.
	switch(e->reg) {
	case RK618_CODEC_SET_SPK:
		rk618->spk_gpio_level = ucontrol->value.enumerated.item[0];
		break;
	case RK618_CODEC_SET_HP:
		rk618->hp_gpio_level = ucontrol->value.enumerated.item[0];
		break;
	case RK618_CODEC_SET_RCV:
		rk618->rcv_gpio_level = ucontrol->value.enumerated.item[0];
		break;
	case RK618_CODEC_SET_MIC:
		rk618->mic_gpio_level = ucontrol->value.enumerated.item[0];
		return gpiod_direction_output(rk618->mic_sel_gpio, ucontrol->value.enumerated.item[0]);
	default:
		return -EINVAL;
	}

	return 0;
}

#define SOC_DOUBLE_R_STEP_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = snd_soc_put_step_volsw_2r, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		.max = xmax, .platform_max = xmax, .invert = xinvert} }

#define SOC_GPIO_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_gpio_enum_double, .put = snd_soc_put_gpio_enum_double, \
	.private_value = (unsigned long)&xenum }

static const struct snd_kcontrol_new rk618_snd_controls[] = {

	//add for incall volume setting
	SOC_DOUBLE_R_STEP_TLV("Speaker Playback Volume", RK618_SPKL_CTL,
			RK618_SPKR_CTL, RK618_VOL_SFT, SPKOUT_VOLUME, 0, out_vol_tlv),
	SOC_DOUBLE_R_STEP_TLV("Headphone Playback Volume", RK618_HPL_CTL,
			RK618_HPR_CTL, RK618_VOL_SFT, HPOUT_VOLUME, 0, out_vol_tlv),
	SOC_DOUBLE_R_STEP_TLV("Earpiece Playback Volume", RK618_SPKL_CTL,
			RK618_SPKR_CTL, RK618_VOL_SFT, SPKOUT_VOLUME, 0, out_vol_tlv),

	SOC_DOUBLE_R("Speaker Playback Switch", RK618_SPKL_CTL,
		RK618_SPKR_CTL, RK618_MUTE_SFT, 1, 1),

	SOC_DOUBLE_R("Headphone Playback Switch", RK618_HPL_CTL,
		RK618_HPR_CTL, RK618_MUTE_SFT, 1, 1),

	SOC_DOUBLE_R("Earpiece Playback Switch", RK618_HPL_CTL,
		RK618_HPR_CTL, RK618_MUTE_SFT, 1, 1),

	SOC_SINGLE_TLV("LINEOUT1 Playback Volume", RK618_LINEOUT1_CTL,
		RK618_LINEOUT_VOL_SFT, 31, 0, out_vol_tlv),
	SOC_SINGLE("LINEOUT1 Playback Switch", RK618_LINEOUT1_CTL,
		RK618_LINEOUT_MUTE_SFT, 1, 1),
	SOC_SINGLE_TLV("LINEOUT2 Playback Volume", RK618_LINEOUT2_CTL,
		RK618_LINEOUT_VOL_SFT, 31, 0, out_vol_tlv),
	SOC_SINGLE("LINEOUT2 Playback Switch", RK618_LINEOUT2_CTL,
		RK618_LINEOUT_MUTE_SFT, 1, 1),

	SOC_SINGLE_TLV("PGAL Capture Volume", RK618_PGAL_CTL,
		RK618_PGA_VOL_SFT, 31, 0, pga_vol_tlv),//0x0a bit 5 is 0
	{
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = "PGAL Capture Switch", \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_pgal_volsw, \
	.private_value =  SOC_SINGLE_VALUE(RK618_PGAL_CTL, RK618_PGA_MUTE_SFT, 1, 1, 0)
	},
	SOC_SINGLE_TLV("PGAR Capture Volume", RK618_PGAR_CTL,
		RK618_PGA_VOL_SFT, 31, 0, pga_vol_tlv),//0x0a bit 4 is 0
	SOC_SINGLE("PGAR Capture Switch", RK618_PGAR_CTL,
		RK618_PGA_MUTE_SFT, 1, 1),

	SOC_SINGLE_TLV("DIFFIN Capture Volume", RK618_DIFFIN_CTL,
		RK618_DIFFIN_GAIN_SFT, 1, 0, bst_vol_tlv),
	SOC_SINGLE("DIFFIN Capture Switch", RK618_DIFFIN_CTL,
		RK618_DIFFIN_MUTE_SFT, 1, 1),

	//Add for set capture mute
	SOC_SINGLE_TLV("Main Mic Capture Volume", RK618_BST_CTL,
		RK618_BSTL_GAIN_SFT, 1, 0, bst_vol_tlv),
	SOC_SINGLE("Main Mic Capture Switch", RK618_BST_CTL,
		RK618_BSTL_MUTE_SFT, 1, 1),
	SOC_SINGLE_TLV("Headset Mic Capture Volume", RK618_BST_CTL,
		RK618_BSTR_GAIN_SFT, 1, 0, bst_vol_tlv),
	SOC_SINGLE("Headset Mic Capture Switch", RK618_BST_CTL,
		RK618_BSTR_MUTE_SFT, 1, 1),

	SOC_ENUM("BST_L Mode",  rk618_bst_enum[0]),
	SOC_ENUM("BST_R Mode",  rk618_bst_enum[1]),
	SOC_ENUM("DIFFIN Mode",  rk618_diffin_enum),

	SOC_SINGLE_TLV("MUXMIC to MIXINL Volume", RK618_MIXINL_VOL1,
		RK618_MIL_F_MUX_VOL_SFT, 7, 0, mix_vol_tlv),
	SOC_SINGLE_TLV("IN1P to MIXINL Volume", RK618_MIXINL_VOL1,
		RK618_MIL_F_IN1P_VOL_SFT, 7, 0, mix_vol_tlv),
	SOC_SINGLE_TLV("IN3L to MIXINL Volume", RK618_MIXINL_VOL2,
		RK618_MIL_F_IN3L_VOL_SFT, 7, 0, mix_vol_tlv),

	SOC_SINGLE_TLV("MIXINR MUX to MIXINR Volume", RK618_MIXINR_VOL1,
		RK618_MIR_F_MIRM_VOL_SFT, 7, 0, mix_vol_tlv),
	SOC_SINGLE_TLV("IN3R to MIXINR Volume", RK618_MIXINR_VOL1,
		RK618_MIR_F_IN3R_VOL_SFT, 7, 0, mix_vol_tlv),
	SOC_SINGLE_TLV("MIC2N to MIXINR Volume", RK618_MIXINR_VOL2,
		RK618_MIR_F_MIC2N_VOL_SFT, 7, 0, mix_vol_tlv),
	SOC_SINGLE_TLV("IN1P to MIXINR Volume", RK618_MIXINR_VOL2,
		RK618_MIR_F_IN1P_VOL_SFT, 7, 0, mix_vol_tlv),

	SOC_SINGLE("MIXINL Switch", RK618_MIXINL_CTL,
		RK618_MIL_MUTE_SFT, 1, 1),
	SOC_SINGLE("MIXINR Switch", RK618_MIXINR_CTL,
		RK618_MIR_MUTE_SFT, 1, 1),

	SOC_SINGLE_TLV("IN1P to HPMIXL Volume", RK618_HPMIX_VOL1,
		RK618_HML_F_IN1P_VOL_SFT, 7, 0, mix_vol_tlv),
	SOC_SINGLE_TLV("HPMIX MUX to HPMIXL Volume", RK618_HPMIX_VOL2,
		RK618_HML_F_HMM_VOL_SFT, 7, 0, mix_vol_tlv),
	SOC_SINGLE_TLV("HPMIX MUX to HPMIXR Volume", RK618_HPMIX_VOL2,
		RK618_HMR_F_HMM_VOL_SFT, 7, 0, mix_vol_tlv),

	SOC_ENUM("Micbias1 Voltage",  rk618_micbias_enum[0]),
	SOC_ENUM("Micbias2 Voltage",  rk618_micbias_enum[1]),

	SOC_ENUM("MIC1 Key Detection Enable",  rk618_mickey_enum[0]),
	SOC_ENUM("MIC2 Key Detection Enable",  rk618_mickey_enum[1]),
	SOC_ENUM("MIC1 Key Range",  rk618_mickey_enum[2]),
	SOC_ENUM("MIC2 Key Range",  rk618_mickey_enum[3]),

	SOC_ENUM("PGAL Gain Control",  rk618_agcl_enum[0]),
	SOC_ENUM("PGAL AGC Way",  rk618_agcl_enum[1]),
	SOC_ENUM("PGAL AGC Hold Time",  rk618_agcl_enum[2]),
	SOC_ENUM("PGAL AGC Ramp Up Time",  rk618_agcl_enum[3]),
	SOC_ENUM("PGAL AGC Ramp Down Time",  rk618_agcl_enum[4]),
	SOC_ENUM("PGAL AGC Mode",  rk618_agcl_enum[5]),
	SOC_ENUM("PGAL AGC Gain Update Zero Enable",  rk618_agcl_enum[6]),
	SOC_ENUM("PGAL AGC Gain Recovery LPGA VOL",  rk618_agcl_enum[7]),
	SOC_ENUM("PGAL AGC Fast Decrement Enable",  rk618_agcl_enum[8]),
	SOC_ENUM("PGAL AGC Noise Gate Enable",  rk618_agcl_enum[9]),
	SOC_ENUM("PGAL AGC Noise Gate Threhold",  rk618_agcl_enum[10]),
	SOC_ENUM("PGAL AGC Upate Gain",  rk618_agcl_enum[11]),
	SOC_ENUM("PGAL AGC Slow Clock Enable",  rk618_agcl_enum[12]),
	SOC_ENUM("PGAL AGC Approximate Sample Rate",  rk618_agcl_enum[13]),
	SOC_ENUM("PGAL AGC Enable",  rk618_agcl_enum[14]),

	SOC_SINGLE_TLV("PGAL AGC Volume", RK618_PGAL_AGC_CTL4,
		RK618_PGA_AGC_VOL_SFT, 31, 0, pga_vol_tlv),//AGC disable and 0x0a bit 5 is 1

	SOC_SINGLE("PGAL AGC Max Level High 8 Bits", RK618_PGAL_AGC_MAX_H,
		0, 255, 0),
	SOC_SINGLE("PGAL AGC Max Level Low 8 Bits", RK618_PGAL_AGC_MAX_L,
		0, 255, 0),
	SOC_SINGLE("PGAL AGC Min Level High 8 Bits", RK618_PGAL_AGC_MIN_H,
		0, 255, 0),
	SOC_SINGLE("PGAL AGC Min Level Low 8 Bits", RK618_PGAL_AGC_MIN_L,
		0, 255, 0),

	SOC_SINGLE_TLV("PGAL AGC Max Gain", RK618_PGAL_AGC_CTL5,
		RK618_PGA_AGC_MAX_G_SFT, 7, 0, pga_agc_max_vol_tlv),//AGC enable and 0x0a bit 5 is 1
	SOC_SINGLE_TLV("PGAL AGC Min Gain", RK618_PGAL_AGC_CTL5,
		RK618_PGA_AGC_MIN_G_SFT, 7, 0, pga_agc_min_vol_tlv),//AGC enable and 0x0a bit 5 is 1

	SOC_ENUM("PGAR Gain Control",  rk618_agcr_enum[0]),
	SOC_ENUM("PGAR AGC Way",  rk618_agcr_enum[1]),
	SOC_ENUM("PGAR AGC Hold Time",  rk618_agcr_enum[2]),
	SOC_ENUM("PGAR AGC Ramp Up Time",  rk618_agcr_enum[3]),
	SOC_ENUM("PGAR AGC Ramp Down Time",  rk618_agcr_enum[4]),
	SOC_ENUM("PGAR AGC Mode",  rk618_agcr_enum[5]),
	SOC_ENUM("PGAR AGC Gain Update Zero Enable",  rk618_agcr_enum[6]),
	SOC_ENUM("PGAR AGC Gain Recovery LPGA VOL",  rk618_agcr_enum[7]),
	SOC_ENUM("PGAR AGC Fast Decrement Enable",  rk618_agcr_enum[8]),
	SOC_ENUM("PGAR AGC Noise Gate Enable",  rk618_agcr_enum[9]),
	SOC_ENUM("PGAR AGC Noise Gate Threhold",  rk618_agcr_enum[10]),
	SOC_ENUM("PGAR AGC Upate Gain",  rk618_agcr_enum[11]),
	SOC_ENUM("PGAR AGC Slow Clock Enable",  rk618_agcr_enum[12]),
	SOC_ENUM("PGAR AGC Approximate Sample Rate",  rk618_agcr_enum[13]),
	SOC_ENUM("PGAR AGC Enable",  rk618_agcr_enum[14]),

	SOC_SINGLE_TLV("PGAR AGC Volume", RK618_PGAR_AGC_CTL4,
		RK618_PGA_AGC_VOL_SFT, 31, 0, pga_vol_tlv),//AGC disable and 0x0a bit 4 is 1

	SOC_SINGLE("PGAR AGC Max Level High 8 Bits", RK618_PGAR_AGC_MAX_H,
		0, 255, 0),
	SOC_SINGLE("PGAR AGC Max Level Low 8 Bits", RK618_PGAR_AGC_MAX_L,
		0, 255, 0),
	SOC_SINGLE("PGAR AGC Min Level High 8 Bits", RK618_PGAR_AGC_MIN_H,
		0, 255, 0),
	SOC_SINGLE("PGAR AGC Min Level Low 8 Bits", RK618_PGAR_AGC_MIN_L,
		0, 255, 0),

	SOC_SINGLE_TLV("PGAR AGC Max Gain", RK618_PGAR_AGC_CTL5,
		RK618_PGA_AGC_MAX_G_SFT, 7, 0, pga_agc_max_vol_tlv),//AGC enable and 0x06 bit 4 is 1
	SOC_SINGLE_TLV("PGAR AGC Min Gain", RK618_PGAR_AGC_CTL5,
		RK618_PGA_AGC_MIN_G_SFT, 7, 0, pga_agc_min_vol_tlv),//AGC enable and 0x06 bit 4 is 1

	SOC_ENUM("I2S Loop Enable",  rk618_loop_enum),

	SOC_GPIO_ENUM("SPK GPIO Control",  rk618_gpio_enum[0]),
	SOC_GPIO_ENUM("HP GPIO Control",  rk618_gpio_enum[1]),
	SOC_GPIO_ENUM("RCV GPIO Control",  rk618_gpio_enum[2]),
	SOC_GPIO_ENUM("MIC GPIO Control",  rk618_gpio_enum[3]),
};

//For tiny alsa playback/capture/voice call path
static const char *rk618_playback_path_mode[] = {"OFF", "RCV", "SPK", "HP", "HP_NO_MIC", "BT", "SPK_HP", //0-6
		"RING_SPK", "RING_HP", "RING_HP_NO_MIC", "RING_SPK_HP"};//7-10

static const char *rk618_capture_path_mode[] = {"MIC OFF", "Main Mic", "Hands Free Mic", "BT Sco Mic"};

static const char *rk618_call_path_mode[] = {"OFF", "RCV", "SPK", "HP", "HP_NO_MIC", "BT"};//0-5

static const char *rk618_modem_input_mode[] = {"OFF", "ON"};

static SOC_ENUM_SINGLE_DECL(rk618_playback_path_type, 0, 0, rk618_playback_path_mode);

static SOC_ENUM_SINGLE_DECL(rk618_capture_path_type, 0, 0, rk618_capture_path_mode);

static SOC_ENUM_SINGLE_DECL(rk618_call_path_type, 0, 0, rk618_call_path_mode);

static SOC_ENUM_SINGLE_DECL(rk618_modem_input_type, 0, 0, rk618_modem_input_mode);

static int rk618_dacl_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACL_INIT_MASK, 0);
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACL_PWRD | RK618_DACL_CLK_PWRD |
			RK618_DACL_INIT_MASK, 0);
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACL_INIT_MASK, RK618_DACL_INIT_WORK);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACL_PWRD | RK618_DACL_CLK_PWRD |
			RK618_DACL_INIT_MASK,
			RK618_DACL_PWRD | RK618_DACL_CLK_PWRD |
			RK618_DACL_INIT_WORK);
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACL_INIT_MASK, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rk618_dacr_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACR_INIT_MASK, 0);
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACR_PWRD | RK618_DACR_CLK_PWRD |
			RK618_DACR_INIT_MASK, 0);
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACR_INIT_MASK, RK618_DACR_INIT_WORK);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACR_PWRD | RK618_DACR_CLK_PWRD |
			RK618_DACR_INIT_MASK,
			RK618_DACR_PWRD | RK618_DACR_CLK_PWRD |
			RK618_DACR_INIT_WORK);
		snd_soc_component_update_bits(component, RK618_DAC_CTL,
			RK618_DACR_INIT_MASK, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rk618_adcl_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK618_ADC_CTL,
			RK618_ADCL_CLK_PWRD | RK618_ADCL_PWRD, 0);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK618_ADC_CTL,
			RK618_ADCL_CLK_PWRD | RK618_ADCL_PWRD,
			RK618_ADCL_CLK_PWRD | RK618_ADCL_PWRD);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rk618_adcr_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK618_ADC_CTL,
			RK618_ADCR_CLK_PWRD | RK618_ADCR_PWRD, 0);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK618_ADC_CTL,
			RK618_ADCR_CLK_PWRD | RK618_ADCR_PWRD,
			RK618_ADCR_CLK_PWRD | RK618_ADCR_PWRD);
		break;

	default:
		return 0;
	}

	return 0;
}

/* Mixin */
static const struct snd_kcontrol_new rk618_mixinl[] = {
	SOC_DAPM_SINGLE("IN3L Switch", RK618_MIXINL_CTL,
				RK618_MIL_F_IN3L_SFT, 1, 1),
	SOC_DAPM_SINGLE("IN1P Switch", RK618_MIXINL_CTL,
				RK618_MIL_F_IN1P_SFT, 1, 1),
	SOC_DAPM_SINGLE("MUXMIC Switch", RK618_MIXINL_CTL,
				RK618_MIL_F_MUX_SFT, 1, 1),
};

static const struct snd_kcontrol_new rk618_mixinr[] = {
	SOC_DAPM_SINGLE("MIC2N Switch", RK618_MIXINR_CTL,
				RK618_MIR_F_MIC2N_SFT, 1, 1),
	SOC_DAPM_SINGLE("IN1P Switch", RK618_MIXINR_CTL,
				RK618_MIR_F_IN1P_SFT, 1, 1),
	SOC_DAPM_SINGLE("IN3R Switch", RK618_MIXINR_CTL,
				RK618_MIR_F_IN3R_SFT, 1, 1),
	SOC_DAPM_SINGLE("MIXINR Mux Switch", RK618_MIXINR_CTL,
				RK618_MIR_F_MIRM_SFT, 1, 1),
};

/* Linemix */
static const struct snd_kcontrol_new rk618_linemix[] = {
	SOC_DAPM_SINGLE("PGAR Switch", RK618_LINEMIX_CTL,
				RK618_LM_F_PGAR_SFT, 1, 1),
	SOC_DAPM_SINGLE("PGAL Switch", RK618_LINEMIX_CTL,
				RK618_LM_F_PGAL_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACR Switch", RK618_LINEMIX_CTL,
				RK618_LM_F_DACR_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACL Switch", RK618_LINEMIX_CTL,
				RK618_LM_F_DACL_SFT, 1, 1),
};

/* HPmix */
static const struct snd_kcontrol_new rk618_hpmixl[] = {
	SOC_DAPM_SINGLE("HPMix Mux Switch", RK618_HPMIX_CTL,
				RK618_HML_F_HMM_SFT, 1, 1),
	SOC_DAPM_SINGLE("IN1P Switch", RK618_HPMIX_CTL,
				RK618_HML_F_IN1P_SFT, 1, 1),
	SOC_DAPM_SINGLE("PGAL Switch", RK618_HPMIX_CTL,
				RK618_HML_F_PGAL_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACL Switch", RK618_HPMIX_CTL,
				RK618_HML_F_DACL_SFT, 1, 1),
};

static const struct snd_kcontrol_new rk618_hpmixr[] = {
	SOC_DAPM_SINGLE("HPMix Mux Switch", RK618_HPMIX_CTL,
				RK618_HMR_F_HMM_SFT, 1, 1),
	SOC_DAPM_SINGLE("PGAR Switch", RK618_HPMIX_CTL,
				RK618_HMR_F_PGAR_SFT, 1, 1),
	SOC_DAPM_SINGLE("PGAL Switch", RK618_HPMIX_CTL,
				RK618_HMR_F_PGAL_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACR Switch", RK618_HPMIX_CTL,
				RK618_HMR_F_DACR_SFT, 1, 1),
};

/* HP MUX */
static const char *hpl_sel[] = {"HPMIXL", "DACL"};

static const struct soc_enum hpl_sel_enum =
	SOC_ENUM_SINGLE(RK618_MUXHP_HPMIX_CTL, RK618_MHL_F_SFT,
			ARRAY_SIZE(hpl_sel), hpl_sel);

static const struct snd_kcontrol_new hpl_sel_mux =
	SOC_DAPM_ENUM("HPL select Mux", hpl_sel_enum);

static const char *hpr_sel[] = {"HPMIXR", "DACR"};

static const struct soc_enum hpr_sel_enum =
	SOC_ENUM_SINGLE(RK618_MUXHP_HPMIX_CTL, RK618_MHR_F_SFT,
			ARRAY_SIZE(hpr_sel), hpr_sel);

static const struct snd_kcontrol_new hpr_sel_mux =
	SOC_DAPM_ENUM("HPR select Mux", hpr_sel_enum);

/* MIC MUX */
static const char *mic_sel[] = {"BSTL", "BSTR"};

static const struct soc_enum mic_sel_enum =
	SOC_ENUM_SINGLE(RK618_MIXINL_CTL, RK618_MM_F_SFT,
			ARRAY_SIZE(mic_sel), mic_sel);

static const struct snd_kcontrol_new mic_sel_mux =
	SOC_DAPM_ENUM("Mic select Mux", mic_sel_enum);

/* MIXINR MUX */
static const char *mixinr_sel[] = {"DIFFIN", "IN1N"};

static const struct soc_enum mixinr_sel_enum =
	SOC_ENUM_SINGLE(RK618_DIFFIN_CTL, RK618_MIRM_F_SFT,
			ARRAY_SIZE(mixinr_sel), mixinr_sel);

static const struct snd_kcontrol_new mixinr_sel_mux =
	SOC_DAPM_ENUM("Mixinr select Mux", mixinr_sel_enum);

/* HPMIX MUX */
static const char *hpmix_sel[] = {"DIFFIN", "IN1N"};

static const struct soc_enum hpmix_sel_enum =
	SOC_ENUM_SINGLE(RK618_DIFFIN_CTL, RK618_HMM_F_SFT,
			ARRAY_SIZE(hpmix_sel), hpmix_sel);

static const struct snd_kcontrol_new hpmix_sel_mux =
	SOC_DAPM_ENUM("HPMix select Mux", hpmix_sel_enum);


static const struct snd_soc_dapm_widget rk618_dapm_widgets[] = {
	/* supply */
	SND_SOC_DAPM_SUPPLY("I2S0 Interface", RK618_IO_CON0,
		3, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("I2S1 Interface", RK618_IO_CON0,
		4, 1, NULL, 0),

	/* microphone bias */
	SND_SOC_DAPM_MICBIAS("Mic1 Bias", RK618_MICBIAS_CTL,
		RK618_MICBIAS1_PWRD_SFT, 1),
	SND_SOC_DAPM_MICBIAS("Mic2 Bias", RK618_MICBIAS_CTL,
		RK618_MICBIAS2_PWRD_SFT, 1),

	/* DACs */
	SND_SOC_DAPM_ADC_E("DACL", NULL, SND_SOC_NOPM,
		0, 0, rk618_dacl_event,
		SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_ADC_E("DACR", NULL, SND_SOC_NOPM,
		0, 0, rk618_dacr_event,
		SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),

	/* ADCs */
	SND_SOC_DAPM_ADC_E("ADCL", NULL, SND_SOC_NOPM,
		0, 0, rk618_adcl_event,
		SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_ADC_E("ADCR", NULL, SND_SOC_NOPM,
		0, 0, rk618_adcr_event,
		SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),

	/* PGA */
	SND_SOC_DAPM_PGA("BSTL", RK618_BST_CTL,
		RK618_BSTL_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("BSTR", RK618_BST_CTL,
		RK618_BSTR_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("DIFFIN", RK618_DIFFIN_CTL,
		RK618_DIFFIN_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("PGAL", RK618_PGAL_CTL,
		RK618_PGA_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("PGAR", RK618_PGAR_CTL,
		RK618_PGA_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("SPKL", RK618_SPKL_CTL,
		RK618_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("SPKR", RK618_SPKR_CTL,
		RK618_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("HPL", RK618_HPL_CTL,
		RK618_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("HPR", RK618_HPR_CTL,
		RK618_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("LINE1", RK618_LINEOUT1_CTL,
		RK618_LINEOUT_PWRD_SFT, 1, NULL, 0),
	SND_SOC_DAPM_PGA("LINE2", RK618_LINEOUT2_CTL,
		RK618_LINEOUT_PWRD_SFT, 1, NULL, 0),

	/* MIXER */
	SND_SOC_DAPM_MIXER("MIXINL", RK618_MIXINL_CTL,
		RK618_MIL_PWRD_SFT, 1, rk618_mixinl,
		ARRAY_SIZE(rk618_mixinl)),
	SND_SOC_DAPM_MIXER("MIXINR", RK618_MIXINR_CTL,
		RK618_MIR_PWRD_SFT, 1, rk618_mixinr,
		ARRAY_SIZE(rk618_mixinr)),
	SND_SOC_DAPM_MIXER("LINEMIX", RK618_LINEMIX_CTL,
		RK618_LM_PWRD_SFT, 1, rk618_linemix,
		ARRAY_SIZE(rk618_linemix)),
	SND_SOC_DAPM_MIXER("HPMIXL", RK618_MUXHP_HPMIX_CTL,
		RK618_HML_PWRD_SFT, 1, rk618_hpmixl,
		ARRAY_SIZE(rk618_hpmixl)),
	SND_SOC_DAPM_MIXER("HPMIXR", RK618_MUXHP_HPMIX_CTL,
		RK618_HMR_PWRD_SFT, 1, rk618_hpmixr,
		ARRAY_SIZE(rk618_hpmixr)),

	/* MUX */
	SND_SOC_DAPM_MUX("HPL Mux", SND_SOC_NOPM, 0, 0,
		&hpl_sel_mux),
	SND_SOC_DAPM_MUX("HPR Mux", SND_SOC_NOPM, 0, 0,
		&hpr_sel_mux),
	SND_SOC_DAPM_MUX("Mic Mux", SND_SOC_NOPM, 0, 0,
		&mic_sel_mux),
	SND_SOC_DAPM_MUX("MIXINR Mux", SND_SOC_NOPM, 0, 0,
		&mixinr_sel_mux),
	SND_SOC_DAPM_MUX("HPMix Mux", SND_SOC_NOPM, 0, 0,
		&hpmix_sel_mux),

	/* Audio Interface */
	SND_SOC_DAPM_AIF_IN("I2S0 DAC", "HiFi Playback", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S0 ADC", "HiFi Capture", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S1 DAC", "Voice Playback", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("I2S1 ADC", "Voice Capture", 0,
		SND_SOC_NOPM, 0, 0),

	/* Input */
	SND_SOC_DAPM_INPUT("IN3L"),
	SND_SOC_DAPM_INPUT("IN3R"),
	SND_SOC_DAPM_INPUT("IN1P"),
	SND_SOC_DAPM_INPUT("IN1N"),
	SND_SOC_DAPM_INPUT("MIC2P"),
	SND_SOC_DAPM_INPUT("MIC2N"),
	SND_SOC_DAPM_INPUT("MIC1P"),
	SND_SOC_DAPM_INPUT("MIC1N"),

	/* Output */
	SND_SOC_DAPM_OUTPUT("SPKOUTL"),
	SND_SOC_DAPM_OUTPUT("SPKOUTR"),
	SND_SOC_DAPM_OUTPUT("HPOUTL"),
	SND_SOC_DAPM_OUTPUT("HPOUTR"),
	SND_SOC_DAPM_OUTPUT("LINEOUT1"),
	SND_SOC_DAPM_OUTPUT("LINEOUT2"),
};

static const struct snd_soc_dapm_route rk618_dapm_routes[] = {
	{"I2S0 DAC", NULL, "I2S0 Interface"},
	{"I2S0 ADC", NULL, "I2S0 Interface"},
	{"I2S1 DAC", NULL, "I2S1 Interface"},
	{"I2S1 ADC", NULL, "I2S1 Interface"},

	/* Input */
	{"DIFFIN", NULL, "IN1P"},
	{"DIFFIN", NULL, "IN1N"},

	{"BSTR", NULL, "MIC2P"},
	{"BSTR", NULL, "MIC2N"},
	{"BSTL", NULL, "MIC1P"},
	{"BSTL", NULL, "MIC1N"},

	{"HPMix Mux", "DIFFIN", "DIFFIN"},
	{"HPMix Mux", "IN1N", "IN1N"},

	{"MIXINR Mux", "DIFFIN", "DIFFIN"},
	{"MIXINR Mux", "IN1N", "IN1N"},

	{"Mic Mux", "BSTR", "BSTR"},
	{"Mic Mux", "BSTL", "BSTL"},

	{"MIXINR", "MIC2N Switch", "MIC2N"},
	{"MIXINR", "IN1P Switch", "IN1P"},
	{"MIXINR", "IN3R Switch", "IN3R"},
	{"MIXINR", "MIXINR Mux Switch", "MIXINR Mux"},

	{"MIXINL", "IN3L Switch", "IN3L"},
	{"MIXINL", "IN1P Switch", "IN1P"},
	{"MIXINL", "MUXMIC Switch", "Mic Mux"},

	{"PGAR", NULL, "MIXINR"},
	{"PGAL", NULL, "MIXINL"},

	{"ADCR", NULL, "PGAR"},
	{"ADCL", NULL, "PGAL"},

	{"I2S0 ADC", NULL, "ADCR"},
	{"I2S0 ADC", NULL, "ADCL"},

	{"I2S1 ADC", NULL, "ADCR"},
	{"I2S1 ADC", NULL, "ADCL"},

	/* Output */
	{"DACR", NULL, "I2S0 DAC"},
	{"DACL", NULL, "I2S0 DAC"},

	{"DACR", NULL, "I2S1 DAC"},
	{"DACL", NULL, "I2S1 DAC"},

	{"LINEMIX", "PGAR Switch", "PGAR"},
	{"LINEMIX", "PGAL Switch", "PGAL"},
	{"LINEMIX", "DACR Switch", "DACR"},
	{"LINEMIX", "DACL Switch", "DACL"},

	{"HPMIXR", "HPMix Mux Switch", "HPMix Mux"},
	{"HPMIXR", "PGAR Switch", "PGAR"},
	{"HPMIXR", "PGAL Switch", "PGAL"},
	{"HPMIXR", "DACR Switch", "DACR"},

	{"HPMIXL", "HPMix Mux Switch", "HPMix Mux"},
	{"HPMIXL", "IN1P Switch", "IN1P"},
	{"HPMIXL", "PGAL Switch", "PGAL"},
	{"HPMIXL", "DACL Switch", "DACL"},

	{"HPR Mux", "DACR", "DACR"},
	{"HPR Mux", "HPMIXR", "HPMIXR"},
	{"HPL Mux", "DACL", "DACL"},
	{"HPL Mux", "HPMIXL", "HPMIXL"},

	{"LINE1", NULL, "LINEMIX"},
	{"LINE2", NULL, "LINEMIX"},
	{"SPKR", NULL, "HPR Mux"},
	{"SPKL", NULL, "HPL Mux"},
	{"HPR", NULL, "HPR Mux"},
	{"HPL", NULL, "HPL Mux"},

	{"LINEOUT1", NULL, "LINE1"},
	{"LINEOUT2", NULL, "LINE2"},
	{"SPKOUTR", NULL, "SPKR"},
	{"SPKOUTL", NULL, "SPKL"},
	{"HPOUTR", NULL, "HPR"},
	{"HPOUTL", NULL, "HPL"},
};

static int rk618_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
        snd_soc_component_update_bits(component, RK618_MICBIAS_CTL,
            RK618_MICBIAS2_PWRD | RK618_MICBIAS2_V_MASK,
            RK618_MICBIAS2_V_1_7);
        mdelay(100);
		break;

	case SND_SOC_BIAS_STANDBY:
		if (component->dapm.bias_level == SND_SOC_BIAS_OFF) {
			/* set power */
			snd_soc_component_update_bits(component, RK618_PWR_ADD1, 
				RK618_ADC_PWRD | RK618_DIFFIN_MIR_PGAR_RLPWRD |
				RK618_MIC1_MIC2_MIL_PGAL_RLPWRD |
				RK618_ADCL_RLPWRD | RK618_ADCR_RLPWRD, 0);

			snd_soc_component_update_bits(component, RK618_PWR_ADD2, 
				RK618_HPL_HPR_PWRD | RK618_DAC_PWRD |
				RK618_DACL_SPKL_RLPWRD | RK618_DACL_RLPWRD |
				RK618_DACR_SPKR_RLPWRD | RK618_DACR_RLPWRD |
				RK618_LM_LO_RLPWRD | RK618_HM_RLPWRD, 0);

			snd_soc_component_update_bits(component, RK618_PWR_ADD3, 
				RK618_ADCL_ZO_PWRD | RK618_ADCR_ZO_PWRD |
				RK618_DACL_ZO_PWRD | RK618_DACR_ZO_PWRD,
				RK618_ADCL_ZO_PWRD | RK618_ADCR_ZO_PWRD |
				RK618_DACL_ZO_PWRD | RK618_DACR_ZO_PWRD );

            snd_soc_component_update_bits(component, RK618_MICBIAS_CTL,
                RK618_MICBIAS2_PWRD | RK618_MICBIAS2_V_MASK,
                RK618_MICBIAS2_V_1_7);
		}
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_component_write(component, RK618_PWR_ADD1, rk618_reg_defaults[RK618_PWR_ADD1] & ~RK618_ADC_PWRD);
		snd_soc_component_write(component, RK618_PWR_ADD2, rk618_reg_defaults[RK618_PWR_ADD2]);
		snd_soc_component_write(component, RK618_PWR_ADD3, rk618_reg_defaults[RK618_PWR_ADD3]);
        snd_soc_component_update_bits(component, RK618_MICBIAS_CTL,
            RK618_MICBIAS1_PWRD,
            RK618_MICBIAS1_PWRD);
		break;
	}

	component->dapm.bias_level = level;

	return 0;
}

static int rk618_set_dai_sysclk(struct snd_soc_dai *component_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct rk618_codec_priv *rk618 = rk618_priv;

	if (!rk618) {
		printk("%s: rk618 is NULL\n", __func__);
		return -EINVAL;
	}

	rk618->stereo_sysclk = freq;

	//set I2S mclk for mipi
	clk_set_rate(rk618->mclk, freq);

	return 0;
}

static int rk618_set_dai_fmt(struct snd_soc_dai *component_dai,
			      unsigned int fmt)
{
	struct snd_soc_component *component = component_dai->component;
	unsigned int adc_aif1 = 0, adc_aif2 = 0, dac_aif1 = 0, dac_aif2 = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		adc_aif2 |= RK618_I2S_MODE_SLV;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		adc_aif2 |= RK618_I2S_MODE_MST;
		break;
	default:
		printk("%s : set master mask failed!\n", __func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		adc_aif1 |= RK618_ADC_DF_PCM;
		dac_aif1 |= RK618_DAC_DF_PCM;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		break;
	case SND_SOC_DAIFMT_I2S:
		adc_aif1 |= RK618_ADC_DF_I2S;
		dac_aif1 |= RK618_DAC_DF_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		adc_aif1 |= RK618_ADC_DF_RJ;
		dac_aif1 |= RK618_DAC_DF_RJ;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		adc_aif1 |= RK618_ADC_DF_LJ;
		dac_aif1 |= RK618_DAC_DF_LJ;
		break;
	default:
		printk("%s : set format failed!\n", __func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		adc_aif1 |= RK618_ALRCK_POL_DIS;
		adc_aif2 |= RK618_ABCLK_POL_DIS;
		dac_aif1 |= RK618_DLRCK_POL_DIS;
		dac_aif2 |= RK618_DBCLK_POL_DIS;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		adc_aif1 |= RK618_ALRCK_POL_EN;
		adc_aif2 |= RK618_ABCLK_POL_EN;
		dac_aif1 |= RK618_DLRCK_POL_EN;
		dac_aif2 |= RK618_DBCLK_POL_EN;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		adc_aif1 |= RK618_ALRCK_POL_DIS;
		adc_aif2 |= RK618_ABCLK_POL_EN;
		dac_aif1 |= RK618_DLRCK_POL_DIS;
		dac_aif2 |= RK618_DBCLK_POL_EN;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		adc_aif1 |= RK618_ALRCK_POL_EN;
		adc_aif2 |= RK618_ABCLK_POL_DIS;
		dac_aif1 |= RK618_DLRCK_POL_EN;
		dac_aif2 |= RK618_DBCLK_POL_DIS;
		break;
	default:
		printk("%s : set dai format failed!\n", __func__);
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, RK618_ADC_INT_CTL1,
			RK618_ALRCK_POL_MASK | RK618_ADC_DF_MASK, adc_aif1);
	snd_soc_component_update_bits(component, RK618_ADC_INT_CTL2,
			RK618_ABCLK_POL_MASK | RK618_I2S_MODE_MASK, adc_aif2);
	snd_soc_component_update_bits(component, RK618_DAC_INT_CTL1,
			RK618_DLRCK_POL_MASK | RK618_DAC_DF_MASK, dac_aif1);
	snd_soc_component_update_bits(component, RK618_DAC_INT_CTL2,
			RK618_DBCLK_POL_MASK, dac_aif2);

	return 0;
}

static int rk618_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = asoc_rtd_to_codec(rtd, 0)->component;
	struct rk618_codec_priv *rk618 = rk618_priv;
	unsigned int rate = params_rate(params);
	unsigned int div;
	unsigned int adc_aif1 = 0, adc_aif2  = 0, dac_aif1 = 0, dac_aif2  = 0;
	u32 mfd_aif1 = 0, mfd_aif2 = 0, mfd_i2s_ctl = 0;

	if (!rk618) {
		printk("%s : rk618 is NULL\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_SND_RK29_CODEC_SOC_MASTER
	// bclk = component_clk / 4
	// lrck = bclk / (wl * 2)
	div = (((rk618->stereo_sysclk / 4) / rate) / 2);

	if ((rk618->stereo_sysclk % (4 * rate * 2) > 0) ||
	    (div != 16 && div != 20 && div != 24 && div != 32)) {
		printk("%s : need PLL\n", __func__);
		return -EINVAL;
	}
#else
	//If component is slave mode, it don't need to set div
	//according to sysclk and rate.
	div = 32;
#endif

	switch (div) {
	case 16:
		adc_aif2 |= RK618_ADC_WL_16;
		dac_aif2 |= RK618_DAC_WL_16;
		break;
	case 20:
		adc_aif2 |= RK618_ADC_WL_20;
		dac_aif2 |= RK618_DAC_WL_20;
		break;
	case 24:
		adc_aif2 |= RK618_ADC_WL_24;
		dac_aif2 |= RK618_DAC_WL_24;
		break;
	case 32:
		adc_aif2 |= RK618_ADC_WL_32;
		dac_aif2 |= RK618_DAC_WL_32;
		break;
	default:
		return -EINVAL;
	}


	dev_dbg(rk618_priv->component->dev, "%s : MCLK = %dHz, sample rate = %dHz, div = %d\n", __func__,
		rk618->stereo_sysclk, rate, div);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		adc_aif1 |= RK618_ADC_VWL_16;
		dac_aif1 |= RK618_DAC_VWL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		adc_aif1 |= RK618_ADC_VWL_20;
		dac_aif1 |= RK618_DAC_VWL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		adc_aif1 |= RK618_ADC_VWL_24;
		dac_aif1 |= RK618_DAC_VWL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adc_aif1 |= RK618_ADC_VWL_32;
		dac_aif1 |= RK618_DAC_VWL_32;
		break;
	default:
		return -EINVAL;
	}

	/*switch (params_channels(params)) {
	case RK618_MONO:
		adc_aif1 |= RK618_ADC_TYPE_MONO;
		break;
	case RK618_STEREO:
		adc_aif1 |= RK618_ADC_TYPE_STEREO;
		break;
	default:
		return -EINVAL;
	}*/

	//MIC1N/P and MIC2N/P can only line to ADCL, so set mono type.
	adc_aif1 |= RK618_ADC_TYPE_MONO;

	adc_aif1 |= RK618_ADC_SWAP_DIS;
	adc_aif2 |= RK618_ADC_RST_DIS;
	dac_aif1 |= RK618_DAC_SWAP_DIS;
	dac_aif2 |= RK618_DAC_RST_DIS;

	rk618->rate = rate;

	snd_soc_component_update_bits(component, RK618_ADC_INT_CTL1,
			 RK618_ADC_VWL_MASK | RK618_ADC_SWAP_MASK |
			 RK618_ADC_TYPE_MASK, adc_aif1);
	snd_soc_component_update_bits(component, RK618_ADC_INT_CTL2,
			RK618_ADC_WL_MASK | RK618_ADC_RST_MASK, adc_aif2);
	snd_soc_component_update_bits(component, RK618_DAC_INT_CTL1,
			 RK618_DAC_VWL_MASK | RK618_DAC_SWAP_MASK, dac_aif1);
	snd_soc_component_update_bits(component, RK618_DAC_INT_CTL2,
			RK618_DAC_WL_MASK | RK618_DAC_RST_MASK, dac_aif2);

	switch (dai->id) {
	case RK618_HIFI:
		mfd_aif1 |= I2S1_OUTPUT_DISABLE | I2S0_IO_PULL_DOWN_DISABLE;
		mfd_aif2 |= I2S0_IO_SCHMITT_INPUT_ENABLE;
		mfd_i2s_ctl |= 0;
		break;
	case RK618_VOICE:
		mfd_aif1 |= I2S0_OUTPUT_DISABLE | I2S1_IO_PULL_DOWN_DISABLE;
		mfd_aif2 |= I2S1_IO_SCHMITT_INPUT_ENABLE;
		mfd_i2s_ctl |= I2S_CHANNEL_SEL | PCM_TO_I2S_MUX;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, RK618_IO_CON0,
			I2S1_OUTPUT_DISABLE | I2S0_OUTPUT_DISABLE |
			I2S1_IO_PULL_DOWN_DISABLE | I2S0_IO_PULL_DOWN_DISABLE, mfd_aif1);
	snd_soc_component_update_bits(component, RK618_IO_CON1,
			I2S1_IO_SCHMITT_INPUT_ENABLE | I2S0_IO_SCHMITT_INPUT_ENABLE, mfd_aif2);
	snd_soc_component_update_bits(component, RK618_PCM2IS2_CON2,
			APS_SEL | APS_CLR | I2S_CHANNEL_SEL,
			mfd_i2s_ctl);
	return 0;
}

static int rk618_digital_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct rk618_codec_priv *rk618 = rk618_priv;

	if (!rk618) {
		printk("%s : rk618_priv is NULL\n", __func__);
		return -EINVAL;
	}

	if (mute) {
        gpiod_direction_output(rk618->spk_ctl_gpio, 0);
        gpiod_direction_output(rk618->hp_ctl_gpio, 0);
        gpiod_direction_output(rk618->rcv_ctl_gpio, 0);
	} else {
		if (rk618->spk_gpio_level)
            gpiod_direction_output(rk618->spk_ctl_gpio, rk618->spk_gpio_level);

		if (rk618->hp_gpio_level)
            gpiod_direction_output(rk618->hp_ctl_gpio, rk618->hp_gpio_level);

		if (rk618->rcv_gpio_level)
            gpiod_direction_output(rk618->rcv_ctl_gpio, rk618->rcv_gpio_level);
	}

	return 0;
}

#define RK618_PLAYBACK_RATES (SNDRV_PCM_RATE_8000 |\
			      SNDRV_PCM_RATE_16000 |	\
			      SNDRV_PCM_RATE_32000 |	\
			      SNDRV_PCM_RATE_44100 |	\
			      SNDRV_PCM_RATE_48000 |	\
			      SNDRV_PCM_RATE_96000)

#define RK618_CAPTURE_RATES (SNDRV_PCM_RATE_8000 |\
			      SNDRV_PCM_RATE_16000 |	\
			      SNDRV_PCM_RATE_32000 |	\
			      SNDRV_PCM_RATE_44100 |	\
			      SNDRV_PCM_RATE_48000 |	\
			      SNDRV_PCM_RATE_96000)

#define RK618_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops rk618_dai_ops = {
	.hw_params	= rk618_hw_params,
	.set_fmt	= rk618_set_dai_fmt,
	.set_sysclk	= rk618_set_dai_sysclk,
	.mute_stream	= rk618_digital_mute,
};

static struct snd_soc_dai_driver rk618_dai[] = {
	{
		.name = "rk618-hifi",
		.id = RK618_HIFI,
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RK618_PLAYBACK_RATES,
			.formats = RK618_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RK618_CAPTURE_RATES,
			.formats = RK618_FORMATS,
		},
		.ops = &rk618_dai_ops,
	},
	{
		.name = "rk618-voice",
		.id = RK618_VOICE,
		.playback = {
			.stream_name = "Voice Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RK618_PLAYBACK_RATES,
			.formats = RK618_FORMATS,
		},
		.capture = {
			.stream_name = "Voice Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RK618_CAPTURE_RATES,
			.formats = RK618_FORMATS,
		},
		.ops = &rk618_dai_ops,
	},

};

static int rk618_suspend(struct snd_soc_component *component)
{
    rk618_set_bias_level(component, SND_SOC_BIAS_OFF);

	return 0;
}

static int rk618_resume(struct snd_soc_component *component)
{
    rk618_set_bias_level(component, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int rk618_probe(struct snd_soc_component *component)
{
	struct rk618_codec_priv *rk618_codec_data = snd_soc_component_get_drvdata(component);
	struct rk618 *rk618 = dev_get_drvdata(component->dev->parent);
    int ret;
    unsigned int val;

	snd_soc_component_init_regmap(component, rk618->regmap);
	rk618_codec_data->component = component;

	rk618_codec_data->spk_gpio_level = 0;
	rk618_codec_data->hp_gpio_level = 0;
	rk618_codec_data->rcv_gpio_level = 0;
	rk618_codec_data->mic_gpio_level = 0;

	rk618_priv = rk618_codec_data;

	val = snd_soc_component_read(component, RK618_RESET);
	if (val != rk618_reg_defaults[RK618_RESET]) {
		printk("%s : component register 0: %x is not a 0x00000003\n", __func__, val);
		ret = -ENODEV;
		goto err__;
	}

	rk618_reset(component);

	return 0;
err__:
	kfree(rk618_codec_data);
	rk618_codec_data = NULL;
	rk618_priv = NULL;

	return ret;
}

/* power down chip */
static void rk618_remove(struct snd_soc_component *component)
{
	struct rk618_codec_priv *rk618 = rk618_priv;

	dev_dbg(rk618_priv->component->dev, "%s\n", __func__);

	if (!rk618) {
		printk("%s : rk618_priv is NULL\n", __func__);
		return;
	}

    gpiod_direction_output(rk618->spk_ctl_gpio, 0);
    gpiod_direction_output(rk618->hp_ctl_gpio, 0);

	mdelay(10);

	snd_soc_component_write(component, RK618_RESET, 0xfc);
	mdelay(10);
	snd_soc_component_write(component, RK618_RESET, 0x3);
	mdelay(10);

	if (rk618)
		kfree(rk618);

    return;
}

static struct snd_soc_component_driver soc_codec_dev_rk618 = {
	.probe =	rk618_probe,
	.remove =	rk618_remove,
	.suspend =	rk618_suspend,
	.resume =	rk618_resume,
	.idle_bias_on = 1,
	.use_pmdown_time = 1,
	.endianness = 1,
	.set_bias_level = rk618_set_bias_level,
	.controls = rk618_snd_controls,
	.num_controls = ARRAY_SIZE(rk618_snd_controls),
	.dapm_routes = rk618_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rk618_dapm_routes),
	.dapm_widgets = rk618_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rk618_dapm_widgets),
};

static int rk618_codec_parse_dt_property(struct device *dev,
					 struct rk618_codec_priv *rk618)
{
	struct device_node *node;
    int ret = 0;

	node = of_get_child_by_name(dev->parent->of_node, "codec");
	if (!node) {
		dev_dbg(dev, "%s() Can not get child: codec\n",
			__func__);
	}

	rk618->spk_ctl_gpio = devm_gpiod_get_optional(dev, "spkctl", 0);
	if (IS_ERR(rk618->spk_ctl_gpio)) {
		ret = PTR_ERR(rk618->spk_ctl_gpio);
		dev_err(dev, "failed to request spkctl GPIO: %d\n", ret);
		return ret;
	}

	rk618->hp_ctl_gpio = devm_gpiod_get_optional(dev, "hpctl", 0);
	if (IS_ERR(rk618->hp_ctl_gpio)) {
		ret = PTR_ERR(rk618->hp_ctl_gpio);
		dev_err(dev, "failed to request hpctl GPIO: %d\n", ret);
		return ret;
	}

	rk618->rcv_ctl_gpio = devm_gpiod_get_optional(dev, "rcvctl", 0);
	if (IS_ERR(rk618->rcv_ctl_gpio)) {
		ret = PTR_ERR(rk618->rcv_ctl_gpio);
		dev_err(dev, "failed to request rcvctl GPIO: %d\n", ret);
		return ret;
	}

	rk618->mic_sel_gpio = devm_gpiod_get_optional(dev, "micsel", 0);
	if (IS_ERR(rk618->mic_sel_gpio)) {
		ret = PTR_ERR(rk618->mic_sel_gpio);
		dev_err(dev, "failed to request micsel GPIO: %d\n", ret);
		return ret;
	}

	of_node_put(node);

    return ret;
}

static int rk618_platform_probe(struct platform_device *pdev)
{
	struct rk618 *rk618 = dev_get_drvdata(pdev->dev.parent);
	struct rk618_codec_priv *rk618_codec_data;
	int ret;

	rk618_codec_data = devm_kzalloc(&pdev->dev,
					sizeof(struct rk618_codec_priv),
					GFP_KERNEL);
	if (!rk618_codec_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, rk618_codec_data);

	rk618_codec_data->rk618 = rk618;

	ret = rk618_codec_parse_dt_property(&pdev->dev, rk618_codec_data);

    if (ret < 0) {
        goto err_;
    }

	rk618_codec_data->mclk = devm_clk_get(pdev->dev.parent, "clkin");
	if (IS_ERR(rk618_codec_data->mclk)) {
		dev_dbg(&pdev->dev, "Unable to get clkin\n");
		ret = -ENXIO;
		goto err_;
	}

	ret = clk_prepare_enable(rk618_codec_data->mclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s() clock prepare error %d\n",
			__func__, ret);
		goto err_;
	}

	ret = devm_snd_soc_register_component(&pdev->dev, &soc_codec_dev_rk618,
					      rk618_dai, ARRAY_SIZE(rk618_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "%s() register codec error %d\n",
			__func__, ret);
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable_unprepare(rk618_codec_data->mclk);
err_:
	return ret;
}

void rk618_platform_shutdown(struct platform_device *pdev)
{
	struct rk618_codec_priv *rk618 = rk618_priv;
	struct snd_soc_component *component;

	dev_dbg(rk618_priv->component->dev, "%s\n", __func__);

	if (!rk618 || !rk618->component) {
		printk("%s : rk618_priv or rk618_priv->codec is NULL\n", __func__);
		return;
	}

	component = rk618->component;

    gpiod_direction_output(rk618->spk_ctl_gpio, 0);
    gpiod_direction_output(rk618->hp_ctl_gpio, 0);

	mdelay(10);

	snd_soc_component_write(component, RK618_RESET, 0xfc);
	mdelay(10);
	snd_soc_component_write(component, RK618_RESET, 0x3);

	if (rk618)
		kfree(rk618);
}

static const struct of_device_id rk618_codec_of_match[] = {
	{ .compatible = "rockchip,rk618-codec", },
	{},
};
MODULE_DEVICE_TABLE(of, rk618_codec_of_match);

static void rk618_platform_remove(struct platform_device *pdev)
{
	struct rk618_codec_priv *rk618 = platform_get_drvdata(pdev);

	clk_disable_unprepare(rk618->mclk);
}

static struct platform_driver rk618_codec_driver = {
	.driver = {
		   .name = "rk618-codec",
           .of_match_table = of_match_ptr(rk618_codec_of_match),
		   },
	.probe = rk618_platform_probe,
	.remove_new = rk618_platform_remove,
};

module_platform_driver(rk618_codec_driver);

MODULE_DESCRIPTION("ASoC RK618 driver");
MODULE_AUTHOR("chenjq <chenjq@rock-chips.com>");
MODULE_LICENSE("GPL");
